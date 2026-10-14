#!/usr/bin/env python3
"""Pi camera/heartbeat bridge ONLY. All Task 2 motion decisions are on STM32.

Default is a description, with no serial connection. --execute authorizes motion.
Stop the old KC_robot_monitor first: one process must own the serial port.
"""
import argparse
import concurrent.futures
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time

STAGES = {'o1': 1, 'o2-prepare': 2, 'o2-bypass': 3, 'full': 0, 'servo-test': 4}


def frame(command_id, component, command, first, second):
    return f':{command_id}/{component}/{command}/{first}/{second};'.encode('ascii')


class Camera:
    """Keeps the camera open. rpicam-still restarts the camera for every photo
    (over a second each); a running Picamera2 returns a frame in ~50 ms."""

    def __init__(self):
        self.cam = None
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            cam.configure(cam.create_still_configuration(main={'size': (640, 480)}, buffer_count=2))
            try:
                # Prefer short exposures: frames taken at 60-70 cm/s came back blank.
                from libcamera import controls
                cam.set_controls({'AeExposureMode': controls.AeExposureModeEnum.Short})
            except Exception as exc:
                print(f'[CAMERA] short exposure not set: {exc}', flush=True)
            cam.start()
            time.sleep(0.5)  # auto exposure settles once, before the run
            self.cam = cam
            print('[CAMERA] Picamera2 running (continuous capture)', flush=True)
        except Exception as exc:
            print(f'[CAMERA] Picamera2 unavailable ({exc}); using rpicam-still', flush=True)

    def capture(self, path):
        if self.cam is not None:
            self.cam.capture_file(str(path))
            return
        subprocess.run(['rpicam-still', '-t', '500', '-o', str(path),
                        '--width', '640', '--height', '480', '-n'],
                       check=True, timeout=4, capture_output=True)

    def close(self):
        if self.cam is not None:
            try:
                self.cam.stop()
                self.cam.close()
            except Exception:
                pass


def capture_arrow(vision, folder, obstacle, cancelled, stationary=None, camera=None):
    """Keep classifying frames while the robot drives; up to four more at a stop.

    While moving, accept one frame >= 75%, or two frames >= 50% for the same side
    with none for the other side (blank frames in between do not reset the count),
    so a single blurred frame cannot steer the robot. Stopped: one frame >= 50%.
    Only a stationary failure is terminal. One worker owns the camera throughout.
    """
    folder.mkdir(parents=True, exist_ok=False)
    if stationary is None:
        stationary = threading.Event()
        stationary.set()
    if camera is None:
        camera = Camera()
    url = next(url for kind, url in vision.backends if kind == 'image-api')
    stopped_attempts = attempt = 0
    stopped_deadline = None
    votes = {'left': 0, 'right': 0}
    while not cancelled.is_set():
        stopped = stationary.is_set()
        if stopped:
            if stopped_deadline is None:
                stopped_deadline = time.monotonic() + 20
            if stopped_attempts >= 4 or time.monotonic() >= stopped_deadline:
                raise RuntimeError('No arrow met 50% confidence after stationary retries')
            stopped_attempts += 1
        image = folder / f'o{obstacle}_{attempt}.jpg'
        attempt += 1
        try:
            camera.capture(image)
            side, confidence = vision._query('image-api', url, image)
            print(f'[VISION] O{obstacle} {"stationary" if stopped else "moving"}: '
                  f'{side}, confidence={confidence:.3f}, image={image}', flush=True)
            if cancelled.is_set():
                break
            good = side in ('left', 'right') and math.isfinite(confidence) and confidence >= 0.50
            if good and (stopped or confidence >= 0.75):
                return side
            if good:
                other = 'right' if side == 'left' else 'left'
                if votes[other]:
                    votes = {'left': 0, 'right': 0}  # frames disagree: start counting again
                votes[side] += 1
                if votes[side] >= 2:
                    return side
        except Exception as exc:
            print(f'[VISION RETRY] O{obstacle}: {exc}', flush=True)
            time.sleep(0.1)
    raise RuntimeError('Capture cancelled')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--laptop-ip', default='192.168.50.218')
    parser.add_argument('--stage', choices=STAGES, default='o1')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    if not args.execute:
        print(f'DRY RUN: native STM32 TASK2KC, stage={args.stage}; Pi supplies camera arrows and heartbeat only.')
        print('Requires KC_Task2_Fast_O1.hex (Claude fast-O1 build). Stop KC_robot_monitor first. Add --execute to start movement.')
        return 0
    import serial
    from KC_stm32_link import VisionClient
    vision = VisionClient(args.laptop_ip, timeout=4)
    cancelled = threading.Event()
    write_lock = threading.Lock()
    incoming = queue.Queue()
    run_id = int(time.time_ns() % 1_000_000_000) + 1000
    command_id = run_id
    last_write = 0.0
    log_path = Path(f'KC_native_task2_{time.time_ns()}.jsonl')
    capture_root = Path(f'KC_native_images_{time.time_ns()}')
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    capture = None
    camera = None
    requested = set()
    stationary_requests = {}
    started = False
    serial_link = None
    try:
        if args.stage != 'servo-test':
            # Open the camera BEFORE the serial port, so no libcamera helper
            # process can inherit (and keep locked) /dev/ttyACM0 after we exit.
            camera = Camera()
        serial_link = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1, exclusive=True)
        def send(component, command, first, second, identity=None):
            nonlocal command_id, last_write
            with write_lock:
                # Legacy firmware consumes a single UART frame every 5 ms.
                # Leave a scheduling margin so the next colon cannot erase it.
                delay = 0.03 - (time.monotonic() - last_write)
                if delay > 0:
                    time.sleep(delay)
                if identity is None:
                    command_id += 1
                    identity = command_id
                serial_link.write(frame(identity, component, command, first, second))
                serial_link.flush()
                last_write = time.monotonic()
            return identity
        def reader():
            pending = bytearray()
            try:
                while not cancelled.is_set():
                    data = serial_link.read(256)
                    pending.extend(data)
                    while b';' in pending:
                        end = pending.index(b';')
                        raw = bytes(pending[:end+1]).decode('ascii', errors='replace')
                        del pending[:end+1]
                        if '!' in raw:
                            incoming.put(raw[raw.index('!'):])
                    if len(pending)>4096:
                        pending.clear()
            except Exception as exc:
                incoming.put(exc)
        threading.Thread(target=reader, daemon=True).start()
        # Identify firmware without movement. Old firmware cannot pass this check.
        send('T2', 'INFO', 0, 0)
        deadline = time.monotonic()+3
        identified = False
        while time.monotonic()<deadline:
            try: item=incoming.get(timeout=0.1)
            except queue.Empty: continue
            if isinstance(item, Exception): raise item
            if item=='!KC_T2/BUILD/2/0;': identified=True;break
        if not identified:
            raise RuntimeError('TASK2KC firmware not identified; no motion command sent')
        send('MOTOR', 'TASK2KC', 50, STAGES[args.stage], identity=run_id)
        started = True
        # START must be consumed before PING: otherwise the single UART buffer
        # can discard START and every heartbeat reports TASK2KC_TOKEN.
        startup_deadline = time.monotonic() + 3
        acknowledged = False
        while time.monotonic() < startup_deadline:
            try: item = incoming.get(timeout=0.1)
            except queue.Empty: continue
            if isinstance(item, Exception): raise item
            if not item.startswith('!SENSOR/'):
                print(item, flush=True)
            if item.startswith(f'!{run_id}/ERROR/') or item == f'!{run_id}/ABORTED;':
                raise RuntimeError(f'STM32 rejected START: {item}')
            if item == '!KC_T2/BUILD_TASK2KC/2.00/0.00;':
                acknowledged = True
                break
        if not acknowledged:
            raise RuntimeError('STM32 did not confirm START; stopping instead of sending heartbeats')
        next_ping=0
        finish_deadline=time.monotonic()+185
        with log_path.open('w') as log:
            while time.monotonic()<finish_deadline:
                if time.monotonic()>=next_ping:
                    send('T2','PING',0,run_id);next_ping=time.monotonic()+0.5
                if capture is not None and capture[2].done():
                    obstacle,token,future=capture
                    try: side=future.result()
                    except Exception as exc:
                        print(f'[VISION ERROR] {exc}',flush=True)
                        send('T2','FAIL',obstacle,token)
                    else: send('T2',side.upper(),obstacle,token)
                    capture=None
                try:item=incoming.get(timeout=0.05)
                except queue.Empty:continue
                if isinstance(item,Exception):raise item
                log.write(json.dumps({'time':time.time(),'frame':item})+'\n');log.flush()
                if not item.startswith('!SENSOR/'):
                    print(item,flush=True)
                if item.startswith(('!KC_T2/CAPTURE/', '!KC_T2/CAPTURE_EARLY/')):
                    parts=item.rstrip(';').split('/')
                    obstacle,token=int(parts[2]),int(parts[3])
                    if token!=run_id or obstacle not in (1,2):continue
                    key=(obstacle,token)
                    stationary = stationary_requests.setdefault(key, threading.Event())
                    if parts[1] == 'CAPTURE':
                        stationary.set()
                    if key not in requested:
                        if capture is not None:raise RuntimeError('Overlapping camera requests')
                        requested.add(key)
                        future=executor.submit(capture_arrow,vision,capture_root/f'O{obstacle}',obstacle,cancelled,stationary,camera)
                        capture=(obstacle,token,future)
                if item==f'!{run_id}/DONE;':
                    print(f'Native stage {args.stage} completed. Log: {log_path}',flush=True)
                    return 0
                if item.startswith(f'!{run_id}/ERROR/') or item==f'!{run_id}/ABORTED;':
                    raise RuntimeError(f'STM32 stopped: {item}; log: {log_path}')
        raise RuntimeError('Native run time limit reached')
    except KeyboardInterrupt:
        print('STOP requested.',flush=True)
        return 130
    finally:
        cancelled.set()
        # A valid STOP is sent even if capture or serial reading fails.
        if serial_link is not None:
            if started:
                try:
                    with write_lock:serial_link.write(frame(command_id+1,'MOTOR','STOP',0,0))
                except Exception:pass  # STM32 heartbeat watchdog is independent.
            serial_link.close()
        executor.shutdown(wait=False,cancel_futures=True)
        if camera is not None:
            camera.close()


if __name__=='__main__':
    raise SystemExit(main())
