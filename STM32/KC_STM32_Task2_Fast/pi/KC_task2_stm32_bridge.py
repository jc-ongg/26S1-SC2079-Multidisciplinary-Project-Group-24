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

STAGES = {'o1': 1, 'o2-prepare': 2, 'o2-bypass': 3, 'full': 0, 'servo-test': 4,
          'full-pi-return': 3}  # native O1+O2, then the Pi drives the two-step return (KC_task2_pi_return.py)


RUN_T0 = [None]   # monotonic time of the acknowledged START: every printed line carries the run time


def tsay(text, flush=True):
    """Print with the run time so far, e.g. '[ 12.34 s] !KC_T2/STATE/...'."""
    t0 = RUN_T0[0]
    print(f'[{time.monotonic() - t0:6.2f} s] {text}' if t0 is not None else text, flush=True)


def frame(command_id, component, command, first, second):
    return f':{command_id}/{component}/{command}/{first}/{second};'.encode('ascii')


class Camera:
    """Keeps the camera open. rpicam-still restarts the camera for every photo
    (over a second each); a running Picamera2 returns a frame in ~50 ms.

    full_sensor=False keeps the 640x480 sensor mode, which only sees the centre
    1280x960 of the IMX219 (about +/-13 deg). full_sensor=True reads the whole
    sensor (1640x1232 binned) and set_view() moves a window of that same size
    and zoom left or right, so side views keep the straight-on calibration.
    """

    SENSOR = (3280, 2464)
    WINDOW = (1280, 960)

    def __init__(self, full_sensor=False):
        self.cam = None
        self.full_sensor = False
        try:
            from picamera2 import Picamera2
            cam = Picamera2()
            config = None
            if full_sensor:
                try:
                    config = cam.create_still_configuration(main={'size': (640, 480)}, buffer_count=2,
                                                            sensor={'output_size': (1640, 1232)})
                    self.full_sensor = True
                except Exception as exc:
                    print(f'[CAMERA] full-sensor mode unavailable ({exc}); using the centre-only mode', flush=True)
            cam.configure(config or cam.create_still_configuration(main={'size': (640, 480)}, buffer_count=2))
            try:
                # Prefer short exposures: frames taken at 60-70 cm/s came back blank.
                from libcamera import controls
                cam.set_controls({'AeExposureMode': controls.AeExposureModeEnum.Short})
            except Exception as exc:
                print(f'[CAMERA] short exposure not set: {exc}', flush=True)
            cam.start()
            time.sleep(0.5)  # auto exposure settles once, before the run
            self.cam = cam
            if self.full_sensor:
                self.set_view(0.0)
            print(f'[CAMERA] Picamera2 running ({"full sensor" if self.full_sensor else "centre-only"} mode)', flush=True)
        except Exception as exc:
            print(f'[CAMERA] Picamera2 unavailable ({exc}); using rpicam-still', flush=True)

    def set_view(self, shift):
        """shift -1 = window at the far left of the sensor, 0 = centre, +1 = far right
        (about 20 deg off centre). Takes a few frames to apply."""
        if self.cam is None or not self.full_sensor:
            return False
        shift = max(-1.0, min(1.0, shift))
        x = int((self.SENSOR[0] - self.WINDOW[0]) / 2 * (1 + shift))
        y = (self.SENSOR[1] - self.WINDOW[1]) // 2
        self.cam.set_controls({'ScalerCrop': (x, y, self.WINDOW[0], self.WINDOW[1])})
        time.sleep(0.15)
        return True

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


def pick_arrow(detections):
    """Best real-looking arrow box, or None.

    Arrow cards come out roughly square. The model also fires on poster icons
    (long thin boxes, e.g. 39x14) and on slivers cut off at the picture edge;
    a calibration photo with no arrow in view came back RIGHT 0.58 that way.
    """
    good = []
    for d in detections:
        if d.get('class') not in ('left', 'right'):
            continue
        x0, y0, x1, y1 = d['xyxy']
        w, h = x1 - x0, y1 - y0
        # A card 25 cm from the bumper is about 200 px wide; bigger boxes are not cards.
        if w < 18 or h < 18 or w > 260 or not 0.5 <= w / h <= 1.6:
            continue
        if (x0 <= 3 or x1 >= 637) and w < 90:  # small box cut off at the edge
            continue
        good.append(d)
    return max(good, key=lambda d: d['confidence'], default=None)


def query_filtered(url, image, timeout=4):
    """Ask the laptop API, then choose the arrow ourselves with pick_arrow."""
    import requests
    with open(image, 'rb') as f:
        response = requests.post(url, files={'image': (image.name, f, 'image/jpeg')}, timeout=timeout)
    response.raise_for_status()
    data = response.json()
    if 'detections' not in data:  # older API without the detection list
        side = str(data.get('image_id', '')).strip().lower()
        return (side, float(data.get('conf', 0.0))) if side in ('left', 'right') else (None, 0.0)
    best = pick_arrow(data['detections'])
    return (best['class'], float(best['confidence'])) if best else (None, 0.0)


# The STM32 no longer guesses an arrow (KC_NO_STALL): never report FAIL. At a
# stop, after four frames under 50%, two agreeing frames >= 35% (none for the
# other side) are the model's answer; otherwise keep capturing, and keep
# retrying an unreachable server (fix --laptop-ip; the robot waits meanwhile).
VISION_KEEP_TRYING = True
WEAK_CONFIDENCE = 0.35


def capture_arrow(vision, folder, obstacle, cancelled, stationary=None, camera=None, query=None):
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
    weak = {'left': 0, 'right': 0}         # stationary frames >= WEAK_CONFIDENCE
    while not cancelled.is_set():
        stopped = stationary.is_set()
        if stopped:
            if stopped_deadline is None:
                stopped_deadline = time.monotonic() + 20
            if stopped_attempts >= 4 or time.monotonic() >= stopped_deadline:
                if not VISION_KEEP_TRYING:
                    raise RuntimeError('No arrow met 50% confidence after stationary retries')
                for s in ('left', 'right'):
                    other = 'right' if s == 'left' else 'left'
                    if weak[s] >= 2 and not weak[other]:
                        tsay(f'[VISION] O{obstacle}: {s} from {weak[s]} stationary frames >= {WEAK_CONFIDENCE:.0%}', flush=True)
                        return s
            stopped_attempts += 1
        image = folder / f'o{obstacle}_{attempt}.jpg'
        attempt += 1
        try:
            camera.capture(image)
            side, confidence = query(url, image) if query else vision._query('image-api', url, image)
            tsay(f'[VISION] O{obstacle} {"stationary" if stopped else "moving"}: '
                  f'{side}, confidence={confidence:.3f}, image={image}', flush=True)
            if cancelled.is_set():
                break
            good = side in ('left', 'right') and math.isfinite(confidence) and confidence >= 0.50
            if good and (stopped or confidence >= 0.75):
                return side
            if (stopped and side in ('left', 'right') and math.isfinite(confidence)
                    and confidence >= WEAK_CONFIDENCE):
                weak[side] += 1
            if good:
                other = 'right' if side == 'left' else 'left'
                if votes[other]:
                    votes = {'left': 0, 'right': 0}  # frames disagree: start counting again
                votes[side] += 1
                if votes[side] >= 2:
                    return side
        except Exception as exc:
            tsay(f'[VISION RETRY] O{obstacle}: {exc}  (robot waits; check --laptop-ip and the server)')
            time.sleep(0.1)
    raise RuntimeError('Capture cancelled')


class BridgeIO:
    """Plain motor commands after a native run, for KC_task2_pi_return."""

    def __init__(self, send, incoming, log):
        self.send, self.incoming, self.logf = send, incoming, log

    @staticmethod
    def parse(item):
        p = item.rstrip(';').split('/')
        try:
            return {'left': float(p[1]), 'right': float(p[2]), 'us': float(p[3]),
                    'us_valid': int(p[4]) == 1, 'heading': float(p[5])}
        except (IndexError, ValueError):
            return None

    def _next(self, timeout):
        item = self.incoming.get(timeout=timeout)
        if isinstance(item, Exception):
            raise item
        self.logf.write(json.dumps({'time': time.time(), 'frame': item}) + '\n'); self.logf.flush()
        if not item.startswith('!SENSOR/'):
            tsay(item)
        return item

    def move(self, command, speed, value, on_sensor=None, timeout=20.0):
        ident = self.send('MOTOR', command, int(speed), int(value))
        tsay(f'[RETURN] {command} speed {speed} {value}')
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                item = self._next(0.2)
            except queue.Empty:
                continue
            if item.startswith('!SENSOR/'):
                if on_sensor is not None:
                    frame = self.parse(item)
                    if frame:
                        on_sensor(frame)
            elif item == f'!{ident}/DONE;':
                return
            elif item.startswith(f'!{ident}/'):
                raise RuntimeError(f'{command} {value} failed: {item}')
        raise RuntimeError(f'{command} {value} timed out')

    def sensors(self, n, timeout=3.0):
        frames, deadline = [], time.monotonic() + timeout
        while len(frames) < n and time.monotonic() < deadline:
            try:
                item = self._next(0.2)
            except queue.Empty:
                continue
            if item.startswith('!SENSOR/'):
                frame = self.parse(item)
                if frame:
                    frames.append(frame)
        if len(frames) < n:
            raise RuntimeError('no SENSOR telemetry from the STM32')
        return frames

    def log(self, text):
        tsay(text)
        self.logf.write(json.dumps({'time': time.time(), 'frame': text}) + '\n'); self.logf.flush()


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
    timing = {'t0': None, 'reported': False}   # run timer: START acknowledged -> end
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
        timing['t0'] = time.monotonic()
        RUN_T0[0] = timing['t0']

        def report(log, label, native_s=None):
            total = time.monotonic() - timing['t0']
            parts = f'{label}: {total:.1f} s'
            if native_s is not None:
                parts += f' (native O1+O2 {native_s:.1f} s, Pi return {total - native_s:.1f} s)'
            print(f'\n===== RUN TIME {parts} =====', flush=True)
            log.write(json.dumps({'time': time.time(), 'frame': f'RUN_TIME {parts}'}) + '\n'); log.flush()
            timing['reported'] = True
        next_ping=0
        native_frames=[]
        finish_deadline=time.monotonic()+185
        with log_path.open('w') as log:
            while time.monotonic()<finish_deadline:
                if time.monotonic()>=next_ping:
                    send('T2','PING',0,run_id);next_ping=time.monotonic()+0.5
                if capture is not None and capture[2].done():
                    obstacle,token,future=capture
                    try: side=future.result()
                    except Exception as exc:
                        tsay(f'[VISION ERROR] {exc}')
                        send('T2','FAIL',obstacle,token)
                    else: send('T2',side.upper(),obstacle,token)
                    capture=None
                try:item=incoming.get(timeout=0.05)
                except queue.Empty:continue
                if isinstance(item,Exception):raise item
                log.write(json.dumps({'time':time.time(),'frame':item})+'\n');log.flush()
                if not item.startswith('!SENSOR/'):
                    tsay(item)
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
                        future=executor.submit(capture_arrow,vision,capture_root/f'O{obstacle}',obstacle,cancelled,stationary,camera,query_filtered)
                        capture=(obstacle,token,future)
                if item.startswith('!KC_T2/'):native_frames.append(item)
                if item==f'!{run_id}/DONE;' and args.stage=='full-pi-return':
                    from KC_task2_pi_return import PiReturn, facts_from_frames
                    facts=facts_from_frames(native_frames)
                    tsay(f'[RETURN] native O1+O2 done; facts {facts}')
                    native_s=time.monotonic()-timing['t0']
                    tsay(f'[RETURN] native O1+O2 took {native_s:.1f} s')
                    try:
                        PiReturn(BridgeIO(send,incoming,log),facts).run()
                    except Exception:
                        report(log,'stopped during the Pi return after',native_s)
                        raise
                    print(f'Native O1+O2 and Pi return completed. Log: {log_path}',flush=True)
                    report(log,'START to parked',native_s)
                    return 0
                if item==f'!{run_id}/DONE;':
                    print(f'Native stage {args.stage} completed. Log: {log_path}',flush=True)
                    report(log,f'START to end of stage {args.stage}')
                    return 0
                if item.startswith(f'!{run_id}/ERROR/') or item==f'!{run_id}/ABORTED;':
                    report(log,f'stopped ({item.strip(";")}) after')
                    raise RuntimeError(f'STM32 stopped: {item}; log: {log_path}')
        raise RuntimeError('Native run time limit reached')
    except KeyboardInterrupt:
        print('STOP requested.',flush=True)
        return 130
    finally:
        if timing['t0'] is not None and not timing['reported']:
            print(f"\n===== RUN TIME: stopped after {time.monotonic() - timing['t0']:.1f} s =====", flush=True)
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
