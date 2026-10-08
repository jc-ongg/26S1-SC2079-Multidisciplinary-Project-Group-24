#!/usr/bin/env python3
import json
import re
import queue
import time
import subprocess
from multiprocessing import Process, Manager, Event
import signal
from typing import Optional
import os
from pathlib import Path
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT, API_PORT_IMAGE, API_IP_IMAGE
from path_plan import parse_plan
from recognition import normalize_recognition
from camera import WarmCamera

# A missing STM completion pauses dispatch; it must never permit another move.
# Camera retries run synchronously and do not use this motor ACK deadline.
# A long compressed straight can take more than 15 s at the current speed.
# This is a completion watchdog, not an inter-command animation delay.
ACK_TIMEOUT = 60
ALGO_TIMEOUT = (3, 180)  # Connection timeout, then up to three minutes for planning.
CAPTURE_ATTEMPTS = 3  # Up to 3 photos; stops at the first recognised one (warm camera: ~1 s each).
CAPTURE_TIMEOUT = 10
IMAGE_API_TIMEOUT = (3, 10)
CAPTURE_DIR = Path(__file__).resolve().parent / 'captures'


def run_worker(target):
    # Ctrl-C is handled once by the parent, which stops and joins its workers.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    target()


class PiAction:
    """
    Class that represents an action that the RPi needs to take.
    """

    def __init__(self, cat, value):
        """
        :param cat: The category of the action. Can be 'info', 'mode', 'path', 'snap', 'obstacle', 'location', 'failed', 'success'
        :param value: The value of the action. Can be a string, a list of coordinates, or a list of obstacles.
        """
        self._cat = cat
        self._value = value

    @property
    def cat(self):
        return self._cat

    @property
    def value(self):
        return self._value


class RaspberryPi:
    """
    Class that represents the Raspberry Pi.
    """

    def __init__(self):
        """
        Initializes the Raspberry Pi.
        """
        self.logger = prepare_logger()
        self.android_link = AndroidLink()
        # Keep sensor readings active, but silence the ~8 Hz values in Task 1 logs.
        self.stm_link = STMLink(log_telemetry=False)

        self.manager = Manager()

        # Interrupting a blocking Manager.Event.wait leaves its RPC pending;
        # a subsequent clear on that connection can hang during shutdown.
        self.android_dropped = Event()
        self.unpause = Event()

        self.movement_lock = self.manager.Lock()
        self.motion_fault = Event()
        self.stm_ack_guard = self.manager.Lock()
        self.stm_pending_id = self.manager.Value('i', 0)
        self.stm_pending_location = self.manager.dict()
        self.stm_pending_manual = self.manager.dict()
        self.plan_guard = self.manager.Lock()
        self.plan_state = self.manager.dict(id='', revision=0, status='unsent')

        self.android_queue = self.manager.Queue()  # Messages to send to Android
        # Messages that need to be processed by RPi
        self.rpi_action_queue = self.manager.Queue()
        # Messages that need to be processed by STM32, as well as snap commands
        self.command_queue = self.manager.Queue()
        # X,Y,D coordinates of the robot after execution of a command
        self.path_queue = self.manager.Queue()

        self.proc_recv_android = None
        self.proc_recv_stm32 = None
        self.proc_android_sender = None
        self.proc_command_follower = None
        self.proc_rpi_action = None
        self.rs_flag = False
        self.success_obstacles = self.manager.list()
        self.failed_obstacles = self.manager.list()
        self.recognized_images = self.manager.dict()
        self.obstacles = self.manager.dict()
        self.current_location = self.manager.dict()
        self.failed_attempt = False

    def start(self):
        """Starts the RPi orchestrator"""
        try:
            ### Start up initialization ###

            self.android_link.connect()
            self.android_queue.put(AndroidMessage(
                'info', 'You are connected to the RPi!'))
            if not self.check_api():
                self.android_link.send(AndroidMessage(
                    'error', 'Laptop Algo/CV is not ready with best_v11i_m.onnx. Start both laptop servers, then restart Task 1.'))
                return
            self.stm_link.connect()

            # Define child processes
            self.proc_recv_android = Process(target=run_worker, args=(self.recv_android,))
            self.proc_recv_stm32 = Process(target=run_worker, args=(self.recv_stm,))
            self.proc_android_sender = Process(target=run_worker, args=(self.android_sender,))
            self.proc_command_follower = Process(target=run_worker, args=(self.command_follower_with_camera,))
            self.proc_rpi_action = Process(target=run_worker, args=(self.rpi_action,))

            # Start child processes
            self.proc_recv_android.start()
            self.proc_recv_stm32.start()
            self.proc_android_sender.start()
            self.proc_command_follower.start()
            self.proc_rpi_action.start()

            self.logger.info("Child Processes started")

            ### Start up complete ###

            # Send success message to Android
            self.android_queue.put(AndroidMessage('info', 'Robot is ready!'))
            self.android_queue.put(AndroidMessage('mode', 'path'))

            self.reconnect_android()

        except KeyboardInterrupt:
            self.logger.info('Task 1 interrupted.')
        finally:
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.unpause.clear()
        workers = [self.proc_recv_android, self.proc_recv_stm32,
                   self.proc_android_sender, self.proc_command_follower, self.proc_rpi_action]
        for worker in workers:
            if worker is not None and worker.is_alive():
                worker.terminate()
        for worker in workers:
            if worker is not None and worker.pid is not None:
                worker.join(timeout=2)
                if worker.is_alive():
                    worker.kill()
                    worker.join(timeout=2)
        self.android_link.disconnect()
        self.stm_link.disconnect()
        self.manager.shutdown()
        self.logger.info("Program exited!")

    def reconnect_android(self):
        """Handles the reconnection to Android in the event of a lost connection."""
        self.logger.info("Reconnection handler is watching...")

        while True:
            # Wait for android connection to drop
            self.android_dropped.wait()

            self.logger.error("Android link is down!")

            # Kill child processes
            self.logger.debug("Killing android child processes")
            self.proc_android_sender.kill()
            self.proc_recv_android.kill()

            # Wait for the child processes to finish
            self.proc_android_sender.join()
            self.proc_recv_android.join()
            assert self.proc_android_sender.is_alive() is False
            assert self.proc_recv_android.is_alive() is False
            self.logger.debug("Android child processes killed")

            # Clean up old sockets
            self.android_link.disconnect()

            # Reconnect; retry until the listener is back up rather than letting
            # one failed attempt kill the reconnection handler for good.
            while True:
                try:
                    self.android_link.connect()
                    break
                except Exception as e:
                    self.logger.error(
                        f"Reconnect attempt failed: {e}; retrying in 3s")
                    time.sleep(3)

            # Recreate Android processes
            self.proc_recv_android = Process(target=run_worker, args=(self.recv_android,))
            self.proc_android_sender = Process(target=run_worker, args=(self.android_sender,))

            # Start previously killed processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage(
                "info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path'))
            # Replay results whose send may have overlapped the disconnect.
            self._replay_recognized_targets()

            self.android_dropped.clear()

    def recv_android(self) -> None:
        """
        [Child Process] Processes the messages received from Android
        """
        while True:
            msg_str: Optional[str] = None
            try:
                msg_str = self.android_link.recv()
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android connection dropped")

            if msg_str is None:
                continue

            message: dict = json.loads(msg_str)

            if self.motion_fault.is_set():
                self.android_queue.put(AndroidMessage(
                    'error', 'Task 1 stopped after a motor fault. Restart Task 1 before trying again.'))
                continue

            ## Command: Set obstacles ##
            if message['cat'] == "obstacles":
                value = dict(message['value'])
                with self.plan_guard:
                    if self.plan_state['status'] == 'running' or self.stm_pending_id.value:
                        self.android_queue.put(AndroidMessage('plan', {
                            'id': value.get('plan_id', ''), 'state': 'ERROR',
                            'reason': 'Robot is moving; finish the current run first'}))
                        continue
                    self.unpause.clear()
                    self.plan_state['revision'] += 1
                    self.plan_state['id'] = value.get('plan_id', 'legacy')
                    self.plan_state['status'] = 'planning'
                    if hasattr(self, 'recognized_images'):
                        self.recognized_images.clear()
                    value['revision'] = self.plan_state['revision']
                self.rpi_action_queue.put(PiAction('obstacles', value))

            elif message['cat'] == 'invalidate':
                with self.plan_guard:
                    if self.plan_state['status'] != 'running':
                        self.unpause.clear()
                        self.plan_state['revision'] += 1
                        self.plan_state['status'] = 'unsent'

            ## Command: Start Moving ##
            elif message['cat'] == "control":
                if message['value'] == "start":
                    with self.plan_guard:
                        if (self.plan_state['status'] != 'ready' or
                                message.get('plan_id') != self.plan_state['id']):
                            self.android_queue.put(AndroidMessage('error',
                                'Send the map and wait for its route before pressing Start.'))
                            continue
                        self.plan_state['status'] = 'running'
                        self.unpause.set()
                    self.logger.info(
                        "Start command received, starting robot on path!")
                    self.android_queue.put(AndroidMessage(
                        'info', 'Starting robot on path!'))
                    self.android_queue.put(
                        AndroidMessage('status', 'running'))

            elif message['cat'] == "manual":
                with self.plan_guard:
                    if self.plan_state['status'] == 'running':
                        self.android_queue.put(AndroidMessage('error', 'Manual controls are disabled during Task 1.'))
                        continue
                    self.plan_state['revision'] += 1
                    self.plan_state['status'] = 'unsent'
                    self.clear_queues()
                command = message['value']
                self.command_queue.put({'command': command, 'pose': None,
                    'revision': self.plan_state['revision'], 'manual': True})
                self.unpause.set()

            elif message['cat'] == 'result':
                # Stop further dispatch. An in-flight motor command may finish;
                # this is not a firmware emergency stop or an automatic finish.
                with self.plan_guard:
                    self.unpause.clear()
                    self.plan_state['revision'] += 1
                    self.plan_state['status'] = 'unsent'
                self.android_queue.put(AndroidMessage('status', 'Run stopped manually; incomplete'))

    def recv_stm(self) -> None:
        """
        [Child Process] Receive acknowledgement messages from STM32, and release the movement lock
        """
        while True:
            message: str = self.stm_link.recv()

            # recv() returns None when the read times out with no frame. Treat
            # that as "nothing happened yet" rather than an acknowledgement --
            # releasing the movement lock here would let the next command go
            # out before the STM finished the current one.
            if not message:
                continue

            if message.startswith("ACK"):
                # Only the outstanding motor command may release this lock.
                # A late/duplicate ACK must not unlock a camera operation.
                with self.stm_ack_guard:
                    pending = self.stm_pending_id.value
                    if not pending or self.stm_link.last_done_id != pending:
                        self.logger.warning(f"Ignoring unexpected STM ACK: {message}")
                        continue
                    self.stm_pending_id.value = 0
                    cur_location = dict(self.stm_pending_location)
                    self.stm_pending_location.clear()
                    if cur_location:
                        self._report_location(cur_location)
                    manual_command = self.stm_pending_manual.pop('command', None)
                    if manual_command:
                        name = {'FW010': 'forward', 'BW010': 'backward',
                                'FL090': 'left', 'FR090': 'right'}[manual_command]
                        self.android_queue.put(AndroidMessage('status', f'Manual {name} complete'))
                    self.movement_lock.release()
                    self.logger.debug(f"STM command {pending} completed; movement lock released.")
            elif message.startswith("ERROR"):
                self._pause_motion(f"STM rejected a command: {message}")
            elif message != "":
                self.logger.warning(
                    f"Ignored unknown message from STM: {message}")

    def android_sender(self) -> None:
        """
        [Child process] Responsible for retrieving messages from android_queue and sending them over the Android link.
        """
        while True:
            # Retrieve from queue
            try:
                message: AndroidMessage = self.android_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                self.android_link.send(message)
            except OSError:
                self.android_dropped.set()
                self.logger.debug("Event set: Android dropped")

    def _pause_motion(self, reason):
        """Latch a fault: no further dispatch until the user restarts Task 1."""
        self.motion_fault.set()
        self.unpause.clear()
        message = f"{reason} No further commands sent; restart Task 1 after checking the robot."
        self.logger.error(message)
        self.android_queue.put(AndroidMessage('error', message))

    def _report_location(self, pose):
        self.current_location.update(pose)
        self.android_queue.put(AndroidMessage('location', dict(pose)))

    def _replay_recognized_targets(self):
        for obstacle_id, image_id in getattr(self, 'recognized_images', {}).items():
            self.android_queue.put(AndroidMessage('image-rec', {
                'obstacle_id': obstacle_id, 'image_id': image_id}))

    def _finish_run(self, revision=None, reason=None):
        """Finish after the last confirmed motor completion, including partial results.

        The caller owns movement_lock and releases it. An image/revisit failure
        is not a motor fault, and must still send the tablet its finished status.
        """
        with self.plan_guard:
            if self.motion_fault.is_set() or (revision is not None and (
                    revision != self.plan_state['revision'] or
                    self.plan_state['status'] != 'running')):
                return
            self.unpause.clear()
            self.plan_state['status'] = 'finished'
        recognized = {str(oid) for oid in self.success_obstacles}
        expected = {str(oid) for oid in self.obstacles.keys()}
        missing = sorted(expected - recognized, key=int)
        summary = f'Task 1 finished: {len(recognized & expected)}/{len(expected)} targets recognized.'
        if missing:
            summary += f' Unrecognized obstacles: {", ".join(missing)}.'
        if reason:
            summary += f' {reason}'
        self.logger.info(summary)
        self.android_queue.put(AndroidMessage('info', summary))
        self.android_queue.put(AndroidMessage('status', 'finished'))
        self.rpi_action_queue.put(PiAction(cat='stitch', value=''))

    def command_follower_with_camera(self) -> None:
        """
        [Child Process] command_follower, holding the camera open for the run.
        snap_and_rec runs in this process, so the camera must be opened here.
        """
        self.camera = WarmCamera(self.logger)
        try:
            self.command_follower()
        finally:
            self.camera.close()

    def command_follower(self) -> None:
        """
        [Child Process]
        """
        active_revision = None
        while True:
            # Retrieve next movement command
            command: str = self.command_queue.get()
            self.logger.debug("wait for unpause")
            # Wait for unpause event to be true [Main Trigger]
            self.unpause.wait()
            if self.motion_fault.is_set():
                return
            pose = None
            revision = None
            manual = False
            if isinstance(command, dict):
                manual = command.get('manual', False)
                pose = command['pose']
                revision = command['revision']
                if revision != self.plan_state['revision'] or (not manual and self.plan_state['status'] != 'running'):
                    continue
                command = command['command']
                if not manual and active_revision != revision:
                    self.failed_attempt = False
                    active_revision = revision
            self.logger.debug("wait for movelock")
            # This wait is now only for the preceding MOTOR command, not CV.
            if not self.movement_lock.acquire(timeout=ACK_TIMEOUT):
                self._pause_motion(f"No matching STM ACK within {ACK_TIMEOUT}s.")
                return
            if self.motion_fault.is_set():
                self.movement_lock.release()
                return
            if not self.unpause.is_set() or (revision is not None and revision != self.plan_state['revision']):
                self.movement_lock.release()
                continue

            # STM32 Commands - Send straight to STM32
            # Legacy FW010-style prefixes plus the Algo server's
            # "<steering>,<angle>,<direction>,<distance>" form. STMLink.translate()
            # understands both and turns them into <id>/MOTOR/<CMD>/<p1>/<p2>;
            stm32_prefixes = ("FS", "BS", "FW", "BW", "FL", "FR", "BL",
                              "BR", "TL", "TR", "A", "C", "DT", "STOP", "ZZ", "RS",
                              "center", "left", "right")
            if command.startswith(stm32_prefixes):
                # NOTE: turns are sent plain here. recv_stm is the sole serial
                # reader (separate process); reading acks here too collides
                # ("multiple access on port"). Gyro turn correction belongs in
                # recv_stm, not here -- see TODO.
                try:
                    # Guard publication against a very fast ACK in recv_stm.
                    with self.stm_ack_guard:
                        self.stm_pending_location.clear()
                        if pose is not None:
                            self.stm_pending_location.update(pose)
                        self.stm_pending_manual.clear()
                        if manual:
                            self.stm_pending_manual['command'] = command
                        self.stm_pending_id.value = self.stm_link.send(command)
                except Exception as exc:
                    self._pause_motion(f"Could not send STM command: {exc}")
                    return
                self.logger.debug(f"Sending to STM32: {command}")

            # Snap command
            elif command.startswith("SNAP"):
                if pose is not None:
                    self._report_location(pose)
                # The Algo server tags snaps with a direction suffix, e.g.
                # "SNAP1_L" / "SNAP2". Preserve the camera alignment as well.
                snap_parts = command[4:].split('_', 1)
                obstacle_id = snap_parts[0]
                camera_signal = snap_parts[1] if len(snap_parts) > 1 else 'C'

                # Keep the command follower here until capture AND result
                # bookkeeping finish. FIN cannot overtake camera retries.
                try:
                    self.snap_and_rec(obstacle_id=obstacle_id, signal=camera_signal)
                finally:
                    self.movement_lock.release()

            # End of path
            elif command == "FIN":
                if pose is not None:
                    self._report_location(pose)
                self.logger.info(
                    f"At FIN, self.failed_obstacles: {self.failed_obstacles}")
                self.logger.info(
                    f"At FIN, self.current_location: {self.current_location}")
                if len(self.failed_obstacles) != 0 and self.failed_attempt == False:

                    # Retain every physical obstacle for collision checking.
                    # d=8 makes already-recognized obstacles non-targets in Algo.
                    failed_ids = {str(oid) for oid in self.failed_obstacles}
                    new_obstacle_list = []
                    for obstacle in self.obstacles.values():
                        ob = dict(obstacle)
                        if str(ob['id']) not in failed_ids:
                            ob['d'] = 8
                        new_obstacle_list.append(ob)
                    if not new_obstacle_list:
                        self.logger.warning(
                            "No obstacle data for retry; finishing instead.")
                        self.failed_attempt = True

                if (len(self.failed_obstacles) != 0 and self.failed_attempt == False
                        and new_obstacle_list):
                    self.logger.info("Attempting to go to failed obstacles")
                    self.failed_attempt = True
                    prepared = self.request_algo({'obstacles': new_obstacle_list, 'mode': '0'},
                                                self.current_location['x'], self.current_location['y'],
                                                self.current_location['d'], retrying=True)
                    try:
                        if not prepared:
                            self._finish_run(revision, 'No revisit route available; ending with the current results.')
                    finally:
                        self.movement_lock.release()
                    continue

                try:
                    self._finish_run(revision)
                finally:
                    self.movement_lock.release()
            else:
                raise Exception(f"Unknown command: {command}")

    def rpi_action(self):
        """
        [Child Process]
        """
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "obstacles":
                with self.plan_guard:
                    if action.value['revision'] != self.plan_state['revision']:
                        continue
                    self.obstacles.clear()
                for obs in action.value['obstacles']:
                    self.obstacles[obs['id']] = obs
                # Android tells us where it placed the robot; without this the
                # defaults (1, 1, NORTH) silently win and the path is planned
                # from the wrong start.
                robot = action.value.get('robot') or {}
                self.request_algo(
                    action.value,
                    robot_x=robot.get('x', 1),
                    robot_y=robot.get('y', 1),
                    robot_dir=robot.get('d', 0))
            elif action.cat == "stitch":
                self.request_stitch()

    def snap_and_rec(self, obstacle_id: str, signal: str = 'C') -> None:
        """
        Capture and record a result while the caller owns movement_lock.
        This method never acquires/releases movement_lock itself.
        :param obstacle_id: the current obstacle ID
        """
        obstacle_id = str(obstacle_id)
        run_id = self.plan_state['id']
        revision = self.plan_state['revision']
        known_image = getattr(self, 'recognized_images', {}).get(obstacle_id)
        if known_image is not None:
            self.android_queue.put(AndroidMessage('image-rec', {
                'obstacle_id': obstacle_id, 'image_id': known_image}))
            return
        started = time.time()
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        self.android_queue.put(AndroidMessage(
            "info", f"Capturing image for obstacle id: {obstacle_id}"))

        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
        CAPTURE_DIR.mkdir(parents=True, exist_ok=True)
        prefix = f"{int(time.time())}_{obstacle_id}"
        results = {'image_id': 'NA', 'obstacle_id': 'NA'}
        for attempt in range(1, CAPTURE_ATTEMPTS + 1):
            try:
                filename = str(CAPTURE_DIR / f'{prefix}_attempt-{attempt}.jpg')
                self.logger.info(f"Capturing obstacle {obstacle_id}: attempt {attempt}/{CAPTURE_ATTEMPTS}")
                # Fast path: the camera kept running by command_follower_with_camera.
                camera = getattr(self, 'camera', None)
                if camera is None or not camera.capture(filename):
                    # Cold start: ~2-3 s per photo. Bound camera hangs and keep
                    # diagnostics without flooding the console with libcamera
                    # mode-selection messages.
                    capture_command = ['rpicam-still', '-e', 'jpg', '-n', '-t', '500', '-o', filename]
                    subprocess.run(capture_command, check=True, timeout=CAPTURE_TIMEOUT,
                                   capture_output=True, text=True)
                with open(filename, 'rb') as image_file:
                    self.logger.debug("Requesting from image API")
                    response = requests.post(url, data={
                        'run_id': run_id, 'target_obstacle_id': obstacle_id,
                        'signal': signal, 'attempt': str(attempt)}, files={
                        'image': (filename, image_file, 'image/jpeg')
                    }, timeout=IMAGE_API_TIMEOUT)
                # The original ONNX API uses 404 for a valid frame with no
                # detections. Treat it like NA, allowing the normal retries.
                if response.status_code == 404:
                    candidate = {'image_id': 'NA', 'obstacle_id': 'NA'}
                else:
                    response.raise_for_status()
                    candidate = response.json()
                if isinstance(candidate, dict) and candidate.get('run_id') not in (None, run_id):
                    raise ValueError('Image API returned a result for a different run')
                results = normalize_recognition(candidate, obstacle_id)
                self.logger.info(f"Image recognition results: {results}")
                if results['image_id'] != 'NA':
                    break
                if attempt < CAPTURE_ATTEMPTS:
                    self.logger.info("No accepted detection; recapturing with the same camera settings.")
            except Exception as exc:
                self.logger.error(f"Capture/image API failed for obstacle {obstacle_id}: {exc}")
                if isinstance(exc, subprocess.CalledProcessError):
                    self.logger.error(f"Camera output: {exc.stderr}")
                self.android_queue.put(AndroidMessage(
                    'error', f'Image capture/recognition failed for obstacle {obstacle_id}.'))
                results = {'image_id': 'NA', 'obstacle_id': 'NA'}
                if attempt >= CAPTURE_ATTEMPTS:
                    break

        if self.plan_state['id'] != run_id or self.plan_state['revision'] != revision:
            self.logger.info('Discarding recognition for a superseded run.')
            return

        self.logger.info(f"Obstacle {obstacle_id}: stop took {time.time() - started:.1f}s")
        self.logger.info(f"results: {results}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['obstacle_id'])})")

        # Handle results - store just the obstacle_id since self.obstacles is empty
        if results['image_id'] == 'NA':
            if obstacle_id not in self.failed_obstacles:
                self.failed_obstacles.append(obstacle_id)
            self.logger.info(
                f"Added Obstacle {obstacle_id} to failed obstacles.")
            self.logger.info(f"self.failed_obstacles: {self.failed_obstacles}")
        else:
            if obstacle_id in self.failed_obstacles:
                self.failed_obstacles.remove(obstacle_id)
            if obstacle_id not in self.success_obstacles:
                self.success_obstacles.append(obstacle_id)
            if hasattr(self, 'recognized_images'):
                self.recognized_images[obstacle_id] = results['obstacle_id']
            self.logger.info(
                f"self.success_obstacles: {self.success_obstacles}")

        # Tablet frame must be TARGET,<obstacle number>,<numeric image id>.
        # The app int-parses both fields and silently drops the frame if either
        # fails, and the grader reads the id off the obstacle on the tablet --
        # so this message is the thing Task 1 actually scores.
        # results['obstacle_id'] is (confusingly) the numeric image id.
        if results.get('image_id') not in (None, 'NA'):
            self.android_queue.put(AndroidMessage("image-rec", {
                "obstacle_id": obstacle_id,
                "image_id": results.get('obstacle_id'),
            }))

    def request_algo(self, data, robot_x=1, robot_y=1, robot_dir=0, retrying=False):
        """
        Requests for a series of commands and the path from the Algo API.
        The received commands and path are then queued in the respective queues
        """
        self.logger.info("Requesting path from algo...")
        revision = data.get('revision', self.plan_state['revision'])
        plan_id = data.get('plan_id', self.plan_state['id'])
        self.android_queue.put(AndroidMessage(
            "info", "Requesting path from algo..."))
        self.logger.info(f"data: {data}")

        # The Algo server (Flask, /path) parses a text/plain payload, not JSON:
        #   STATUS,OBSTACLE_LIST,<retrying>,<rx>;<ry>;<DIR>,false,<id>;<x>;<y>;<DIR> | ... ,true,true
        # convert_input() reads fields 2, 3 and 5 out of that comma split, and
        # its face_map only accepts the spelled-out directions.
        dir_name = {0: "NORTH", 2: "EAST", 4: "SOUTH", 6: "WEST",
                    1: "NORTH", 3: "EAST", 5: "SOUTH", 7: "WEST",
                    "N": "NORTH", "E": "EAST", "S": "SOUTH", "W": "WEST",
                    "NORTH": "NORTH", "EAST": "EAST",
                    "SOUTH": "SOUTH", "WEST": "WEST", 8: "SKIP"}

        obstacle_list = " | ".join(
            "{};{};{};{}".format(o["id"], o["x"], o["y"], dir_name[o["d"]])
            for o in data["obstacles"])

        payload = "STATUS,OBSTACLE_LIST,{},{};{};{},false,{},true,true".format(
            "true" if retrying else "false",
            robot_x, robot_y, dir_name[robot_dir],
            obstacle_list)

        self.logger.debug(f"Algo payload: {payload}")
        url = f"http://{API_IP}:{API_PORT}/path"
        try:
            response = requests.post(url, data=payload,
                headers={"Content-Type": "text/plain"}, timeout=ALGO_TIMEOUT)
            if response.status_code != 200:
                raise ValueError(f'Algo returned HTTP {response.status_code}')
            plan = parse_plan(response.json())
        except Exception as exc:
            self.logger.error(f'Route preparation failed: {exc}')
            with self.plan_guard:
                if revision == self.plan_state['revision']:
                    if retrying:
                        self.android_queue.put(AndroidMessage('info', f'Revisit unavailable: {exc}'))
                    else:
                        self.plan_state['status'] = 'error'
                        self.android_queue.put(AndroidMessage('plan', {
                            'id': plan_id, 'state': 'ERROR', 'reason': str(exc)}))
            return False

        with self.plan_guard:
            if revision != self.plan_state['revision']:
                self.logger.info('Discarding route for an outdated map.')
                return False
            self.clear_queues()
            if not retrying:
                self.success_obstacles[:] = []
                self.failed_obstacles[:] = []
                if hasattr(self, 'recognized_images'):
                    self.recognized_images.clear()
                self.current_location.clear()
                self.current_location.update(x=robot_x, y=robot_y, d=robot_dir)
            for command, pose in plan:
                self.command_queue.put({'command': command, 'pose': pose, 'revision': revision})
            visited = {int(command[4:].split('_')[0]) for command, _ in plan if command.startswith('SNAP')}
            requested = {int(ob['id']) for ob in data['obstacles'] if ob['d'] != 8}
            omitted = sorted(requested - visited)
            summary = f'Route prepared for {len(visited)}/{len(requested)} targets.'
            if omitted:
                summary += f' No reachable visit in this route for obstacles {omitted}; check the map before Start.'
            self.android_queue.put(AndroidMessage('info', summary))
            if not retrying:
                self.plan_state['status'] = 'ready'
                self.android_queue.put(AndroidMessage('plan', {'id': plan_id, 'state': 'READY'}))
        self.logger.info('Prepared route queued; waiting for Android Start.' if not retrying else 'Revisit route queued.')
        return True

    def request_stitch(self):
        """Sends a stitch request to the image recognition API to stitch the different images together"""
        # The tiled RAW display lives on image-api (:8001), not the Algo
        # server, whose /stitch is commented out. FAQ 16: no display, no score.
        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/stitch"
        try:
            response = requests.get(url, params={'run_id': self.plan_state['id']}, timeout=IMAGE_API_TIMEOUT)
        except requests.RequestException as exc:
            self.logger.error(f"Image stitching request failed: {exc}")
            self.android_queue.put(AndroidMessage('error', 'Image stitching request failed.'))
            return

        # If error, then log, and send error to Android
        if response.status_code != 200:
            # Notify android
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting stitch from the API."))
            self.logger.error(
                "Something went wrong when requesting stitch from the API.")
            return

        self.logger.info("Images stitched!")
        self.android_queue.put(AndroidMessage("info", "Images stitched!"))

    def clear_queues(self):
        """Clear both command and path queues"""
        for pending_queue in (self.command_queue, self.path_queue):
            while True:
                try:
                    pending_queue.get_nowait()
                except queue.Empty:
                    break

    def check_api(self) -> bool:
        """Require both planning and the released CV model before motor use."""
        try:
            algo = requests.get(f"http://{API_IP}:{API_PORT}/status", timeout=3)
            algo.raise_for_status()
            cv = requests.get(f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/health", timeout=3)
            cv.raise_for_status()
            model_name = str(cv.json().get('model', '')).replace(chr(92), '/').rsplit('/', 1)[-1]
            if model_name != 'best_v11i_m.onnx':
                self.logger.error(f'Unexpected CV model: {model_name}; Task 1 requires best_v11i_m.onnx.')
                return False
            self.logger.info('Algo and best_v11i_m.onnx CV API are ready.')
            return True
        except (requests.RequestException, ValueError) as exc:
            self.logger.error(f'Laptop API check failed: {exc}')
            return False


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
