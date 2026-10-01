#!/usr/bin/env python3
import json
import re
import queue
import time
import subprocess
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from consts import SYMBOL_MAP
from logger import prepare_logger
from settings import API_IP, API_PORT, API_PORT_IMAGE, API_IP_IMAGE

# A missing STM completion pauses dispatch; it must never permit another move.
# Camera retries run synchronously and do not use this motor ACK deadline.
ACK_TIMEOUT = 15
CAPTURE_ATTEMPTS = 7
CAPTURE_TIMEOUT = 10
IMAGE_API_TIMEOUT = (3, 10)


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

        self.android_dropped = self.manager.Event()
        self.unpause = self.manager.Event()

        self.movement_lock = self.manager.Lock()
        self.motion_fault = self.manager.Event()
        self.stm_ack_guard = self.manager.Lock()
        self.stm_pending_id = self.manager.Value('i', 0)

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
            self.stm_link.connect()
            self.check_api()

            # Define child processes
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_recv_stm32 = Process(target=self.recv_stm)
            self.proc_android_sender = Process(target=self.android_sender)
            self.proc_command_follower = Process(target=self.command_follower)
            self.proc_rpi_action = Process(target=self.rpi_action)

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
            self.stop()

    def stop(self):
        """Stops all processes on the RPi and disconnects gracefully with Android and STM32"""
        self.android_link.disconnect()
        self.stm_link.disconnect()
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
            self.proc_recv_android = Process(target=self.recv_android)
            self.proc_android_sender = Process(target=self.android_sender)

            # Start previously killed processes
            self.proc_recv_android.start()
            self.proc_android_sender.start()

            self.logger.info("Android child processes restarted")
            self.android_queue.put(AndroidMessage(
                "info", "You are reconnected!"))
            self.android_queue.put(AndroidMessage('mode', 'path'))

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
                self.rpi_action_queue.put(PiAction(**message))
                self.logger.debug(
                    f"Set obstacles PiAction added to queue: {message}")

            ## Command: Start Moving ##
            elif message['cat'] == "control":
                if message['value'] == "start":
                    # Check API
                    # if not self.check_api():
                    #     self.logger.error(
                    #         "API is down! Start command aborted.")
                    #     self.android_queue.put(AndroidMessage(
                    #         'error', "API is down, start command aborted."))

                    # Commencing path following
                    # Do NOT gate on the queue being non-empty. BEGIN sends
                    # obstacles + control:start back-to-back, and the queue is
                    # filled by rpi_action in a SEPARATE process that usually
                    # has not run yet here -- so the old check saw an empty
                    # queue, skipped unpause.set(), and the robot never moved.
                    # command_follower blocks on command_queue.get() anyway.
                    self.unpause.set()
                    self.logger.info(
                        "Start command received, starting robot on path!")
                    self.android_queue.put(AndroidMessage(
                        'info', 'Starting robot on path!'))
                    self.android_queue.put(
                        AndroidMessage('status', 'running'))

            elif message['cat'] == "manual":
                command = message['value']
                self.command_queue.put(command)
                self.unpause.set()

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
                    try:
                        cur_location = self.path_queue.get_nowait()
                    except queue.Empty:
                        cur_location = None
                    if cur_location is not None:
                        self.current_location.update(cur_location)
                        self.android_queue.put(AndroidMessage('location', {
                            key: cur_location[key] for key in ('x', 'y', 'd')
                        }))
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

    def command_follower(self) -> None:
        """
        [Child Process] 
        """
        while True:
            # Retrieve next movement command
            command: str = self.command_queue.get()
            self.logger.debug("wait for unpause")
            # Wait for unpause event to be true [Main Trigger]
            self.unpause.wait()
            if self.motion_fault.is_set():
                return
            self.logger.debug("wait for movelock")
            # This wait is now only for the preceding MOTOR command, not CV.
            if not self.movement_lock.acquire(timeout=ACK_TIMEOUT):
                self._pause_motion(f"No matching STM ACK within {ACK_TIMEOUT}s.")
                return
            if self.motion_fault.is_set():
                self.movement_lock.release()
                return

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
                        self.stm_pending_id.value = self.stm_link.send(command)
                except Exception as exc:
                    self._pause_motion(f"Could not send STM command: {exc}")
                    return
                self.logger.debug(f"Sending to STM32: {command}")

            # Snap command
            elif command.startswith("SNAP"):
                # The Algo server tags snaps with a direction suffix, e.g.
                # "SNAP1_L" / "SNAP2". Keep only the obstacle id.
                obstacle_id = command.replace("SNAP", "").split("_")[0]

                # Keep the command follower here until capture AND result
                # bookkeeping finish. FIN cannot overtake camera retries.
                try:
                    self.snap_and_rec(obstacle_id=obstacle_id)
                finally:
                    self.movement_lock.release()

            # End of path
            elif command == "FIN":
                self.logger.info(
                    f"At FIN, self.failed_obstacles: {self.failed_obstacles}")
                self.logger.info(
                    f"At FIN, self.current_location: {self.current_location}")
                if len(self.failed_obstacles) != 0 and self.failed_attempt == False:

                    # failed_obstacles holds obstacle-id STRINGS, but the algo
                    # needs the obstacle dicts -- look them up. The old code
                    # also re-added successful obstacles with d=8, which (a)
                    # crashed on strings and (b) the current algo treats every
                    # obstacle as a target to visit, so it would re-drive them.
                    new_obstacle_list = []
                    for oid in list(self.failed_obstacles):
                        ob = None
                        try:
                            ob = self.obstacles.get(int(oid))
                        except (TypeError, ValueError):
                            pass
                        if ob is None:
                            ob = self.obstacles.get(oid)
                        if ob:
                            new_obstacle_list.append(dict(ob))
                    if not new_obstacle_list:
                        self.logger.warning(
                            "No obstacle data for retry; finishing instead.")
                        self.failed_attempt = True

                if (len(self.failed_obstacles) != 0 and self.failed_attempt == False
                        and new_obstacle_list):
                    self.logger.info("Attempting to go to failed obstacles")
                    self.failed_attempt = True
                    self.request_algo({'obstacles': new_obstacle_list, 'mode': '0'},
                                      self.current_location['x'], self.current_location['y'], self.current_location['d'], retrying=True)
                    self.movement_lock.release()
                    continue

                self.unpause.clear()
                self.movement_lock.release()
                self.logger.info("Commands queue finished.")
                self.android_queue.put(AndroidMessage(
                    "info", "Commands queue finished."))
                self.android_queue.put(AndroidMessage("status", "finished"))
                self.rpi_action_queue.put(PiAction(cat="stitch", value=""))
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

    def snap_and_rec(self, obstacle_id: str) -> None:
        """
        Capture and record a result while the caller owns movement_lock.
        This method never acquires/releases movement_lock itself.
        :param obstacle_id: the current obstacle ID
        """
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        self.android_queue.put(AndroidMessage(
            "info", f"Capturing image for obstacle id: {obstacle_id}"))
        
        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
        filename = f"{int(time.time())}_{obstacle_id}.jpg"
        
        capture_command = ['rpicam-still', '-e', 'jpg', '-n', '-t', '500', '-o', filename]
        results = {'image_id': 'NA', 'obstacle_id': 'NA'}
        for attempt in range(1, CAPTURE_ATTEMPTS + 1):
            try:
                self.logger.info(f"Capturing obstacle {obstacle_id}: attempt {attempt}/{CAPTURE_ATTEMPTS}")
                # Bound camera hangs and retain diagnostics without flooding
                # the console with normal libcamera mode-selection messages.
                subprocess.run(capture_command, check=True, timeout=CAPTURE_TIMEOUT,
                               capture_output=True, text=True)
                with open(filename, 'rb') as image_file:
                    self.logger.debug("Requesting from image API")
                    response = requests.post(url, files={
                        'image': (filename, image_file, 'image/jpeg')
                    }, timeout=IMAGE_API_TIMEOUT)
                response.raise_for_status()
                candidate = response.json()
                if not isinstance(candidate, dict) or not candidate.get('image_id'):
                    raise ValueError('Image API returned no image_id')
                results = candidate
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
                break
        
        self.logger.info(f"results: {results}")
        self.logger.info(f"self.obstacles: {self.obstacles}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")
        
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
                    "SOUTH": "SOUTH", "WEST": "WEST"}

        obstacle_list = " | ".join(
            "{};{};{};{}".format(o["id"], o["x"], o["y"], dir_name[o["d"]])
            for o in data["obstacles"])

        payload = "STATUS,OBSTACLE_LIST,{},{};{};{},false,{},true,true".format(
            "true" if retrying else "false",
            robot_x, robot_y, dir_name[robot_dir],
            obstacle_list)

        self.logger.debug(f"Algo payload: {payload}")
        url = f"http://{API_IP}:{API_PORT}/path"
        response = requests.post(
            url, data=payload,
            headers={"Content-Type": "text/plain"}, timeout=60)

        # Error encountered at the server, return early
        if response.status_code != 200:
            self.android_queue.put(AndroidMessage(
                "error", "Something went wrong when requesting path from Algo API."))
            self.logger.error(
                "Something went wrong when requesting path from Algo API.")
            return

        # Parse response. The Algo server nests everything under "data" and
        # returns the STM-ready list as stm_commands ("center,0,forward,120"),
        # while "commands" is the human-readable FW/FR form used by the sim.
        result = response.json()
        payload_data = result.get("data", result)
        commands = payload_data.get("stm_commands") or payload_data.get("commands", [])
        path = payload_data.get("path", [])
        if result.get("error"):
            self.logger.error(f"Algo API reported an error: {result['error']}")

        # Log commands received
        self.logger.debug(f"Commands received from API: {commands}")

        # Put commands and paths into respective queues
        self.clear_queues()
        for c in commands:
            self.command_queue.put(c)
        for p in path[1:]:  # ignore first element as it is the starting position of the robot
            self.path_queue.put(p)

        self.android_queue.put(AndroidMessage(
            "info", "Commands and path received Algo API. Robot is ready to move."))
        self.logger.info(
            "Commands and path received Algo API. Robot is ready to move.")

    def request_stitch(self):
        """Sends a stitch request to the image recognition API to stitch the different images together"""
        # The tiled RAW display lives on image-api (:8001), not the Algo
        # server, whose /stitch is commented out. FAQ 16: no display, no score.
        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/stitch"
        try:
            response = requests.get(url, timeout=IMAGE_API_TIMEOUT)
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
        while not self.command_queue.empty():
            self.command_queue.get()
        while not self.path_queue.empty():
            self.path_queue.get()

    def check_api(self) -> bool:
        """Check whether image recognition and algorithm API server is up and running

        Returns:
            bool: True if running, False if not.
        """
        # Check image recognition API
        url = f"http://{API_IP}:{API_PORT}/status"
        try:
            response = requests.get(url, timeout=1)
            if response.status_code == 200:
                self.logger.debug("API is up!")
                return True
            return False
        # If error, then log, and return False
        except ConnectionError:
            self.logger.warning("API Connection Error")
            return False
        except requests.Timeout:
            self.logger.warning("API Timeout")
            return False
        except Exception as e:
            self.logger.warning(f"API Exception: {e}")
            return False


if __name__ == "__main__":
    rpi = RaspberryPi()
    rpi.start()
