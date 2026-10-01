#!/usr/bin/env python3
import json
import queue
import time
from multiprocessing import Process, Manager
from typing import Optional
import os
import requests
from communication.android import AndroidLink, AndroidMessage
from communication.stm32 import STMLink
from logger import prepare_logger
from settings import API_IP, API_PORT, API_PORT_IMAGE, API_IP_IMAGE

try:
    from consts import SYMBOL_MAP
except ImportError:          # only used for a log line
    SYMBOL_MAP = {}

# How long to wait for the STM's !<id>/DONE; before giving up on the movement
# lock. Longest observed real move is ~3.5s, so this is generous while still
# recovering quickly from a missed reply.
ACK_TIMEOUT = 15

# Directions the Algo server's convert_input() accepts. The tablet and the path
# use the 0/2/4/6 encoding (odd values are diagonals in the old planner).
DIR_NAME = {0: "NORTH", 2: "EAST", 4: "SOUTH", 6: "WEST",
            1: "NORTH", 3: "EAST", 5: "SOUTH", 7: "WEST",
            "N": "NORTH", "E": "EAST", "S": "SOUTH", "W": "WEST",
            "NORTH": "NORTH", "EAST": "EAST", "SOUTH": "SOUTH", "WEST": "WEST"}

# Commands command_follower forwards to the STM. Legacy FW010-style prefixes
# plus the Algo server's "<steering>,<angle>,<direction>,<distance>" form;
# STMLink.translate() turns either into <id>/MOTOR/<CMD>/<p1>/<p2>;
STM32_PREFIXES = ("FS", "BS", "FW", "BW", "FL", "FR", "BL", "BR", "TL", "TR",
                  "A", "C", "DT", "STOP", "ZZ", "RS",
                  "center", "left", "right")


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
        self.stm_link = STMLink()

        self.manager = Manager()

        self.android_dropped = self.manager.Event()
        self.unpause = self.manager.Event()
        # Set once the end-of-path retry of failed obstacles has been used.
        # An Event rather than a plain attribute so a new run (handled in the
        # rpi_action process) can reset it for command_follower.
        self.retry_done = self.manager.Event()

        self.movement_lock = self.manager.Lock()

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

            ## Command: Set obstacles ##
            if message['cat'] == "obstacles":
                self.rpi_action_queue.put(PiAction(**message))
                self.logger.debug(
                    f"Set obstacles PiAction added to queue: {message}")

            ## Command: Start Moving ##
            elif message['cat'] == "control":
                if message['value'] == "start":
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
                try:
                    self.movement_lock.release()
                    self.logger.info("Movement lock released")
                    try:
                        self.retrylock.release()
                    except:
                        pass
                    self.logger.debug(
                        "ACK from STM32 received, movement lock released.")

                    cur_location = self.path_queue.get_nowait()

                    self.current_location['x'] = cur_location['x']
                    self.current_location['y'] = cur_location['y']
                    self.current_location['d'] = cur_location['d']
                    self.logger.info(
                        f"self.current_location = {self.current_location}")
                    self.android_queue.put(AndroidMessage('location', {
                        "x": cur_location['x'],
                        "y": cur_location['y'],
                        "d": cur_location['d'],
                    }))

                except Exception:
                    self.logger.warning("Tried to release a released lock!")
            elif message.startswith("ERROR"):
                # The command is over even though it failed (e.g. the firmware
                # abandoned a turn with TURN_TIMEOUT). Release the lock so the
                # run carries on instead of stalling for ACK_TIMEOUT.
                self.logger.error(f"STM reported failure: {message}")
                try:
                    self.movement_lock.release()
                except Exception:
                    pass
            else:
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

    def command_follower(self) -> None:
        """
        [Child Process]
        """
        while True:
            # Retrieve next movement command
            command: str = self.command_queue.get()
            self.logger.debug("wait for unpause")
            # Wait for unpause event to be true [Main Trigger]
            try:
                self.logger.debug("wait for retrylock")
                self.retrylock.acquire()
                self.retrylock.release()
            except:
                self.logger.debug("wait for unpause")
                self.unpause.wait()
            self.logger.debug("wait for movelock")
            # Acquire the lock, but do not wait forever. recv_stm releases it on
            # an ACK; if the STM misses one reply the lock would otherwise stay
            # held and every later command would block here permanently.
            if not self.movement_lock.acquire(timeout=ACK_TIMEOUT):
                self.logger.warning(
                    f"No ACK from STM32 within {ACK_TIMEOUT}s -- proceeding anyway. "
                    "The board may be desynced; press its reset button if this repeats.")
                self.android_queue.put(AndroidMessage(
                    "error", "STM not responding - continuing without ACK"))

            # STM32 Commands - Send straight to STM32.
            # Turns are sent plain. recv_stm is the sole serial reader (separate
            # process); reading acks here as well collides ("multiple access
            # on port"), so gyro-verified turning cannot live in this process.
            if command.startswith(STM32_PREFIXES):
                try:
                    self.stm_link.send(command)
                    self.logger.debug(f"Sending to STM32: {command}")
                except ValueError as e:
                    self.logger.error(f"Cannot send {command!r}: {e}")
                    self.movement_lock.release()

            # Snap command
            elif command.startswith("SNAP"):
                # The Algo server tags snaps with a direction suffix, e.g.
                # "SNAP1_L" / "SNAP2". Keep only the obstacle id.
                obstacle_id = command.replace("SNAP", "").split("_")[0]

                self.rpi_action_queue.put(
                    PiAction(cat="snap", value=obstacle_id))

            # End of path
            elif command == "FIN":
                self.logger.info(
                    f"At FIN, self.failed_obstacles: {list(self.failed_obstacles)}")
                self.logger.info(
                    f"At FIN, self.current_location: {dict(self.current_location)}")

                if self.retry_failed_obstacles():
                    self.retrylock = self.manager.Lock()
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
                # Raising here used to kill command_follower, after which no
                # command (manual or path) ever reached the STM again.
                self.logger.error(f"Unknown command, skipped: {command}")
                self.movement_lock.release()

    def retry_failed_obstacles(self) -> bool:
        """Re-plan a path to the obstacles that came back NA, once per run.

        failed_obstacles holds obstacle ids (strings), not obstacle dicts, so
        they are looked up in self.obstacles. The previous version passed the
        ids straight to request_algo and set a dict key on a string, which
        crashed command_follower at FIN.

        Returns True if a retry path was requested.
        """
        if not self.failed_obstacles or self.retry_done.is_set():
            return False
        self.retry_done.set()

        failed_ids = {str(i) for i in self.failed_obstacles}
        retry_list = [dict(o) for k, o in self.obstacles.items()
                      if str(k) in failed_ids]
        if not retry_list:
            return False

        loc = dict(self.current_location)
        self.logger.info(f"Attempting to go to failed obstacles: {sorted(failed_ids)}")
        self.request_algo({'obstacles': retry_list, 'mode': '0'},
                          loc.get('x', 1), loc.get('y', 1), loc.get('d', 0),
                          retrying=True)
        return True

    def rpi_action(self):
        """
        [Child Process]
        """
        while True:
            action: PiAction = self.rpi_action_queue.get()
            self.logger.debug(
                f"PiAction retrieved from queue: {action.cat} {action.value}")

            if action.cat == "obstacles":
                # A new BEGIN is a fresh run: forget the previous run's map and
                # results, so a Reset Map + re-place on the tablet starts clean.
                self.obstacles.clear()
                del self.success_obstacles[:]
                del self.failed_obstacles[:]
                self.current_location.clear()
                self.retry_done.clear()

                for obs in action.value['obstacles']:
                    self.obstacles[obs['id']] = obs
                # Use where the tablet actually placed the robot, not (1,1,0).
                robot = (action.value.get("robot") or {}) if isinstance(action.value, dict) else {}
                self.request_algo(
                    action.value,
                    robot_x=robot.get("x", 1),
                    robot_y=robot.get("y", 1),
                    robot_dir=robot.get("d", 0))
            elif action.cat == "snap":
                self.snap_and_rec(obstacle_id=action.value)
            elif action.cat == "stitch":
                self.request_stitch()

    def snap_and_rec(self, obstacle_id: str) -> None:
        """
        RPi snaps an image and calls the API for image-rec.
        The response is then forwarded back to the android
        :param obstacle_id: the current obstacle ID
        """
        self.logger.info(f"Capturing image for obstacle id: {obstacle_id}")
        self.android_queue.put(AndroidMessage(
            "info", f"Capturing image for obstacle id: {obstacle_id}"))

        # image-api on the laptop (best_v11i_m.onnx). Field name is "image".
        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
        filename = f"{int(time.time())}_{obstacle_id}.jpg"

        # libcamera-still does not exist on this OS -- it is rpicam-still.
        rpistr = f"rpicam-still -e jpg -n -t 500 -o {filename}"
        os.system(rpistr)

        retry_count = 0

        while True:
            retry_count += 1

            self.logger.debug("Requesting from image API")

            # Send image to API
            try:
                image_file = open(filename, 'rb')
                files = {
                    "image": (
                        filename,
                        image_file,
                        "image/jpeg"  # Explicitly set content type
                    )
                }
                response = requests.post(url, files=files)
                image_file.close()

                # image-api answers 404 when the frame has no objects at all.
                # That is "nothing recognised", not a server failure -- retry
                # like any other NA.
                if response.status_code == 404:
                    results = {'image_id': 'NA'}
                elif response.status_code != 200:
                    self.logger.error(
                        f"Something went wrong when requesting path from image-rec API. Status code: {response.status_code}")
                    # Release before bailing out, otherwise command_follower
                    # blocks on the next movement_lock.acquire().
                    self.movement_lock.release()
                    return
                else:
                    results = json.loads(response.content)

                # Break if we got a valid result or exceeded max retries
                if results['image_id'] != 'NA' or retry_count > 6:
                    break

                # Capture logic remains the same
                if retry_count > 3:
                    self.logger.info(f"Image recognition results: {results}")
                    self.logger.info("Recapturing with lower shutter speed...")
                else:
                    self.logger.info(f"Image recognition results: {results}")
                    self.logger.info("Recapturing with higher shutter speed...")

                # Recapture image
                os.system(rpistr)

            except Exception as e:
                self.logger.error(f"Error during API request: {e}")
                # Same deadlock as above: a failed capture or API call must not
                # leave the movement lock held.
                self.movement_lock.release()
                return

        # Release locks
        self.movement_lock.release()
        try:
            self.retrylock.release()
        except:
            pass

        self.logger.info(f"results: {results}")
        self.logger.info(
            f"Image recognition results: {results} ({SYMBOL_MAP.get(results['image_id'])})")

        if results['image_id'] == 'NA':
            if obstacle_id not in self.failed_obstacles:
                self.failed_obstacles.append(obstacle_id)
            self.logger.info(
                f"Added Obstacle {obstacle_id} to failed obstacles.")
            self.logger.info(f"self.failed_obstacles: {list(self.failed_obstacles)}")
        else:
            self.success_obstacles.append(obstacle_id)
            self.logger.info(
                f"self.success_obstacles: {list(self.success_obstacles)}")
            # Tablet frame must be TARGET,<obstacle number>,<numeric image id>.
            # The app int-parses both fields and silently drops the frame if
            # either fails, and the grader reads the id off the obstacle on the
            # tablet -- so this message is the thing Task 1 actually scores.
            # results['obstacle_id'] is (confusingly) the numeric image id.
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
        obstacle_list = " | ".join(
            "{};{};{};{}".format(o["id"], o["x"], o["y"], DIR_NAME[o["d"]])
            for o in data["obstacles"])

        payload = "STATUS,OBSTACLE_LIST,{},{};{};{},false,{},true,true".format(
            "true" if retrying else "false",
            robot_x, robot_y, DIR_NAME[robot_dir],
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
        """Ask image-api to tile this run's annotated captures (Task 1 RAW display).

        The stitch lives on image-api (:8001), not the Algo server, whose
        /stitch is commented out. FAQ 16: no tiled display, no score.
        """
        url = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/stitch"
        try:
            response = requests.get(url, timeout=30)
        except requests.RequestException as e:
            self.logger.error(f"Stitch request failed: {e}")
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
