#!/usr/bin/env python3
"""Android manual controls, independent of path planning and image recognition."""

import json
import logging
import re
import threading
import time

from communication.android import AndroidLink, AndroidMessage
from communication.android_protocol import MANUAL_MOVES
from communication.stm32 import READ_TIMEOUT, STMLink


COMMAND_NAMES = dict(zip(MANUAL_MOVES.values(), ("forward", "backward", "left", "right")))


class ManualController:
    def __init__(self, android=None, stm=None):
        self.android = android if android is not None else AndroidLink()
        self.stm = stm if stm is not None else STMLink()
        self.logger = logging.getLogger("manual_control")
        self.busy = threading.Lock()
        self.write_lock = threading.Lock()
        self.status_lock = threading.Lock()
        self.shutdown = threading.Event()
        self.worker = None
        self.fault = None

    def status(self, text):
        self.logger.info(text)
        if self.shutdown.is_set():
            return
        try:
            with self.status_lock:
                self.android.send(AndroidMessage("status", text))
        except OSError:
            self.logger.warning("Could not send manual-control status to Android")

    def handle_message(self, message):
        if message.get("cat") != "manual":
            if message.get("cat") == "control":
                self.status("Manual mode: use the arrow buttons; Task 1 Start is disabled")
            return

        command = message.get("value")
        if command not in COMMAND_NAMES:
            self.status("Unsupported manual command")
            return
        if self.fault:
            self.status("Manual controls paused after STM32 error; restart this controller")
            return
        if not self.busy.acquire(blocking=False):
            self.status("Robot is busy; wait for movement to finish before pressing again")
            return

        self.worker = threading.Thread(target=self.execute, args=(command,), daemon=True)
        self.worker.start()

    def stop_motor(self):
        try:
            with self.write_lock:
                self.stm.send("STOP")
        except Exception:
            self.logger.exception("Could not send STOP to STM32")

    def fail(self, reason):
        self.fault = reason
        self.stop_motor()
        self.status(reason + "; manual controls paused")

    def execute(self, command):
        name = COMMAND_NAMES[command]
        try:
            with self.write_lock:
                command_id = self.stm.send(command)
            self.status("Manual " + name + " sent; waiting for STM32")
            deadline = time.monotonic() + READ_TIMEOUT

            while not self.shutdown.is_set() and time.monotonic() < deadline:
                reply = self.stm.recv()
                if not reply:
                    break
                if reply.startswith("ACK") and self.stm.last_done_id == command_id:
                    self.status("Manual " + name + " complete")
                    return
                if reply.startswith("ERROR"):
                    match = re.search(r"!(\d+)/ERROR/([^;]+)", reply)
                    if match and int(match.group(1)) == command_id:
                        self.fail("STM32 rejected " + name + ": " + match.group(2))
                        return
                self.logger.warning("Ignoring reply for a different command: %s", reply)

            if not self.shutdown.is_set():
                self.fail("No matching STM32 completion for " + name)
        except Exception as exc:
            if not self.shutdown.is_set():
                self.logger.exception("Manual movement failed")
                self.fail("Manual " + name + " failed: " + str(exc))
        finally:
            self.busy.release()

    def run(self):
        self.stm.connect()
        self.android.connect()
        self.status("Manual controls ready: Forward/Back 10 cm; Left/Right 90 degrees")
        while not self.shutdown.is_set():
            raw = self.android.recv()
            if raw:
                self.handle_message(json.loads(raw))

    def close(self):
        self.shutdown.set()
        if self.busy.locked():
            self.stop_motor()
        self.android.disconnect()
        self.stm.disconnect()
        if self.worker is not None:
            self.worker.join(timeout=2)


def main():
    controller = ManualController()
    try:
        controller.run()
    except KeyboardInterrupt:
        controller.logger.info("Manual controller stopped")
    except OSError as exc:
        controller.logger.warning("Manual connection closed: %s", exc)
    finally:
        controller.close()


if __name__ == "__main__":
    main()
