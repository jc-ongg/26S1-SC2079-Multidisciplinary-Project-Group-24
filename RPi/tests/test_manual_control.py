"""Regression tests with fake links: no sockets, serial port or motor movement."""

import threading
import unittest
from collections import deque
from unittest.mock import Mock, patch

from communication.android_protocol import AndroidState, FrameBuffer
from communication.stm32 import STMLink
from manual_control import ManualController


class FakeAndroid:
    def __init__(self):
        self.messages = []

    def send(self, message):
        self.messages.append(message.value)

    def disconnect(self):
        pass


class FakeSTM:
    def __init__(self, replies):
        self.replies = deque(replies)
        self.sent = []
        self.last_done_id = None
        self.current_id = 0

    def send(self, command):
        self.sent.append(command)
        self.current_id += 1
        return self.current_id

    def recv(self):
        reply, done_id = self.replies.popleft() if self.replies else (None, None)
        self.last_done_id = done_id
        return reply

    def disconnect(self):
        pass


class ManualControlTests(unittest.TestCase):
    def controller(self, replies):
        return ManualController(android=FakeAndroid(), stm=FakeSTM(replies))

    def test_buttons_produce_correct_firmware_frames(self):
        link = STMLink.__new__(STMLink)
        link.speed = 50
        link._id = 0
        link._id_lock = threading.Lock()
        expected = {
            "F": "1/MOTOR/FWD/50/10;",
            "B": "2/MOTOR/REVS/50/10;",
            "L": "3/MOTOR/TURNL/50/90;",
            "R": "4/MOTOR/TURNR/50/90;",
        }
        state = AndroidState()
        frames = FrameBuffer()
        self.assertEqual(frames.feed("//F/"), [])
        payloads = frames.feed("///B////L////R//")
        self.assertEqual(payloads, list(expected))
        for button in payloads:
            command = state.handle(button)[0]["value"]
            self.assertEqual(link.translate(command), expected[button])

    def test_send_returns_id_of_actual_written_frame(self):
        link = STMLink.__new__(STMLink)
        link.speed = 50
        link._id = 0
        link._id_lock = threading.Lock()
        link.serial_link = Mock()
        link.logger = Mock()
        self.assertEqual(link.send("FW010"), 1)
        link.serial_link.write.assert_called_once_with(b"1/MOTOR/FWD/50/10;")
        link.serial_link.flush.assert_called_once()

    def test_only_matching_ack_completes_move(self):
        controller = self.controller([("ACK !99/DONE;", 99), ("ACK !1/DONE;", 1)])
        controller.busy.acquire()
        controller.execute("FW010")
        self.assertEqual(controller.stm.sent, ["FW010"])
        self.assertIsNone(controller.fault)
        self.assertIn("Manual forward complete", controller.android.messages)
        self.assertFalse(controller.busy.locked())

    def test_timeout_stops_and_blocks_later_commands(self):
        controller = self.controller([(None, None)])
        controller.busy.acquire()
        controller.execute("BW010")
        self.assertEqual(controller.stm.sent, ["BW010", "STOP"])
        controller.handle_message({"cat": "manual", "value": "FW010"})
        self.assertEqual(controller.stm.sent, ["BW010", "STOP"])

    def test_matching_stm_error_is_reported_and_stops(self):
        controller = self.controller([("ERROR !1/ERROR/NOT_READY;", None)])
        controller.busy.acquire()
        controller.execute("FL090")
        self.assertEqual(controller.stm.sent, ["FL090", "STOP"])
        self.assertTrue(any("NOT_READY" in message for message in controller.android.messages))

    def test_busy_button_presses_do_not_queue_stale_moves(self):
        controller = self.controller([])
        controller.busy.acquire()
        controller.handle_message({"cat": "manual", "value": "FW010"})
        self.assertEqual(controller.stm.sent, [])
        self.assertTrue(any("busy" in message for message in controller.android.messages))
        controller.busy.release()

    def test_task_start_does_not_launch_a_path_in_manual_mode(self):
        controller = self.controller([])
        controller.handle_message({"cat": "obstacles", "value": {"obstacles": []}})
        controller.handle_message({"cat": "control", "value": "start"})
        self.assertEqual(controller.stm.sent, [])
        self.assertTrue(any("Start is disabled" in message for message in controller.android.messages))

    def test_disconnect_during_move_sends_stop(self):
        controller = self.controller([])
        controller.busy.acquire()
        controller.close()
        self.assertEqual(controller.stm.sent, ["STOP"])
        self.assertTrue(controller.shutdown.is_set())
        controller.busy.release()

    def test_idle_disconnect_does_not_send_motor_commands(self):
        controller = self.controller([])
        controller.close()
        self.assertEqual(controller.stm.sent, [])

    def test_button_handler_dispatches_one_move(self):
        controller = self.controller([("ACK !1/DONE;", 1)])
        controller.handle_message({"cat": "manual", "value": "FW010"})
        controller.worker.join(timeout=2)
        self.assertFalse(controller.worker.is_alive())
        self.assertEqual(controller.stm.sent, ["FW010"])


if __name__ == "__main__":
    unittest.main()
