import unittest
from unittest.mock import Mock
from communication.stm32 import STMLink


class TelemetryLoggingTests(unittest.TestCase):
    def make_link(self, **kwargs):
        link = STMLink(**kwargs)
        link.logger = Mock()
        link.serial_link = Mock()
        link.serial_link.read_until.side_effect = [
            b"!SENSOR/80.0/90.0/100.0/1/0.0/0.00;",
            b"!1/DONE;",
        ]
        return link

    def test_task1_quiet_mode_keeps_sensor_data_and_ack(self):
        link = self.make_link(log_telemetry=False)
        self.assertEqual(link.recv(), "ACK !1/DONE;")
        self.assertEqual(link.last_sensor, ["80.0", "90.0", "100.0", "1", "0.0", "0.00"])
        self.assertEqual(link.last_done_id, 1)
        messages = [call.args[0] for call in link.logger.debug.call_args_list]
        self.assertFalse(any("STM32 telemetry:" in message for message in messages))
        self.assertTrue(any("Received from STM32: !1/DONE;" in message for message in messages))
        link.serial_link.write.assert_not_called()

    def test_other_programs_keep_default_sensor_logging(self):
        link = self.make_link()
        self.assertEqual(link.recv(), "ACK !1/DONE;")
        messages = [call.args[0] for call in link.logger.debug.call_args_list]
        self.assertTrue(any("STM32 telemetry:" in message for message in messages))

    def test_quiet_mode_preserves_command_errors(self):
        link = self.make_link(log_telemetry=False)
        link.serial_link.read_until.side_effect = [b"!2/ERROR/INVALID_SPEED;"]
        self.assertEqual(link.recv(), "ERROR !2/ERROR/INVALID_SPEED;")
        link.logger.error.assert_called_once()


if __name__ == "__main__":
    unittest.main()
