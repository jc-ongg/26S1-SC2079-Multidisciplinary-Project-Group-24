"""Launcher tests use fake hardware and HTTP; never start the actual robot."""
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import task1_main


class Task1LauncherTests(unittest.TestCase):
    def status_reply(self, **overrides):
        status = {'status': 'ok', 'movement_profile': 'restored-speed50-grid10',
                  'recommended_motor_speed': 50, 'grid_cell_cm': 10,
                  'input_cell_cm': 10, 'arena_cells': 20, 'turn_clearance_cm': 40}
        status.update(overrides)
        return io.BytesIO(json.dumps(status).encode())

    def test_matching_profile_speed(self):
        with patch.dict(sys.modules, {'settings': SimpleNamespace(API_IP='127.0.0.1', API_PORT=8000)}), \
                patch.object(task1_main.urllib.request, 'urlopen', return_value=self.status_reply()) as get:
            self.assertEqual(task1_main.check_profile(), 50)
            get.assert_called_once_with('http://127.0.0.1:8000/status', timeout=5)

    def test_mismatched_profile_grid_speed_and_missing_status_block_hardware(self):
        for overrides in [{'movement_profile': 'original'}, {'movement_profile': None},
                          {'recommended_motor_speed': 25}, {'grid_cell_cm': 5},
                          {'input_cell_cm': 5}, {'arena_cells': 40},
                          {'turn_clearance_cm': 30}, {'status': 'error'}]:
            robot = Mock()
            with patch.object(sys, 'argv', ['task1_main.py']), \
                    patch.dict(sys.modules, {'settings': SimpleNamespace(API_IP='127.0.0.1', API_PORT=8000),
                                             'task1_controller': SimpleNamespace(RaspberryPi=robot)}), \
                    patch.object(task1_main.urllib.request, 'urlopen', return_value=self.status_reply(**overrides)):
                with self.assertRaises(SystemExit) as error:
                    task1_main.main()
                self.assertEqual(error.exception.code, 1)
                robot.assert_not_called()

    def test_matching_setup_sets_speed_before_start(self):
        robot = Mock()
        robot.start.side_effect = lambda: self.assertEqual(robot.stm_link.speed, 50)
        factory = Mock(return_value=robot)
        with patch.object(sys, 'argv', ['task1_main.py']), \
                patch.object(task1_main, 'check_profile', return_value=50), \
                patch.dict(sys.modules, {'task1_controller': SimpleNamespace(RaspberryPi=factory)}):
            task1_main.main()
        factory.assert_called_once()
        robot.start.assert_called_once()

    def test_unreachable_algo_blocks_hardware(self):
        factory = Mock()
        with patch.object(sys, 'argv', ['task1_main.py']), \
                patch.object(task1_main, 'check_profile', side_effect=OSError('unreachable')), \
                patch.dict(sys.modules, {'task1_controller': SimpleNamespace(RaspberryPi=factory)}):
            with self.assertRaises(SystemExit):
                task1_main.main()
        factory.assert_not_called()

    def test_manual_bypasses_profile_check(self):
        manual = Mock()
        with patch.object(sys, 'argv', ['task1_main.py', '--manual']), \
                patch.object(task1_main, 'check_profile') as check, \
                patch.dict(sys.modules, {'manual_control': SimpleNamespace(main=manual)}):
            task1_main.main()
        check.assert_not_called()
        manual.assert_called_once()

    def test_obsolete_calibration_option_is_rejected(self):
        with patch.object(sys, 'argv', ['task1_main.py', '--calibrated']):
            with self.assertRaises(SystemExit) as exit:
                task1_main.main()
            self.assertEqual(exit.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
