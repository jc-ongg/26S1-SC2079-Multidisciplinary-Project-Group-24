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
    def status_reply(self, profile=None):
        status = {'status': 'ok'}
        if profile is not None:
            status['movement_profile'] = profile
        return io.BytesIO(json.dumps(status).encode())

    def test_matching_profile_speed(self):
        for calibrated, profile, speed in [(True, 'measured25', 25), (False, 'original', 50)]:
            with patch.dict(sys.modules, {'settings': SimpleNamespace(API_IP='127.0.0.1', API_PORT=8000)}), \
                    patch.object(task1_main.urllib.request, 'urlopen', return_value=self.status_reply(profile)) as get:
                self.assertEqual(task1_main.check_profile(calibrated), speed)
                get.assert_called_once_with('http://127.0.0.1:8000/status', timeout=5)

    def test_mismatched_or_missing_calibrated_profile_blocks_hardware(self):
        for arguments, profile in [(['--calibrated'], 'original'), (['--calibrated'], None), ([], 'measured25')]:
            robot = Mock()
            with patch.object(sys, 'argv', ['task1_main.py'] + arguments), \
                    patch.dict(sys.modules, {'settings': SimpleNamespace(API_IP='127.0.0.1', API_PORT=8000),
                                             'final_week_8_backup': SimpleNamespace(RaspberryPi=robot)}), \
                    patch.object(task1_main.urllib.request, 'urlopen', return_value=self.status_reply(profile)):
                with self.assertRaises(SystemExit) as exit:
                    task1_main.main()
                self.assertEqual(exit.exception.code, 1)
                robot.assert_not_called()

    def test_matching_trial_sets_speed_before_start(self):
        robot = Mock()
        robot.start.side_effect = lambda: self.assertEqual(robot.stm_link.speed, 25)
        factory = Mock(return_value=robot)
        with patch.object(sys, 'argv', ['task1_main.py', '--calibrated']), \
                patch.object(task1_main, 'check_profile', return_value=25), \
                patch.dict(sys.modules, {'final_week_8_backup': SimpleNamespace(RaspberryPi=factory)}):
            task1_main.main()
        factory.assert_called_once()
        robot.start.assert_called_once()

    def test_unreachable_algo_blocks_hardware(self):
        factory = Mock()
        with patch.object(sys, 'argv', ['task1_main.py', '--calibrated']), \
                patch.object(task1_main, 'check_profile', side_effect=OSError('unreachable')), \
                patch.dict(sys.modules, {'final_week_8_backup': SimpleNamespace(RaspberryPi=factory)}):
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

    def test_manual_and_calibrated_cannot_be_combined(self):
        with patch.object(sys, 'argv', ['task1_main.py', '--manual', '--calibrated']):
            with self.assertRaises(SystemExit) as exit:
                task1_main.main()
            self.assertEqual(exit.exception.code, 2)


if __name__ == '__main__':
    unittest.main()
