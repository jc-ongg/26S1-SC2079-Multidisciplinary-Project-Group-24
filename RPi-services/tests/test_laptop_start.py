import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
import laptop_start


class LaptopStartTests(unittest.TestCase):
    def args(self):
        return SimpleNamespace(python=sys.executable, algo_port=8000, image_port=8001)

    def test_single_configuration_has_no_overlay_environment(self):
        specs, _ = laptop_start.service_specs(self.args())
        self.assertEqual(specs[0][3], {'ALGO_PORT': '8000', 'CV_BASE_URL': 'http://127.0.0.1:8001'})

    def test_obsolete_profile_switches_are_rejected_before_spawn(self):
        for flag in ['--movement-profile', '--forward-calibration', '--reverse-calibration']:
            with patch.object(sys, 'argv', ['start', flag, 'original']), \
                    patch.object(laptop_start.subprocess, 'Popen') as spawn:
                with self.assertRaises(SystemExit) as error:
                    laptop_start.main()
                self.assertEqual(error.exception.code, 2)
                spawn.assert_not_called()

    def test_only_onnx_image_api_is_started(self):
        specs, paths = laptop_start.service_specs(self.args())
        self.assertEqual(specs[1][0], 'image-api')
        self.assertEqual(specs[1][1][1:], ['main.py'])
        self.assertEqual(paths[-1].name, 'best_v11i_m.onnx')

    def test_best_pt_backend_is_rejected(self):
        with patch.object(sys, 'argv', ['start', '--cv-backend', 'trial']), \
                patch.object(laptop_start.subprocess, 'Popen') as spawn:
            with self.assertRaises(SystemExit) as error:
                laptop_start.main()
            self.assertEqual(error.exception.code, 2)
            spawn.assert_not_called()

    def test_no_best_pt_anywhere_in_repo(self):
        repo = ROOT.parent
        self.assertFalse((repo / 'Algo' / 'best.pt').exists())
        self.assertFalse((repo / 'Algo' / 'cv_trial_server.py').exists())

    def test_check_never_starts_processes(self):
        with patch.object(sys, 'argv', ['start', '--check']), patch.object(laptop_start, 'up', return_value=False), \
                patch.object(laptop_start, 'lan_ips', return_value=[]), patch.object(laptop_start.subprocess, 'Popen') as spawn:
            laptop_start.main()
            spawn.assert_not_called()

    def test_busy_second_port_prevents_first_server_start(self):
        with patch.object(sys, 'argv', ['start']), patch.object(Path, 'is_file', return_value=True), \
                patch.object(laptop_start, 'up', return_value=False), patch.object(laptop_start, 'lan_ips', return_value=[]), \
                patch.object(laptop_start, 'port_busy', side_effect=[False, True]), \
                patch.object(laptop_start.subprocess, 'Popen') as spawn:
            self.assertEqual(laptop_start.main(), 1)
            spawn.assert_not_called()

    def test_cleanup_touches_only_owned_children(self):
        log, proc = Mock(), Mock()
        proc.poll.return_value = None
        laptop_start.stop_children([('Algo', '', log, proc)])
        proc.terminate.assert_called_once()
        proc.wait.assert_called_once_with(timeout=10)
        log.close.assert_called_once()
