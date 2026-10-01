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
    def args(self, backend):
        return SimpleNamespace(python=sys.executable, cv_backend=backend,
                               algo_port=8000, image_port=8001, movement_profile='original',
                               reverse_calibration='original', forward_calibration='original')

    def test_combined_calibration_is_forwarded_only_to_algo(self):
        args = self.args('trial')
        args.forward_calibration = args.reverse_calibration = 'speed50-v2'
        specs, _ = laptop_start.service_specs(args)
        self.assertEqual(specs[0][3]['MDP_FORWARD_CALIBRATION'], 'speed50-v2')
        self.assertEqual(specs[0][3]['MDP_REVERSE_CALIBRATION'], 'speed50-v2')
        self.assertNotIn('MDP_FORWARD_CALIBRATION', specs[1][3])

    def test_forward_calibration_rejects_speed25_profile_before_spawn(self):
        with patch.object(sys, 'argv', ['start', '--movement-profile', 'measured25',
                                      '--forward-calibration', 'speed50-v2']), \
                patch.object(laptop_start.subprocess, 'Popen') as spawn:
            with self.assertRaises(SystemExit) as error:
                laptop_start.main()
            self.assertEqual(error.exception.code, 2)
            spawn.assert_not_called()

    def test_reverse_calibration_is_forwarded_only_to_algo(self):
        args = self.args('trial')
        args.reverse_calibration = 'speed50-v2'
        specs, _ = laptop_start.service_specs(args)
        self.assertEqual(specs[0][3]['MDP_REVERSE_CALIBRATION'], 'speed50-v2')
        self.assertNotIn('MDP_REVERSE_CALIBRATION', specs[1][3])

    def test_measured_profile_is_forwarded_only_to_algo(self):
        args = self.args('trial')
        args.movement_profile = 'measured25'
        specs, _ = laptop_start.service_specs(args)
        self.assertEqual(specs[0][3]['MDP_MOVEMENT_PROFILE'], 'measured25')
        self.assertNotIn('MDP_MOVEMENT_PROFILE', specs[1][3])

    def test_trial_uses_explicit_fallback_and_port(self):
        specs, paths = laptop_start.service_specs(self.args('trial'))
        self.assertEqual(specs[1][1][1:], ['cv_trial_server.py', '--port', '8001'])
        self.assertEqual(paths[-1].name, 'best.pt')

    def test_onnx_uses_original_model(self):
        specs, paths = laptop_start.service_specs(self.args('onnx'))
        self.assertEqual(specs[1][1][1:], ['main.py'])
        self.assertEqual(paths[-1].name, 'best_v11i_m.onnx')

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
