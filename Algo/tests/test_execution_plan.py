"""Motor endpoints must follow compound turns and cancelled moves, never precede them."""
import sys
from pathlib import Path
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from execution_plan import build_command_poses
from consts import FORWARD_LEFT, UNIT_TO_CM_MULTIPLIER


class ExecutionPlanTests(unittest.TestCase):
    def test_turn_endpoint_is_only_on_last_nonzero_command(self):
        start = {'x': 5, 'y': 5, 'd': 0}
        end = {'x': 1, 'y': 7, 'd': 6}
        pads = [int(value / 10 * UNIT_TO_CM_MULTIPLIER) for value in FORWARD_LEFT]
        commands = ([f'center,0,forward,{pads[0]}'] if pads[0] else []) + ['left,90,forward,0']
        if pads[1]: commands.append(f'center,0,forward,{pads[1]}')
        poses = build_command_poses(['FL01', 'FIN'], [start, end], commands + ['FIN'])
        self.assertEqual(poses[:len(commands)-1], [None] * (len(commands)-1))
        self.assertEqual(poses[-2:], [end, end])

    def test_compressed_move_reports_its_full_endpoint(self):
        start = {'x': 1, 'y': 1, 'd': 0}
        end = {'x': 1, 'y': 4, 'd': 0}
        self.assertEqual(build_command_poses(['FW03', 'SNAP1', 'FIN'], [start, end],
            ['center,0,forward,30', 'SNAP1', 'FIN']), [end, end, end])

    def test_fully_cancelled_straights_do_not_consume_motor_poses(self):
        start = {'x': 1, 'y': 1, 'd': 0}
        middle = {'x': 1, 'y': 2, 'd': 0}
        self.assertEqual(build_command_poses(['FW01', 'BW01', 'SNAP1', 'FIN'],
            [start, middle, start], ['SNAP1', 'FIN']), [start, start])

    def test_schedule_mismatch_is_rejected(self):
        pose = {'x': 1, 'y': 1, 'd': 0}
        with self.assertRaises(ValueError):
            build_command_poses(['FIN'], [pose], ['center,0,forward,10', 'FIN'])


if __name__ == '__main__':
    unittest.main()
