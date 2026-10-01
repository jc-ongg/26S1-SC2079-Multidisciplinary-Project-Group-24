"""Software scaling checks, not claims of physical turn calibration."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import consts
from helper import command_converter, command_generator


class MovementBaselineTests(unittest.TestCase):
    def test_forward_and_reverse_cells_are_ten_cm(self):
        for prefix, direction in [('FW', 'forward'), ('BW', 'reverse')]:
            for cells in [1, 2, 4, 8, 12]:
                with self.subTest(prefix=prefix, cells=cells):
                    self.assertEqual(command_converter(f'{prefix}{cells:02}'),
                                     f'center,0,{direction},{cells * 10}')

    def test_original_turn_geometry_is_preserved(self):
        self.assertEqual((consts.FORWARD_LEFT_LONGITUDINAL_OFFSET,
                          consts.FORWARD_LEFT_LATERAL_OFFSET), (2, 4))
        self.assertEqual((consts.FORWARD_RIGHT_LONGITUDINAL_OFFSET,
                          consts.FORWARD_RIGHT_LATERAL_OFFSET), (2, 4))
        self.assertEqual((consts.BACKWARD_LEFT_LONGITUDINAL_OFFSET,
                          consts.BACKWARD_LEFT_LATERAL_OFFSET), (3, 2))
        self.assertEqual((consts.BACKWARD_RIGHT_LONGITUDINAL_OFFSET,
                          consts.BACKWARD_RIGHT_LATERAL_OFFSET), (3, 2))
        self.assertEqual((consts.FORWARD_LEFT, consts.FORWARD_RIGHT,
                          consts.REVERSE_LEFT, consts.REVERSE_RIGHT),
                         ((3, 5), (0, 4), (0, 2), (0, 4)))

    def test_original_turn_padding_commands(self):
        # North-facing representative endpoints for FL, FR, BL and BR.
        cases = [
            ((6, 12, 6), ['center,0,forward,3', 'left,90,forward,0', 'center,0,forward,5']),
            ((14, 12, 2), ['right,90,forward,0', 'center,0,forward,4']),
            ((8, 7, 2), ['left,90,reverse,0', 'center,0,reverse,2']),
            ((12, 7, 6), ['right,90,reverse,0', 'center,0,reverse,4']),
        ]
        start = SimpleNamespace(x=10, y=10, direction=0, screenshot_id=-1)
        for (x, y, direction), expected in cases:
            end = SimpleNamespace(x=x, y=y, direction=direction, screenshot_id=-1)
            self.assertEqual(command_generator([start, end], [])[2], expected + ['FIN'])


if __name__ == '__main__':
    unittest.main()
