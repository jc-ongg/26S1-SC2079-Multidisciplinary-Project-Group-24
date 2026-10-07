"""New speed50 reverse measurements: software checks, no physical movement."""
import contextlib
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import consts
from algo.algo import PathFinder, LEFT_FRONT_STEER_RADIUS, RIGHT_FRONT_STEER_RADIUS, LEFT_REAR_STEER_RADIUS, RIGHT_REAR_STEER_RADIUS
from entities.Entity import CellState
from helper import command_generator
import test_movement_calibration as calibration
from test_movement_calibration import HEADINGS, MEASURED, simulate_measured_commands, turn_endpoint


@unittest.skipUnless(consts.REVERSE_CALIBRATION == 'speed50-v2', 'reverse50-only trial')
class ReverseCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.stdout = contextlib.redirect_stdout(io.StringIO())
        self.stdout.__enter__()
        self.addCleanup(self.stdout.__exit__, None, None, None)

    @unittest.skipUnless(consts.FORWARD_CALIBRATION == 'original', 'reverse-only isolation check')
    def test_forward_settings_and_speed_are_unchanged(self):
        self.assertEqual(consts.RECOMMENDED_MOTOR_SPEED, 50)
        self.assertEqual((consts.FORWARD_LEFT_LONGITUDINAL_OFFSET, consts.FORWARD_LEFT_LATERAL_OFFSET), (2, 4))
        self.assertEqual((consts.FORWARD_RIGHT_LONGITUDINAL_OFFSET, consts.FORWARD_RIGHT_LATERAL_OFFSET), (2, 4))
        self.assertEqual((consts.FORWARD_LEFT, consts.FORWARD_RIGHT), ((3, 5), (0, 4)))
        self.assertEqual((consts.UNIT_TO_CM_MULTIPLIER, consts.UNIT_TO_CM_MULTIPLIER_REVERSE), (10, 10))

    def test_reverse_commands_match_new_measurements_in_all_headings(self):
        planner = PathFinder(20, 20, 10, 10, consts.Direction.NORTH)
        cases = {'BL': ((3, 2), (1, 5), (29.5, 15)),
                 'BR': ((4, 3), (6, 8), (34, 21.5))}
        for heading in HEADINGS:
            mapped = planner._map_turn_logic(10, 10, heading, LEFT_FRONT_STEER_RADIUS,
                                            RIGHT_FRONT_STEER_RADIUS, LEFT_REAR_STEER_RADIUS,
                                            RIGHT_REAR_STEER_RADIUS)
            for turn, (geometry, padding, measured) in cases.items():
                with self.subTest(heading=heading, turn=turn):
                    endpoint = turn_endpoint(10, 10, heading, turn, *geometry)
                    self.assertIn(endpoint, mapped)
                    commands = command_generator([CellState(10, 10, heading), CellState(*endpoint)], [])[2]
                    steering = 'left' if turn == 'BL' else 'right'
                    self.assertEqual(commands, [f'center,0,reverse,{padding[0]}',
                                                f'{steering},90,reverse,0',
                                                f'center,0,reverse,{padding[1]}', 'FIN'])
                    with patch.dict(MEASURED, {turn: measured}):
                        actual = simulate_measured_commands(commands, (100, 100, heading))
                    self.assertAlmostEqual(actual[0], endpoint[0] * 10, delta=.51)
                    self.assertAlmostEqual(actual[1], endpoint[1] * 10, delta=.51)
                    self.assertEqual(actual[2], endpoint[2])

    def test_reverse_padding_precedes_capture(self):
        end = CellState(13, 6, consts.Direction.WEST, screenshot_id=1)
        commands = command_generator([CellState(10, 10, consts.Direction.NORTH), end],
                                     [{'id': 1, 'x': 9, 'y': 6, 'd': 2}])[2]
        self.assertEqual(commands[:3], ['center,0,reverse,6', 'right,90,reverse,0', 'center,0,reverse,8'])
        self.assertTrue(commands[-2].startswith('SNAP1'))
        self.assertEqual(commands[-1], 'FIN')

    # Shared structural invariants apply to either asymmetric trial profile.
    assert_valid_edges = calibration.MovementCalibrationTests.assert_valid_edges
    test_archiving_does_not_invent_a_reverse_route = calibration.MovementCalibrationTests.test_archiving_does_not_invent_a_reverse_route
    test_batch_plans_both_directions_independently = calibration.MovementCalibrationTests.test_batch_plans_both_directions_independently
    test_tsp_matrix_preserves_directional_costs = calibration.MovementCalibrationTests.test_tsp_matrix_preserves_directional_costs
    test_two_obstacle_mission_uses_valid_directed_movements = calibration.MovementCalibrationTests.test_two_obstacle_mission_uses_valid_directed_movements
