"""Speed50 forward endpoint checks using the user's new single-trial measurements."""
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
from main import app
from test_movement_calibration import HEADINGS, MEASURED, simulate_measured_commands, turn_endpoint


@unittest.skipUnless(consts.FORWARD_CALIBRATION == 'speed50-v2', 'forward50-only trial')
class ForwardCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.stdout = contextlib.redirect_stdout(io.StringIO())
        self.stdout.__enter__()
        self.addCleanup(self.stdout.__exit__, None, None, None)

    def test_reverse_settings_are_preserved(self):
        self.assertEqual(consts.RECOMMENDED_MOTOR_SPEED, 50)
        self.assertEqual((consts.UNIT_TO_CM_MULTIPLIER, consts.UNIT_TO_CM_MULTIPLIER_REVERSE), (10, 10))
        expected = ((3, 2), (4, 3), (1, 5), (6, 8)) if consts.REVERSE_CALIBRATION == 'speed50-v2' else ((3, 2), (3, 2), (0, 2), (0, 4))
        actual = ((consts.BACKWARD_LEFT_LONGITUDINAL_OFFSET, consts.BACKWARD_LEFT_LATERAL_OFFSET),
                  (consts.BACKWARD_RIGHT_LONGITUDINAL_OFFSET, consts.BACKWARD_RIGHT_LATERAL_OFFSET),
                  consts.REVERSE_LEFT, consts.REVERSE_RIGHT)
        self.assertEqual(actual, expected)

    def test_forward_endpoints_and_commands_in_all_headings(self):
        planner = PathFinder(20, 20, 10, 10, consts.Direction.NORTH)
        cases = {'FL': ((2, 4), (5, 7), (15, 33)),
                 'FR': ((3, 4), (5, 3), (25, 37.5))}
        for heading in HEADINGS:
            mapped = planner._map_turn_logic(10, 10, heading, LEFT_FRONT_STEER_RADIUS,
                                            RIGHT_FRONT_STEER_RADIUS, LEFT_REAR_STEER_RADIUS,
                                            RIGHT_REAR_STEER_RADIUS)
            for turn, (geometry, padding, measured) in cases.items():
                with self.subTest(heading=heading, turn=turn):
                    endpoint = turn_endpoint(10, 10, heading, turn, *geometry)
                    self.assertIn(endpoint, mapped)
                    commands = command_generator([CellState(10, 10, heading), CellState(*endpoint)], [])[2]
                    steering = 'left' if turn == 'FL' else 'right'
                    self.assertEqual(commands, [f'center,0,forward,{padding[0]}',
                                                f'{steering},90,forward,0',
                                                f'center,0,forward,{padding[1]}', 'FIN'])
                    with patch.dict(MEASURED, {turn: measured}):
                        actual = simulate_measured_commands(commands, (100, 100, heading))
                    self.assertAlmostEqual(actual[0], endpoint[0] * 10, delta=.51)
                    self.assertAlmostEqual(actual[1], endpoint[1] * 10, delta=.51)
                    self.assertEqual(actual[2], endpoint[2])

    def test_opposite_straight_cancellation_preserves_endpoint(self):
        states = [CellState(10, 11, consts.Direction.NORTH),
                  CellState(10, 10, consts.Direction.NORTH),
                  CellState(14, 13, consts.Direction.EAST)]
        commands = command_generator(states, [])[2]
        self.assertEqual(commands, ['center,0,reverse,5', 'right,90,forward,0', 'center,0,forward,3', 'FIN'])
        with patch.dict(MEASURED, {'FR': (25, 37.5)}):
            actual = simulate_measured_commands(commands, (100, 110, consts.Direction.NORTH))
        self.assertAlmostEqual(actual[0], 140, delta=.51)
        self.assertAlmostEqual(actual[1], 130, delta=.51)

    def test_padding_finishes_before_snapshot(self):
        end = CellState(14, 13, consts.Direction.EAST, screenshot_id=1)
        commands = command_generator([CellState(10, 10, consts.Direction.NORTH), end],
                                     [{'id': 1, 'x': 18, 'y': 13, 'd': 6}])[2]
        self.assertEqual(commands[:3], ['center,0,forward,5', 'right,90,forward,0', 'center,0,forward,3'])
        self.assertTrue(commands[-2].startswith('SNAP1'))
        self.assertEqual(commands[-1], 'FIN')

    def test_recorded_map_api_and_route_edges(self):
        obstacles = [{'x': 10, 'y': 10, 'd': 4, 'id': 1}, {'x': 15, 'y': 15, 'd': 6, 'id': 2}]
        planner = PathFinder(20, 20, 1, 1, consts.Direction.NORTH)
        for ob in obstacles:
            planner.insert_target(ob['x'], ob['y'], consts.Direction(ob['d']), ob['id'])
        path, _ = planner.solve_mission(False)
        self.assertEqual({s.screenshot_id for s in path if s.screenshot_id != -1}, {1, 2})
        for start, end in zip(path, path[1:]):
            neighbors = [tuple(n[:3]) for n in planner._fetch_neighbors(start.x, start.y, start.direction)]
            self.assertIn((end.x, end.y, end.direction), neighbors)
        expected_commands = command_generator(path, obstacles)[2]
        response = app.test_client().post('/path', content_type='text/plain',
                    data='STATUS,OBSTACLE_LIST,false,1;1;NORTH,false,1;10;10;SOUTH | 2;15;15;WEST,true,true')
        self.assertEqual(response.status_code, 200)
        self.assertIsNone(response.json['error'])
        self.assertEqual(response.json['data']['stm_commands'], expected_commands)
        self.assertEqual(expected_commands[-1], 'FIN')
