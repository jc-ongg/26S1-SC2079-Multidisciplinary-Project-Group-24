"""Recorded-map regression in Flask's test client, never the live robot."""
import contextlib
import io
import json
import os
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ['YOLO_AUTOINSTALL'] = 'false'
from main import app
import consts
import importlib.util

spec = importlib.util.spec_from_file_location('task1_plan_validation', Path(__file__).resolve().parents[2] / 'RPi/path_plan.py')
validation = importlib.util.module_from_spec(spec)
spec.loader.exec_module(validation)


class PathApiTests(unittest.TestCase):
    def test_eight_obstacle_layout_rejects_unreachable_candidate_without_crashing(self):
        payload = ('STATUS,OBSTACLE_LIST,false,1;1;NORTH,false,'
                   '1;5;13;WEST | 2;5;7;SOUTH | 3;12;9;EAST | '
                   '4;15;15;SOUTH | 5;15;4;NORTH | 6;3;17;EAST | '
                   '7;9;17;SOUTH | 8;17;10;WEST,true,true')
        with contextlib.redirect_stdout(io.StringIO()):
            response = app.test_client().post('/path', data=payload, content_type='text/plain')
        self.assertEqual(response.status_code, 200)
        plan = validation.parse_plan(response.json)
        photographed = {int(command[4:].split('_')[0]) for command, _ in plan
                        if command.startswith('SNAP')}
        # The planner can fall back to a valid subset in a crowded layout.
        # A crashed API or a fabricated full route cannot earn partial credit.
        self.assertTrue(photographed)
        self.assertTrue(photographed.issubset(set(range(1, 9))))
        self.assertEqual(plan[-1][0], 'FIN')

    def test_real_algo_response_is_accepted_by_pi_with_aligned_poses(self):
        golden = Path(__file__).resolve().parents[2] / 'RPi-services/golden/expected_path.json'
        for case in json.loads(golden.read_text()):
            with self.subTest(payload=case['payload']), contextlib.redirect_stdout(io.StringIO()):
                response = app.test_client().post('/path', data=case['payload'], content_type='text/plain')
                self.assertEqual(response.status_code, 200)
                plan = validation.parse_plan(response.json)
                self.assertEqual(len(plan), len(response.json['data']['stm_commands']))
                self.assertEqual(plan[-1][0], 'FIN')
                self.assertEqual(plan[-1][1], {key: response.json['data']['path'][-1][key]
                    for key in ('x', 'y', 'd')})

    def test_revisit_skip_obstacle_is_retained_but_not_photographed(self):
        payload = 'STATUS,OBSTACLE_LIST,true,1;1;NORTH,false,1;5;10;SOUTH | 2;15;15;SKIP,true,true'
        with contextlib.redirect_stdout(io.StringIO()):
            response = app.test_client().post('/path', data=payload, content_type='text/plain')
        self.assertEqual(response.status_code, 200)
        commands = response.json['data']['stm_commands']
        self.assertTrue(any(command.startswith('SNAP1') for command in commands))
        self.assertFalse(any(command.startswith('SNAP2') for command in commands))
        from entities.Entity import Grid, Obstacle
        arena = Grid(20, 20)
        arena.add_obstacle(Obstacle(15, 15, 8, 2))
        self.assertFalse(arena.reachable(15, 15))

    def test_status_identifies_the_loaded_profile_and_speed(self):
        response = app.test_client().get('/status')
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json, {'status': 'ok',
                                        'movement_profile': consts.MOVEMENT_PROFILE,
                                        'recommended_motor_speed': consts.RECOMMENDED_MOTOR_SPEED,
                                        'grid_cell_cm': 10, 'input_cell_cm': 10,
                                        'arena_cells': 20, 'turn_clearance_cm': 30,
                                        'forward_calibration': consts.FORWARD_CALIBRATION,
                                        'reverse_calibration': consts.REVERSE_CALIBRATION})

    def test_recorded_maps_cover_both_targets_on_restored_grid(self):
        golden = Path(__file__).resolve().parents[2] / 'RPi-services/golden/expected_path.json'
        for case in json.loads(golden.read_text()):
            with self.subTest(payload=case['payload']), contextlib.redirect_stdout(io.StringIO()):
                response = app.test_client().post('/path', data=case['payload'], content_type='text/plain')
                self.assertEqual(response.status_code, 200)
                self.assertIsNone(response.json['error'])
                commands = response.json['data']['stm_commands']
                self.assertEqual({int(c[4:].split('_')[0]) for c in commands if c.startswith('SNAP')}, {1, 2})
                self.assertEqual(response.json['data']['grid_cell_cm'], 10)
                self.assertEqual(response.json['data']['input_cell_cm'], 10)
