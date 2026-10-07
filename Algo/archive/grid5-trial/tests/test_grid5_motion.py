"""Speed-50 measured endpoint checks; no claim of physical arena validation."""
import contextlib
import io
import itertools
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import consts
from coordinates import to_planner, to_tablet_pose
from entities.Entity import CellState, Grid, Obstacle
from algo.algo import PathFinder
from helper import command_converter, command_generator
from execution_plan import build_command_poses
from motor_schedule import coalesce

VECTORS = ((0, 1), (1, 0), (0, -1), (-1, 0))
BARE_CM = {'FL': (17, 30), 'FR': (22, 35), 'BL': (30, 15), 'BR': (35, 23)}
TOTAL_CM = {'FL': (20, 30), 'FR': (25, 35), 'BL': (30, 15), 'BR': (35, 25)}

def relative_turn(x, y, heading, turn, longitudinal, lateral):
    fx, fy = VECTORS[heading // 2]
    side = -1 if turn.endswith('L') else 1
    travel = -1 if turn.startswith('B') else 1
    return (x + travel * longitudinal * fx + side * lateral * fy,
            y + travel * longitudinal * fy - side * lateral * fx,
            (heading + side * travel * 2) % 8)

def replay(commands, start):
    x, y, heading = start
    for command in commands:
        if command.startswith(('SNAP', 'FIN')):
            continue
        steering, angle, travel, cm = command.split(',')
        if steering == 'center':
            fx, fy = VECTORS[heading // 2]
            distance = int(cm) * (-1 if travel == 'reverse' else 1)
            x, y = x + fx * distance, y + fy * distance
        else:
            assert angle == '90'
            turn = ('B' if travel == 'reverse' else 'F') + ('L' if steering == 'left' else 'R')
            x, y, heading = relative_turn(x, y, heading, turn, *BARE_CM[turn])
    return x, y, heading

class Grid5MotionTests(unittest.TestCase):
    def test_five_cm_steps_both_directions(self):
        self.assertEqual(consts.RECOMMENDED_MOTOR_SPEED, 50)
        for steps in (1, 2, 4, 8, 12):
            for prefix, travel in [('FW', 'forward'), ('BW', 'reverse')]:
                self.assertEqual(command_converter(f'{prefix}{steps:02}'), f'center,0,{travel},{steps * 5}')

    def test_all_turns_in_all_headings_match_user_measurements(self):
        planner = PathFinder(40, 40, 20, 20, 0)
        for heading, turn in itertools.product(range(0, 8, 2), TOTAL_CM):
            longitudinal, lateral = TOTAL_CM[turn]
            end = relative_turn(20, 20, heading, turn, longitudinal // 5, lateral // 5)
            self.assertIn(end, [tuple(n[:3]) for n in planner._fetch_neighbors(20, 20, heading)])
            states = [CellState(20, 20, heading), CellState(*end)]
            raw, _, motors = command_generator(states, [])
            self.assertEqual(raw, [turn + '01', 'FIN'])
            self.assertEqual(replay(motors, (100, 100, heading)), (end[0] * 5, end[1] * 5, end[2]))
            endpoints = build_command_poses(raw, [s.get_dict() for s in states], motors)
            self.assertEqual(endpoints[-2], {k: states[1].get_dict()[k] for k in ('x', 'y', 'd')})
            self.assertEqual(endpoints[:-2], [None] * (len(motors) - 2))

    def test_merging_and_cancelling_do_not_cross_capture_or_turn(self):
        values = [('center,0,forward,20', 'first'), ('center,0,forward,3', None),
                  ('right,90,forward,0', 'turn'), ('center,0,reverse,10', 'a'),
                  ('center,0,forward,5', 'b'), ('SNAP1', 'b'), ('center,0,forward,5', 'c'), ('FIN', 'c')]
        self.assertEqual(coalesce(values), [('center,0,forward,23', None), ('right,90,forward,0', 'turn'),
            ('center,0,reverse,5', 'b'), ('SNAP1', 'b'), ('center,0,forward,5', 'c'), ('FIN', 'c')])

    def test_exact_revisit_coordinates_survive_tablet_rounding(self):
        self.assertEqual(to_planner(19), 38)
        self.assertEqual(to_planner(5.5), 11)
        pose = to_tablet_pose({'x': 11, 'y': 9, 'd': 2})
        self.assertEqual(pose, {'x': 6, 'y': 5, 'd': 2, 'planner_x': 11, 'planner_y': 9})
        for value in (-1, 20, 1.25):
            with self.assertRaises(ValueError):
                to_planner(value)

    def test_arena_bounds_and_obstacle_clearance_preserve_centimetres(self):
        arena = Grid(40, 40)
        self.assertTrue(arena.is_valid_coord(36, 2))
        self.assertFalse(arena.is_valid_coord(37, 2))
        arena.add_obstacle(Obstacle(20, 20, 8, 1))
        self.assertFalse(arena.reachable(25, 20))  # 25 cm below 30 cm clearance.
        self.assertTrue(arena.reachable(26, 20))
        self.assertFalse(arena.reachable(26, 20, turn=True))
        views = Obstacle(20, 20, 0, 1).get_view_state(False)
        self.assertEqual({(v.x, v.y) for v in views}, {(20, 26), (20, 28), (22, 28), (18, 28)})

    def test_batched_directed_paths_agree_with_independent_astar(self):
        states = [CellState(4, 4, 0), CellState(20, 24, 2), CellState(32, 30, 4)]
        batch = PathFinder(40, 40, 4, 4, 0)
        batch._batch_calculate_segments(states)
        for start, end in itertools.permutations(states, 2):
            reference = PathFinder(40, 40, 4, 4, 0)
            reference._run_astar(start, end)
            self.assertEqual(batch.weight_cache[(start, end)], reference.weight_cache[(start, end)])
            for a, b in zip(batch.route_cache[(start, end)], batch.route_cache[(start, end)][1:]):
                self.assertIn(b, [tuple(n[:3]) for n in batch._fetch_neighbors(*a)])
        # Cloned pose keys reuse the cache and cannot mutate costs by screenshot ID.
        clone = CellState(4, 4, 0, 99)
        self.assertIn((clone, states[1]), batch.route_cache)

    def test_mission_chooses_best_directed_route_and_keeps_unreachable_target_out(self):
        planner = PathFinder(40, 40, 4, 4, 0)
        a, b, isolated = CellState(10, 10, 0, 1), CellState(20, 20, 0, 2), CellState(30, 30, 0, 3)
        start = planner.agent.get_start_state()
        def segments(states):
            for s, e, cost in [(start, a, 20), (start, b, 1), (a, b, 1), (b, a, 2)]:
                planner.weight_cache[(s, e)] = cost
                planner.route_cache[(s, e)] = [(s.x, s.y, s.direction), (e.x, e.y, e.direction)]
        with patch.object(planner.arena, 'get_view_obstacle_positions', return_value=[[a], [b], [isolated]]), \
                patch.object(planner, '_batch_calculate_segments', side_effect=segments):
            path, score = planner.solve_mission(False)
        self.assertEqual(score, 3)
        self.assertEqual([s.screenshot_id for s in path if s.screenshot_id != -1], [2, 1])

    def test_shared_view_pose_captures_both_targets_without_extra_motion(self):
        planner = PathFinder(40, 40, 10, 10, 0)
        views = [[CellState(10, 10, 0, i)] for i in (1, 2)]
        with patch.object(planner.arena, 'get_view_obstacle_positions', return_value=views):
            path, _ = planner.solve_mission(False)
        obstacles = [{'id': i, 'x': 10, 'y': 16, 'd': 4} for i in (1, 2)]
        raw, _, motors = command_generator(path, obstacles)
        self.assertEqual(motors, ['SNAP1', 'SNAP2', 'FIN'])
        build_command_poses(raw, [p.get_dict() for p in path], motors)

if __name__ == '__main__':
    unittest.main()
