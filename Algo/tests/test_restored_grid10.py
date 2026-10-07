"""Restored movement configuration and the current 40 cm turn-clearance trial."""
import contextlib
import io
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import consts
from helper import command_converter, command_generator
from entities.Entity import CellState, Grid, Obstacle
from main import app
from algo.algo import PathFinder

VECTORS = ((0, 1), (1, 0), (0, -1), (-1, 0))
GEOMETRY = {'FL': (2, 4), 'FR': (3, 4), 'BL': (3, 2), 'BR': (4, 3)}
PADDING = {'FL': (5, 9), 'FR': (10, 2), 'BL': (1, 7), 'BR': (4, 9)}

class RestoredGridTests(unittest.TestCase):
    def test_measured_speed50_turn_commands_in_all_headings(self):
        self.assertEqual(consts.RECOMMENDED_MOTOR_SPEED, 50)
        for heading in range(0, 8, 2):
            fx, fy = VECTORS[heading // 2]
            for turn, (longitudinal, lateral) in GEOMETRY.items():
                side = -1 if turn.endswith('L') else 1
                travel = -1 if turn.startswith('B') else 1
                end = CellState(10 + travel * longitudinal * fx + side * lateral * fy,
                                10 + travel * longitudinal * fy - side * lateral * fx,
                                (heading + side * travel * 2) % 8)
                _, _, motors = command_generator([CellState(10, 10, heading), end], [])
                before, after = PADDING[turn]
                direction = 'forward' if travel == 1 else 'reverse'
                steering = 'left' if side == -1 else 'right'
                self.assertEqual(motors, [f'center,0,{direction},{before}',
                    f'{steering},90,{direction},0', f'center,0,{direction},{after}', 'FIN'])
        self.assertEqual(command_converter('FW03'), 'center,0,forward,30')
        self.assertEqual(command_converter('BW03'), 'center,0,reverse,30')

    def test_turn_clearance_restored_and_straight_edges_preserved(self):
        grid = Grid(20, 20)
        grid.add_obstacle(Obstacle(10, 10, 0, 1))
        self.assertFalse(grid.reachable(10, 13, turn=True))
        self.assertFalse(grid.reachable(10, 13, preTurn=True))
        self.assertTrue(grid.reachable(10, 14, turn=True))
        self.assertTrue(grid.reachable(10, 14, preTurn=True))
        self.assertFalse(grid.reachable(10, 12, turn=True))
        self.assertFalse(grid.reachable(12, 11))
        self.assertTrue(grid.reachable(13, 10))
        for value in (1, 18):
            self.assertTrue(grid.is_valid_coord(value, 1))
        for value in (0, 19):
            self.assertFalse(grid.is_valid_coord(value, 1))

    def test_old_eight_target_map_returns_valid_subset_at_40cm(self):
        payload = ('STATUS,OBSTACLE_LIST,false,1;1;NORTH,false,'
                   '1;0;19;SOUTH | 2;10;19;SOUTH | 3;5;14;SOUTH | '
                   '4;9;4;WEST | 5;19;4;WEST | 6;10;9;WEST | '
                   '7;15;9;NORTH | 8;15;15;WEST,true,true')
        with contextlib.redirect_stdout(io.StringIO()):
            response = app.test_client().post('/path', data=payload, content_type='text/plain')
        self.assertEqual(response.status_code, 200)
        data = response.json['data']
        ids = {int(c[4:].split('_')[0]) for c in data['stm_commands'] if c.startswith('SNAP')}
        # Obstacle 7's available view poses cannot be reached with 40 cm turns.
        self.assertEqual(ids, set(range(1, 9)) - {7})
        self.assertEqual(data['grid_cell_cm'], 10)
        self.assertEqual(len(data['command_poses']), len(data['stm_commands']))
        planner = PathFinder(20, 20, 1, 1, 0)
        for x, y, d, i in [(0,19,4,1),(10,19,4,2),(5,14,4,3),(9,4,6,4),
                            (19,4,6,5),(10,9,6,6),(15,9,0,7),(15,15,6,8)]:
            planner.insert_target(x, y, d, i)
        pose, endpoints = data['path'][0], iter(data['path'][1:])
        for command in data['commands']:
            if command.startswith(('SNAP', 'FIN')):
                continue
            end = next(endpoints)
            if command.startswith(('FW', 'BW')):
                fx, fy = VECTORS[pose['d'] // 2]
                sign = 1 if command.startswith('FW') else -1
                for step in range(1, int(command[2:]) + 1):
                    self.assertTrue(planner.arena.reachable(pose['x'] + sign * fx * step,
                                                           pose['y'] + sign * fy * step))
            else:
                self.assertIn((end['x'], end['y'], end['d']),
                    [tuple(n[:3]) for n in planner._fetch_neighbors(pose['x'], pose['y'], pose['d'])])
            pose = end

if __name__ == '__main__':
    unittest.main()
