"""Provisional measured25 trial; endpoint math is not a physical validation."""

import contextlib
import io
import unittest
from unittest.mock import patch

from algo.algo import (
    LEFT_FRONT_STEER_RADIUS,
    LEFT_REAR_STEER_RADIUS,
    RIGHT_FRONT_STEER_RADIUS,
    RIGHT_REAR_STEER_RADIUS,
    PathFinder,
    solve_tsp_dynamic_programming,
)
from consts import Direction, MOVEMENT_PROFILE
from entities.Entity import CellState
from helper import command_converter, command_generator


HEADINGS = (Direction.NORTH, Direction.EAST, Direction.SOUTH, Direction.WEST)
FORWARD_VECTORS = ((0, 1), (1, 0), (0, -1), (-1, 0))
# Measured bare-turn means (longitudinal, lateral), in centimetres at speed 25.
MEASURED = {
    "FL": (25.5, 22),
    "FR": (32.5, 26),
    "BL": (45, (34 + 32 + 32.5) / 3),
    "BR": ((44 + 44 + 45) / 3, (26.5 + 27.5 + 28) / 3),
}
# Whole-grid endpoint and additional straight centimetres (before, after).
PROFILE = {
    "FL": ((3, 3), (5, 8)),
    "FR": ((4, 3), (8, 4)),
    "BL": ((5, 4), (5, 7)),
    "BR": ((5, 3), (6, 3)),
}


def turn_endpoint(x, y, heading, turn, longitudinal, lateral):
    """Independent robot-relative displacement, not the planner's mapping."""
    fx, fy = FORWARD_VECTORS[int(heading) // 2]
    reverse = turn.startswith("B")
    left = turn.endswith("L")
    side = -1 if left else 1
    travel = -1 if reverse else 1
    next_heading = Direction((int(heading) + side * travel * 2) % 8)
    return (
        x + travel * longitudinal * fx + side * lateral * fy,
        y + travel * longitudinal * fy - side * lateral * fx,
        next_heading,
    )


def simulate_measured_commands(commands, start):
    """Replay commands using measured turn means and exact straight distances."""
    x, y, heading = start
    for command in commands:
        if command.startswith(("SNAP", "FIN")):
            continue
        steering, angle, travel, distance = command.split(",")
        if steering == "center":
            fx, fy = FORWARD_VECTORS[int(heading) // 2]
            signed_distance = int(distance) * (-1 if travel == "reverse" else 1)
            x += fx * signed_distance
            y += fy * signed_distance
        else:
            assert angle == "90"
            turn = ("B" if travel == "reverse" else "F") + (
                "L" if steering == "left" else "R"
            )
            x, y, heading = turn_endpoint(x, y, heading, turn, *MEASURED[turn])
    return x, y, heading


@unittest.skipUnless(MOVEMENT_PROFILE == "measured25", "measured25-only trial")
class MovementCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.stdout = contextlib.redirect_stdout(io.StringIO())
        self.stdout.__enter__()
        self.addCleanup(self.stdout.__exit__, None, None, None)

    def test_straight_grid_steps_are_ten_centimetres(self):
        for direction, travel, steps in (
            ("FW", "forward", (1, 2, 4, 8, 12)),
            ("BW", "reverse", (1, 2, 4, 8)),
        ):
            for step in steps:
                with self.subTest(direction=direction, steps=step):
                    self.assertEqual(
                        command_converter(f"{direction}{step:02d}"),
                        f"center,0,{travel},{10 * step}",
                    )

    def test_planner_endpoints_and_compensated_commands_in_all_headings(self):
        planner = PathFinder(20, 20, 10, 10, Direction.NORTH)
        for heading in HEADINGS:
            endpoints = planner._map_turn_logic(
                10, 10, heading, LEFT_FRONT_STEER_RADIUS, RIGHT_FRONT_STEER_RADIUS,
                LEFT_REAR_STEER_RADIUS, RIGHT_REAR_STEER_RADIUS,
            )
            for turn, ((longitudinal, lateral), (before, after)) in PROFILE.items():
                with self.subTest(heading=heading, turn=turn):
                    endpoint = turn_endpoint(10, 10, heading, turn, longitudinal, lateral)
                    self.assertIn(endpoint, endpoints)
                    raw, _, adjusted = command_generator(
                        [CellState(10, 10, heading), CellState(*endpoint)], []
                    )
                    travel = "reverse" if turn.startswith("B") else "forward"
                    steering = "left" if turn.endswith("L") else "right"
                    self.assertEqual(raw, [f"{turn}01", "FIN"])
                    self.assertEqual(adjusted, [
                        f"center,0,{travel},{before}",
                        f"{steering},90,{travel},0",
                        f"center,0,{travel},{after}",
                        "FIN",
                    ])
                    actual = simulate_measured_commands(adjusted, (100, 100, heading))
                    self.assertEqual(actual[2], endpoint[2])
                    self.assertAlmostEqual(actual[0], endpoint[0] * 10, delta=0.51)
                    self.assertAlmostEqual(actual[1], endpoint[1] * 10, delta=0.51)

    def test_snap_and_finish_follow_turn_compensation(self):
        end = CellState(13, 14, Direction.EAST, screenshot_id=1)
        raw, _, adjusted = command_generator(
            [CellState(10, 10, Direction.NORTH), end],
            [{"id": 1, "x": 17, "y": 14, "d": int(Direction.WEST)}],
        )
        self.assertTrue(raw[-2].startswith("SNAP1"))
        self.assertEqual(adjusted[-3], "center,0,forward,4")
        self.assertEqual(adjusted[-2:], raw[-2:])

    def test_cancelling_opposite_straights_preserves_endpoint(self):
        # One grid step backwards followed by a forward-right turn.
        states = [CellState(10, 11, Direction.NORTH),
                  CellState(10, 10, Direction.NORTH),
                  CellState(13, 14, Direction.EAST)]
        _, _, adjusted = command_generator(states, [])
        self.assertEqual(adjusted[0], "center,0,reverse,2")
        actual = simulate_measured_commands(adjusted, (100, 110, Direction.NORTH))
        self.assertAlmostEqual(actual[0], 130, delta=0.51)
        self.assertAlmostEqual(actual[1], 140, delta=0.51)

    def test_archiving_does_not_invent_a_reverse_route(self):
        planner = PathFinder(20, 20, 10, 10, Direction.NORTH)
        start = CellState(10, 10, Direction.NORTH)
        end = CellState(13, 14, Direction.EAST)
        planner._run_astar(start, end)
        self.assertIn((start, end), planner.route_cache)
        self.assertNotIn((end, start), planner.route_cache)
        self.assertNotIn((end, start), planner.weight_cache)
        planner._run_astar(end, start)
        self.assert_valid_edges(planner, planner.route_cache[(end, start)])

    def test_batch_plans_both_directions_independently(self):
        planner = PathFinder(20, 20, 10, 10, Direction.NORTH)
        states = [CellState(10, 10, Direction.NORTH),
                  CellState(13, 14, Direction.EAST),
                  CellState(5, 6, Direction.SOUTH)]
        with patch.object(planner, "_run_astar") as astar:
            planner._batch_calculate_segments(states)
        self.assertEqual(astar.call_count, 6)
        for start in states:
            for end in states:
                if start is not end:
                    astar.assert_any_call(start, end)

    def assert_valid_edges(self, planner, coordinates):
        for start, end in zip(coordinates, coordinates[1:]):
            neighbors = [tuple(n[:3]) for n in planner._fetch_neighbors(*start)]
            self.assertIn(tuple(end), neighbors, f"Invalid movement: {start} -> {end}")

    def test_tsp_matrix_preserves_directional_costs(self):
        planner = PathFinder(20, 20, 1, 1, Direction.NORTH)
        targets = [[CellState(10, 6, Direction.NORTH, screenshot_id=1)],
                   [CellState(11, 15, Direction.EAST, screenshot_id=2)]]

        def supply_directed_segments(states):
            # Synthetic directed costs isolate matrix construction from A*.
            for i, start in enumerate(states):
                for j, end in enumerate(states):
                    if i != j:
                        planner.weight_cache[(start, end)] = 10 * (i + 1) + j
                        planner.route_cache[(start, end)] = [
                            (start.x, start.y, start.direction),
                            (end.x, end.y, end.direction),
                        ]

        with patch.object(planner.arena, "get_view_obstacle_positions", return_value=targets), \
                patch.object(planner, "_batch_calculate_segments", side_effect=supply_directed_segments), \
                patch("algo.algo.solve_tsp_dynamic_programming", wraps=solve_tsp_dynamic_programming) as tsp:
            planner.solve_mission(False)
        self.assertEqual(tsp.call_args_list[0].args[0].tolist(),
                         [[0, 11, 12], [0, 0, 22], [0, 31, 0]])

    def test_two_obstacle_mission_uses_valid_directed_movements(self):
        planner = PathFinder(20, 20, 1, 1, Direction.NORTH)
        planner.insert_target(10, 10, Direction.SOUTH, 1)
        planner.insert_target(15, 15, Direction.WEST, 2)
        path, _ = planner.solve_mission(False)
        self.assertEqual({state.screenshot_id for state in path if state.screenshot_id != -1}, {1, 2})
        self.assert_valid_edges(planner, [(s.x, s.y, s.direction) for s in path])


if __name__ == "__main__":
    unittest.main()
