"""Small regressions for directed coverage selection and stale segment caches."""
import itertools
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from algo.algo import PathFinder
from entities.Entity import CellState


class PlannerOptimizationTests(unittest.TestCase):
    def test_full_and_partial_order_match_exhaustive_reference(self):
        planner = PathFinder(20, 20, 1, 1, 0)
        states = [CellState(i + 1, 1, 0, penalty=i % 3) for i in range(7)]
        groups = [[1, 2], [3, 4], [5, 6]]
        rng = random.Random(24)
        for blocked in (None, 1):
            with self.subTest(blocked=blocked):
                planner.weight_cache.clear()
                for start in range(len(states)):
                    for end in range(1, len(states)):
                        planner.weight_cache[planner._segment_key(states[start], states[end])] = (
                            1e9 if blocked is not None and end in groups[blocked]
                            else rng.randrange(1, 30))
                order, score = planner._select_view_order(states, groups)
                best = (0, 0)
                for count in range(1, len(groups) + 1):
                    for subset in itertools.combinations(groups, count):
                        for targets in itertools.product(*subset):
                            for permutation in itertools.permutations(targets):
                                previous, total = 0, 0
                                for goal in permutation:
                                    edge = planner.weight_cache[
                                        planner._segment_key(states[previous], states[goal])]
                                    if edge >= 1e9:
                                        break
                                    total += edge + states[goal].penalty
                                    previous = goal
                                else:
                                    best = min(best, (-count, total))
                self.assertEqual((-len(order), score), best)

    def test_failed_clone_is_cached_and_obstacle_edits_invalidate_results(self):
        planner = PathFinder(20, 20, 1, 1, 0)
        start, goal = CellState(1, 1, 0), CellState(1, 8, 0)
        key = planner._segment_key(start, goal)
        planner._run_astar(start, goal)
        self.assertIsNotNone(planner.route_cache[key])
        planner.insert_target(1, 8, 0, 17)
        planner._run_astar(start, goal)
        self.assertIsNone(planner.route_cache[key])
        with patch.object(planner, "_fetch_neighbors", side_effect=AssertionError("repeated failed search")):
            planner._run_astar(CellState(1, 1, 0), CellState(1, 8, 0))
        planner.clear_targets()
        planner._run_astar(start, goal)
        self.assertIsNotNone(planner.route_cache[key])
