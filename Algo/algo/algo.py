import heapq
import math
from collections import deque
from typing import List, Dict, Tuple
from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import *

"""
Example of outputs are given along the way to help understand the different components.
The examples are based on these inputs:
Robot position = (1, 1), facing RIGHT (2)
Obstacles = [
    {id: 1, x: 10, y: 10, d: UP (0)},
    {id: 2, x: 15, y: 18, d: LEFT (6)},
"""

# Configuration for turning radii based on steering mode and direction
LEFT_FRONT_STEER_RADIUS = [FORWARD_LEFT_LATERAL_OFFSET * TURN_RADIUS, FORWARD_LEFT_LONGITUDINAL_OFFSET * TURN_RADIUS]
RIGHT_FRONT_STEER_RADIUS = [FORWARD_RIGHT_LATERAL_OFFSET * TURN_RADIUS, FORWARD_RIGHT_LONGITUDINAL_OFFSET * TURN_RADIUS]
LEFT_REAR_STEER_RADIUS = [BACKWARD_LEFT_LATERAL_OFFSET * TURN_RADIUS, BACKWARD_LEFT_LONGITUDINAL_OFFSET * TURN_RADIUS]
RIGHT_REAR_STEER_RADIUS = [BACKWARD_RIGHT_LATERAL_OFFSET * TURN_RADIUS, BACKWARD_RIGHT_LONGITUDINAL_OFFSET * TURN_RADIUS]

# LEFT FRONT
#  <-
#    |
# RIGHT FRONT
#   ->
#  |
# LEFT REAR
#    |
#  <-
# RIGHT REAR
#  |
#   ->

# Backup values
#FRONT_STEER_RADIUS = [[3 * TURN_RADIUS, 2 * TURN_RADIUS], [3 * TURN_RADIUS, 2 * TURN_RADIUS]]
#REAR_STEER_RADIUS = [[2 * TURN_RADIUS, 3 * TURN_RADIUS], [3 * TURN_RADIUS, 2 * TURN_RADIUS]]

class PathFinder:
    """
    Core navigation engine responsible for grid management,
    A* pathfinding, and multi-objective sequence optimization.
    """
    def __init__(self, x_limit: int, y_limit: int, start_x: int, start_y: int,
                 initial_heading: Direction):
        # Setup arena and robot entities
        self.arena = Grid(x_limit, y_limit)
        self.agent = Robot(start_x, start_y, initial_heading)

        # Caching mechanisms for graph traversal
        self.route_cache = {}
        self.weight_cache = {}
        self._neighbor_cache = {}

    @staticmethod
    def _pose_key(state):
        """Geometry is independent of CellState identity and screenshot metadata."""
        return state.x, state.y, int(state.direction)

    @classmethod
    def _segment_key(cls, start, end):
        return cls._pose_key(start), cls._pose_key(end)

    def _clear_caches(self):
        self.route_cache.clear()
        self.weight_cache.clear()
        self._neighbor_cache.clear()

    def _reachable_states(self, start):
        """Find legal directed poses before enumerating camera combinations."""
        start_key = self._pose_key(start)
        reached = {start_key}
        pending = deque([start_key])
        while pending:
            for x, y, direction, _ in self._fetch_neighbors(*pending.popleft()):
                pose = (x, y, int(direction))
                if pose not in reached:
                    reached.add(pose)
                    pending.append(pose)
        return reached

    def insert_target(self, x: int, y: int, face: Direction, target_id: int):
        """Registers an obstacle into the internal grid."""
        target = Obstacle(x, y, face, target_id)
        self.arena.add_obstacle(target)
        self._clear_caches()

    def clear_targets(self):
        """Wipes all obstacles from the current arena."""
        self.arena.reset_obstacles()
        self._clear_caches()

    @staticmethod
    def get_distance(p1_x: int, p1_y: int, p2_x: int, p2_y: int, norm=1):
        """Calculates L1 or L2 distance between two points."""
        dx, dy = p1_x - p2_x, p1_y - p2_y
        return math.sqrt(dx**2 + dy**2) if norm == 2 else abs(dx) + abs(dy)

    def solve_mission(self, is_retry: bool) -> Tuple[List[CellState], float]:
        """Choose camera poses and visit order in one subset dynamic program."""
        start = self.agent.get_start_state()
        reachable = self._reachable_states(start)
        groups = [[state for state in group if self._pose_key(state) in reachable]
                  for group in self.arena.get_view_obstacle_positions(is_retry)]
        # Unreachable targets are omitted, but remain physical obstacles.
        groups = [group for group in groups if group]
        states = [start]
        indices = []
        for group in groups:
            indices.append(list(range(len(states), len(states) + len(group))))
            states.extend(group)
        self._batch_calculate_segments(states)
        order, cost = self._select_view_order(states, indices)

        path = [CellState(start.x, start.y, start.direction)]
        current = start
        for index in order:
            goal = states[index]
            segment = self.route_cache[self._segment_key(current, goal)]
            if segment is None:
                raise ValueError("Selected route contains an unreachable segment")
            # Keep a distinct photo event even when this pose needs no movement.
            points = segment[1:] if len(segment) > 1 else segment
            path.extend(CellState(x, y, direction) for x, y, direction in points)
            path[-1].set_screenshot(goal.screenshot_id)
            current = path[-1]
        return path, cost

    def _select_view_order(self, states, groups):
        """Maximize targets visited, then minimize travel and camera penalties.

        A state is (visited-obstacle mask, last camera pose). Each obstacle can
        have several poses, but contributes exactly one visit to a route.
        Unlike repeated TSP calls, all camera choices share one calculation.
        """
        size = len(states)
        edges = [[1e9] * size for _ in states]
        for start in range(size):
            for end in range(1, size):
                if start != end:
                    edges[start][end] = self.weight_cache.get(
                        self._segment_key(states[start], states[end]), 1e9)

        costs = [{} for _ in range(1 << len(groups))]
        costs[0][0] = 0
        previous = {}
        for mask, endings in enumerate(costs):
            for last, cost in endings.items():
                for group_index, goals in enumerate(groups):
                    bit = 1 << group_index
                    if mask & bit:
                        continue
                    next_mask = mask | bit
                    for goal in goals:
                        distance = edges[last][goal]
                        if distance >= 1e9:
                            continue
                        candidate = cost + distance + states[goal].penalty
                        if candidate < costs[next_mask].get(goal, float('inf')):
                            costs[next_mask][goal] = candidate
                            previous[(next_mask, goal)] = (mask, last)

        best = (0, 0)
        best_count, best_cost = 0, 0
        for mask, endings in enumerate(costs):
            count = mask.bit_count()
            for last, cost in endings.items():
                if count > best_count or (count == best_count and cost < best_cost):
                    best, best_count, best_cost = (mask, last), count, cost

        order = []
        while best != (0, 0):
            order.append(best[1])
            best = previous[best]
        return order[::-1], best_cost

    def _get_proximity_cost(self, x: int, y: int) -> int:
        """Adds a penalty if the robot is too close to obstacle corners."""
        for obs in self.arena.obstacles:
            if abs(obs.x - x) <= 2 and abs(obs.y - y) <= 2:
                if abs(obs.x - x) + abs(obs.y - y) >= 3: # Specialized proximity logic
                    return SAFE_COST
        return 0

    def _fetch_neighbors(self, cur_x: int, cur_y: int, heading: Direction):
        key = (cur_x, cur_y, int(heading))
        if key not in self._neighbor_cache:
            self._neighbor_cache[key] = tuple(self._compute_neighbors(cur_x, cur_y, heading))
        return self._neighbor_cache[key]

    def _compute_neighbors(self, cur_x: int, cur_y: int, heading: Direction):
        """Generates valid adjacent states including straight moves and turns."""
        adjacents = []

        # 1. Straight movements (Forward/Backward)
        for dx, dy, move_dir in MOVE_DIRECTION:
            if move_dir == heading:
                # Forward move
                if self.arena.reachable(cur_x + dx, cur_y + dy):
                    adjacents.append((cur_x + dx, cur_y + dy, move_dir, self._get_proximity_cost(cur_x + dx, cur_y + dy)))
                # Reverse move
                if self.arena.reachable(cur_x - dx, cur_y - dy):
                    adjacents.append((cur_x - dx, cur_y - dy, move_dir, self._get_proximity_cost(cur_x - dx, cur_y - dy)))

        # 2. Turning maneuvers
        # Logic matches existing turn radius calculations but restructured for clarity

        # Mapping of (Current Direction, Target Direction) to coordinate offsets
        # This handles all 8 specific turning cases (North-East, East-South, etc.)
        turn_patterns = self._map_turn_logic(cur_x, cur_y, heading, LEFT_FRONT_STEER_RADIUS, RIGHT_FRONT_STEER_RADIUS, LEFT_REAR_STEER_RADIUS, RIGHT_REAR_STEER_RADIUS)
        for nx, ny, ndir in turn_patterns:
            if self.arena.reachable(nx, ny, turn=True) and self.arena.reachable(cur_x, cur_y, preTurn=True): #and self.arena.reachable((nx + cur_x)//2, (ny+cur_y)//2, during_turn=True):
                cost = self._get_proximity_cost(nx, ny) + 10 # Base turn penalty
                adjacents.append((nx, ny, ndir, cost))
        return adjacents

    def _batch_calculate_segments(self, states: List[CellState]):
        """Runs A* between all pairs of required states."""
        for i in range(len(states)):
            for j in range(0 if DIRECTIONAL_MOVEMENT else i + 1, len(states)):
                if i == j:
                    continue
                self._run_astar(states[i], states[j])

    def _run_astar(self, start: CellState, goal: CellState):
        """Standard A* search using a priority queue."""
        segment_key = self._segment_key(start, goal)
        if segment_key in self.route_cache: return

        start_key = self._pose_key(start)
        g_scores = {start_key: 0}
        pq = [(self.get_distance(start.x, start.y, goal.x, goal.y), start.x, start.y, start.direction)]

        route = {}
        visited = set()

        while pq:
            _, x, y, head = heapq.heappop(pq)
            if (x, y, head) in visited: continue

            if goal.is_eq(x, y, head):
                self._archive_path(start, goal, route, g_scores[(x, y, head)])
                return

            visited.add((x, y, head))
            current_g = g_scores[(x, y, head)]

            for nx, ny, nhead, safe_bonus in self._fetch_neighbors(x, y, head):
                if (nx, ny, nhead) in visited: continue

                # Cost = Rotation + Step + Proximity Penalty
                step_cost = Direction.rotation_cost(nhead, head) * TURN_FACTOR + 1 + safe_bonus
                if self._checkReverseTurn(x, y, head, nx, ny, nhead):
                    step_cost += REVERSE_TURN_PENALTY
                new_g = current_g + step_cost

                # f = g + h
                f_score = new_g + self.get_distance(nx, ny, goal.x, goal.y)

                if new_g < g_scores.get((nx, ny, nhead), 1e9):
                    g_scores[(nx, ny, nhead)] = new_g
                    route[(nx, ny, nhead)] = (x, y, head)
                    heapq.heappush(pq, (f_score, nx, ny, nhead))

        # Remember failed searches as well as successful ones.
        self.route_cache[segment_key] = None
        self.weight_cache[segment_key] = 1e9

    def _archive_path(self, s, e, route, total_cost):
        """Stores the result of a successful A* search."""
        segment_key = self._segment_key(s, e)
        self.weight_cache[segment_key] = total_cost
        if not DIRECTIONAL_MOVEMENT:
            self.weight_cache[self._segment_key(e, s)] = total_cost

        coords = []
        curr = (e.x, e.y, e.direction)
        while curr in route:
            coords.append(curr)
            curr = route[curr]
        coords.append(curr)

        self.route_cache[segment_key] = coords[::-1]
        if not DIRECTIONAL_MOVEMENT:
            self.route_cache[self._segment_key(e, s)] = coords

    def _map_turn_logic(self, x, y, h, left_front, right_front, left_rear, right_rear):
        # Helper to condense the 8 turning conditions from original algo.py
        res = []

        # Right turns
        front_right_large_displacement = right_front[0]
        front_right_small_displacement = right_front[1]
        rear_left_large_displacement = left_rear[0]
        rear_left_small_displacement = left_rear[1]
        # Left turns
        front_left_large_displacement = left_front[0]
        front_left_small_displacement = left_front[1]
        rear_right_large_displacement = right_rear[0]
        rear_right_small_displacement = right_rear[1]

        if h == Direction.NORTH:
            # Right turns to EAST
            # Left turns to WEST
            res.extend([(x + front_right_large_displacement, y + front_right_small_displacement, Direction.EAST), \
                        (x - rear_left_large_displacement, y - rear_left_small_displacement, Direction.EAST), \
                        (x - front_left_large_displacement, y + front_left_small_displacement, Direction.WEST), \
                        (x + rear_right_large_displacement, y - rear_right_small_displacement, Direction.WEST)])
        elif h == Direction.EAST:
            # Right turns to SOUTH
            # Left turns to NORTH
            res.extend([(x + front_right_small_displacement, y - front_right_large_displacement, Direction.SOUTH), \
                        (x - rear_left_small_displacement, y + rear_left_large_displacement, Direction.SOUTH), \
                        (x + front_left_small_displacement, y + front_left_large_displacement, Direction.NORTH), \
                        (x - rear_right_small_displacement, y - rear_right_large_displacement, Direction.NORTH)])
        elif h == Direction.SOUTH:
            # Right turns to WEST
            # Left turns to EAST
            res.extend([
                (x - front_right_large_displacement, y - front_right_small_displacement, Direction.WEST), \
                (x + rear_left_large_displacement, y + rear_left_small_displacement, Direction.WEST), \
                (x + front_left_large_displacement, y - front_left_small_displacement, Direction.EAST), \
                (x - rear_right_large_displacement, y + rear_right_small_displacement, Direction.EAST)])
        elif h == Direction.WEST:
            # Right turns to NORTH
            # Left turns to SOUTH
            res.extend([(x - front_right_small_displacement, y + front_right_large_displacement, Direction.NORTH), \
                        (x + rear_left_small_displacement, y - rear_left_large_displacement, Direction.NORTH), \
                        (x - front_left_small_displacement, y - front_left_large_displacement, Direction.SOUTH), \
                        (x + rear_right_small_displacement, y + rear_right_large_displacement, Direction.SOUTH)])

        return res

    def _checkReverseTurn(self, x, y, d, newX, newY, newD):
        dx = newX - x
        dy = newY - y

        # Forward direction vectors
        forward = {
            Direction.NORTH: (0, 1),
            Direction.SOUTH: (0, -1),
            Direction.EAST: (1, 0),
            Direction.WEST: (-1, 0)
        }

        fx, fy = forward[d]

        # Dot product to determine if movement is backward
        dot = dx * fx + dy * fy

        moved_backward = dot < 0
        turned = d != newD

        return moved_backward and turned
