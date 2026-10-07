import heapq
import math
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
        self.neighbor_cache = {}

    def insert_target(self, x: int, y: int, face: Direction, target_id: int):
        """Registers an obstacle into the internal grid."""
        target = Obstacle(x, y, face, target_id)
        self.arena.add_obstacle(target)
        self.route_cache.clear()
        self.weight_cache.clear()
        self.neighbor_cache.clear()

    def clear_targets(self):
        """Wipes all obstacles from the current arena."""
        self.arena.reset_obstacles()
        self.route_cache.clear()
        self.weight_cache.clear()
        self.neighbor_cache.clear()

    @staticmethod
    def get_distance(p1_x: int, p1_y: int, p2_x: int, p2_y: int, norm=1):
        """Calculates L1 or L2 distance between two points."""
        dx, dy = p1_x - p2_x, p1_y - p2_y
        return math.sqrt(dx**2 + dy**2) if norm == 2 else abs(dx) + abs(dy)

    def solve_mission(self, is_retry: bool) -> Tuple[List[CellState], float]:
        """Choose one camera pose per target and order them in one dynamic program.

        Prioritize the number of photographed targets, then minimize directed
        travel cost plus camera-pose penalties. Unreachable edges are excluded.
        This avoids thousands of repeated TSP solves during preparation.
        """
        groups = self.arena.get_view_obstacle_positions(is_retry)
        start = self.agent.get_start_state()
        poses = [pose for group in groups for pose in group]
        self._batch_calculate_segments([start, *poses])
        by_group, offset = [], 0
        for group in groups:
            by_group.append(list(range(offset, offset + len(group))))
            offset += len(group)
        costs = {(0, -1): 0}
        parents = {}
        for mask in range(1 << len(groups)):
            for last in [-1, *range(len(poses))]:
                key = (mask, last)
                if key not in costs:
                    continue
                source = start if last == -1 else poses[last]
                for group_index, candidates in enumerate(by_group):
                    if mask & (1 << group_index):
                        continue
                    for candidate in candidates:
                        target = poses[candidate]
                        if source == target:
                            edge = 0
                        elif self.route_cache.get((source, target)) is None:
                            continue
                        else:
                            edge = self.weight_cache[(source, target)]
                        next_key = (mask | (1 << group_index), candidate)
                        cost = costs[key] + edge + target.penalty
                        if cost < costs.get(next_key, math.inf):
                            costs[next_key] = cost
                            parents[next_key] = key
        end = min(costs, key=lambda key: (-key[0].bit_count(), costs[key]))
        if end[0] == 0 and groups:
            return [], math.inf
        selected = []
        key = end
        while key in parents:
            selected.append(poses[key[1]])
            key = parents[key]
        path, source = [CellState(start.x, start.y, start.direction)], start
        for target in reversed(selected):
            segment = ([(source.x, source.y, source.direction)] if source == target
                       else self.route_cache[(source, target)])
            for x, y, direction in segment[1:]:
                path.append(CellState(x, y, direction))
            # A second target can share the same camera pose; keep both SNAPs.
            if len(segment) == 1:
                path.append(CellState(target.x, target.y, target.direction))
            path[-1].set_screenshot(target.screenshot_id)
            source = target
        return path, costs[end]

    def _get_proximity_cost(self, x: int, y: int) -> int:
        """Adds a penalty if the robot is too close to obstacle corners."""
        for obs in self.arena.obstacles:
            if abs(obs.x - x) <= 2 * MAP_TO_PLANNER and abs(obs.y - y) <= 2 * MAP_TO_PLANNER:
                if abs(obs.x - x) + abs(obs.y - y) >= 3 * MAP_TO_PLANNER: # Specialized proximity logic
                    return SAFE_COST
        return 0

    def _fetch_neighbors(self, cur_x: int, cur_y: int, heading: Direction):
        """Generates valid adjacent states including straight moves and turns."""
        key = (cur_x, cur_y, heading)
        if key in self.neighbor_cache:
            return self.neighbor_cache[key]
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
        self.neighbor_cache[key] = adjacents
        return adjacents

    def _batch_calculate_segments(self, states: List[CellState]):
        """One directed shortest-path search per source, shared across goals.

        Repeating A* for every pair was too slow on the finer 5 cm grid.
        Dijkstra preserves the exact same edge costs and does not invent
        a reverse path for the asymmetric measured turns.
        """
        unique_states = list(dict.fromkeys(states))
        for start in unique_states:
            remaining = {(goal.x, goal.y, goal.direction): goal for goal in unique_states
                         if goal != start and (start, goal) not in self.route_cache}
            if not remaining:
                continue
            start_key = (start.x, start.y, start.direction)
            costs, parents, visited = {start_key: 0}, {}, set()
            pending = [(0, *start_key)]
            while pending and remaining:
                cost, x, y, heading = heapq.heappop(pending)
                key = (x, y, heading)
                if key in visited:
                    continue
                visited.add(key)
                goal = remaining.pop(key, None)
                if goal is not None:
                    self._archive_path(start, goal, parents, cost)
                for nx, ny, ndir, penalty in self._fetch_neighbors(x, y, heading):
                    next_key = (nx, ny, ndir)
                    if next_key in visited:
                        continue
                    step_cost = Direction.rotation_cost(ndir, heading) * TURN_FACTOR + 1 + penalty
                    if self._checkReverseTurn(x, y, heading, nx, ny, ndir):
                        step_cost += REVERSE_TURN_PENALTY
                    next_cost = cost + step_cost
                    if next_cost < costs.get(next_key, math.inf):
                        costs[next_key] = next_cost
                        parents[next_key] = key
                        heapq.heappush(pending, (next_cost, *next_key))
            for goal in remaining.values():
                self.route_cache[(start, goal)] = None
                self.weight_cache[(start, goal)] = math.inf

    def _run_astar(self, start: CellState, goal: CellState):
        """Standard A* search using a priority queue."""
        if (start, goal) in self.route_cache: return

        start_key = (start.x, start.y, start.direction)
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

    def _archive_path(self, s, e, route, total_cost):
        """Stores the result of a successful A* search."""
        self.weight_cache[(s, e)] = total_cost
        if not DIRECTIONAL_MOVEMENT:
            self.weight_cache[(e, s)] = total_cost
        
        coords = []
        curr = (e.x, e.y, e.direction)
        while curr in route:
            coords.append(curr)
            curr = route[curr]
        coords.append(curr)
        
        self.route_cache[(s, e)] = coords[::-1]
        if not DIRECTIONAL_MOVEMENT:
            self.route_cache[(e, s)] = coords

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
