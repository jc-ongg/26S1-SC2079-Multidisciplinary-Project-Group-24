import heapq
import math
import numpy as np
from typing import List, Dict, Tuple
from entities.Robot import Robot
from entities.Entity import Obstacle, CellState, Grid
from consts import *
from python_tsp.exact import solve_tsp_dynamic_programming

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

    def insert_target(self, x: int, y: int, face: Direction, target_id: int):
        """Registers an obstacle into the internal grid."""
        target = Obstacle(x, y, face, target_id)
        self.arena.add_obstacle(target)

    def clear_targets(self):
        """Wipes all obstacles from the current arena."""
        self.arena.reset_obstacles()

    @staticmethod
    def get_distance(p1_x: int, p1_y: int, p2_x: int, p2_y: int, norm=1):
        """Calculates L1 or L2 distance between two points."""
        dx, dy = p1_x - p2_x, p1_y - p2_y
        return math.sqrt(dx**2 + dy**2) if norm == 2 else abs(dx) + abs(dy)

    def solve_mission(self, is_retry: bool) -> Tuple[List[CellState], float]:
        """
        Determines the best sequence of obstacles to visit using TSP,
        building the final path from cached A* segments.
        """
        best_overall_score = 100000000
        final_sequence = []
        # Find all valid vantage points for the given obstacles
        vantage_groups = self.arena.get_view_obstacle_positions(is_retry)
        """" 
        print(vantage_groups) 
            -> each nested list represents an obstacle's vantage points
            -> screenshot id corresponds to obstacle id
            [
                [x: 10, y: 13, d: 4, screenshot: 7, 
                 x: 10, y: 14, d: 4, screenshot: 7, 
                 x: 11, y: 14, d: 4, screenshot: 7, 
                 x: 9, y: 14, d: 4, screenshot: 7], 
                [x: 12, y: 18, d: 2, screenshot: 3, 
                 x: 11, y: 18, d: 2, screenshot: 3, 
                 x: 11, y: 17, d: 2, screenshot: 3]
            ]
        """

        # Iterate through combinations of obstacles to visit (bitmask-style)
        for selection_mask in self._generate_bitmasks(len(vantage_groups)):
            active_items = [self.agent.get_start_state()]
            active_vantage_groups = []
            
            for i, bit in enumerate(selection_mask):
                if bit == '1':
                    active_items.extend(vantage_groups[i])
                    active_vantage_groups.append(vantage_groups[i])
            """
            active_items: 
                -> flat list that contains the robot's starting position and every possible viewing position for every selected obstacle. 
                -> serves as a master registry of all possible coordinates (nodes) for this specific iteration
                -> used for A* searches between all possible pairs of states
                -> eg. [Start State, Obstacle1_PosA, Obstacle1_PosB, Obstacle2_PosA, ...]

            active_vantage_groups:
                -> a nested list where each sub-list contains only the vantage points for a single specific obstacle.
                -> since the robot only needs to visit one vantage point per obstacle to take a picture, the algorithm needs to know which positions belong to which obstacle.
                -> Structure: [[Obstacle1_PosA, Obstacle1_PosB], [Obstacle2_PosA, Obstacle2_PosB, Obstacle2_PosC]].
            """

            # Pre-calculate segment costs using A*
            self._batch_calculate_segments(active_items)
            
            possible_permutations = []
            self._recursive_combinations(active_vantage_groups, 0, [], possible_permutations, [ITERATIONS])
            """
            possible_permutations:
                -> generates all possible ways to pick one view position per obstacle.
                -> Example:
                    If 2 obstacles each have 3 view positions, it generates indices like:
                    [0, 0], [0, 1], [0, 2], [1, 0], [1, 1], [1, 2], ...
                    representing "pick position 0 from obstacle 0 and position 0 from obstacle 1," etc.
                    These combinations are used to try different ways the robot can view each obstacle, then picks the cheapest route via TSP.
            """

            for perm in possible_permutations:
                # Build the cost matrix for the TSP solver
                indices = [0]
                offset = 1
                static_penalty = 0
                
                for i, group in enumerate(active_vantage_groups):
                    chosen_idx = offset + perm[i]
                    indices.append(chosen_idx)
                    static_penalty += group[perm[i]].penalty
                    offset += len(group)
                
                """
                    indices:
                        -> stores the positions of the CellState objects in the flattened active_items list that the robot will visit in this specific iteration
                        -> starts with 0 because active_items[0] is the robot's starting position
                        -> eg. [0, 2, 5] means starting position, then 2nd and 5th items in active_items
                        -> by this point, each obstacle will only be left with 1 vantage point

                    group:
                        -> each group contains all possible valid positions to view a specific obstacle

                    chosen_idx:
                        -> perm[i] indicates which specific vantage point was chosen for the current obstacle

                    offset: 
                        -> since active_items is a flat list, the offset keeps track of where the next obstacle's group of points begins in that list
                    
                    static_penalty:
                        -> each vantage point has a correspondng penalty defined in its CellState, it represents how "hard" it is to take a picture from that position
                """
                
                size = len(indices)
                adj_matrix = np.zeros((size, size))
                if DIRECTIONAL_MOVEMENT:
                    for s in range(size):
                        for e in range(size):
                            if s != e:
                                adj_matrix[s][e] = self.weight_cache.get(
                                    (active_items[indices[s]], active_items[indices[e]]), 1e9)
                else:
                    for s in range(size - 1):
                        for e in range(s + 1, size):
                            start_node = active_items[indices[s]]
                            end_node = active_items[indices[e]]
                            dist = self.weight_cache.get((start_node, end_node), 1e9)
                            adj_matrix[s][e] = adj_matrix[e][s] = dist
                
                # Treat as an open path by zeroing return-to-start cost
                adj_matrix[:, 0] = 0
                # order represents the order of visiting obstacles
                order, travel_dist = solve_tsp_dynamic_programming(adj_matrix)
                # ... (inside solve_mission after travel_dist calculation)
                if travel_dist + static_penalty < best_overall_score:
                    # Start the path with the robot's initial state
                    current_path = [active_items[0]] 
                    
                    # Must track the ACTUAL current state of the robot to ensure the next segment starts exactly where the last one ended.
                    actual_current_state = active_items[0]

                    for k in range(len(order) - 1):
                        # The next target state chosen by the TSP solver
                        target_node = active_items[indices[order[k+1]]]
                        
                        # Retrieve the segment starting from our ACTUAL current position 
                        # to the next target. If it's missing, we run A* on the fly.
                        if (actual_current_state, target_node) not in self.route_cache:
                            self._run_astar(actual_current_state, target_node)
                            
                        segment = self.route_cache.get((actual_current_state, target_node))
                        if segment is None:
                            # A symmetric candidate cost can describe an edge
                            # that A* cannot drive in this direction. Reject
                            # this candidate before accepting its score/path.
                            break
                        
                        # Add steps to the path (skip the first to avoid duplicates)
                        for step in segment[1:]:
                            new_state = CellState(step[0], step[1], step[2])
                            current_path.append(new_state)
                        
                        # Mark the last state of this segment for a screenshot
                        current_path[-1].set_screenshot(target_node.screenshot_id)
                        
                        # UPDATE the actual current state to be the end of this segment
                        actual_current_state = current_path[-1]
                    
                    else:
                        best_overall_score = travel_dist + static_penalty
                        final_sequence = current_path

                    """
                    order:
                        -> the optimized order of visiting the selected obstacles
                        -> eg. [0, 2, 1, 3] means start -> obstacle at index 2 of indices -> obstacle at index 1 of indices -> ...

                    current_path:
                        -> the final stitched path the robot will take, composed of multiple A* segments
                    
                    segments:
                        -> loops through the optimized order. for every pair of origin and target in that order, it retrieves the pre-calculated A* path from the route_cache
                    Stitching Coordinates: It appends every coordinate step from the cached segment (skipping the first step to avoid duplicates) into the current_path as a new CellState.

                    Marking Screenshots: At the end of each segment, it calls set_screenshot(target.screenshot_id) on the last CellState to mark that specific coordinate as the location where the robot must take a picture of the obstacle.
                    """

            if final_sequence: break # found a valid complete path, since we prioritize more obstacles, we can exit early as the further iterations would have less obstacles

        return final_sequence, best_overall_score

    def _get_proximity_cost(self, x: int, y: int) -> int:
        """Adds a penalty if the robot is too close to obstacle corners."""
        for obs in self.arena.obstacles:
            if abs(obs.x - x) <= 2 and abs(obs.y - y) <= 2:
                if abs(obs.x - x) + abs(obs.y - y) >= 3: # Specialized proximity logic
                    return SAFE_COST
        return 0

    def _fetch_neighbors(self, cur_x: int, cur_y: int, heading: Direction):
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
        if cur_x == 3 and cur_y == 12 and heading == Direction.NORTH:
            print("DEBUG NEIGHBORS: {adjacents}".format(adjacents=adjacents))
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

    @staticmethod
    def _generate_bitmasks(n):
        masks = [bin(i)[2:].zfill(n) for i in range(2**n)]
        return sorted(masks, key=lambda x: x.count('1'), reverse=True)

    @staticmethod
    def _recursive_combinations(groups, idx, current, results, limit):
        if idx == len(groups):
            results.append(list(current))
            return
        if limit[0] <= 0: return
        limit[0] -= 1
        for i in range(len(groups[idx])):
            current.append(i)
            PathFinder._recursive_combinations(groups, idx + 1, current, results, limit)
            current.pop()

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
