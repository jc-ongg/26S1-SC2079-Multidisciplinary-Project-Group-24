"""One Task 1 test configuration: speed50, 5 cm planner, 10 cm tablet map."""
from enum import Enum

MOVEMENT_PROFILE = "turn-guide-speed50-grid5"
RECOMMENDED_MOTOR_SPEED = 50
GRID_CELL_CM = 5
MAP_CELL_CM = 10
MAP_SIZE = 20
MAP_TO_PLANNER = MAP_CELL_CM // GRID_CELL_CM
WIDTH = HEIGHT = MAP_SIZE * MAP_TO_PLANNER
UNIT_TO_CM_MULTIPLIER = UNIT_TO_CM_MULTIPLIER_REVERSE = GRID_CELL_CM

# User-confirmed bare-turn measurements at speed50, plus physical cm padding.
# FL: 17/30 + 3 before -> 20/30; FR: 22/35 + 3 before -> 25/35.
# BL: 30/15; BR: 35/23 + 2 after -> 35/25.
FORWARD_LEFT_LONGITUDINAL_OFFSET, FORWARD_LEFT_LATERAL_OFFSET = 4, 6
FORWARD_RIGHT_LONGITUDINAL_OFFSET, FORWARD_RIGHT_LATERAL_OFFSET = 5, 7
BACKWARD_LEFT_LONGITUDINAL_OFFSET, BACKWARD_LEFT_LATERAL_OFFSET = 6, 3
BACKWARD_RIGHT_LONGITUDINAL_OFFSET, BACKWARD_RIGHT_LATERAL_OFFSET = 7, 5
FORWARD_LEFT = FORWARD_RIGHT = (3, 0)
REVERSE_LEFT, REVERSE_RIGHT = (0, 0), (0, 2)
DIRECTIONAL_MOVEMENT = True

class Direction(int, Enum):
    NORTH = 0
    EAST = 2
    SOUTH = 4
    WEST = 6
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTION = [(1, 0, Direction.EAST), (-1, 0, Direction.WEST),
                  (0, 1, Direction.NORTH), (0, -1, Direction.SOUTH)]
TURN_FACTOR = 10 * MAP_TO_PLANNER
REVERSE_TURN_PENALTY = 0
EXPANDED_CELL = MAP_TO_PLANNER
ITERATIONS = 2000
TURN_RADIUS = 1
SAFE_COST = 1000 * MAP_TO_PLANNER
SCREENSHOT_COST = 10 * MAP_TO_PLANNER

# Historical checklist inputs remain callable; they do not select another profile.
A5_INITIAL_INPUT = "CONTROL - ['center,0,forward,40', 'SNAP1']"
A5_PIVOT_INPUT = "CONTROL - ['left,90,reverse,0', 'center,0,forward,120', 'left,90,forward,0', 'left,90,forward,0', 'SNAP1']"
TASK_2 = "CONTROL - []"
