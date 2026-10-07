"""One restored Task 1 setup: previous speed50 turns, 10 cm cells."""
from enum import Enum

MOVEMENT_PROFILE = "restored-speed50-grid10"
RECOMMENDED_MOTOR_SPEED = 50
GRID_CELL_CM = MAP_CELL_CM = 10
WIDTH = HEIGHT = 20
UNIT_TO_CM_MULTIPLIER = UNIT_TO_CM_MULTIPLIER_REVERSE = 10
FORWARD_CALIBRATION = REVERSE_CALIBRATION = "speed50-v2"
DIRECTIONAL_MOVEMENT = True

# Exact resolved geometry/padding of the server running before the 5 cm trial.
FORWARD_LEFT_LONGITUDINAL_OFFSET, FORWARD_LEFT_LATERAL_OFFSET = 2, 4
FORWARD_RIGHT_LONGITUDINAL_OFFSET, FORWARD_RIGHT_LATERAL_OFFSET = 3, 4
BACKWARD_LEFT_LONGITUDINAL_OFFSET, BACKWARD_LEFT_LATERAL_OFFSET = 3, 2
BACKWARD_RIGHT_LONGITUDINAL_OFFSET, BACKWARD_RIGHT_LATERAL_OFFSET = 4, 3
FORWARD_LEFT, FORWARD_RIGHT = (5, 7), (5, 3)
REVERSE_LEFT, REVERSE_RIGHT = (1, 5), (6, 8)

# User's current trial: restore 40 cm turn clearance; keep straight/edge rules.
TURN_CLEARANCE_CM = 40
STRAIGHT_AXIS_CLEARANCE_CM = 30
STRAIGHT_MANHATTAN_CLEARANCE_CM = 40

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
TURN_FACTOR = 10
REVERSE_TURN_PENALTY = 0
EXPANDED_CELL = 1
ITERATIONS = 2000
TURN_RADIUS = 1
SAFE_COST = 1000
SCREENSHOT_COST = 10
A5_INITIAL_INPUT = "CONTROL - ['center,0,forward,40', 'SNAP1']"
A5_PIVOT_INPUT = "CONTROL - ['left,90,reverse,0', 'center,0,forward,120', 'left,90,forward,0', 'left,90,forward,0', 'SNAP1']"
TASK_2 = "CONTROL - []"
