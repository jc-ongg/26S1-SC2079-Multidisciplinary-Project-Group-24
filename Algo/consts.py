from enum import Enum

# =========== ADJUST ACCORDINGLY TO ROBOT'S MOVEMENT CHARACTERISTICS ===========
UNIT_TO_CM_MULTIPLIER = 20
UNIT_TO_CM_MULTIPLIER_REVERSE = 20

FORWARD_LEFT_LONGITUDINAL_OFFSET = 2
FORWARD_LEFT_LATERAL_OFFSET = 4

FORWARD_RIGHT_LONGITUDINAL_OFFSET = 2
FORWARD_RIGHT_LATERAL_OFFSET = 4

BACKWARD_LEFT_LONGITUDINAL_OFFSET = 3
BACKWARD_LEFT_LATERAL_OFFSET = 2

BACKWARD_RIGHT_LONGITUDINAL_OFFSET = 3
BACKWARD_RIGHT_LATERAL_OFFSET = 2
# ==============================================================================
FORWARD_RIGHT = (0, 4) # Move forward X cm, turn right, move forward Y cm
FORWARD_LEFT = (3, 5)
REVERSE_RIGHT = (0, 4) # Move backward X cm, turn right, move backward Y cm
REVERSE_LEFT = (0, 2)
# ============================ A5 Hardcoded Path ===============================
A5_INITIAL_INPUT = "CONTROL - ['center,0,forward,40', 'SNAP1']"
A5_PIVOT_INPUT = "CONTROL - ['left,90,reverse,0', 'center,0,forward,120', 'left,90,forward,0', 'left,90,forward,0', 'SNAP1']"
TASK_2 = "CONTROL - []" # TO BE FILLED IN LATER
# ==============================================================================

class Direction(int, Enum):
    NORTH = 0
    # NORTH_EAST = 1
    EAST = 2
    # SOUTH_EAST = 3
    SOUTH = 4
    # SOUTH_WEST = 5
    WEST = 6
    # NORTH_WEST = 7
    SKIP = 8

    def __int__(self):
        return self.value

    @staticmethod
    def rotation_cost(d1, d2):
        diff = abs(d1 - d2)
        return min(diff, 8 - diff)

MOVE_DIRECTION = [
    (1, 0, Direction.EAST),
    (-1, 0, Direction.WEST),
    (0, 1, Direction.NORTH),
    (0, -1, Direction.SOUTH),
    # (1, 1, Direction.NORTH_EAST),
    # (1, -1, Direction.SOUTH_EAST),
    # (-1, 1, Direction.NORTH_WEST),
    # (-1, -1, Direction.SOUTH_WEST)
]

TURN_FACTOR = 10

REVERSE_TURN_PENALTY = 0

EXPANDED_CELL = 1 # for both agent and obstacles

WIDTH = 20
HEIGHT = 20

# JJ Originally 2000, reduce to speed up computation
ITERATIONS = 2000
TURN_RADIUS = 1

SAFE_COST = 1000 # the cost for the turn in case there is a chance that the robot is touch some obstacle
SCREENSHOT_COST = 10 # the cost for the place where the picture is taken