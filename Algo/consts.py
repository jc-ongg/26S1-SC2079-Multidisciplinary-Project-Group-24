from enum import Enum
import os

# Explicit opt-in: the previously tried speed-25 profile is NOT the default.
MOVEMENT_PROFILE = os.environ.get("MDP_MOVEMENT_PROFILE", "original")
if MOVEMENT_PROFILE not in ("original", "measured25"):
    raise ValueError("MDP_MOVEMENT_PROFILE must be original or measured25")
RECOMMENDED_MOTOR_SPEED = 25 if MOVEMENT_PROFILE == "measured25" else 50
REVERSE_CALIBRATION = os.environ.get("MDP_REVERSE_CALIBRATION", "original")
FORWARD_CALIBRATION = os.environ.get("MDP_FORWARD_CALIBRATION", "original")
if FORWARD_CALIBRATION not in ("original", "speed50-v2"):
    raise ValueError("MDP_FORWARD_CALIBRATION must be original or speed50-v2")
if REVERSE_CALIBRATION not in ("original", "speed50-v2"):
    raise ValueError("MDP_REVERSE_CALIBRATION must be original or speed50-v2")
if MOVEMENT_PROFILE != "original" and (REVERSE_CALIBRATION != "original" or FORWARD_CALIBRATION != "original"):
    raise ValueError("speed50-v2 calibration requires the original speed-50 profile")
DIRECTIONAL_MOVEMENT = (MOVEMENT_PROFILE == "measured25" or
                        REVERSE_CALIBRATION == "speed50-v2" or FORWARD_CALIBRATION == "speed50-v2")

# =========== ADJUST ACCORDINGLY TO ROBOT'S MOVEMENT CHARACTERISTICS ===========
# Arena cells are 10cm (2m / 20 cells) and the STM's FWD unit is ~1cm
# (measured: ultrasonic fell ~21cm over two forward-10 commands). 20 here
# doubled every distance and drove the robot straight through obstacles.
UNIT_TO_CM_MULTIPLIER = 10
UNIT_TO_CM_MULTIPLIER_REVERSE = 10

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
if MOVEMENT_PROFILE == "measured25":
    # Rounded grid endpoints with before/after straight corrections (cm).
    # Provisional trial derived from the user's bare-turn measurements at 25.
    FORWARD_LEFT_LONGITUDINAL_OFFSET, FORWARD_LEFT_LATERAL_OFFSET = 3, 3
    FORWARD_RIGHT_LONGITUDINAL_OFFSET, FORWARD_RIGHT_LATERAL_OFFSET = 4, 3
    BACKWARD_LEFT_LONGITUDINAL_OFFSET, BACKWARD_LEFT_LATERAL_OFFSET = 5, 4
    BACKWARD_RIGHT_LONGITUDINAL_OFFSET, BACKWARD_RIGHT_LATERAL_OFFSET = 5, 3
    FORWARD_LEFT, FORWARD_RIGHT = (5, 8), (8, 4)
    REVERSE_LEFT, REVERSE_RIGHT = (5, 7), (6, 3)
if REVERSE_CALIBRATION == "speed50-v2":
    # New bare-turn measurements at speed50: REVL 29.5/15cm, REVR 34/21.5cm.
    # Whole-cell targets and integer straight compensation; trial, not validated.
    BACKWARD_LEFT_LONGITUDINAL_OFFSET, BACKWARD_LEFT_LATERAL_OFFSET = 3, 2
    BACKWARD_RIGHT_LONGITUDINAL_OFFSET, BACKWARD_RIGHT_LATERAL_OFFSET = 4, 3
    REVERSE_LEFT, REVERSE_RIGHT = (1, 5), (6, 8)
if FORWARD_CALIBRATION == "speed50-v2":
    # New table results at speed50: TURNL 15/33cm, TURNR 25/37.5cm.
    # Treat as bare 90-degree turns; preserve the reverse profile independently.
    FORWARD_LEFT_LONGITUDINAL_OFFSET, FORWARD_LEFT_LATERAL_OFFSET = 2, 4
    FORWARD_RIGHT_LONGITUDINAL_OFFSET, FORWARD_RIGHT_LATERAL_OFFSET = 3, 4
    FORWARD_LEFT, FORWARD_RIGHT = (5, 7), (5, 3)
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
