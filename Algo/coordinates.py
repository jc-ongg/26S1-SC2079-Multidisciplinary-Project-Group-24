"""Convert the public 10 cm map coordinates to/from the internal 5 cm grid."""
from consts import MAP_SIZE, MAP_TO_PLANNER

def to_planner(value):
    scaled = float(value) * MAP_TO_PLANNER
    if not scaled.is_integer() or not 0 <= scaled < MAP_SIZE * MAP_TO_PLANNER:
        raise ValueError("Robot coordinates must lie on a 5 cm position inside the arena")
    return int(scaled)

def to_tablet_pose(pose):
    # The tablet accepts integer 10 cm cells. Keep the precise planner pose
    # separately so revisits never start from the rounded displayed position.
    def cell(value):
        return min(MAP_SIZE - 2, max(1, (value + MAP_TO_PLANNER // 2) // MAP_TO_PLANNER))
    return {**pose, 'x': cell(pose['x']), 'y': cell(pose['y']),
            'planner_x': pose['x'], 'planner_y': pose['y']}
