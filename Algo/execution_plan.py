"""Attach planner endpoints to the actual padded/cancelled STM commands.

No endpoint is advertised during the middle of a compound turn. These are
planned positions at command completion, not measured odometry.
"""
from consts import (FORWARD_LEFT, FORWARD_RIGHT, REVERSE_LEFT, REVERSE_RIGHT,
                    UNIT_TO_CM_MULTIPLIER, UNIT_TO_CM_MULTIPLIER_REVERSE)
from helper import command_converter


def build_command_poses(commands, path, stm_commands):
    pose = path[0]
    endpoints = iter(path[1:])
    expanded = []
    for command in commands:
        if not command.startswith(("SNAP", "FIN")):
            pose = next(endpoints)
        endpoint = {key: pose[key] for key in ("x", "y", "d")}
        motor = command_converter(command)
        if motor.startswith(("center", "SNAP", "FIN")):
            parts = [motor]
        else:
            steering, _, direction, _ = motor.split(",")
            pads = {("left", "forward"): FORWARD_LEFT,
                    ("right", "forward"): FORWARD_RIGHT,
                    ("left", "reverse"): REVERSE_LEFT,
                    ("right", "reverse"): REVERSE_RIGHT}[(steering, direction)]
            scale = UNIT_TO_CM_MULTIPLIER if direction == "forward" else UNIT_TO_CM_MULTIPLIER_REVERSE
            parts = [f"center,0,{direction},{int(pads[0] / 10 * scale)}", motor,
                     f"center,0,{direction},{int(pads[1] / 10 * scale)}"]
        parts = [part for part in parts if not part.startswith("center,0,") or int(part.split(",")[3]) != 0]
        for index, part in enumerate(parts):
            expanded.append((part, endpoint if index == len(parts) - 1 else None))

    # Same cancellation as command_generator, carrying the later endpoint.
    # A completely cancelled move has no ACK; SNAP/FIN carries its endpoint.
    adjusted = []
    previous = None
    for command, endpoint in expanded:
        if command.startswith(("SNAP", "FIN")):
            if previous:
                adjusted.append(previous)
                previous = None
            adjusted.append((command, endpoint))
            continue
        if previous is None:
            previous = (command, endpoint)
            continue
        current_fields = command.split(",")
        previous_fields = previous[0].split(",")
        if current_fields[0] == previous_fields[0] == "center" and current_fields[2] != previous_fields[2]:
            current_distance, previous_distance = int(current_fields[3]), int(previous_fields[3])
            difference = abs(current_distance - previous_distance)
            if difference == 0:
                previous = None
            else:
                direction = current_fields[2] if current_distance > previous_distance else previous_fields[2]
                previous = (f"center,0,{direction},{difference}", endpoint)
        else:
            adjusted.append(previous)
            previous = (command, endpoint)
    if previous:
        adjusted.append(previous)
    if [command for command, _ in adjusted] != stm_commands:
        raise ValueError("Execution pose schedule does not match STM command generation")
    return [endpoint for _, endpoint in adjusted]
