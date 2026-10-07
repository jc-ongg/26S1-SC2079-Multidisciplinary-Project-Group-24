"""Attach planner endpoints to the actual padded/cancelled STM commands.

No endpoint is advertised during the middle of a compound turn. These are
planned positions at command completion, not measured odometry.
"""
from helper import expand_motor_command
from motor_schedule import coalesce


def build_command_poses(commands, path, stm_commands):
    pose = path[0]
    endpoints = iter(path[1:])
    expanded = []
    for command in commands:
        if not command.startswith(('SNAP', 'FIN')):
            pose = next(endpoints)
        endpoint = {key: pose[key] for key in ('x', 'y', 'd')}
        parts = expand_motor_command(command)
        for index, part in enumerate(parts):
            expanded.append((part, endpoint if index == len(parts) - 1 else None))
    adjusted = coalesce(expanded)
    if [command for command, _ in adjusted] != stm_commands:
        raise ValueError('Execution pose schedule does not match STM command generation')
    return [endpoint for _, endpoint in adjusted]
