"""Validate an Algo route without sockets, serial, or multiprocessing."""


def parse_plan(result):
    if not isinstance(result, dict) or result.get("error"):
        raise ValueError("Algo returned an error")
    data = result.get("data", result)
    commands = data.get("stm_commands")
    poses = data.get("command_poses")
    if not isinstance(commands, list) or not commands or commands[-1] != "FIN":
        raise ValueError("Algo returned no complete route")
    if not any(isinstance(command, str) and command.startswith("SNAP") for command in commands):
        raise ValueError("Algo route does not visit any image targets")
    if not isinstance(poses, list) or len(poses) != len(commands):
        raise ValueError("Algo needs the command_poses update; restart the updated laptop server")
    for command, pose in zip(commands, poses):
        if not isinstance(command, str) or not command.startswith(("center,", "left,", "right,", "SNAP", "FIN")):
            raise ValueError("Algo returned an unsupported command")
        if pose is not None and (not isinstance(pose, dict) or
            any(type(pose.get(key)) is not int for key in ("x", "y", "d")) or
            pose["d"] not in (0, 2, 4, 6) or
            not (0 <= pose["x"] < 20 and 0 <= pose["y"] < 20)):
            raise ValueError("Algo returned an invalid command position")
    return list(zip(commands, poses))
