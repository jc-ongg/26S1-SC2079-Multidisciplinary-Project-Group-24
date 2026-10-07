from consts import *

def is_valid(center_x: int, center_y: int):
    """Checks if given position is within bounds

    Inputs
    ------
    center_x (int): x-coordinate
    center_y (int): y-coordinate

    Returns
    -------
    bool: True if valid, False otherwise
    """
    maximum = (MAP_SIZE - 2) * MAP_TO_PLANNER
    return MAP_TO_PLANNER <= center_x <= maximum and MAP_TO_PLANNER <= center_y <= maximum

def command_converter(movement):
    final_command_dict = {
        "FW": "center,0,forward,",
        "BW": "center,0,reverse,",
        "FR": "right,90,forward,",
        "FL": "left,90,forward,",
        "BR": "right,90,reverse,",
        "BL": "left,90,reverse,",
        "SNAP": "SNAP",
        "FIN": "FIN"
    }
    movement_step = set(["FW", "BW", "FR", "FL", "BR", "BL"])
    turn_step = set(["FR", "FL", "BR", "BL"])
    if movement[:2] in movement_step:
        if movement[:2] in turn_step:
            return final_command_dict[movement[:2]] + "0"
        if movement[:2] == "BW":
            steps = int(int(movement[2:]) * UNIT_TO_CM_MULTIPLIER_REVERSE)
        else:
            steps = int(int(movement[2:]) * UNIT_TO_CM_MULTIPLIER)
        return final_command_dict[movement[:2]] + str(steps)
    else:
        return movement
        

def expand_motor_command(command):
    """Padding values are physical cm, independent of the planner cell size."""
    motor = command_converter(command)
    if motor.startswith(('center', 'SNAP', 'FIN')):
        return [motor]
    steering, _, direction, _ = motor.split(',')
    pads = {('left', 'forward'): FORWARD_LEFT, ('right', 'forward'): FORWARD_RIGHT,
            ('left', 'reverse'): REVERSE_LEFT, ('right', 'reverse'): REVERSE_RIGHT}[(steering, direction)]
    return ([f'center,0,{direction},{pads[0]}'] if pads[0] else []) + [motor] + (
        [f'center,0,{direction},{pads[1]}'] if pads[1] else [])


def command_generator(states, obstacles, step_compression=True):
    """
        Iterates through the list of CellState objects produced by the algorithm and compares each state with the previous one to determine the necessary movement:
            -> Straight Movements (FW / BW): If the robot's direction remains the same between two states, the function calculates if it moved forward or backward.
                -> Forward (FW): Occurs if the robot moves in the direction it is facing (e.g., facing North and Y increases, or facing East and X increases).
                -> Backward (BW): Occurs for all other movements where the direction stays the same but the position changes.

            -> Turn Movements: When the direction changes between states, the function identifies one of four 90-degree turns:
                -> FR / FL: Forward Right or Forward Left.
                -> BR / BL: Backward Right or Backward Left.

            -> Image Capture (SNAP): Whenever a state has a valid screenshot_id, the function appends a SNAP command. It also determines the relative position of the obstacle to help the camera center the image:
                -> Sub-labels (_L, _C, _R): By comparing the robot's coordinates to the obstacle's coordinates, it attaches a suffix for Left, Center, or Right. For example, if the robot is facing North to view an obstacle and the obstacle's X is greater than the robot's X, it adds _R (Right).

            -> Command Compression: To make the movement more efficient, the function post-processes the list to combine consecutive 1-unit movements. For example, three separate FW01 commands are compressed into a single FW03 command.

            -> Termination: The very last command added is always FIN, signaling the robot has completed its path.
    """

    # Convert the list of obstacles into a dictionary with key as the obstacle id and value as the obstacle
    obstacles_dict = {ob['id']: ob for ob in obstacles}
    
    # Initialize commands list
    commands = []

    # Iterate through each state in the list of states
    for i in range(1, len(states)):
        steps = "1"  # Default step size

        # If previous state and current state are the same direction,
        if states[i].direction == states[i - 1].direction:
            # Forward - Must be (east facing AND x value increased) OR (north facing AND y value increased)
            if states[i].x == states[i - 1].x and states[i].y == states[i - 1].y:
                # Two targets may share a view pose: capture both without motion.
                commands.append("FW00")
            elif (states[i].x > states[i - 1].x and states[i].direction == Direction.EAST) or (states[i].y > states[i - 1].y and states[i].direction == Direction.NORTH):
                commands.append("FW0{}".format(steps))
            # Forward - Must be (west facing AND x value decreased) OR (south facing AND y value decreased)
            elif (states[i].x < states[i-1].x and states[i].direction == Direction.WEST) or (
                    states[i].y < states[i-1].y and states[i].direction == Direction.SOUTH):
                commands.append("FW0{}".format(steps))
            # Backward - All other cases where the previous and current state is the same direction
            else:
                commands.append("BW0{}".format(steps))

            # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
            if states[i].screenshot_id != -1:
                current_ob_dict = obstacles_dict[states[i].screenshot_id]
                current_robot_position = states[i]

                # Obstacle facing WEST, robot facing EAST
                if current_ob_dict['d'] == 6 and current_robot_position.direction == 2:
                    """ if current_ob_dict['y'] > current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    elif current_ob_dict['y'] == current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['y'] < current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}") """
                    commands.append(f"SNAP{states[i].screenshot_id}")
                
                # Obstacle facing EAST, robot facing WEST
                elif current_ob_dict['d'] == 2 and current_robot_position.direction == 6:
                    """ if current_ob_dict['y'] > current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    elif current_ob_dict['y'] == current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['y'] < current_robot_position.y:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}") """
                    commands.append(f"SNAP{states[i].screenshot_id}")

                # Obstacle facing NORTH, robot facing SOUTH
                elif current_ob_dict['d'] == 0 and current_robot_position.direction == 4:
                    """ if current_ob_dict['x'] > current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    elif current_ob_dict['x'] == current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['x'] < current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}") """
                    commands.append(f"SNAP{states[i].screenshot_id}")
                    

                # Obstacle facing SOUTH, robot facing NORTH
                elif current_ob_dict['d'] == 4 and current_robot_position.direction == 0:
                    """ if current_ob_dict['x'] > current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_R")
                    elif current_ob_dict['x'] == current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_C")
                    elif current_ob_dict['x'] < current_robot_position.x:
                        commands.append(f"SNAP{states[i].screenshot_id}_L")
                    else:
                        commands.append(f"SNAP{states[i].screenshot_id}") """
                    commands.append(f"SNAP{states[i].screenshot_id}")
            continue

        # If previous state and current state are not the same direction, it means that there will be a turn command involved
        # Assume there are 4 turning command: FR, FL, BL, BR (the turn command will turn the robot 90 degrees)
        # FR00 | FR30: Forward Right;
        # FL00 | FL30: Forward Left;
        # BR00 | BR30: Backward Right;
        # BL00 | BL30: Backward Left;

        # Facing north previously
        if states[i - 1].direction == Direction.NORTH:
            if states[i].direction == Direction.EAST:
                if states[i].y > states[i - 1].y:
                    commands.append("FR0{}".format(steps))
                else:
                    commands.append("BL0{}".format(steps))
            elif states[i].direction == Direction.WEST:
                if states[i].y > states[i - 1].y:
                    commands.append("FL0{}".format(steps))
                else:
                    commands.append("BR0{}".format(steps))
            # elif states[i].direction == Direction.NORTH_EAST:
            #     commands.append("FWD_NE0{}".format(steps))
            #     commands.append("TURN_NORTH")
            # elif states[i].direction == Direction.NORTH_WEST:
            #     commands.append("FWD_NW0{}".format(steps))
            #     commands.append("TURN_NORTH")
            # else:
            #     raise Exception("Invalid turning direction")

        elif states[i - 1].direction == Direction.EAST:
            if states[i].direction == Direction.NORTH:
                if states[i].y > states[i - 1].y:
                    commands.append("FL0{}".format(steps))
                else:
                    commands.append("BR0{}".format(steps))
            elif states[i].direction == Direction.SOUTH:
                if states[i].y > states[i - 1].y:
                    commands.append("BL0{}".format(steps))
                else:
                    commands.append("FR0{}".format(steps))
            # elif states[i].direction == Direction.SOUTH_EAST:
            #     commands.append("FWD_SE0{}".format(steps))
            #     commands.append("TURN_EAST")
            # elif states[i].direction == Direction.NORTH_EAST:
            #     commands.append("FWD_NE0{}".format(steps))
            #     commands.append("TURN_EAST")
            # else:
            #     raise Exception("Invalid turning direction")

        elif states[i - 1].direction == Direction.SOUTH:
            if states[i].direction == Direction.EAST:
                if states[i].y > states[i - 1].y:
                    commands.append("BR0{}".format(steps))
                else:
                    commands.append("FL0{}".format(steps))
            elif states[i].direction == Direction.WEST:
                if states[i].y > states[i - 1].y:
                    commands.append("BL0{}".format(steps))
                else:
                    commands.append("FR0{}".format(steps))
            # elif states[i].direction == Direction.SOUTH_EAST:
            #     commands.append("FWD_SE0{}".format(steps))
            #     commands.append("TURN_SOUTH")
            # elif states[i].direction == Direction.SOUTH_WEST:
            #     commands.append("FWD_SW0{}".format(steps))
            #     commands.append("TURN_SOUTH")
            # else:
            #     raise Exception("Invalid turning direction")

        elif states[i - 1].direction == Direction.WEST:
            if states[i].direction == Direction.NORTH:
                if states[i].y > states[i - 1].y:
                    commands.append("FR0{}".format(steps))
                else:
                    commands.append("BL0{}".format(steps))
            elif states[i].direction == Direction.SOUTH:
                if states[i].y > states[i - 1].y:
                    commands.append("BR0{}".format(steps))
                else:
                    commands.append("FL0{}".format(steps))
            # elif states[i].direction == Direction.NORTH_WEST:
            #     commands.append("FWD_NW0{}".format(steps))
            #     commands.append("TURN_WEST")
            # elif states[i].direction == Direction.SOUTH_WEST:
            #     commands.append("FWD_SW0{}".format(steps))
            #     commands.append("TURN_WEST")
            # else:
            #     raise Exception("Invalid turning direction")
        else:
            raise Exception("Invalid position")

        # If any of these states has a valid screenshot ID, then add a SNAP command as well to take a picture
        if states[i].screenshot_id != -1:
            current_ob_dict = obstacles_dict[states[i].screenshot_id]
            current_robot_position = states[i]

            # Obstacle facing WEST, robot facing EAST
            if current_ob_dict['d'] == 6 and current_robot_position.direction == 2:
                if current_ob_dict['y'] > current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                elif current_ob_dict['y'] == current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['y'] < current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")
            
            # Obstacle facing EAST, robot facing WEST
            elif current_ob_dict['d'] == 2 and current_robot_position.direction == 6:
                if current_ob_dict['y'] > current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                elif current_ob_dict['y'] == current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['y'] < current_robot_position.y:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

            # Obstacle facing NORTH, robot facing SOUTH
            elif current_ob_dict['d'] == 0 and current_robot_position.direction == 4:
                if current_ob_dict['x'] > current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                elif current_ob_dict['x'] == current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['x'] < current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

            # Obstacle facing SOUTH, robot facing NORTH
            elif current_ob_dict['d'] == 4 and current_robot_position.direction == 0:
                if current_ob_dict['x'] > current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_R")
                elif current_ob_dict['x'] == current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_C")
                elif current_ob_dict['x'] < current_robot_position.x:
                    commands.append(f"SNAP{states[i].screenshot_id}_L")
                else:
                    commands.append(f"SNAP{states[i].screenshot_id}")

    # Final command is the stop command (FIN)
    commands.append("FIN")  

    # Compress commands if there are consecutive forward or backward commands
    compressed_commands = commands
    if step_compression:
        compressed_commands = [commands[0]]

        for i in range(1, len(commands)):
            # If both commands are BW
            if commands[i].startswith("BW") and compressed_commands[-1].startswith("BW"):
                # Get the number of steps of previous command
                steps = int(compressed_commands[-1][2:])
                # If steps are not 90, add 10 to the steps
                if steps < 9:
                    compressed_commands[-1] = "BW0{}".format(steps + 1)
                    continue
                else:
                    compressed_commands[-1] = "BW{}".format(steps + 1)
                    continue

            # If both commands are FW
            elif commands[i].startswith("FW") and compressed_commands[-1].startswith("FW"):
                # Get the number of steps of previous command
                steps = int(compressed_commands[-1][2:])
                # If steps are not 90, add 10 to the steps
                if steps < 9:
                    compressed_commands[-1] = "FW0{}".format(steps + 1)
                    continue
                else:
                    compressed_commands[-1] = "FW{}".format(steps + 1)
                    continue
            
            # Otherwise, just add as usual
            compressed_commands.append(commands[i])
    
    # Convert the command into STM format
    stm_commands = [command_converter(command) for command in compressed_commands]
    from motor_schedule import coalesce
    expanded = [(motor, None) for command in compressed_commands
                for motor in expand_motor_command(command)]
    adjusted_stm_commands = [command for command, _ in coalesce(expanded)]

    return compressed_commands, stm_commands, adjusted_stm_commands
