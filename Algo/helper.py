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
    return center_x > 0 and center_y > 0 and center_x < WIDTH - 1 and center_y < HEIGHT - 1

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
            if (states[i].x > states[i - 1].x and states[i].direction == Direction.EAST) or (states[i].y > states[i - 1].y and states[i].direction == Direction.NORTH):
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
    adjusted_stm_commands = []
    for cmd in stm_commands:
        if cmd.startswith("center") or cmd.startswith("SNAP") or cmd.startswith("FIN"):
            adjusted_stm_commands.append(cmd)
        else:
            if "right" in cmd and "forward" in cmd:
                adjusted_stm_commands.append(f"center,0,forward,{int(FORWARD_RIGHT[0] / 10 * UNIT_TO_CM_MULTIPLIER)}")
                adjusted_stm_commands.append(cmd)
                adjusted_stm_commands.append(f"center,0,forward,{int(FORWARD_RIGHT[1] / 10 * UNIT_TO_CM_MULTIPLIER)}")
            elif "left" in cmd and "forward" in cmd:
                adjusted_stm_commands.append(f"center,0,forward,{int(FORWARD_LEFT[0] / 10 * UNIT_TO_CM_MULTIPLIER)}")
                adjusted_stm_commands.append(cmd)
                adjusted_stm_commands.append(f"center,0,forward,{int(FORWARD_LEFT[1] / 10 * UNIT_TO_CM_MULTIPLIER)}")
            elif "right" in cmd and "reverse" in cmd:
                adjusted_stm_commands.append(f"center,0,reverse,{int(REVERSE_RIGHT[0] / 10 * UNIT_TO_CM_MULTIPLIER_REVERSE)}")
                adjusted_stm_commands.append(cmd)
                adjusted_stm_commands.append(f"center,0,reverse,{int(REVERSE_RIGHT[1] / 10 * UNIT_TO_CM_MULTIPLIER_REVERSE)}")
            elif "left" in cmd and "reverse" in cmd:
                adjusted_stm_commands.append(f"center,0,reverse,{int(REVERSE_LEFT[0] / 10 * UNIT_TO_CM_MULTIPLIER_REVERSE)}")
                adjusted_stm_commands.append(cmd)
                adjusted_stm_commands.append(f"center,0,reverse,{int(REVERSE_LEFT[1] / 10 * UNIT_TO_CM_MULTIPLIER_REVERSE)}")


    # remove empty commands
    new_adjusted_cmds = []
    for cmd in adjusted_stm_commands:
        if cmd.startswith("SNAP") or cmd.startswith("FIN"):
            new_adjusted_cmds.append(cmd)
            continue
        
        individual_cmd = cmd.split(",")
        if individual_cmd[1] == "0" and individual_cmd[3] == "0":
            continue
        else:
            new_adjusted_cmds.append(cmd)
    

    adjusted_stm_commands = []

    print("\nBefore removing redundant commands: ", new_adjusted_cmds)

    prev_cmd = None

    for cmd in new_adjusted_cmds:

        # SNAP / FIN always pass through
        if cmd.startswith("SNAP") or cmd.startswith("FIN"):
            if prev_cmd:
                adjusted_stm_commands.append(",".join(prev_cmd))
                prev_cmd = None
            adjusted_stm_commands.append(cmd)
            continue

        curr = cmd.split(",")

        # if first command
        if prev_cmd is None:
            prev_cmd = curr
            continue

        # check if both are straight movement commands
        if (
            curr[0] == "center"
            and prev_cmd[0] == "center"
            and curr[2] != prev_cmd[2]   # opposite direction
        ):

            curr_dist = int(curr[3])
            prev_dist = int(prev_cmd[3])

            diff = abs(curr_dist - prev_dist)

            if diff == 0:
                prev_cmd = None
            else:
                direction = curr[2] if curr_dist > prev_dist else prev_cmd[2]
                prev_cmd = ["center", "0", direction, str(diff)]

        else:
            # commands cannot merge
            adjusted_stm_commands.append(",".join(prev_cmd))
            prev_cmd = curr

    # append last remaining command
    if prev_cmd:
        adjusted_stm_commands.append(",".join(prev_cmd))

    print("\nAfter removing redundant commands: ", adjusted_stm_commands, "\n")

    return compressed_commands, stm_commands, adjusted_stm_commands