import turtle
import math
import time
from consts import *



# =========== ADJUST ACCORDINGLY TO ROBOT'S MOVEMENT CHARACTERISTICS ===========
#FORWARD_LONGITUDINAL_OFFSET = 1
#FORWARD_LATERAL_OFFSET = 2
#BACKWARD_LONGITUDINAL_OFFSET = 2
#BACKWARD_LATERAL_OFFSET = 1
# ==============================================================================
# Backup values
#FORWARD_LONGITUDINAL_OFFSET = 2
#FORWARD_LATERAL_OFFSET = 3
#BACKWARD_LONGITUDINAL_OFFSET = 3
#BACKWARD_LATERAL_OFFSET = 2



# --- CONFIGURATION ---
GRID_SIZE, CELL_DIM = 20, 30
ARENA_PIXELS = GRID_SIZE * CELL_DIM
SAFETY_BOX_SIZE = CELL_DIM * 3
MOVE_SPEED, OFFSET = 0.005, 15
DISTANCE_MULTIPLIER = 1 

# YOUR NEW CARDINAL MAPPING:
# North: 0 (90 deg), East: 2 (0 deg), South: 4 (270 deg), West: 6 (180 deg)
DIRECTION_MAP = {0: 90, 2: 0, 4: 270, 6: 180}

def sync_safety_zone(car, zone):
    zone.clear()
    zone.penup()
    zone.goto(car.pos())
    zone.setheading(car.heading())
    half = SAFETY_BOX_SIZE / 2
    zone.backward(half); zone.left(90); zone.forward(half); zone.right(90)
    zone.pendown()
    zone.color("blue")
    for _ in range(4):
        zone.forward(SAFETY_BOX_SIZE); zone.right(90)

def draw_directed_obstacle(painter, x, y, direction_code):
    # Use the unified direction mapping
    heading = DIRECTION_MAP.get(direction_code, 90)
    
    px, py = x * CELL_DIM, y * CELL_DIM
    painter.penup(); painter.goto(px, py); painter.setheading(0)
    painter.color("red"); painter.begin_fill()
    for _ in range(4): painter.forward(CELL_DIM); painter.left(90)
    painter.end_fill()
    
    painter.color("purple"); painter.pensize(4); painter.penup()
    # Purple line placement logic
    if heading == 90: # North
        painter.goto(px, py + CELL_DIM); painter.setheading(0)
    elif heading == 270: # South
        painter.goto(px, py); painter.setheading(0)
    elif heading == 180: # West
        painter.goto(px, py); painter.setheading(90)
    else: # East (0 deg)
        painter.goto(px + CELL_DIM, py); painter.setheading(90)
    
    painter.pendown(); painter.forward(CELL_DIM); painter.penup(); painter.pensize(1)

def run_turtle_sim(data, repeat=True, pause_between=1, trail=True, trail_color="green", trail_width=2, clear_trail_on_repeat=True):
    start_info = {'x': data['robotX'], 'y': data['robotY'], 'dir': data['robotDir']}
    obstacle_list = data['obstacles']
    action_list = data['commands']
    
    turtle.resetscreen()
    screen = turtle.Screen()
    screen.tracer(0)
    screen.setworldcoordinates(-50, -50, ARENA_PIXELS + 50, ARENA_PIXELS + 50)
    
    painter = turtle.Turtle(); painter.hideturtle()
    painter.color("#E0E0E0")
    for i in range(GRID_SIZE + 1):
        painter.penup(); painter.goto(i*CELL_DIM, 0); painter.pendown(); painter.goto(i*CELL_DIM, ARENA_PIXELS)
        painter.penup(); painter.goto(0, i*CELL_DIM); painter.pendown(); painter.goto(ARENA_PIXELS, i*CELL_DIM)
    
    for obs in obstacle_list:
        draw_directed_obstacle(painter, obs['x'], obs['y'], obs['d'])
    
    car = turtle.Turtle(); car.shape("classic"); car.color("blue"); car.penup()
    # Apply correctly mapped heading
    car.goto(start_info['x'] * CELL_DIM + OFFSET, start_info['y'] * CELL_DIM + OFFSET) 
    car.setheading(DIRECTION_MAP.get(int(start_info['dir']), 90))
    
    trail_t = None
    if trail:
        trail_t = turtle.Turtle(); trail_t.hideturtle(); trail_t.penup()
        trail_t.color(trail_color); trail_t.pensize(trail_width)
        trail_t.goto(car.pos()); trail_t.pendown()
    
    zone = turtle.Turtle(); zone.hideturtle()
    sync_safety_zone(car, zone); screen.update()

    while True:
        for action in action_list:
            if action == "FIN": break
            if action.startswith("SNAP"):
                time.sleep(1); continue

            cmd, val = action[:2], int(action[2:]) * DISTANCE_MULTIPLIER

            if cmd in ["FW", "BW"]:
                for _ in range(val * CELL_DIM):
                    car.forward(1 if cmd == "FW" else -1)
                    if trail and trail_t: trail_t.goto(car.pos())
                    sync_safety_zone(car, zone); screen.update(); time.sleep(MOVE_SPEED)

            elif cmd in ["FL", "FR", "BL", "BR"]:
                start_pos = car.pos()
                start_heading_deg = car.heading()
                start_heading_rad = math.radians(start_heading_deg)
                
                # Forward Vector
                f_vec = (math.cos(start_heading_rad), math.sin(start_heading_rad))
                
                is_left = "L" in cmd
                is_back = "B" in cmd
                
                # Forward: 2 fwd, 3 lat | Backward: 3 back, 2 lat
                if is_back: # Backward turn
                    current_f_offset = BACKWARD_LEFT_LONGITUDINAL_OFFSET * CELL_DIM # Longitudinal offset (Forward/Backward)
                    current_s_offset = BACKWARD_LEFT_LATERAL_OFFSET * CELL_DIM # Lateral offset (Sideways)
                else: # Forward turn
                    current_f_offset = FORWARD_LEFT_LONGITUDINAL_OFFSET * CELL_DIM
                    current_s_offset = FORWARD_LEFT_LATERAL_OFFSET * CELL_DIM
                
                # Sideways Vector
                s_vec = (-f_vec[1], f_vec[0]) if is_left else (f_vec[1], -f_vec[0])
                
                for i in range(1, 91):
                    angle_rad = math.radians(i)
                    
                    # Elliptical progression
                    f_dist = current_f_offset * math.sin(angle_rad)
                    s_dist = current_s_offset * (1 - math.cos(angle_rad))
                    
                    # Invert the forward vector component if moving backward
                    move_f = -f_dist if is_back else f_dist
                    
                    new_x = start_pos[0] + (move_f * f_vec[0]) + (s_dist * s_vec[0])
                    new_y = start_pos[1] + (move_f * f_vec[1]) + (s_dist * s_vec[1])
                    
                    car.goto(new_x, new_y)
                    if trail and trail_t:
                        trail_t.goto((new_x, new_y))
                    
                    # Heading Update
                    if not is_back:
                        car.setheading(start_heading_deg + (i if is_left else -i))
                    else:
                        # When backing up, turning the rear left/right 
                        # changes the front heading in the opposite way
                        car.setheading(start_heading_deg + (-i if is_left else i))
                    
                    sync_safety_zone(car, zone)
                    screen.update()
                    time.sleep(MOVE_SPEED * 2)

        if not repeat: break

        # Loop reset using new heading logic
        car.penup()
        car.goto(start_info['x'] * CELL_DIM + OFFSET, start_info['y'] * CELL_DIM + OFFSET)
        car.setheading(DIRECTION_MAP.get(int(start_info['dir']), 90))
        zone.clear()
        if trail and trail_t:
            if clear_trail_on_repeat:
                trail_t.clear(); trail_t.penup(); trail_t.goto(car.pos()); trail_t.pendown()
            else:
                trail_t.penup(); trail_t.goto(car.pos()); trail_t.pendown()
        sync_safety_zone(car, zone)
        screen.update()
        time.sleep(pause_between)