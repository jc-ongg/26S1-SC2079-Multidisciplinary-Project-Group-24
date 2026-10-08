import time
from algo.algo import PathFinder
from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from helper import command_generator
import os
from car_animation import run_turtle_sim
import queue
import threading
import turtle
from threading import Lock
import ast
import consts

# All recognition uses the same CV service and released ONNX model.
# /infer is a compatibility adapter; it never loads the old best.pt.
import requests
CV_BASE_URL = os.environ.get(
    'CV_BASE_URL', 'http://127.0.0.1:' + os.environ.get('IMAGE_API_PORT', '8001')).rstrip('/')

# Create upload directory if not exists
UPLOAD_FOLDER = "uploads"
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

app = Flask(__name__)
sim_queue = queue.Queue()
CORS(app)
#model = load_model()
model = None
@app.route('/status', methods=['GET'])
def status():
    """
    This is a health check endpoint to check if the server is running
    :return: a json object with a key "result" and value "ok"
    """
    return jsonify({"status": "ok", "movement_profile": consts.MOVEMENT_PROFILE,
                    "recommended_motor_speed": consts.RECOMMENDED_MOTOR_SPEED,
                    "reverse_calibration": consts.REVERSE_CALIBRATION,
                    "forward_calibration": consts.FORWARD_CALIBRATION,
                    "grid_cell_cm": consts.GRID_CELL_CM, "input_cell_cm": consts.MAP_CELL_CM,
                    "arena_cells": consts.WIDTH, "turn_clearance_cm": consts.TURN_CLEARANCE_CM})

def convert_input(request):
    # Example : STATUS, RETRY_FALSE, OBSTACLE LIST, 4, 1; 11; 12; UP | 2; 4; 7; UP | 3; 14; 6; UP | 4; 6; 13; UP
    face_map = {
            "NORTH": 0,
            "EAST": 2,
            "SOUTH": 4,
            "WEST": 6,
            "SKIP": 8
        }

    andriod_input = request.get_data().decode()
    print(f'===========================\nFrontend/Android Input: \n{andriod_input}\n===========================')

    check_input = andriod_input.split("-")
    if check_input[0].strip() == "CONTROL":
        print("Manual control from Android app.")
        return andriod_input
    elif check_input[0].strip() == "A5_Initial":
        print("A5 Initial from RPI.")
        return consts.A5_INITIAL_INPUT
    elif check_input[0].strip() == "A5_Pivot":
        print("A5 Pivot from RPI.")
        return consts.A5_PIVOT_INPUT
    elif check_input[0].strip() == "Task_2":
        print("Task 2 hardcoded input from RPI.")
        return consts.TASK_2

    print("Converting input from frontend/android app.\n")

    input_array = andriod_input.split(",")
    content = {
        "obstacles": [],
        "robot_x": 1,
        "robot_y": 1,
        "robot_dir": 0,
        "retrying": False,
        "step_compression": True
    }

    print(f'Input Obstacles: \n{input_array}===========================')
    robot_info = input_array[3].strip().split(";")
    print(f'Robot Info: {robot_info}')

    content["robot_x"] = int(robot_info[0])
    content["robot_y"] = int(robot_info[1])
    content["robot_dir"] = face_map[robot_info[2].strip()]

    if input_array[2].strip() == "false":
        content["retrying"] = False
    else:
        content["retrying"] = True

    input_obstacles = input_array[5].split('|')
    for individual_input_obstacle in input_obstacles:
        content_y = individual_input_obstacle.strip().split(";")
        content["obstacles"].append({
            "x": int(content_y[1]),
            "y": int(content_y[2]),
            "d": face_map[content_y[3].strip()],
            "id": int(content_y[0])
        })
    return content

@app.route('/path', methods=['POST'])
def path_finding():
    """
    This is the main endpoint for the path finding algorithm
    :return: a json object with a key "data" and value a dictionary with keys "distance", "path", and "commands"
    """
    # Convert input from raw format to json format
    """     try:
        # Get the json data from the request
        content = request.json
    except:
        content = convert_input(request) """

    content = convert_input(request)

    if not isinstance(content, dict) and content.startswith("CONTROL"):
        # eg. "CONTROL - ['center,0,forward,120', 'right,90,forward,0', 'center,0,forward,40', 'SNAP1', 'FIN']"
        print("Hardcoded Input:", ast.literal_eval(content.split("-")[1].strip()))
        return jsonify({
        "data": {
            'distance': 0,
            'path': [],
            'commands': [],
            'stm_commands': ast.literal_eval(content.split("-")[1].strip())
        },
        "error": None
        })
    print("\nERROR CHECKING ==>>> Right before algorithm: ", content)
    # Get the obstacles, big_turn, retrying, robot_x, robot_y, and robot_direction from the json data
    obstacles = content['obstacles']
    # big_turn = int(content['big_turn'])
    retrying = content['retrying']
    robot_x, robot_y = content['robot_x'], content['robot_y']
    robot_direction = int(content['robot_dir'])
    step_compression = content['step_compression']

    # Initialize PathFinder object with robot size of 20x20, bottom left corner of robot at (1,1), facing north, and whether to use a big turn or not.
    path_finder = PathFinder(20, 20, robot_x, robot_y, robot_direction)

    # Add each obstacle into the PathFinder. Each obstacle is defined by its x,y positions, its direction, and its id
    for ob in obstacles:
        path_finder.insert_target(ob['x'], ob['y'], ob['d'], ob['id'])
    start = time.time()
    # Get shortest path
    optimal_path, distance = path_finder.solve_mission(is_retry=retrying)
    print(f"Time taken to find shortest path using A* search: {time.time() - start}s")
    print(f"Distance to travel: {distance} units")
    print(f"Optimal path: \n{optimal_path}\n\n")

    # Based on the shortest path, generate commands for the robot
    commands, unadjusted_stm_commands, stm_commands = command_generator(optimal_path, obstacles, step_compression)
    print(f"Commands generated: \n{commands}\n\n")
    print(f"Unadjusted STM Commands generated: \n{unadjusted_stm_commands}\n\n")
    print(f"STM Commands generated: \n{stm_commands}\n\n")

    # Get the starting location and add it to path_results
    path_results = [optimal_path[0].get_dict()]
    # Process each command individually and append the location the robot should be after executing that command to path_results
    i = 0
    for command in commands:
        if command.startswith("SNAP"):
            # Consume a photo-only state without shifting later movement poses.
            target_id = int(command[4:].split("_")[0])
            if (i + 1 < len(optimal_path)
                    and optimal_path[i + 1].screenshot_id == target_id
                    and optimal_path[i + 1].is_eq(
                        optimal_path[i].x, optimal_path[i].y, optimal_path[i].direction)):
                i += 1
            continue
        if command.startswith("FIN"):
            continue
        elif command.startswith("FW") or command.startswith("FS"):
            i += int(command[2:]) #// 10
        elif command.startswith("BW") or command.startswith("BS"):
            i += int(command[2:]) #// 10
        else:
            i += 1
        path_results.append(optimal_path[i].get_dict())
    from execution_plan import build_command_poses
    command_poses = build_command_poses(commands, path_results, stm_commands)
    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands,
            # 'stm_commands': unadjusted_stm_commands
            'stm_commands': stm_commands,
            'command_poses': command_poses,
            'grid_cell_cm': consts.GRID_CELL_CM, 'input_cell_cm': consts.MAP_CELL_CM
        },
        "error": None
        })
        #stm_commands)


@app.route('/infer', methods=['POST'])
def image_predict():
    upload = request.files.get('file')
    if upload is None:
        upload = request.files.get('image')
    if upload is None:
        return jsonify(ok=False, error='Upload an image using file or image'), 400
    started = time.monotonic()
    try:
        response = requests.post(
            CV_BASE_URL + '/predict',
            data={key: value for key, value in request.form.items()
                  if key in ('run_id', 'target_obstacle_id', 'signal', 'attempt')},
            files={'image': (upload.filename or 'frame.jpg', upload.stream,
                             upload.content_type or 'image/jpeg')},
            timeout=(3, 30))
        result = response.json()
        if not response.ok:
            return jsonify(ok=False, error=result.get('error', 'CV recognition failed'),
                           capture=result), response.status_code
        detections = result.get('detections', [])
        return jsonify(ok=True, n=len(detections), detections=detections,
                       latency_ms=round((time.monotonic() - started) * 1000, 1),
                       model=result.get('model'), capture=result)
    except (requests.RequestException, ValueError) as exc:
        return jsonify(ok=False, error=f'CV service unavailable: {exc}'), 503


@app.route('/stitch', methods=['GET'])
def image_stitch():
    try:
        response = requests.get(CV_BASE_URL + '/stitch',
                                params=request.args.to_dict(), timeout=(3, 30))
        headers = {'Content-Type': response.headers.get('Content-Type', 'application/json')}
        if response.headers.get('X-Stitch-File'):
            headers['X-Stitch-File'] = response.headers['X-Stitch-File']
        return response.content, response.status_code, headers
    except requests.RequestException as exc:
        return jsonify(error=f'CV service unavailable: {exc}'), 503


@app.route('/animate', methods=['POST'])
def animate():
    # Receive data and put it in the queue for the main thread
    #print("DATA: ", data)
    #{'commands': ['FL01', 'FW01', 'BR01', 'FW02', 'BL01', 'FW01', 'FR01', 'FW01', 'FL01', 'FW04', 'FR01', 'BW03', 'BL01', 'FW01', 'FIN'], 'obstacles': [{'x': 10, 'y': 10, 'd': 0, 'id': 10}, {'x': 7, 'y': 6, 'd': 4, 'id': 1}], 'robotX': 1, 'robotY': 1, 'robotDir': '2'}
    data = request.json
    sim_queue.put(data)
    return jsonify({"status": "queued", "message": "Simulation will start on main thread"}), 200

def check_queue():
    """Main thread function to check for new simulation jobs."""
    try:
        # Check if there is data in the queue without blocking
        data = sim_queue.get_nowait()
        run_turtle_sim(data)
    except queue.Empty:
        pass
    # Re-run this check every 100ms
    turtle.ontimer(check_queue, 100)

# @app.route('/stitch', methods=['GET'])
# def stitch():
#     """
#     This is the main endpoint for the stitching command. Stitches the images using two different functions, in effect creating two stitches, just for redundancy purposes
#     """
#     # img = stitch_image()
#     # img.show()
#     # img2 = stitch_image_own()
#     # img2.show()
#     return jsonify({"result": "ok"})


def run_flask():
    # Run Flask in a background thread
    # ALGO_PORT lets a second copy run alongside (e.g. for laptop_verify.py).
    app.run(host='0.0.0.0', port=int(os.environ.get("ALGO_PORT", 8000)),
            debug=True, use_reloader=False)

if __name__ == '__main__':
    run_turtle = False

    if run_turtle:
        # Run Flask in a background thread and run Turtle on the main thread.
        threading.Thread(target=run_flask, daemon=True).start()

        # Setup the Turtle window on the Main Thread
        screen = turtle.Screen()
        screen.title("Simulation")

        check_queue()

        turtle.mainloop()
    else:
        # Run Flask on the main thread (blocking). Turtle GUI is disabled.
        print("RUN_TURTLE disabled; running Flask in main thread (no Turtle).")
        run_flask()
