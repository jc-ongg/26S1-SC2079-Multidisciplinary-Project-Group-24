import time
from datetime import datetime
from algo.algo import PathFinder 
from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from model import *
from helper import command_generator
import os
from car_animation import run_turtle_sim
import queue
import threading
import turtle
from ultralytics import YOLO
from PIL import Image, ImageOps
import io
from threading import Lock
import ast
import consts

# Image recognition is NOT done here -- it runs on image-api (:8001) with
# best_v11i_m.onnx, which reads the arrows and digits more reliably. This server
# is for path planning only.
#
# The weights are therefore optional: if best.pt is absent the server still
# starts and /path works normally, and /infer answers with a clear pointer
# instead of a stack trace. Loading is lazy so a missing file costs nothing.
MODEL_PATH = "./best.pt"
IMAGE_API_HINT = "image recognition is served by image-api on :8001 (/predict)"
yolo_model = None


def get_yolo_model():
    """Load best.pt on first use, or return None if it is not present."""
    global yolo_model
    if yolo_model is None and os.path.exists(MODEL_PATH):
        yolo_model = YOLO(MODEL_PATH)
    return yolo_model

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
    return jsonify({"status": "ok"})

def convert_input(request):
    # Example : STATUS, RETRY_FALSE, OBSTACLE LIST, 4, 1; 11; 12; UP | 2; 4; 7; UP | 3; 14; 6; UP | 4; 6; 13; UP
    face_map = {
            "NORTH": 0,
            "EAST": 2,
            "SOUTH": 4,
            "WEST": 6
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
    return jsonify({
        "data": {
            'distance': distance,
            'path': path_results,
            'commands': commands,
            # 'stm_commands': unadjusted_stm_commands
            'stm_commands': stm_commands
        },
        "error": None
        })
        #stm_commands)


@app.route('/infer', methods=['POST'])
def image_predict():
    """
    Receive image file, save it to disk,
    run YOLO inference,
    return detection results only.
    """

    t0 = time.time()

    if 'file' not in request.files:
        return jsonify({"ok": False, "error": "No file uploaded"}), 400

    try:
        file = request.files['file']
        filename = file.filename
        img_bytes = file.read()
        img = Image.open(io.BytesIO(img_bytes))
        img = ImageOps.exif_transpose(img).convert("RGB")

        # Save original image with timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name, ext = os.path.splitext(filename)
        timestamped_filename = f"{name}_{timestamp}{ext}"
        save_path = os.path.join(UPLOAD_FOLDER, timestamped_filename)
        img.save(save_path)

    except Exception as e:
        return jsonify({"ok": False, "error": f"Invalid image: {e}"}), 400

    # =========================
    # CLASS ID MAPPING
    # =========================
    CLS_MAP = {
        0:  (20, 'A'),
        1:  (21, 'B'),
        # 2: Bullseye -> ignored
        3:  (22, 'C'),
        4:  (23, 'D'),
        5:  (24, 'E'),
        6:  (25, 'F'),
        7:  (26, 'G'),
        8:  (27, 'H'),
        9:  (28, 'S'),
        10: (29, 'T'),
        11: (30, 'U'),
        12: (31, 'V'),
        13: (32, 'W'),
        14: (33, 'X'),
        15: (34, 'Y'),
        16: (35, 'Z'),
        17: (40, 'stop'),
        18: (37, 'down'),
        19: (18, '8'),
        20: (15, '5'),
        21: (14, '4'),
        22: (39, 'left'),
        23: (19, '9'),
        24: (11, '1'),
        25: (38, 'right'),
        26: (17, '7'),
        27: (16, '6'),
        28: (13, '3'),
        29: (12, '2'),
        30: (36, 'up'),
    }

    # =========================
    # YOLO INFERENCE
    # =========================
    model = get_yolo_model()
    if model is None:
        return jsonify({
            "ok": False,
            "error": f"/infer is disabled on this server -- {IMAGE_API_HINT}",
        }), 503

    result = model.predict(img, imgsz=640, conf=0.30, verbose=False)[0]

    # Save annotated image (with bounding boxes)
    annotated_array = result.plot()  # numpy array (BGR)
    annotated_img = Image.fromarray(annotated_array[..., ::-1])  # BGR -> RGB
    annotated_filename = f"{name}_{timestamp}_annotated{ext}"
    annotated_save_path = os.path.join(UPLOAD_FOLDER, annotated_filename)
    annotated_img.save(annotated_save_path)

    detections = []

    if result.boxes is not None and len(result.boxes) > 0:
        for box in result.boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            
            # Skip if not in CLS_MAP (e.g. Bullseye)
            if cls_id not in CLS_MAP:
                continue

            # Map to CLS_MAP
            mapped_id, label = CLS_MAP[cls_id]

            # Checklist A.2 wants a bounding rectangle drawn around the detected
            # image on the PC, so the box coordinates have to leave the server.
            xyxy = [round(float(v), 1) for v in box.xyxy[0].tolist()]

            detections.append({
                #"cls_id": cls_id,
                "cls_id": mapped_id,
                # "cls_name": (
                #     names.get(cls_id, str(cls_id))
                #     if isinstance(names, dict)
                #     else str(cls_id)
                # ),
                "cls_name": label,
                #"mapped_id": mapped_id,
                #"label": label,
                "conf": round(conf, 4),
                "xyxy": xyxy
            })



    latency = round((time.time() - t0) * 1000, 1)
    print ("detections:", detections)
    return jsonify({
        "ok": True,
        "n": len(detections),
        "latency_ms": latency,
        "detections": detections
    })
    
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