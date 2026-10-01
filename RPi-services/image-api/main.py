import logging
import io
import json
import numpy as np
import glob
import torch
import os
import cv2
import time
import pandas as pd
from PIL import Image
from ultralytics import YOLO
from fastapi import FastAPI, File, UploadFile, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

MODEL_NAME = 'best_v11i_m.onnx'
REACT_MODE = False

# The bullseye marks an obstacle face without a target image. It is reported
# separately as "bullseye" rather than as the image_id; set this to allow it to
# be chosen as the target too.
BULLSEYE_AS_TARGET = os.environ.get("BULLSEYE_AS_TARGET", "0") == "1"

# Class mappings remain the same...
CLASS_NAMES = {
    0: '1', 1: '2', 2: '3', 3: '4', 4: '5', 5: '6', 6: '7', 7: '8', 8: '9',
    9: 'A', 10: 'B', 11: 'BULLSEYE', 12: 'C', 13: 'CIRCLE', 14: 'D',
    15: 'DOWN', 16: 'E', 17: 'F', 18: 'G', 19: 'H', 20: 'LEFT',
    21: 'RIGHT', 22: 'S', 23: 'T', 24: 'U', 25: 'UP',
    26: 'V', 27: 'W', 28: 'X', 29: 'Y', 30: 'Z'
}

NAME_TO_ID = {
    "NA": 'NA',
    "BULLSEYE": 10,
    "1": 11,
    "2": 12,
    "3": 13,
    "4": 14,
    "5": 15,
    "6": 16,
    "7": 17,
    "8": 18,
    "9": 19,
    "A": 20,
    "B": 21,
    "C": 22,
    "D": 23,
    "E": 24,
    "F": 25,
    "G": 26,
    "H": 27,
    "S": 28,
    "T": 29,
    "U": 30,
    "V": 31,
    "W": 32,
    "X": 33,
    "Y": 34,
    "Z": 35,
    "UP": 36,
    "DOWN": 37,
    "RIGHT": 38,
    "LEFT": 39,
    "Up Arrow": 36,
    "Down Arrow": 37,
    "Right Arrow": 38,
    "Left Arrow": 39,
    "CIRCLE": 40
}

# Ensure own_results directory exists
os.makedirs('own_results', exist_ok=True)

def convert_to_python_type(obj):
    """
    Convert numpy types to Python native types for JSON serialization
    """
    if isinstance(obj, np.integer):
        return int(obj)
    elif isinstance(obj, np.floating):
        return float(obj)
    elif isinstance(obj, np.ndarray):
        return obj.tolist()
    return obj

def draw_bbox(img, box, image_id, obstacle_id, confidence, color=(36,255,12), text_color=(0,0,0)):
    """
    Draw a bounding box and a legible label.

    The old version drew a 2px box and 0.6-scale text regardless of image size,
    so on a full-res capture (1640px+ wide) the box was a hairline and the label
    an unreadable smudge. Everything here scales with the image, and the label
    sits in a panel to the SIDE of the box -- like the assessment's own example
    screenshot -- so it never covers the character being shown.
    """
    h_img, w_img = img.shape[:2]
    x1, y1, x2, y2 = map(int, box['xyxy'])
    timestamp = str(int(time.time()))

    # Scale line weights and text to the image so they read at any resolution.
    scale = max(w_img, h_img) / 1000.0
    thick = max(3, int(round(4 * scale)))                 # box outline
    font = cv2.FONT_HERSHEY_SIMPLEX
    fs = max(0.8, 1.1 * scale)                            # font scale
    ft = max(2, int(round(2.5 * scale)))                 # font stroke

    cv2.rectangle(img, (x1, y1), (x2, y2), color, thick)

    # Match the assessment's own labelling ("Number 5 / Image id=15"):
    # image_id is the recognised character, obstacle_id is its numeric image id.
    lines = [f"{image_id}", f"Image id {obstacle_id}"]
    sizes = [cv2.getTextSize(t, font, fs, ft)[0] for t in lines]
    lw = max(w for w, _ in sizes)
    lh = max(h for _, h in sizes)
    pad = int(round(12 * scale))
    line_gap = lh + pad
    panel_w = lw + 2 * pad
    panel_h = line_gap * len(lines) + pad

    # Prefer the panel to the right of the box; fall back to the left, then
    # inside, so it is always fully on-screen.
    px = x2 + pad
    if px + panel_w > w_img:
        px = x1 - pad - panel_w
    if px < 0:
        px = min(x1 + pad, w_img - panel_w)
    py = max(0, min(y1, h_img - panel_h))

    cv2.rectangle(img, (px, py), (px + panel_w, py + panel_h), color, -1)
    for i, t in enumerate(lines):
        ty = py + pad + lh + i * line_gap
        cv2.putText(img, t, (px + pad, ty), font, fs, text_color, ft, cv2.LINE_AA)

    # High quality so the character stays sharp in the tiled display.
    cv2.imwrite(
        f"own_results/annotated_image_{image_id}-{obstacle_id}_{timestamp}.jpg",
        img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    return img

def process_legacy_mode(results, signal='C'):
    """
    Process results using the legacy mode logic
    """
    pred_list = []

    # Convert YOLO results to the format we need
    for result in results:
        boxes = result.boxes
        for box in boxes:
            cls_index = int(box.cls[0].item())
            class_name = CLASS_NAMES.get(cls_index, f"Unknown Class ({cls_index})")
            confidence = float(box.conf[0].item())  # Convert to Python float
            xyxy = box.xyxy[0].cpu().numpy()

            pred_list.append({
                'name': class_name,
                'confidence': confidence,
                'xmin': float(xyxy[0]),  # Convert to Python float
                'ymin': float(xyxy[1]),
                'xmax': float(xyxy[2]),
                'ymax': float(xyxy[3]),
                'bboxArea': float((xyxy[2] - xyxy[0]) * (xyxy[3] - xyxy[1]))
            })

    # Sort by area, then split the bullseye out of the target candidates.
    #
    # The bullseye is the marker that says "this is an obstacle", not a target
    # image, so it must not win the image_id -- reporting it would send a bogus
    # target to Android. It is still reported separately under "bullseye" so the
    # viewer can draw it and so checklist A.5 can tell "I am looking at a blank
    # face of the obstacle" apart from "I see nothing at all".
    #
    # Set BULLSEYE_AS_TARGET=1 to let it be selected as the image_id too.
    pred_list.sort(key=lambda x: x['bboxArea'], reverse=True)

    bullseyes = [x for x in pred_list if x['name'] == 'BULLSEYE']
    if not BULLSEYE_AS_TARGET:
        pred_list = [x for x in pred_list if x['name'] != 'BULLSEYE']

    bullseye_out = None
    if bullseyes:
        b = bullseyes[0]
        bullseye_out = {
            'conf': float(b['confidence']),
            'xyxy': [float(b['xmin']), float(b['ymin']),
                     float(b['xmax']), float(b['ymax'])],
        }

    pred = 'NA'

    if len(pred_list) == 1:
        pred = pred_list[0]
    elif len(pred_list) > 1:
        # Filter by confidence and area
        pred_shortlist = []
        current_area = pred_list[0]['bboxArea']

        for pred_item in pred_list:
            if (pred_item['confidence'] > 0.5 and
                ((current_area * 0.8 <= pred_item['bboxArea']) or
                 (pred_item['name'] == '1' and current_area * 0.6 <= pred_item['bboxArea']))):
                pred_shortlist.append(pred_item)
                current_area = pred_item['bboxArea']

        if len(pred_shortlist) == 1:
            pred = pred_shortlist[0]
        else:
            # Sort by x position
            pred_shortlist.sort(key=lambda x: x['xmin'])

            if signal == 'L':
                pred = pred_shortlist[0]
            elif signal == 'R':
                pred = pred_shortlist[-1]
            else:
                for item in pred_shortlist:
                    if 250 < item['xmin'] < 774:
                        pred = item
                        break

                if isinstance(pred, str):
                    pred_shortlist.sort(key=lambda x: x['bboxArea'])
                    pred = pred_shortlist[-1]

    if isinstance(pred, str):
        # No target image. The bullseye still goes out when one was seen, so the
        # caller can distinguish "obstacle, wrong face" from "nothing there".
        return {'image_id': 'NA', 'bullseye': bullseye_out}
    else:
        return {
            'obstacle_id': str(NAME_TO_ID.get(pred['name'], 'NA')),
            'conf': float(pred['confidence']),  # Convert to Python float
            'xyxy': [float(pred['xmin']), float(pred['ymin']),
                    float(pred['xmax']), float(pred['ymax'])],  # Convert to Python float
            'image_id': pred['name'],
            'bullseye': bullseye_out
        }

# Initialize FastAPI application
app = FastAPI(
    title="Object Detection API",
    description="YOLO-based object detection API",
    version="1.0.0"
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

try:
    # best_v11i_m.onnx is a YOLOv8-seg export; without an explicit task ultralytics
    # guesses 'detect' and reads the 32 mask coefficients as class scores, which
    # yields junk class indices and confidences above 1.0.
    model = YOLO(MODEL_NAME, task='segment')
    logger.info("Model loaded successfully")
except Exception as e:
    logger.error(f"Model loading failed: {e}")
    raise

@app.post("/predict")
async def predict(image: UploadFile = File(...)):
    """
    Endpoint for object detection prediction.
    """
    try:
        if not image.content_type.startswith('image/'):
            raise HTTPException(status_code=400, detail="File is not an image")

        contents = await image.read()
        pil_image = Image.open(io.BytesIO(contents)).convert("RGB")
        np_image = np.array(pil_image)

        try:
            results = model.predict(np_image, verbose=True)

            if REACT_MODE:
                processed_results = []

                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        cls_index = int(box.cls[0].item())
                        class_name = CLASS_NAMES.get(cls_index, f"Unknown Class ({cls_index})")
                        confidence = float(box.conf[0].item())  # Convert to Python float

                        processed_box = {
                            'xyxy': [float(x) for x in box.xyxy[0].cpu().numpy()],  # Convert to Python float
                            'conf': confidence,
                            'obstacle_id': cls_index,
                            'image_id': class_name,
                            'xywh': [float(x) for x in box.xywh[0].cpu().numpy()]  # Convert to Python float
                        }
                        processed_results.append(processed_box)

                if processed_results:
                    return JSONResponse(content={"results": processed_results})
            else:
                # Legacy mode processing
                result = process_legacy_mode(results)

                if result['image_id'] != 'NA':
                    # Draw bbox only in non-REACT mode
                    annotated_image = draw_bbox(
                        np_image.copy(),
                        {'xyxy': result['xyxy']},
                        result['image_id'],
                        result['obstacle_id'],
                        result['conf']
                    )

                return JSONResponse(content=result)

            return JSONResponse(
                content={
                    "error": "No objects detected",
                    "details": "The model did not identify any objects in the image"
                },
                status_code=404
            )

        except Exception as inference_error:
            logger.error(f"Inference failed: {inference_error}")
            raise

    except Exception as e:
        logger.error(f"Prediction failed: {e}")
        return JSONResponse(
            content={
                "error": "Prediction process failed",
                "details": str(e)
            },
            status_code=500
        )

@app.get("/stitch")
def stitch(minutes: int = 20):
    """Tile the newest annotated capture per recognised image, in one JPEG.

    Task 1 rules (FAQ 16): the team does not qualify unless the RAW captures,
    with bounding boxes, are shown tiled in one window at the end of the run.
    draw_bbox() already saves an annotated copy of every successful detection
    to own_results/, so this just collects the most recent one for each
    distinct image id from the last `minutes` and grids them.

    Open http://<this mac>:8001/stitch in a browser after the run.
    """
    import glob as _glob
    import time as _time

    cutoff = _time.time() - minutes * 60
    newest = {}
    for p in _glob.glob("own_results/annotated_image_*.jpg"):
        try:
            ts = os.path.getmtime(p)
        except OSError:
            continue
        if ts < cutoff:
            continue
        # filename: annotated_image_{char}-{imageid}_{timestamp}.jpg
        stem = os.path.basename(p)[len("annotated_image_"):]
        key = stem.split("_")[0]          # "{char}-{imageid}"
        if key not in newest or ts > newest[key][0]:
            newest[key] = (ts, p)

    if not newest:
        return JSONResponse(
            content={"error": f"no annotated captures in the last {minutes} minutes"},
            status_code=404)

    imgs = []
    for _, path in sorted(newest.values()):
        im = cv2.imread(path)
        if im is not None:
            imgs.append(im)
    if not imgs:
        return JSONResponse(content={"error": "captures unreadable"}, status_code=500)

    # Same height per tile, three tiles per row.
    tile_h = 360
    imgs = [cv2.resize(im, (int(im.shape[1] * tile_h / im.shape[0]), tile_h))
            for im in imgs]
    cols = 3
    rows = []
    for i in range(0, len(imgs), cols):
        row = imgs[i:i + cols]
        width = sum(im.shape[1] for im in row)
        strip = np.zeros((tile_h, width, 3), dtype=np.uint8)
        x = 0
        for im in row:
            strip[:, x:x + im.shape[1]] = im
            x += im.shape[1]
        rows.append(strip)
    width = max(r.shape[1] for r in rows)
    rows = [cv2.copyMakeBorder(r, 0, 0, 0, width - r.shape[1],
                               cv2.BORDER_CONSTANT, value=(0, 0, 0))
            for r in rows]
    canvas = cv2.vconcat(rows)

    out = f"own_results/stitched_{int(_time.time())}.jpg"
    cv2.imwrite(out, canvas)
    ok, buf = cv2.imencode(".jpg", canvas)
    from fastapi.responses import Response
    return Response(content=buf.tobytes(), media_type="image/jpeg")


@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "model": MODEL_NAME,
        "total_classes": len(CLASS_NAMES),
        "mode": "REACT" if REACT_MODE else "LEGACY"
    }

@app.get("/stitch")
def stitch_image_own():
    """
    Stitches the images in the folder together and saves it into own_results folder

    Basically similar to stitch_image() but with different folder names and slightly different drawing of bounding boxes and text
    """
    imgFolder = 'own_results'
    stitchedPath = os.path.join(imgFolder, f'stitched-{int(time.time())}.jpeg')

    imgPaths = glob.glob(os.path.join(imgFolder+"/annotated_image_*.jpg"))
    imgTimestamps = [imgPath.split("_")[-1][:-4] for imgPath in imgPaths]

    sortedByTimeStampImages = sorted(zip(imgPaths, imgTimestamps), key=lambda x: x[1])

    images = [Image.open(x[0]) for x in sortedByTimeStampImages]
    width, height = zip(*(i.size for i in images))
    total_width = sum(width)
    max_height = max(height)
    stitchedImg = Image.new('RGB', (total_width, max_height))
    x_offset = 0

    for im in images:
        stitchedImg.paste(im, (x_offset, 0))
        x_offset += im.size[0]
    stitchedImg.save(stitchedPath)

    return {"result":"ok"}

if __name__ == "__main__":
    import uvicorn
    import os
    # Port is overridable so this can run alongside the Algo server,
    # which owns 8000. IMAGE_API_PORT=8001 keeps both models available.
    uvicorn.run(app, host="0.0.0.0", port=int(os.environ.get("IMAGE_API_PORT", 8000)))
