"""Task 1 ONNX recognition, local capture evidence, and per-run stitching."""
import hashlib
import io
import json
import logging
import math
import os
from pathlib import Path
import threading
import time
import uuid
from contextlib import asynccontextmanager

import cv2
import numpy as np
from PIL import Image, ImageOps, UnidentifiedImageError
from fastapi import FastAPI, File, Form, HTTPException, UploadFile
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, Response
from fastapi.staticfiles import StaticFiles

logger = logging.getLogger(__name__)
ROOT = Path(__file__).resolve().parent
MODEL_NAME = 'best_v11i_m.onnx'
MODEL_PATH = ROOT / MODEL_NAME
OUTPUT_DIR = ROOT / 'own_results'
CONFIDENCE = float(os.environ.get('CV_CONFIDENCE', '0.25'))
if not 0 < CONFIDENCE <= 1:
    raise ValueError('CV_CONFIDENCE must be in (0, 1]')
CLASS_NAMES = {
    0: '1', 1: '2', 2: '3', 3: '4', 4: '5', 5: '6', 6: '7', 7: '8', 8: '9',
    9: 'A', 10: 'B', 11: 'BULLSEYE', 12: 'C', 13: 'CIRCLE', 14: 'D',
    15: 'DOWN', 16: 'E', 17: 'F', 18: 'G', 19: 'H', 20: 'LEFT',
    21: 'RIGHT', 22: 'S', 23: 'T', 24: 'U', 25: 'UP',
    26: 'V', 27: 'W', 28: 'X', 29: 'Y', 30: 'Z',
}
NAME_TO_ID = {
    **{str(n): n + 10 for n in range(1, 10)},
    **dict(zip('ABCDEFGHSTUVWXYZ', range(20, 36))),
    'BULLSEYE': 10, 'UP': 36, 'DOWN': 37, 'RIGHT': 38, 'LEFT': 39, 'CIRCLE': 40,
}


def collect_detections(results):
    detections = []
    bullseye = None
    for result in results:
        if result.boxes is None:
            continue
        for box in result.boxes:
            name = CLASS_NAMES.get(int(box.cls[0]))
            score = float(box.conf[0])
            xyxy = [float(value) for value in box.xyxy[0].tolist()]
            if name is None or not all(math.isfinite(value) for value in [score, *xyxy]):
                continue
            if not CONFIDENCE <= score <= 1 or xyxy[2] <= xyxy[0] or xyxy[3] <= xyxy[1]:
                continue
            detection = {'cls_id': NAME_TO_ID[name], 'cls_name': name,
                         'conf': score, 'xyxy': xyxy}
            if name == 'BULLSEYE':
                if bullseye is None or score > bullseye['conf']:
                    bullseye = {'conf': score, 'xyxy': xyxy}
            else:
                detections.append(detection)
    return detections, bullseye


def select_target(detections, signal, image_width):
    if not detections:
        return None
    area = lambda item: ((item['xyxy'][2] - item['xyxy'][0]) *
                         (item['xyxy'][3] - item['xyxy'][1]))
    largest = max(area(item) for item in detections)
    # Comparable foreground boxes; the same confidence policy applies whether
    # one or several symbols are visible, avoiding the old shortlist crash.
    candidates = [item for item in detections
                  if area(item) >= largest * (0.6 if item['cls_name'] == '1' else 0.8)]
    centre = lambda item: (item['xyxy'][0] + item['xyxy'][2]) / 2
    if signal == 'L':
        return min(candidates, key=centre)
    if signal == 'R':
        return max(candidates, key=centre)
    return min(candidates, key=lambda item: (abs(centre(item) - image_width / 2), -item['conf']))


def run_key(run_id):
    return hashlib.sha256(run_id.encode('utf-8')).hexdigest()[:12]


def create_app(model=None, output_dir=OUTPUT_DIR):
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    model_lock = threading.Lock()

    @asynccontextmanager
    async def lifespan(api):
        if model is None:
            # Load ONNX Runtime before health becomes ready, rather than making
            # the first robot photo pay the cold-start cost inside its timeout.
            with model_lock:
                get_model().predict(Image.new('RGB', (640, 640)), conf=CONFIDENCE, verbose=False)
        yield

    api = FastAPI(title='Task 1 recognition API', version='2.0', lifespan=lifespan)
    api.add_middleware(CORSMiddleware, allow_origins=['*'], allow_credentials=True,
                       allow_methods=['*'], allow_headers=['*'])
    api.mount('/files', StaticFiles(directory=str(output_dir)), name='capture-files')

    def get_model():
        nonlocal model
        if model is None:
            if not MODEL_PATH.is_file():
                raise RuntimeError(f'Model missing: {MODEL_PATH}')
            from ultralytics import YOLO
            # This export has mask coefficients; task='detect' misreads them.
            model = YOLO(str(MODEL_PATH), task='segment')
        return model

    def save_record(record):
        path = output_dir / (record['capture_id'] + '_result.json')
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(record, indent=2), encoding='utf-8')
        temporary.replace(path)

    def records(run_id=None):
        pattern = f'{run_key(run_id)}_*_result.json' if run_id is not None else '*_result.json'
        found = []
        for path in output_dir.glob(pattern):
            try:
                record = json.loads(path.read_text(encoding='utf-8'))
                if run_id is None or record.get('run_id') == run_id:
                    found.append(record)
            except (OSError, ValueError):
                continue
        return sorted(found, key=lambda record: record['captured_at'])

    def selected_records(run_id):
        found = records(run_id)
        if run_id is None and found:
            latest_run = found[-1]['run_id']
            found = [record for record in found if record['run_id'] == latest_run]
        return found

    @api.post('/predict')
    async def predict(image: UploadFile = File(...), run_id: str = Form('legacy'),
                      target_obstacle_id: str = Form(''), signal: str = Form('C'),
                      attempt: str = Form('')):
        signal = signal.upper()
        if signal not in ('L', 'C', 'R'):
            raise HTTPException(400, 'signal must be L, C, or R')
        captured_at = time.time()
        capture_id = f'{run_key(run_id)}_{time.time_ns()}_{uuid.uuid4().hex}'
        raw_name = capture_id + '_original.jpg'
        record = {
            'capture_id': capture_id, 'captured_at': captured_at, 'run_id': run_id,
            'target_obstacle_id': target_obstacle_id, 'attempt': attempt, 'signal': signal,
            'model': MODEL_NAME, 'status': 'not_detected', 'image_id': 'NA',
            # obstacle_id is retained as the legacy *image number* field.
            'obstacle_id': 'NA', 'mdp_image_id': None, 'accepted': False,
            'conf': 0.0, 'xyxy': None, 'bullseye': None, 'detections': [],
            'raw_file': raw_name, 'raw_url': '/files/' + raw_name,
            'annotated_file': None, 'annotated_url': None,
        }
        try:
            contents = await image.read()
            if len(contents) > 16 * 1024 * 1024:
                raise ValueError('Image exceeds 16 MB')
            with Image.open(io.BytesIO(contents)) as source:
                frame = ImageOps.exif_transpose(source).convert('RGB')
            # Save every valid photo BEFORE inference, including NA/errors.
            frame.save(output_dir / raw_name, quality=95)
        except (ValueError, OSError, UnidentifiedImageError) as exc:
            raise HTTPException(400, f'Invalid image: {exc}') from exc

        try:
            with model_lock:
                # PIL input lets Ultralytics perform its own RGB/BGR handling.
                results = get_model().predict(frame, conf=CONFIDENCE, verbose=False)
            detections, bullseye = collect_detections(results)
            target = select_target(detections, signal, frame.width)
            record.update(bullseye=bullseye, detections=detections)
            if target is not None:
                number = target['cls_id']
                record.update(status='recognized', accepted=True, mdp_image_id=number,
                              obstacle_id=str(number), image_id=target['cls_name'],
                              conf=target['conf'], xyxy=target['xyxy'])
                record['detections'] = [target] + [item for item in detections if item is not target]
                annotated = np.array(frame)
                x1, y1, x2, y2 = map(int, target['xyxy'])
                thickness = max(3, round(max(frame.size) / 300))
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), thickness)
                label = f"{target['cls_name']} / Image id {number} ({target['conf']:.2f})"
                cv2.putText(annotated, label, (10, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            max(0.7, frame.width / 1000), (0, 255, 0), thickness, cv2.LINE_AA)
                annotated_name = capture_id + '_annotated.jpg'
                Image.fromarray(annotated).save(output_dir / annotated_name, quality=95)
                record.update(annotated_file=annotated_name, annotated_url='/files/' + annotated_name)
            save_record(record)
            logger.info('Run %s obstacle %s -> %s (image %s)', run_id,
                        target_obstacle_id, record['image_id'], record['mdp_image_id'])
            return record
        except Exception as exc:
            logger.exception('Recognition failed for %s', capture_id)
            record.update(status='error', accepted=False, image_id='NA', obstacle_id='NA',
                          mdp_image_id=None, error=str(exc))
            save_record(record)
            return JSONResponse(record, status_code=500)

    @api.get('/results')
    def capture_results(run_id: str = None):
        return {'model': MODEL_NAME, 'output_dir': str(output_dir),
                'captures': selected_records(run_id)}

    @api.get('/stitch')
    def stitch(run_id: str = None):
        newest = {}
        found = selected_records(run_id)
        for record in found:
            if not record['accepted'] or not record.get('annotated_file'):
                continue
            key = ('obstacle', record['target_obstacle_id']) if record['target_obstacle_id'] else ('image', record['mdp_image_id'])
            path = output_dir / record['annotated_file']
            if path.is_file():
                newest[key] = path
        # Preserve access to captures made before result manifests existed.
        if not newest and run_id is None and not found:
            for path in sorted(output_dir.glob('annotated_image_*.jpg')):
                newest[path.name.split('_')[2]] = path
        if not newest:
            return JSONResponse({'error': 'No recognized captures for this run'}, status_code=404)
        tiles = []
        for path in newest.values():
            with Image.open(path) as source:
                tile = source.convert('RGB')
            tile.thumbnail((480, 360))
            tiles.append(tile)
        columns = min(3, len(tiles))
        canvas = Image.new('RGB', (columns * 480, ((len(tiles) + columns - 1) // columns) * 360))
        for index, tile in enumerate(tiles):
            canvas.paste(tile, ((index % columns) * 480, (index // columns) * 360))
        actual_run = run_id
        if actual_run is None:
            actual_run = found[-1]['run_id'] if found else 'legacy'
        name = f'stitched-{run_key(actual_run)}.jpg'
        canvas.save(output_dir / name, quality=95)
        buffer = io.BytesIO()
        canvas.save(buffer, format='JPEG', quality=95)
        return Response(buffer.getvalue(), media_type='image/jpeg', headers={'X-Stitch-File': name})

    @api.get('/health')
    def health():
        return {'status': 'healthy', 'model': MODEL_NAME, 'task': 'segment',
                'total_classes': len(CLASS_NAMES), 'mode': 'LEGACY',
                'contract_version': 2, 'confidence': CONFIDENCE,
                'output_dir': str(output_dir)}

    return api


app = create_app()

if __name__ == '__main__':
    import uvicorn
    logging.basicConfig(level=logging.INFO)
    if not MODEL_PATH.is_file():
        raise SystemExit(f'Model missing: {MODEL_PATH}')
    uvicorn.run(app, host='0.0.0.0', port=int(os.environ.get('IMAGE_API_PORT', '8001')))
