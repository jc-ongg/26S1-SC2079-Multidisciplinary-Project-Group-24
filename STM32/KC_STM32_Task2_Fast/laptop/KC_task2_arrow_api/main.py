"""Task2-only API compatible with the existing native STM32 Pi bridge."""
from contextlib import asynccontextmanager
from pathlib import Path
from threading import Lock
import hashlib
import io
import json
import logging
import time
import uuid

from fastapi import FastAPI, File, HTTPException, UploadFile
from PIL import Image, UnidentifiedImageError
from starlette.concurrency import run_in_threadpool

PIL_OPEN = Image.open  # Retain Pillow decoder before model-library plugin patching.
LOGGER = logging.getLogger(__name__)

from model_loader import load_model

ROOT = Path(__file__).resolve().parent
MODEL = ROOT / 'models' / 'best_yolo11m_arrows.pt'
RESULTS = ROOT / 'own_results'
EXPECTED_SHA256 = '5098030850b53b7a49ae7ef6ce3942e25d4863f5308dd71c814c7cd4330fd821'
MIN_CONFIDENCE = .50
INFERENCE_CONFIDENCE = .10
MAX_UPLOAD_BYTES = 10 * 1024 * 1024
MODEL_ID = 'PranavU-Coder/arrow_detection:335c0dc275769edd902c5bb41a99992fa12fff9c'
LOCK = Lock()


def select_arrow(detections):
    """Confidence wins; a low-score full-frame box cannot displace the arrow."""
    arrows = [d for d in detections if d['class'] in ('left', 'right')]
    candidate = max(arrows, key=lambda d: d['confidence'], default=None)
    accepted = candidate if candidate and candidate['confidence'] >= MIN_CONFIDENCE else None
    return {
        'image_id': accepted['class'].upper() if accepted else 'NA',
        'obstacle_id': {'left': 39, 'right': 38}[accepted['class']] if accepted else 'NA',
        'conf': accepted['confidence'] if accepted else 0.0,
        'xyxy': accepted['xyxy'] if accepted else [],
        'candidate': candidate,
        'detections': detections,
        'min_confidence': MIN_CONFIDENCE,
        'backend': 'task2-yolo11m-arrows',
        'model': MODEL.name,
    }


@asynccontextmanager
async def lifespan(app):
    actual = hashlib.sha256(MODEL.read_bytes()).hexdigest()
    if actual != EXPECTED_SHA256:
        raise RuntimeError('Arrow model checksum mismatch')
    app.state.model = load_model(MODEL)
    app.state.model.predict(Image.new('RGB', (640, 480)), conf=INFERENCE_CONFIDENCE,
                            imgsz=640, device='cpu', verbose=False)
    RESULTS.mkdir(exist_ok=True)
    yield
    app.state.model = None


app = FastAPI(title='KC Task2 arrow API', lifespan=lifespan)


@app.get('/health')
def health():
    return {'status': 'healthy', 'task': 'task2', 'model': MODEL.name,
            'model_source': MODEL_ID, 'sha256': EXPECTED_SHA256,
            'total_classes': 2, 'classes': ['left', 'right'],
            'min_confidence': MIN_CONFIDENCE, 'mode': 'LEGACY_COMPATIBLE'}


def infer(contents):
    try:
        with PIL_OPEN(io.BytesIO(contents)) as source:
            suffix = {'JPEG': '.jpg', 'PNG': '.png'}.get(source.format, '.img')
            if source.width * source.height > 20_000_000:
                raise HTTPException(400, 'Image too large')
            photo = source.convert('RGB')
    except (UnidentifiedImageError, OSError, Image.DecompressionBombError):
        raise HTTPException(400, 'Cannot decode image')
    stamp = str(time.time_ns()) + '_' + uuid.uuid4().hex[:8]
    original = RESULTS / (stamp + '_original' + suffix)
    original.write_bytes(contents)
    with LOCK:
        start = time.perf_counter()
        try:
            predictions = app.state.model.predict(photo, conf=INFERENCE_CONFIDENCE,
                imgsz=640, device='cpu', verbose=False)
            detections = []
            for result in predictions:
                for box in result.boxes:
                    detections.append({'class': str(result.names[int(box.cls.item())]).lower(),
                        'confidence': float(box.conf.item()),
                        'xyxy': [float(v) for v in box.xyxy[0]]})
            response = select_arrow(detections)
            response['inference_seconds'] = time.perf_counter() - start
            response['capture_id'] = stamp
            (RESULTS / (stamp + '_result.json')).write_text(json.dumps(response, indent=2))
            return response
        except Exception as exc:
            LOGGER.exception('Arrow inference failed for %s', stamp)
            (RESULTS / (stamp + '_result.json')).write_text(json.dumps(
                {'error': type(exc).__name__, 'capture_id': stamp}))
            raise HTTPException(500, 'Arrow inference failed; inspect server log') from exc


@app.post('/predict')
async def predict(image: UploadFile = File(...)):
    if not image.content_type or not image.content_type.startswith('image/'):
        raise HTTPException(400, 'File must be an image')
    contents = await image.read(MAX_UPLOAD_BYTES + 1)
    if len(contents) > MAX_UPLOAD_BYTES:
        raise HTTPException(413, 'Image exceeds 10 MiB')
    return await run_in_threadpool(infer, contents)


if __name__ == '__main__':
    import argparse
    import uvicorn
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8001)
    args = parser.parse_args()
    uvicorn.run(app, host=args.host, port=args.port)
