r"""Temporary, Pi-compatible CV API using the existing best.pt detection model.

Run from Algo: .\.venv-cv-win\Scripts\python.exe -u cv_trial_server.py
This is separate from the Algo server and does not modify the Pi controller.
"""

import argparse
import io
import logging
from pathlib import Path
import threading
import time
import uuid

from flask import Flask, jsonify, request, send_file
from PIL import Image, ImageDraw, ImageFont, ImageOps, UnidentifiedImageError


LOGGER = logging.getLogger("cv-trial")
IMAGE_LABELS = {
    **{number + 10: str(number) for number in range(1, 10)},
    **dict(zip(range(20, 36), "ABCDEFGHSTUVWXYZ")),
    36: "UP", 37: "DOWN", 38: "RIGHT", 39: "LEFT", 40: "CIRCLE",
}


def validate_names(names):
    """The available best.pt names are already numeric MDP image IDs.

    Do NOT apply the missing ONNX model's class-index mapping to this model.
    """
    values = names.values() if isinstance(names, dict) else names
    for name in values:
        value = str(name)
        if value in ("NA", "10", "BULLSEYE"):
            continue
        if not value.isdigit() or int(value) not in IMAGE_LABELS:
            raise ValueError(f"Unsupported model class label: {name!r}")


def select_target(result, names, minimum_confidence):
    candidates = []
    if result.boxes is None:
        return None
    for box in result.boxes:
        name = str(names[int(box.cls[0])])
        if not name.isdigit() or int(name) not in IMAGE_LABELS:
            continue  # Ignore the bullseye/NA class, not a scored target.
        confidence = float(box.conf[0])
        if confidence < minimum_confidence:
            continue
        xyxy = [float(value) for value in box.xyxy[0].tolist()]
        area = max(0, xyxy[2] - xyxy[0]) * max(0, xyxy[3] - xyxy[1])
        candidates.append((area, confidence, int(name), xyxy))
    return max(candidates, key=lambda item: (item[0], item[1])) if candidates else None


def annotate(image, target):
    output = image.copy()
    draw = ImageDraw.Draw(output)
    _, confidence, image_id, xyxy = target
    width = max(3, round(max(image.size) / 300))
    draw.rectangle(xyxy, outline="lime", width=width)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", max(20, round(image.width / 45)))
    except OSError:
        font = ImageFont.load_default()
    label = f"{IMAGE_LABELS[image_id]} / Image id {image_id} ({confidence:.2f})"
    text_box = draw.textbbox((10, 10), label, font=font)
    draw.rectangle((0, 0, text_box[2] + 10, text_box[3] + 10), fill="black")
    draw.text((10, 10), label, fill="lime", font=font)
    return output


def create_app(model, model_path, output_dir, confidence=0.30):
    validate_names(model.names)
    if getattr(model, "task", None) != "detect":
        raise ValueError("This temporary server requires the best.pt detection model")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    model_lock = threading.Lock()
    run_captures = {}
    current_run = ['legacy']
    app = Flask(__name__)
    app.config["MAX_CONTENT_LENGTH"] = 16 * 1024 * 1024

    @app.get("/health")
    @app.get("/status")
    def health():
        return jsonify(status="healthy", model=str(model_path), temporary=True,
                       task=model.task, total_classes=len(model.names))

    @app.post("/predict")
    def predict():
        upload = request.files.get("image")
        if upload is None:
            return jsonify(error="Upload the image using multipart field 'image'"), 400
        try:
            with Image.open(io.BytesIO(upload.read())) as source:
                image = ImageOps.exif_transpose(source).convert("RGB")
        except (UnidentifiedImageError, OSError, ValueError, Image.DecompressionBombError):
            return jsonify(error="Invalid image"), 400

        started = time.monotonic()
        run_id = request.form.get('run_id', 'legacy')
        try:
            with model_lock:
                result = model.predict(image, imgsz=640, conf=confidence, verbose=False)[0]
            target = select_target(result, model.names, confidence)
            # Ignore client filenames when constructing local file paths.
            capture_id = f"{time.time_ns()}_{uuid.uuid4().hex}"
            # Branch switches/cleanup may remove generated output while we run.
            output_dir.mkdir(parents=True, exist_ok=True)
            image.save(output_dir / f"{capture_id}_original.jpg", quality=95)
            with model_lock:
                current_run[0] = run_id
                run_captures.setdefault(run_id, {})
            if target is None:
                LOGGER.info("No target detected in %s", upload.filename)
                return jsonify(image_id="NA", obstacle_id="NA", conf=0.0,
                               xyxy=None, bullseye=None)

            _, score, image_id, xyxy = target
            annotated_path = output_dir / f"{capture_id}_annotated.jpg"
            annotate(image, target).save(annotated_path, quality=95)
            with model_lock:
                run_captures[run_id][image_id] = annotated_path
            LOGGER.info("%s -> %s, image ID %s, confidence %.3f, %.2fs",
                        upload.filename, IMAGE_LABELS[image_id], image_id,
                        score, time.monotonic() - started)
            # The original Pi API unfortunately calls the numeric image ID
            # 'obstacle_id'. The Pi adds the actual obstacle number separately.
            return jsonify(image_id=IMAGE_LABELS[image_id], obstacle_id=str(image_id),
                           conf=score, xyxy=xyxy, bullseye=None)
        except Exception:
            LOGGER.exception("Recognition failed")
            return jsonify(error="Recognition failed; see CV server log"), 500

    @app.get("/stitch")
    def stitch():
        cutoff = time.time() - 20 * 60
        with model_lock:
            run_id = request.args.get('run_id', current_run[0])
            candidates = list(run_captures.get(run_id, {}).values())
        paths = sorted(path for path in candidates if path.exists() and path.stat().st_mtime >= cutoff)
        if not paths:
            return jsonify(error="No recognized captures for this run in the last 20 minutes"), 404
        tiles = []
        for path in paths:
            with Image.open(path) as source:
                tile = source.convert("RGB")
            tile.thumbnail((480, 360))
            tiles.append(tile)
        columns = min(3, len(tiles))
        canvas = Image.new("RGB", (480 * columns, 360 * ((len(tiles) + columns - 1) // columns)))
        for index, tile in enumerate(tiles):
            canvas.paste(tile, ((index % columns) * 480, (index // columns) * 360))
        buffer = io.BytesIO()
        canvas.save(buffer, format="JPEG", quality=95)
        buffer.seek(0)
        return send_file(buffer, mimetype="image/jpeg")

    return app


def main():
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=root / "best.pt")
    parser.add_argument("--output", type=Path, default=root.parent / "output" / "cv-trial")
    parser.add_argument("--port", type=int, default=8001)
    args = parser.parse_args()
    if not args.model.is_file():
        parser.error(f"Model file not found: {args.model}")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    from ultralytics import YOLO
    model = YOLO(str(args.model), task="detect")
    app = create_app(model, args.model.resolve(), args.output)
    LOGGER.info("TEMPORARY model: %s; outputs: %s", args.model, args.output)
    app.run(host="0.0.0.0", port=args.port, debug=False, use_reloader=False)


if __name__ == "__main__":
    main()
