# RPi services (laptop side)

Recognition and monitoring services that run on the **laptop**, not the Pi.
The laptop is the compute for Task 1: the Pi captures and relays, the laptop
plans paths and recognises images.

## image-api  (port 8001)

YOLO image recognition (`best_v11i_m.onnx`, YOLOv8-seg). Endpoints:

- `POST /predict`  field `image` -> `{image_id, obstacle_id, conf, xyxy, bullseye}`
- `GET  /stitch`   tiled RAW display of the run's annotated captures (Task 1 /
  FAQ 16 requirement) -- open in a browser after a run
- `GET  /health`

```bash
cd image-api
python3 -m venv venv && ./venv/bin/pip install -r requirements.txt   # first time
IMAGE_API_PORT=8001 ./venv/bin/python main.py
```

The bounding box is drawn thick with a legible side-panel label
("<char> / Image id <n>"), scaled to the capture resolution. The bullseye
marker is reported separately from the target so a blank obstacle face is never
mistaken for a real symbol.

## viewer  (port 5173)

Vite dev server: live MJPEG monitor + single-shot / continuous detection.
Proxies `/stream` -> Pi camera, `/api` -> image-api.

```bash
cd viewer
PI_HOST=http://<pi>:5000 API_HOST=http://localhost:8001 npm run dev
```

## Where the Pi orchestration lives

`../RPi/` -- `final_week_8.py` and the `communication/` package (STM protocol,
Bluetooth `//FRAME//` link). Copy it to `~/SC2079-MDP/rpi/` on the Pi. See
`../RPi/README.md`.
