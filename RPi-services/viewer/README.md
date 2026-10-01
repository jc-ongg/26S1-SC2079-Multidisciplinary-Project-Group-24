# MDP Group 6 — Vision viewer

Small Vite + React app that shows the Raspberry Pi camera stream and runs frames
through the YOLO image API.

## What it does

- Displays the live MJPEG stream from the Pi (`videostream/main.py`, port 5000)
- "Capture & detect" freezes the current frame and POSTs it to `image-api` `/predict`
- Draws the returned bounding box over the captured frame and shows the symbol,
  obstacle id and confidence
- Optional auto-capture every 2 s, plus a rolling log of recent detections

## Running

Three things need to be up.

**1. Stream on the Pi**

```bash
ssh natrpi
cd ~/SC2079-MDP/videostream && python3 main.py     # serves :5000
```

**2. Image API on this machine**

```bash
cd ../image-api
./venv/bin/python main.py                          # serves :8000
```

**3. This app**

```bash
npm install
npm run dev
```

Then open the URL Vite prints (http://localhost:5173, or the next free port).

## Pointing at different hosts

Both backends are reached through the Vite dev proxy, so the browser only ever
talks to one origin — no CORS setup, and the canvas isn't tainted when grabbing
a frame. Defaults live in `vite.config.js`; override per-run:

```bash
PI_HOST=http://192.168.50.5:5000 API_HOST=http://localhost:8000 npm run dev
```

Use the `192.168.50.x` address when you're on the Pi's own `MDP-group-6` hotspot
rather than the lab LAN.

## Response shapes

`image-api/main.py` has a `REACT_MODE` flag:

- `REACT_MODE = False` (current) — returns the single best guess,
  `{ image_id, obstacle_id, conf, xyxy }`, or `{ image_id: "NA" }`
- `REACT_MODE = True` — returns every box under `{ results: [...] }`

The app handles both, so flipping the flag needs no frontend change.
