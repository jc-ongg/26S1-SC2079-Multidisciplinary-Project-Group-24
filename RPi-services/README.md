# Laptop services for Task 1

Task 1 runs on three machines: the tablet sends the map over Bluetooth to the
Pi, the Pi asks a **laptop** for the path and for image recognition, and drives
the STM. This folder is the laptop half. Any team laptop can be the one.

```
Algo server   :8000   path planning           (../Algo, POST /path)
image-api     :8001   image recognition       (image-api/, POST /predict)
                      tiled RAW display       (GET /stitch -- Task 1 needs it to qualify)
```

## Set up a laptop (once)

Needs Python 3.12 or 3.13 (or [uv](https://docs.astral.sh/uv/), which fetches
Python itself and installs much faster), and the
[GitHub CLI](https://cli.github.com/) logged in (`gh auth login`) to fetch the
model, since the repo is private.

```bash
cd RPi-services
python laptop_setup.py
```

This builds `Algo/.venv` and `image-api/venv` with the **exact package versions**
from the laptop that ran Task 1 3/3, and downloads `best_v11i_m.onnx` (109 MB,
too big for git) from the
[cv-model-v11i release](https://github.com/jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24/releases/tag/cv-model-v11i),
checking its checksum. Without `gh`, download it from that page into
`image-api/` by hand and run setup again.

## Every session

```bash
python laptop_start.py       # starts both servers, prints this laptop's IP
python laptop_verify.py      # in another terminal: proves it matches the 3/3 run
```

`laptop_verify.py` replays recorded inputs from the 3/3 run (`golden/`) and
checks the answers are the same: two map layouts the tablet sent, against the
exact commands the robot received; the three captures from that run (right
arrow 38, S 28, 2 12) plus a bullseye face; and `/stitch`. If it prints
`ALL MATCH`, this laptop behaves like the one that ran 3/3.

Then point the Pi at this laptop (`laptop_start.py` prints the line to use):

```bash
ssh -t nat@192.168.50.5 'cd ~/SC2079-MDP/rpi && LAPTOP_IP=192.168.50.xxx ./run_week8.sh'
```

The laptop must be on the robot's hotspot (`MDP-group-24`) for the whole run.
Turn off auto-join for other WiFi networks; if the laptop roams off mid-run,
path planning and recognition both disappear.

After the run, open `http://127.0.0.1:8001/stitch` for the tiled captures to
show the supervisor.

## Which model

Recognition uses `best_v11i_m.onnx` (YOLOv8m-seg) through image-api. The Algo
server can also do recognition with `Algo/best.pt` via `/infer`, but that model
read arrows and digits worse and is **not** used: `final_week_8.py` sends
photos to image-api's `/predict`. Keep `task='segment'` in image-api's model
load, or ultralytics returns junk class ids.

## image-api

- `POST /predict` (field `image`) -> `{image_id, obstacle_id, conf, xyxy, bullseye}`.
  `image_id` is the character, `obstacle_id` is (confusingly) its numeric image id.
  A bullseye face comes back as `image_id: "NA"` with `bullseye` set, so it is
  never reported as a target. HTTP 404 means no objects in the frame at all.
- `GET /stitch` tiles the newest annotated capture of each recognised image
  from the last 20 minutes.
- Annotated captures are saved to `image-api/own_results/`.

## viewer (optional)

Live camera monitor with single-shot detection, not needed for a run.

```bash
cd viewer && npm install
PI_HOST=http://192.168.50.5:5000 API_HOST=http://localhost:8001 npm run dev
```

## The Pi side

`../RPi/` -- `final_week_8.py` and the `communication/` package. Copy it to
`~/SC2079-MDP/rpi/` on the Pi. See `../RPi/README.md`.
