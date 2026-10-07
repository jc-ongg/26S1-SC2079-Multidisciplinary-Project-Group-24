# Task 1 CV using best.pt

The root launcher starts `cv_trial_server.py` on port 8001 using the existing
`Algo/best.pt` detection model. The model and 0.30 confidence threshold are unchanged
by the movement update. ONNX class-index mappings do not apply to this model;
its class names already contain numeric MDP image IDs. Bullseye/NA is ignored.

- `GET /health`: model and readiness.
- `POST /predict`: multipart `image` and `run_id`. Returns the symbol as `image_id`
  and its numeric image ID in the historically named `obstacle_id` field.
- `GET /stitch`: current run's tiled boxed detections, at most one per image ID,
  from the last 20 minutes. `?run_id=...` selects an earlier in-memory run.

Windows image directory, from the repository root: **`output/cv-trial/`**.
`*_original.jpg` are raw camera frames, `*_annotated.jpg` contain boxes/IDs.
A successful stitch request also saves **`stitched-<run hash>.jpg`** there.
The run hash prevents client run IDs from becoming arbitrary file paths.
Open `http://127.0.0.1:8001/stitch` on the laptop and save the assessment screenshot.
No recognized current-run captures gives HTTP 404. Restarting CV clears its
in-memory run grouping; existing files and saved montages remain on disk.

Pi captures: `/home/nat/SC2079-MDP/rpi/captures/`, with separate retry files.
Historical root captures are preserved under `captures/archive-pre-grid5/`
when the Pi cleanup is applied. Camera framing and inference are unchanged.

See [current test procedure](../docs/TASK1_GRID5_TEST.md) and
[assessment checklist](../docs/TASK1_RUN_GUIDE_2026-10-07.md).
