# Laptop Algo + CV services

Algo is on port **8000**; CV is on **8001**. Both run on the laptop, not the Pi.
Only one CV backend should own 8001 at a time.

## Existing Windows environment

From the repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --check
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial
```

`--check` starts nothing. Normal launch refuses occupied ports, appends logs
under `RPi-services/logs/`, and Ctrl-C stops only its own child servers.
Stop existing servers yourself before using it. No Pi controller is launched.

## Optional calibration trials

The default command above retains original turns. To select the latest speed50
forward and reverse measurements, add both options:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --forward-calibration speed50-v2 --reverse-calibration speed50-v2
```

`GET /status` identifies the base profile, recommended speed and both overlays.
Each overlay can be rolled back independently with `original`. The rejected
`--movement-profile measured25` is preserved for recovery but cannot be mixed
with these overlays. See [calibration status](../Algo/CALIBRATION.md). Changing
files or making a Git commit does not reload a running server; no service is
restarted as part of repository tidying.

## Fresh temporary-CV environment

Do not copy virtual environments between computers/operating systems. The
checked local `Algo/.venv` was a macOS environment, not usable Windows setup.
Create a NEW environment if an existing one is broken; preserve it first.

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements-cv-trial.txt
```

The temporary detector uses the existing tracked `Algo/best.pt`.
It is explicitly labelled temporary by `GET /health`. Its class names are
numeric MDP image IDs; do not apply the original ONNX index mapping to it.
See [temporary CV details](../Algo/CV_TRIAL.md).

## Original ONNX backend

Expected file: `RPi-services/image-api/best_v11i_m.onnx`.
Expected SHA-256:
`8c4f382f800d123e069fe93f0069e279175594c0a6ef631c699fbc3d957ee7d5`.

The teammate's setup points to the private
[cv-model-v11i release](https://github.com/jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24/releases/tag/cv-model-v11i).
Release availability and model inference must be verified on your machine;
the consolidation did not download this model or validate ONNX predictions.

For a fresh original-backend setup, `laptop_setup.py` uses Python 3.12/3.13
(or uv), installs the teammate's locked packages and downloads about 109 MB.
This is opt-in, not part of `--check`. It refuses to overwrite a different
existing model and verifies a downloaded candidate before installing it.

```powershell
py -3.12 RPi-services/laptop_setup.py
py -3.12 RPi-services/laptop_start.py --cv-backend onnx
```

Do not run setup blindly over someone else's environments. You can also pass
`--python <existing compatible interpreter>` to the launcher.

`laptop_verify.py` compares two recorded map responses, original ONNX image
IDs/confidences and stitching. It is **not** an arena calibration or hardware
test. Temporary `best.pt` is not expected to reproduce ONNX confidences.

## Network and protocol

Keep laptop Wi-Fi on the Pi hotspot and phone USB tethering for internet.
Use the laptop's **192.168.50.x Wi-Fi address** in Pi settings.

- Algo: `GET /status`, `POST /path` with the text map payload.
- CV: `GET /health`, `POST /predict` with multipart field `image`.
- CV result `image_id` is the symbol; confusingly, `obstacle_id` is its numeric
  image ID. The Pi attaches the actual obstacle number for Android TARGET.
- Original ONNX uses HTTP 404 for no detections; the repository Pi code retries
  it as NA. The physical Pi needs the reviewed follow-up patch for that fix.
- `GET /stitch` returns the annotated image montage after detections.

The optional `viewer/` is not required to run Task 1. See [Pi entry points](../RPi/README.md).
