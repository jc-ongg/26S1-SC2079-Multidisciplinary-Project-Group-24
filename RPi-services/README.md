# Laptop Algo + CV

From the repository root (Windows, macOS or Linux):

```powershell
python RPi-services/laptop_setup.py   # once
python RPi-services/laptop_start.py
```

One active movement setup: restored-speed50-grid10, speed50, 10 cm map/planner,
40 cm Manhattan turn clearance. No calibration/profile switches. --check starts
nothing. The launcher refuses occupied ports, logs under RPi-services/logs/,
and Ctrl-C stops its children. It does not start the Pi.

CV is image-api + best_v11i_m.onnx on 8001; Algo is 8000. This is the only
recognition model: Algo/best.pt and cv_trial_server.py were removed because
best.pt read arrows badly (right arrow 0.43 vs 0.93 on the 3/3 run). Annotated
images go to image-api/own_results/. Open http://127.0.0.1:8001/stitch after a run.

laptop_setup.py builds Algo/.venv and image-api/venv with the pinned versions
from the 3/3 laptop and downloads best_v11i_m.onnx from the cv-model-v11i
release (checksum-verified). Uses uv if installed, else Python 3.12/3.13.

Keep Wi-Fi on the Pi hotspot and use the laptop's 192.168.50.x address.
[Pi startup](../RPi/README.md) and [current setup/backups](../docs/TASK1_ROLLBACK_2026-10-07.md).

Use current Algo/Pi tests; laptop_verify.py's exact golden routes are historical.
