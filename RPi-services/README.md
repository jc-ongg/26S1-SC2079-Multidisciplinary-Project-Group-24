# Laptop Algo + CV

From the Windows repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

One active movement setup: restored-speed50-grid10, speed50, 10 cm map/planner,
40 cm Manhattan turn clearance. No calibration/profile switches. --check starts
nothing. The launcher refuses occupied ports, logs under RPi-services/logs/,
and Ctrl-C stops its children. It does not start the Pi.

Default CV is the released image-api/best_v11i_m.onnx on8001; Algo is8000.
The ONNX export uses segmentation (YOLOv8m-seg) and 31 classes. The existing
image-api loads it with task='segment' and maps its class indices to MDP IDs.
Every valid uploaded photo is saved in RPi-services/image-api/own_results/,
including frames with no detection or an inference error. Each capture has an
original JPEG and result JSON; recognised captures also have an annotated JPEG.
Open http://127.0.0.1:8001/stitch after a run. Stitching now respects run_id and
keeps one latest successful tile per obstacle, including repeated image IDs.
Without run_id it selects the latest run; saved evidence survives server restarts.
Use /results for the capture manifest and /files/<filename> for individual photos.
Algo /infer and /stitch forward to this same CV service, using the same ONNX model.

Fresh Windows environment:

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements-cv-onnx-win.txt
```

Keep Wi-Fi on the Pi hotspot and use the laptop's 192.168.50.x address.
[Pi startup](../RPi/README.md) and [current setup/backups](../docs/TASK1_ROLLBACK_2026-10-07.md).

Place best_v11i_m.onnx from the cv-model-v11i release in image-api/.
laptop_setup.py also supports fetching and checksum-verifying that release.
best_v11i_m.onnx is the only recognition model; Algo/best.pt and its trial
server were removed (best.pt read arrows badly). Keep the launcher
in the foreground so Ctrl-C stops both services; it never starts the Pi/camera.
Use current Algo/Pi tests; laptop_verify.py's exact golden routes are historical.

[Recognition handoff and deployment](../docs/TASK1_ONNX_HANDOFF_2026-10-08.md)
