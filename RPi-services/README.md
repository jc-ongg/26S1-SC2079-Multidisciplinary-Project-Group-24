# Laptop Algo + CV

From the Windows repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

One active movement setup: restored-speed50-grid10, speed50, 10 cm map/planner,
40 cm Manhattan turn clearance. No calibration/profile switches. --check starts
nothing. The launcher refuses occupied ports, logs under RPi-services/logs/,
and Ctrl-C stops its children. It does not start the Pi.

Default CV remains Algo/best.pt on8001; Algo is8000. Images and saved montages
are in output/cv-trial/. Open http://127.0.0.1:8001/stitch after a run.

Fresh Windows environment:

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements-cv-trial.txt
```

Keep Wi-Fi on the Pi hotspot and use the laptop's 192.168.50.x address.
[Pi startup](../RPi/README.md) and [current setup/backups](../docs/TASK1_ROLLBACK_2026-10-07.md).

The original ONNX remains an explicit alternative (--cv-backend onnx), requiring
image-api/best_v11i_m.onnx and its dependencies. It is absent on this laptop.
The cv-model-v11i release and laptop_setup.py preserve that optional recovery.
Use current Algo/Pi tests; laptop_verify.py's exact golden routes are historical.
