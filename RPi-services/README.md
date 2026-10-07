# Laptop Algo + CV services

From the Windows repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

One active movement setup: speed 50, 5 cm internal planner, 10 cm tablet map.
There are no movement/calibration overlay options. `--check` starts nothing.
Normal launch refuses occupied ports, logs under `RPi-services/logs/`, and Ctrl-C
stops its children. It does not start the Pi.

The default CV backend is the existing `Algo/best.pt` detector at port 8001.
Algo runs at 8000. Captures and saved montages are in `output/cv-trial/`.
Open `http://127.0.0.1:8001/stitch` after the run; a successful request also writes
`stitched-<run hash>.jpg`. Model labels are MDP image IDs, not ONNX class indices.

To create a fresh Windows environment:

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements-cv-trial.txt
```

Keep Wi-Fi on the Pi hotspot and use the laptop's actual `192.168.50.x` address.
Pi startup/configuration: [RPi guide](../RPi/README.md).

The original ONNX backend remains an explicit alternative (`--cv-backend onnx`)
requiring the separately supplied `image-api/best_v11i_m.onnx` and its dependencies.
Its model is currently absent on this laptop. The release is
[cv-model-v11i](https://github.com/jc-ongg/26S1-SC2079-Multidisciplinary-Project-Group-24/releases/tag/cv-model-v11i).
Expected SHA-256: `8c4f382f800d123e069fe93f0069e279175594c0a6ef631c699fbc3d957ee7d5`.
`laptop_setup.py` installs that optional backend; it is not necessary for today's
best.pt test. Do not run it over a working environment.

`laptop_verify.py` uses historical 10 cm golden route commands; those exact command
expectations are archived baselines. Use the current Algo/Pi test suites for the
5 cm setup. The optional viewer is not required for Task 1.
