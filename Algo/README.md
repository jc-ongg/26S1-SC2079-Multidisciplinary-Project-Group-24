# Algorithm & image recognition

[← Project overview](../README.md)

The laptop runs the Task 1 planner and the selected task's recognition service. Task 1 recognises symbols; Task 2 uses a separate LEFT/RIGHT arrow backend.

| Path | Entry point | Purpose |
| --- | --- | --- |
| `PathPlanning/` | `main.py` | Flask planner API on port 8000. |
| `ImageRecognition/Task1/` | `main.py` | FastAPI symbol API on port 8001. |
| `ImageRecognition/Task2/` | `main.py` | FastAPI arrow API on port 8001. |
| `Services/` | `laptop_start.py` | Starts the Task 1 planner and symbol API together. |

## Planning

The arena is **20 × 20 cells**, with **10 cm per cell**. The planner selects camera-facing states and uses a subset dynamic program to choose viewing poses and visit order. Directed A* segments and neighbours are cached, including failed segments.

The active profile is `restored-speed50-grid10`: motor setting 50, calibrated turn offsets and **40 cm Manhattan turn clearance**. Robot-centre bounds remain cells 1–18. The final schedule accounts for straight-command compression and calibrated padding; command poses match that schedule.

If all targets cannot be reached, the planner can return a valid partial route. Check coverage before starting. Turn checks use the implemented endpoint-clearance model, rather than a continuous swept-body collision proof.

## Setup

Commands run from the repository root. Obtain the Task 1 model before launching.

### Windows

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements.txt
.\Algo\.venv-cv-win\Scripts\python.exe Algo/Services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

### Linux / macOS

```bash
python3 -m venv Algo/.venv
Algo/.venv/bin/python -m pip install -r Algo/requirements.txt
Algo/.venv/bin/python Algo/Services/laptop_start.py --python Algo/.venv/bin/python
```

Add `--check` for readiness checks without starting services. The launcher refuses occupied ports; Ctrl+C stops its own child processes.

`Services/laptop_setup.py` alternatively uses the retained exact dependency lists and can fetch the Task 1 release checkpoint with authenticated GitHub access. Its environment setup requires Python 3.12/3.13 or uv.

## Recognition models

Weights are not tracked or included in the source submission.

| Task | Checkpoint location | Source |
| --- | --- | --- |
| **1** | `Algo/ImageRecognition/Task1/best_v11i_m.onnx` | Group repository release `cv-model-v11i`; 31-class segmentation export. |
| **2** | `Algo/ImageRecognition/Task2/models/best_yolo11m_arrows.pt` | [Arrow detection checkpoint](https://github.com/PranavU-Coder/arrow_detection/tree/335c0dc275769edd902c5bb41a99992fa12fff9c), upstream file `runs/detect/train/weights/best.pt`. |

Expected SHA-256 checksums:

```text
Task 1: 8c4f382f800d123e069fe93f0069e279175594c0a6ef631c699fbc3d957ee7d5
Task 2: 5098030850b53b7a49ae7ef6ce3942e25d4863f5308dd71c814c7cd4330fd821
```

Task 2 verifies its checkpoint at startup. Redistribution and use of that external checkpoint are subject to its upstream licence; it is not represented as a model authored by this team.

### Task 2 server

Install `Algo/ImageRecognition/Task2/requirements.txt` in a separate environment, then:

```bash
python Algo/ImageRecognition/Task2/main.py --port 8001
```

The health response identifies the checkpoint and two classes. The API returns its best arrow, confidence and raw candidates; the Pi applies moving/stationary confirmation rules. Do not run both task recognition services on port 8001 simultaneously.

## Interfaces and evidence

| Service | Endpoint | Contract |
| --- | --- | --- |
| Planner | `GET /status` | Active grid, speed and clearance profile. |
| Planner | `POST /path` | Text map request; JSON route, commands and command poses. |
| Task 1 vision | `GET /health`, `POST /predict` | Model identity; multipart recognition request and JSON result. |
| Task 1 vision | `GET /results`, `GET /stitch` | Capture manifest and per-run montage. |
| Task 2 vision | `GET /health`, `POST /predict` | Arrow-model identity and multipart arrow inference. |

Each recognition directory stores captures/results in its own `own_results/` directory, outside Git.

## Offline tests

From the repository root:

```bash
python -B -m unittest discover -s Algo/PathPlanning/tests -v
python -B -m unittest discover -s Algo/Services/tests -v
python -B -m unittest discover -s Algo/ImageRecognition/Task1/tests -v
```

From `Algo/ImageRecognition/Task2`, run `python -B -m unittest test_api -v`. Selector tests do not need weights. Real-model replay is skipped unless its checkpoint and recorded photos are available.
