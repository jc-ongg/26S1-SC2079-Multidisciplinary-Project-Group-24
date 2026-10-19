# SC2079 MDP - Group 24

This repository contains the selected Task 1 and Task 2 implementations, organised into four components.

| Folder | Contents |
| --- | --- |
| `Algo/PathPlanning` | Task 1 route planning, command generation and tests |
| `Algo/ImageRecognition/Task1` | Task 1 symbol recognition API |
| `Algo/ImageRecognition/Task2` | Task 2 LEFT/RIGHT arrow recognition API |
| `Algo/Services` | Laptop service setup and launchers |
| `STM32/Task1` | Task 1 STM32CubeIDE project |
| `STM32/Task2` | Task 2 STM32CubeIDE project |
| `RPi/Task1` | Task 1 Pi controller, camera and communications |
| `RPi/Task2` | Task 2 camera/heartbeat bridge and Pi return/parking |
| `Android` | One Android application shared by both tasks |

Task 1 is retained from GitHub `main`; Task 2 is the selected `v19b-fix` implementation, including its Pi-driven return and arrow API. The shared starting snapshot is `27cb0792`. Older experiments and handovers remain in Git history.

## Run Task 1

Use Python 3.11 or later. From the repository root on the laptop:

```powershell
py -3.11 -m venv Algo/.venv-cv-win
.\Algo\.venv-cv-win\Scripts\python.exe -m pip install -r Algo/requirements.txt
.\Algo\.venv-cv-win\Scripts\python.exe Algo/Services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

Place `best_v11i_m.onnx` in `Algo/ImageRecognition/Task1/`. `Algo/Services/laptop_setup.py` can fetch and checksum-verify the `cv-model-v11i` release with an authenticated GitHub CLI. Its separate environment setup uses the retained pinned dependency lists and requires Python 3.12/3.13 or uv. The launcher runs the planner on port 8000 and recognition on 8001; `--check` opens no services.

Deploy the contents of `RPi/Task1` to the Pi, install its requirements, then run:

```bash
LAPTOP_IP=<laptop-wifi-ip> python3 -u task1_main.py
```

Use system Picamera2/libcamera or the rpicam-still fallback. Only one controller may own STM32 serial, and only one program may own the camera. The planner uses speed 50, a 10 cm grid and 40 cm turn clearance. Send the Android map, wait for READY, then Start. A valid partial route can run.

## Run Task 2

Install `Algo/ImageRecognition/Task2/requirements.txt` in a separate laptop environment. Download the trained arrow checkpoint from [its upstream repository](https://github.com/PranavU-Coder/arrow_detection/tree/335c0dc275769edd902c5bb41a99992fa12fff9c), file `runs/detect/train/weights/best.pt`, and place it at `Algo/ImageRecognition/Task2/models/best_yolo11m_arrows.pt`. The server verifies SHA-256 `5098030850b53b7a49ae7ef6ce3942e25d4863f5308dd71c814c7cd4330fd821`. The upstream licence governs the model; weights are not tracked here.

```bash
python Algo/ImageRecognition/Task2/main.py --port 8001
```

Stop Task 1 recognition before using the same port. Deploy `RPi/Task2` to the Pi and install its requirements. With the matching Task 2 firmware flashed, run from that deployed directory:

```bash
python3 KC_task2_stm32_bridge.py --execute --stage full-pi-return --port /dev/ttyACM0 --laptop-ip <laptop-wifi-ip>
```

Omitting `--execute` prints a dry-run description. STM32 handles O1/O2; the Pi supplies arrows/heartbeats and drives the return/parking. The bridge retries recognition according to its stationary/moving-frame policy.

## Firmware and Android

Import `STM32/Task1` and `STM32/Task2` as separate existing STM32CubeIDE projects, named `MDP_Task1` and `MDP_Task2`. Each retains its HAL/CMSIS, FreeRTOS, linker scripts and board configuration. Rebuild and flash the appropriate task. The root main.c in each project contains the integrated motor/sensor logic; Task 2 adds the `kc_*.h` helpers and platform adapter.

Open `Android` in Android Studio. Its Gradle wrapper, resources and SDK configuration are retained. Pair the tablet with the Pi and use the same app for both tasks.

## Offline checks

Using a Python environment with the laptop dependencies:

```bash
python -B -m unittest discover -s Algo/PathPlanning/tests -v
python -B RPi/Task1/tests/test_offline.py
python -B -m unittest discover -s Algo/Services/tests -v
python -B -m unittest discover -s Algo/ImageRecognition/Task1/tests -v
```

Run `python -m unittest test_bridge -v` and `python test_pi_return.py` from `RPi/Task2`. Run `python -m unittest test_api -v` from the Task 2 image-recognition directory; real-model replay is skipped when the checkpoint/photos are unavailable. From `STM32/Task2`, compile the classic O1 suite with `gcc -std=gnu11 -O2 -ICore/Inc -DKC_FAST_O1=0 test_task2.c -lm -o test_task2`. Compile `test_fast_o1.c`, `test_o1_range.c` and `test_platform_settle.c` with the same flags but omit `-DKC_FAST_O1=0`; those exercise the selected fast O1 and its adapter. Hardware execution is separate from these checks.

## Source submission

```bash
python build_submission.py
```

This creates only `output/MDP_2026-27S1-TEAM-24_Source_Code.zip`. It contains one combined component tree, source files renamed to `original.extension.txt`, a README and SHA-256 manifest. Configurations, resources, libraries, generated support, weights and binaries remain out of the submission. The repository retains build support so it can still be rebuilt. `output` is ignored by Git.
