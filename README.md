# SC2079 MDP — Group 24

One active Task 1 test setup: **motor speed 50, 5 cm planner, 10 cm tablet map**.
Use `main` for shared work. Older calibration sources and RPi experiments are
under `Algo/archive/` and `RPi/archive/`. They are historical references.

[Current test procedure, backups and changes](docs/TASK1_GRID5_TEST.md)
[Assessment requirements and checklist](docs/TASK1_RUN_GUIDE_2026-10-07.md)

| Component | Entry point | Location |
| --- | --- | --- |
| Android | `android/MDPGroup24` in Android Studio | Tablet |
| Laptop Algo + CV | `RPi-services/laptop_start.py` | Laptop; ports 8000 / 8001 |
| Task 1 | `RPi/task1_main.py` → `task1_controller.py` | Pi: `/home/nat/SC2079-MDP/rpi` |
| Manual arrows | `task1_main.py --manual` | Pi; separate from Task 1 |
| STM32 | Existing flashed firmware | Robot |

From this Windows repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

The launcher uses the existing `best.pt` CV model, refuses occupied ports, and
prints the Wi-Fi address for the Pi. Add `--check` for a read-only health check.
There are no movement/calibration switches. Changing source does not reload a
running server. Stop its existing launcher before starting another.

On the Pi, with other robot controllers/serial monitors stopped:

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use the laptop's actual hotspot Wi-Fi address. Connect the updated tablet app,
enter the map, press **Send Map**, inspect route coverage, wait for **READY**,
then press **Start** when ready to run. The professor allows calculation during
preparation. Preparation overtime still uses the Task 1 runtime allowance.

The tablet map remains 20 × 20 at 10 cm. Algo converts to a 40 × 40 internal
5 cm grid covering the same arena. The tablet icon shows rounded planned poses
after matching motor completion; this is not measured real-time odometry.

Laptop images: `output/cv-trial/`. Open `http://127.0.0.1:8001/stitch` for the
current run montage; it also saves `stitched-<run hash>.jpg` there. Pi captures
are under `/home/nat/SC2079-MDP/rpi/captures/`.

GitHub updates do not deploy the Pi or install the Android app. Physical
movement accuracy and tablet installation still require an integration test.

Offline software checks:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -B RPi/tests/test_offline.py
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s Algo/tests -v
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s RPi-services/tests -v
```

Use the curated Pi suite; archived scripts can contain live motor commands.
