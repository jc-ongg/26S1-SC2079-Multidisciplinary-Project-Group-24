# SC2079 MDP - Group 24

One active Task 1 setup: **speed 50, 10 cm planner/tablet grid, 40 cm turn clearance**.
Use `main` for shared work. Older experiments are archived and backed up.

[Current clearance trial](docs/TASK1_CLEARANCE40_2026-10-07.md)
[Current planner, measured turn padding and all settings](docs/TASK1_SETTINGS_2026-10-07.md)
[Planner rollback and backups](docs/TASK1_ROLLBACK_2026-10-07.md)
[Assessment checklist](docs/TASK1_RUN_GUIDE_2026-10-07.md)
[Current photo retries and finish behavior](docs/TASK1_PHOTO_STOP_2026-10-07.md)
[Proposed improvements and 5 cm / 1 cm trial handover](docs/TASK1_IMPROVEMENT_BACKLOG_2026-10-07.md)

From the repository root (Windows, macOS or Linux):

```powershell
python RPi-services/laptop_setup.py   # once: venvs + best_v11i_m.onnx from the cv-model-v11i release
python RPi-services/laptop_start.py
```

The launcher starts Algo on 8000 and image-api on 8001. Recognition is
**best_v11i_m.onnx only**; the old best.pt detector was removed (it read arrows
badly). The launcher refuses occupied ports and prints the laptop Wi-Fi
address. `--check` starts nothing. Source changes require a service restart.

On the Pi, with other robot controllers/serial monitors stopped:

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use the laptop's actual hotspot IP. Connect Android, enter/resend the map,
check route coverage, wait for **READY**, then **Start**. A valid partial route
such as **7/8** can start and skips the unreachable target.
The updated Android app waits 190 seconds for READY; install the latest build
to replace its old 70-second wait. Android stays on 10 cm coordinates.

Robot-centre arena margins stay at cells **1-18**. Turn start/end positions
require at least 40 cm Manhattan distance from obstacle centres. Straight checks
are preserved. The earlier crowded map returns 7/8 at this clearance; resend
the current map and check coverage before Start. The tablet icon advances to planned
poses after matching motor completion; it is not continuous measured odometry.

Images: `RPi-services/image-api/own_results/`; open `http://127.0.0.1:8001/stitch`
for the current run. Pi originals are under `/home/nat/SC2079-MDP/rpi/captures/`.

The Pi keeps the camera running for the whole run (`RPi/camera.py`), so each
obstacle stop is one frame grab (~0.1 s) plus recognition, not a 2-3 s cold
`rpicam-still` start. The log prints `Obstacle N: stop took X.Xs`. Stop
`videostream/main.py` before Task 1, or it falls back to `rpicam-still`.

Task 1 is `RPi/task1_main.py` -> `task1_controller.py`; `--manual` is a separate
arrow test. The optimized planner uses one visit-order calculation and caches
directed segments, including failures. Turn padding follows the latest bare
speed-50 measurements. It keeps the 180-second planning timeout and
working Ctrl-C shutdown. STM firmware is unchanged.
Git pushes do not deploy the Pi or install the tablet app.

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -B RPi/tests/test_offline.py
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s Algo/tests -v
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s RPi-services/tests -v
```

Use the curated Pi suite; archived scripts may issue live motor commands.
