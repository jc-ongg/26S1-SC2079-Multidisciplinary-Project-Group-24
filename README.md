# SC2079 MDP - Group 24

One active Task 1 setup: **speed 50, 10 cm planner/tablet grid, 30 cm turn clearance**.
Use `main` for shared work. Older experiments are archived and backed up.

[Current setup, clearance, tests and backups](docs/TASK1_ROLLBACK_2026-10-07.md)
[Assessment checklist](docs/TASK1_RUN_GUIDE_2026-10-07.md)

From this Windows repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

The launcher starts Algo8000 and the existing best.pt CV detector8001, refuses
occupied ports, and prints the laptop Wi-Fi address. `--check` starts nothing.
There are no movement profile switches. Source changes require a service restart.

On the Pi, with other robot controllers/serial monitors stopped:

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use the laptop's actual hotspot IP. Connect Android, enter/resend the map,
check **8/8** for the tested layout, wait for **READY**, then **Start**.
No tablet reflash is needed for the rollback. Android stays on 10 cm coordinates.

Robot-centre arena margins stay at cells **1-18**. Only the turn-clearance
threshold was reduced from 40 cm to 30 cm Manhattan distance. Straight checks
are preserved. Route generation includes all eight targets on the tested map;
physical clearance still requires a trial. The tablet icon advances to planned
poses after matching motor completion; it is not continuous measured odometry.

Images: `output/cv-trial/`; open `http://127.0.0.1:8001/stitch` for the current run.
A successful request saves `stitched-<run hash>.jpg` there. Pi originals are
under `/home/nat/SC2079-MDP/rpi/captures/`.

Task 1 is `RPi/task1_main.py` -> `task1_controller.py`; `--manual` is a separate
arrow test. The restored planner keeps the 180-second planning timeout and
working Ctrl-C shutdown. STM firmware and CV weights are unchanged.
Git pushes do not deploy the Pi or install the tablet app.

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -B RPi/tests/test_offline.py
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s Algo/tests -v
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s RPi-services/tests -v
```

Use the curated Pi suite; archived scripts may issue live motor commands.
