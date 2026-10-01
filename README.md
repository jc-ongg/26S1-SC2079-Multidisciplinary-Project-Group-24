# SC2079 MDP — Group 24

**Use `main` as the shared integration branch.** Older branches and both local
GitHub Desktop stashes were kept for recovery; no history was force-pushed.

| Component | Current entry point | Runs on |
| --- | --- | --- |
| Android | `android/MDPGroup24` in Android Studio | Tablet |
| Algo | `Algo/main.py`, port 8000 | Laptop |
| Temporary CV | `Algo/cv_trial_server.py` + `Algo/best.pt`, port 8001 | Laptop |
| Original CV | `RPi-services/image-api/main.py` + `best_v11i_m.onnx`, port 8001 | Laptop, alternative to temporary CV |
| Task 1 | `task1_main.py` → `final_week_8_backup.py` | Physical Pi: `~/SC2079-MDP/rpi` |
| STM32 | Existing flashed firmware | Robot; not reflashed during consolidation |

The physical Pi remains the source of truth for deployed code. The repository
contains its recovered core source plus reviewed follow-up fixes. **A Git push
does not update the Pi. Do not bulk-copy this folder over it.**

## Start on this Windows laptop

From the repository root, inspect without starting anything:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --check
```

When existing servers have been stopped by you, start both laptop services:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial
```

The launcher refuses occupied ports and never starts/stops a Pi program.
It prints the laptop's hotspot address. Keep Wi-Fi connected to the Pi, with
USB tethering for internet. See [laptop setup](RPi-services/README.md) for a fresh
environment and the original ONNX option.

On the Pi, after stopping any existing controller yourself:

```bash
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py
```

The deployed settings must point to the laptop's Wi-Fi address, not its USB
tether address. Repository settings additionally support `LAPTOP_IP=...`;
that override works on the Pi only after deploying the updated settings file.

Use `--manual` only for arrow-button testing; it cannot run a map.
`bt_echo.py` is a Bluetooth echo test, not a robot controller.
**`final_week_8.py` is a sensor/Task 2 helper, not Task 1.**

## What was consolidated

- RPi protocol/A5 history and the teammate's Task 1 services branch.
- Actual Pi controller, manual controls, Bluetooth framing, quiet telemetry and CV lock fixes.
- Android Send Map confirmation and recovered **Save | Use** layout buttons.
- Explicit temporary CV fallback, output-folder recovery and safe laptop launcher.
- Straight scaling **10 cm per grid cell**, with original turn constants unchanged.

[Calibration status](Algo/CALIBRATION.md) distinguishes approved settings from
rejected experiments. Software tests do not prove arena accuracy.

## Checks and maintenance

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -B RPi/tests/test_offline.py
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s Algo/tests -v
.\Algo\.venv-cv-win\Scripts\python.exe -B -m unittest discover -s RPi-services/tests -v
```

Only `RPi/tests/` contains the curated offline suite. Other legacy Pi scripts
may send motor commands on import: do not run blanket test discovery there.

See [repository and deployment guide](docs/REPOSITORY_GUIDE.md) for branch
roles, backup provenance, deployment boundaries and remaining limitations.
