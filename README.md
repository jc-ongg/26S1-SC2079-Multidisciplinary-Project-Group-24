# SC2079 MDP — Group 24

**Task 1 readiness and assessment walkthrough (2026-10-07):** see
[requirements, readiness gaps and run checklist](docs/TASK1_RUN_GUIDE_2026-10-07.md),
plus the [original pending Pi changes](docs/TASK1_READINESS_2026-10-04.md).
The current branch adds Send Map -> route preparation -> enabled Start,
timestamped target findings in Chat, and command-aligned tablet positions.
The four matching Pi runtime files were deployed and verified on 2026-10-07;
see [Pi deployment record](docs/RPI_DEPLOYMENT_2026-10-07.md). Use the updated
Android build for the new READY gate. Tablet installation and a physical run
remain unverified; the Pi controller was left stopped.

**Use `main` as the shared integration branch.** It contains all four components.
The four redundant branches were removed locally and on GitHub on 2026-10-02
after verifying all their commits were included in main. Shared commit history
was not rewritten. Their final shared tip is preserved locally and on GitHub in
`archive/2026-10-02/merged-branches-before-removal`; older component snapshots
remain available through `archive/2026-10-01/*` tags.
The two old active stash entries were later cleared with approval; their full
snapshots remain in local `archive/2026-10-02/stash-*` recovery tags. Useful
source and reference documents are committed. See [stash audit](docs/STASH_AUDIT_2026-10-02.md).

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
Access returned on **2026-10-07** for the narrow Task 1 deployment. Its folder
here remains a source snapshot, not a complete verified live mirror. See
[latest repository status](docs/REPO_STATUS_2026-10-02.md).

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

Later work adds opt-in speed50 forward/reverse calibration trials and a guarded
Pi launcher. Defaults still use original turns. To select both new turn trials,
add `--forward-calibration speed50-v2 --reverse-calibration speed50-v2` to the
laptop launch command. The base profile remains `original`/speed50; do not pass
`--calibrated` to Task 1 for these overlays. Tests do not prove physical accuracy.

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
The detailed [conversation handover](docs/CONVERSATION_HANDOVER_2026-10-01.md)
preserves historical troubleshooting; use the latest status page for current
access and unresolved issues.
