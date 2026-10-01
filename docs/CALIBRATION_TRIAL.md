# Optional speed-25 calibration trial

**Latest outcome:** the user rejected this trial again. Algo was restarted with
`original` (recommended motor speed 50); CV was left unchanged. The optional
profile remains saved for reference, but is not the running Algo profile.
The Pi was not restarted by the assistant. Stop any calibrated Pi controller
before doing new measurements. New calibration should measure at the intended
operating speed and include heading and turn-start behaviour, not endpoints alone.

Restored on request on 2026-10-01. This is the earlier provisional profile,
not a newly verified physical calibration. The user previously found it worse;
`original` remains the default and is available for comparison.

| Turn | Bare-turn mean longitudinal/lateral (cm) | Planner endpoint longitudinal/lateral (cells) | Straight before/after (cm) |
| --- | --- | --- | --- |
| FL | 25.5 / 22 | 3 / 3 | 5 / 8 forward |
| FR | 32.5 / 26 | 4 / 3 | 8 / 4 forward |
| BL | 45 / 32.83 | 5 / 4 | 5 / 7 reverse |
| BR | 44.33 / 27.33 | 5 / 3 | 6 / 3 reverse |

Both straight scales stay **10 cm per cell**. Task 1 motor speed is **25** for
this profile, versus **50** for the original baseline. Directed path caches/TSP
costs are enabled in the trial: asymmetric forward/reverse turns cannot safely
reuse a reversed route. Small straight corrections and final heading still
need physical validation; software endpoint checks assume exact straights and
90-degree turns. Collision checking does not validate the complete physical
swept arc or the extra padding, so first test in a clear area.

## Run it yourself

Stop all previous Task 1 instances and your laptop servers with Ctrl+C in their
own terminals. Keep only **one** Task 1 parent/controller active. Manager and
worker children underneath that parent are normal. No process is restarted by
changing these source files.

Windows PowerShell, from the repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe .\RPi-services\laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --movement-profile measured25
```

In another PowerShell terminal, confirm the loaded server profile:

```powershell
Invoke-RestMethod http://127.0.0.1:8000/status
```

It must show `movement_profile: measured25` and `recommended_motor_speed: 25`.
The CV server remains on 8001 using temporary `Algo/best.pt`; the original ONNX
model is still missing. On the Pi, after the laptop servers are ready:

```bash
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py --calibrated
```

The launcher checks the Algo profile before constructing the hardware controller
and prints motor speed 25. It reads the laptop address from the Pi's existing
`settings.py` (currently 192.168.50.181); confirm that matches your laptop's Pi
Wi-Fi interface. A changed Windows IP needs separate configuration. This trial
does not overwrite the Pi's settings or its Task 1 core/controller.

For comparison/rollback, stop those same programs yourself, run the laptop
command with `--movement-profile original`, then the Pi command **without**
`--calibrated`. This restores original turns and motor speed 50, not the earlier
incorrect 20-cm straight scaling.

Start with a simple clear-area right turn and compare its landing position and
heading. FR's planned move from north is 40 cm forward and 30 cm right, produced
by forward 8 cm, right 90 degrees, then forward 4 cm. Do not start with a dense
obstacle map. Record actual endpoint and heading; speed and heading changes can
make a nominally matching displacement fail in practice.

## Physical deployment backup

Only `task1_main.py` was patched on the Pi. Its previous contents are preserved
at `/home/nat/mdp-backups/calibration-trial-20261001-DLA9Sp/task1_main.py`.
The original controller and settings were left untouched. No programs were
started or stopped. Running processes require a user-controlled restart.
