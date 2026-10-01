# Forward calibration trial, speed 50

Latest forward measurements from the user's result tables: TURNL90 at speed50,
15cm forward/33cm sideways; TURNR90 at speed50, 25cm forward/37.5cm sideways.
The command examples show speed25, but the actual result rows show50; this
implementation uses50. Bare90-degree turns are assumed. One trial each and
no final-heading measurement were supplied: this is provisional calibration.

| Turn | Bare forward / sideways cm | Grid target forward / sideways cm | Extra forward before / after cm | Predicted combined forward / sideways cm |
| --- | --- | --- | --- | --- |
| TURNL | 15 / 33 | 20 / 40 | 5 / 7 | 20 / 40 |
| TURNR | 25 / 37.5 | 30 / 40 | 5 / 3 | 30 / 40.5 |

The planner uses whole10cm cells and the STM accepts integer cm, so the right
turn retains a 0.5cm rounding residual. Extra before-turn straights change
steering onset; endpoint agreement is not agreement with the grey illustrated
curve. Short straights, actual90-degree heading and full swept-arc clearance
must still be validated on the floor. Start in a clear area.

## Configuration

- Forward switch: `MDP_FORWARD_CALIBRATION=speed50-v2`.
- Retain new reverse calibration: `MDP_REVERSE_CALIBRATION=speed50-v2`.
- Keep `MDP_MOVEMENT_PROFILE=original` for motor speed50. The name denotes the
  base speed/profile; `/status` separately identifies both calibration overlays.
- Original forward settings are restored by `MDP_FORWARD_CALIBRATION=original`
  when restarting Algo. This does not undo the independent reverse calibration.
- Do not combine either speed50 overlay with the rejected `measured25` profile.

Laptop launcher from repository root, if starting both servers yourself:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe .\RPi-services\laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial --movement-profile original --forward-calibration speed50-v2 --reverse-calibration speed50-v2
```

The assistant restarted ONLY the already-owned Algo8000 process to load both
overlays. CV8001 and Pi programs were left alone. No Pi source deployment is
needed: the existing speed50 launcher is compatible. The operator must run
Task1 without `--calibrated` (that flag selects the rejected speed25 profile).

Send a fresh map and begin a new run; a robot's already queued route is unchanged.
Save three bare and three padded turn trials at the same battery/surface/speed,
measuring robot centre, final heading and steering onset separately.
