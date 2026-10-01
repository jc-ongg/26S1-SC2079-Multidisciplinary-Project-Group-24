# Reverse-only calibration trial, speed 50

Later update: the user also supplied new forward values. Those are separately
selectable with `MDP_FORWARD_CALIBRATION=speed50-v2` and are currently enabled
alongside this reverse overlay. Reverse values below are unchanged. See
[forward calibration](FORWARD_CALIBRATION_SPEED50.md).

The user supplied one new result per turn and corrected the table's speed25
label to **50**. Values are treated as bare 90-degree turns, with no straight
padding included. Final heading was not supplied. Confirm those assumptions
before drawing conclusions from the trial.

| Turn | Measured backward / sideways cm | Planner backward / sideways cm | Added reverse before / after cm | Predicted combined backward / sideways cm |
| --- | --- | --- | --- | --- |
| REVL | 29.5 / 15 | 30 / 20 | 1 / 5 | 30.5 / 20 |
| REVR | 34 / 21.5 | 40 / 30 | 6 / 8 | 40 / 29.5 |

The planner uses whole 10cm cells, so endpoints are rounded up and compensated
with integer straight commands. The half-centimetre residuals are not evidence
of physical precision: only one trial per turn, unknown heading error, and
unvalidated short straight movements. Extra motion before turning changes the
steering onset. The diagram is only a curve between planner endpoints, not the
true padded trajectory; swept-arc/padding collision clearance is not verified.

Forward-left/right settings, motor speed50, and both 10cm-per-cell scales remain
unchanged. Directed path caches and TSP costs are required for this asymmetric
reverse profile; do not fabricate a return route by reversing a forward route.

## Run / roll back

Algo configuration (PowerShell in `Algo`, existing environment):

```powershell
$env:MDP_MOVEMENT_PROFILE = 'original'
$env:MDP_REVERSE_CALIBRATION = 'speed50-v2'
.\.venv-cv-win\Scripts\python.exe -u main.py
```

Alternatively add `--reverse-calibration speed50-v2` to the laptop launcher,
with `--movement-profile original`. Never combine it with `measured25`.
On `/status` expect profile `original`, recommended speed50, and
`reverse_calibration: speed50-v2`. This extra calibration field deliberately
keeps the physical Pi's existing speed50 launcher compatible; no Pi deployment
is needed. If the Pi was running `--calibrated`, stop it and run Task1 WITHOUT
that flag yourself. The assistant does not start or stop Pi programs.

For rollback, restart only Algo with `MDP_REVERSE_CALIBRATION=original` (or the
laptop launcher's default `--reverse-calibration original`). This restores old
reverse geometry and padding, while keeping 10cm scaling and speed50.

Use a clear-area single reverse turn first, not a crowded full map. Repeat each
bare turn three times at speed50, then measure its padded version separately,
including final heading and robot-centre displacement. Do not change STM
firmware or robot dimensions to match the illustrative curve.
