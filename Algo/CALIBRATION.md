# Active movement baseline — 2026-10-01

User-approved: **10 cm per grid cell in both directions; keep original turns.**

| Setting | Active value |
| --- | --- |
| Straight / reverse cm per grid cell | 10 / 10 |
| Forward-left longitudinal / lateral cells | 2 / 4 |
| Forward-right longitudinal / lateral cells | 2 / 4 |
| Backward-left longitudinal / lateral cells | 3 / 2 |
| Backward-right longitudinal / lateral cells | 3 / 2 |
| FL straight before / after cm | 3 / 5 |
| FR straight before / after cm | 0 / 4 |
| BL straight before / after cm, reverse | 0 / 2 |
| BR straight before / after cm, reverse | 0 / 4 |
| Pi default speed | 50, unchanged |

The rejected padded-turn calibration was NOT reapplied.
[MOVEMENT_CALIBRATION.md](MOVEMENT_CALIBRATION.md) preserves that historical
experiment; its deployment/test commands are not current instructions.

## Supplied measurements

At speed **25**, FWD 10/20/40/80/120 cm and REVS 10/20/40/80 cm travelled the
requested distances. This supports a 1.0 distance correction at that speed.
Grid conversion is separately 10 cm per cell.

| Bare 90-degree turn | Trials | Mean longitudinal cm | Mean lateral cm |
| --- | ---: | ---: | ---: |
| TURNL | 2 | 25.50 | 22.00 |
| TURNR | 1 | 32.50 | 26.00 |
| REVL | 3 | 45.00 | 32.83 |
| REVR | 3 | 44.33 | 27.33 |

These measurements do not validate the original turn geometry or speed 50.
No final heading measurements were supplied. Software tests cannot substitute
for a physical run; do not claim the robot is fully calibrated.

## Next physical tests (run by the operator)

Stop Task 1/manual/echo first so only ONE program owns serial and Bluetooth.
Use a clear arena and be ready to cut motor power. Measure from the robot's
centre, record battery/surface/speed, start pose and final heading.

At the SAME chosen speed, repeat FWD and REVS 10/20/40/80 cm, then three trials
each of TURNL, TURNR, REVL and REVR at 90 degrees. Finally test the original
combined turn padding separately; bare-turn measurements are not the same
as a padded maneuver. Do not run a whole sequence blindly.

Print the firmware test frames without opening any hardware:

```bash
python3 calibration_plan.py --speed 25
# Or --speed 50 to measure the CURRENT Task 1 speed.
```

Every frame needs the trailing semicolon and a unique ID. Wait for the matching
`!<id>/DONE;` before the next command; stop on ERROR or timeout.
The plan printer does not transmit commands.

After changing Algo constants, restart Algo yourself. On-disk changes do not
alter a currently running server. Validate a simple one-obstacle map before
attempting a full course.
