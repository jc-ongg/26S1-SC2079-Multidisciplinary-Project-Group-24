# REVERTED: historical provisional Task 1 movement calibration

**Rolled back at the user's request on 2026-10-01.** The Algo constants and
route-planning changes below are no longer active. Task 1's speed-25 override
was removed, restoring its original default speed 50. The CV crash fix remains.
The rest of this document is a historical record, not current setup instructions.

Prepared from the supplied centre-displacement measurements on 2026-10-01.
Applies to **STM speed 25**, a 10 cm arena grid, and nominal 90-degree turns.
The Algo CSV commands do not include speed: the Pi's `STMLink` selects it.
Do not use this profile at speed 50 without remeasuring. No heading-angle
measurements were supplied, so heading accuracy is not calibrated here.

## Measurements and selected values

Forward commands of 10, 20, 40, 80 and 120 cm all travelled their requested
distances. Reverse commands of 10, 20, 40 and 80 cm did likewise. The empirical
distance correction is therefore 1.0 in both directions. Because
`UNIT_TO_CM_MULTIPLIER` converts grid steps to centimetres, its value and the
reverse equivalent are **10**, not 1 or 20.

Only populated trials were averaged; blank trials are not zero measurements.
Longitudinal means displacement along the starting heading's axis; lateral
means sideways displacement. Reverse values below are positive magnitudes.

| Bare turn | Trials | Mean longitudinal cm | Mean lateral cm | Planner longitudinal/lateral cells | Extra straight before/after cm |
| --- | ---: | ---: | ---: | --- | --- |
| TURNL / FL | 2 | 25.50 | 22.00 | 3 / 3 | 5 / 8 forward |
| TURNR / FR | 1 | 32.50 | 26.00 | 4 / 3 | 8 / 4 forward |
| REVL / BL | 3 | 45.00 | 32.83 | 5 / 4 | 5 / 7 reverse |
| REVR / BR | 3 | 44.33 | 27.33 | 5 / 3 | 6 / 3 reverse |

The planner operates on an integer grid. Putting raw centimetres or fractional
grid offsets into its constants would change that state space and break the
intended one-cell straight-step interpretation. Instead, each mean displacement
is rounded **up** to the next whole grid cell. The existing command generator
adds short straight movements before and after each turn to fill the gap:

```
target_cm = ceil(measured_mean_cm / 10) * 10
before_cm = round_to_nearest_whole_cm(target_longitudinal_cm - measured_longitudinal_cm)
after_cm  = round_to_nearest_whole_cm(target_lateral_cm - measured_lateral_cm)
```

Half-centimetre corrections were rounded upwards. Whole-centimetre commands
avoid the Pi translator's integer truncation. Extra reverse-turn movements are
both backwards: the first adds longitudinal displacement and the second adds
lateral displacement after the heading has changed.

For example, a forward-left maneuver is 5 cm forward, TURNL 90, then 8 cm forward.
Its predicted mean displacement is 30.5 cm longitudinal and 30 cm lateral,
against the planner's 30/30 cm target. The other mean endpoint predictions are
FR 40.5/30 cm, BL 50/39.83 cm, BR 50.33/30.33 cm. These rounding residuals are
not a claim of actual robot accuracy. In particular, straight moves below
10 cm have not yet been measured; acceleration, steering reset, battery level,
surface and turn-angle error may affect the combined maneuver.

## Planner change required by asymmetric turns

`algo/algo.py` previously cached a route in both directions by reversing its
coordinates, and mirrored its cost in the TSP matrix. Different forward and
reverse turn displacements make that assumption invalid. Ordered state pairs
are now searched independently and their costs kept directional. This may
increase planning time and change obstacle visit order.

## Before using on the robot

1. Ensure Task 1's Pi-side movement speed is **25** for both straight moves and
   turns. Changing Algo constants alone does not change that speed.
2. Restart the Algo server yourself to load these on-disk constants. Restart
   Task 1 yourself if its startup speed was changed. Do not restart mid-run.
3. In a clear area, validate each compensated maneuver at least three times,
   measuring robot-centre endpoint and final heading. Complete the missing
   forward-turn trials, especially the right turn (only one sample supplied).
4. Verify short straight corrections (3-8 cm), then test a simple one-obstacle
   route before a full map. Existing collision checks examine states, not a
   measured continuous swept robot footprint; this profile is not proof of
   clearance throughout an arc.

No physical movement is performed by the offline regression suite. From `Algo`,
using a working Python environment with the Algo dependencies installed:

```powershell
python -m unittest discover -s tests -p test_movement_calibration.py -v
```

It checks straight conversion, all four turns in all four headings, mean
endpoint consistency, opposite-straight cancellation, SNAP/FIN ordering,
directional route caching and a two-obstacle mission. Passing these checks
verifies software consistency, not physical calibration on the arena.

## Pi deployment record

The physical Pi's `/home/nat/SC2079-MDP/rpi/task1_main.py` was patched directly,
not replaced from GitHub. Normal Task 1 now sets `robot.stm_link.speed = 25`
before starting its controller. `--manual` and the shared `DEFAULT_SPEED = 50`
are unchanged. Launching `final_week_8_backup.py` directly bypasses this override;
use `python3 -u task1_main.py` for the calibrated Task 1 entry point.

The original launcher is preserved at:

```
/home/nat/mdp-backups/calibration-speed25-20261001-X8FU3G/task1_main.py
SHA256: b9b27a226f8bc8a3d4e22d8611012fc9ca28384d65f4eae2485b045310abcc49
```

Two new Pi launcher tests (`tests/test_task1_calibration.py`) passed with both
controller modules replaced by mocks. No real controller, server or robot was
started or restarted during this work.

Local test environment note: `.venv-cv-win/Scripts/python.exe` currently points
to a missing Python installation. The eight offline Algo tests were run using
the installed Windows Python 3.11 with that environment's existing site-packages
on the test process's import path. The environment was not repaired or replaced;
confirm a working Algo launch environment before attempting a hardware run.
