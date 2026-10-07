# Task 1 algorithm API

The active test configuration is defined once in `consts.py`: speed 50,
5 cm internal cells, measured/padded turn endpoints, 40 × 40 arena.
The public tablet map stays at 10 cm and 20 × 20 cells. Coordinates are scaled
on input; output tablet poses retain exact `planner_x` / `planner_y` for revisits.

Run the root [laptop launcher](../README.md) to start both Algo (8000) and CV (8001).
Standalone planning uses `python main.py` from this directory; install dependencies
from `requirements.txt` or the combined `requirements-cv-trial.txt` environment.

`GET /status` identifies the active profile, speed and grid sizes.
`POST /path` accepts the existing text map protocol. `stm_commands` use physical
centimetres, all turns use 90 degrees, and `command_poses` align with the final
merged motor schedule. Intermediate turn padding has no future endpoint.

Directed shortest paths are calculated once per source pose. One dynamic program
selects camera poses and visiting order, prioritizing the number of targets,
then cost. Missing edges are excluded; unreachable targets are not fabricated.
The Pi reports partial route coverage before Start.

[Measurements, procedure and backups](../docs/TASK1_GRID5_TEST.md)
[CV images and montage](CV_TRIAL.md)

`archive/` contains the former 10 cm / speed25 / speed50-overlay instructions
and tests. They are recovery references and do not run against the active setup.
The optional turtle/frontend simulator is not updated to display the 5 cm planner;
use the tablet and API traces for this test setup.
