# Task 1 algorithm API

The current profile is **restored-speed50-grid10**. The previous 10 cm planner,
turn geometry/padding and motor-command schedule are restored; the requested
turn clearance reduction is 40 cm to 30 cm Manhattan distance. Straight checks
and robot-centre arena limits (cells 1–18) are preserved. Motor speed stays 50.

[Current configuration, commands, tests and backup](../docs/TASK1_ROLLBACK_2026-10-07.md)
[CV captures and montage](CV_TRIAL.md)

Run the root laptop launcher for Algo8000 and CV8001. `GET /status` identifies
the profile, speed, grid sizes and turn clearance. `POST /path` uses the same
text map protocol; `stm_commands` use physical centimetres and 90-degree turns,
and `command_poses` follow the final padded/cancelled motor schedule.

The user's eight-target map returns all eight SNAP IDs with reduced clearance.
Planning coverage is not proof of physical clearance; turns still use endpoint
checks rather than a swept-body collision model. `archive/grid5-trial/` and
commit `4d2bea0f` preserve the 5 cm experiment. Other historical calibration
references under `archive/` are not active profile choices.
