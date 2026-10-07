# Current Task 1 clearance: 40 cm

**Later update:** measured turn padding and the faster planner are recorded in
[current Task 1 settings](TASK1_SETTINGS_2026-10-07.md). The 40 cm turn threshold
and arena-edge limits below remain active; the old compensation was replaced.

Jacob requested returning turn clearance from 30 cm to 40 cm for the next test.
The active setting is `Algo/consts.py: TURN_CLEARANCE_CM = 40`. The Pi launcher
checks the same value at startup, and the laptop launcher prints 40 cm.

Turn start/end positions require `|dx| + |dy| >= 40 cm` from every obstacle
centre. This is Manhattan distance. The 10 cm planner has no 35 cm positions,
so a 35 cm threshold accepts exactly the same positions as 40 cm.

Straight clearance, arena-edge limits (robot-centre cells 1-18), speed 50,
turn geometry/padding, Android coordinates, photo retries and finish behavior
remain as in the restored configuration. No Android reflash is needed.

The earlier layout with obstacle 7 at `(15,9,N)` and obstacle 8 at `(15,15,W)`
returns 7/8 at 40 cm. The 30 cm version is recoverable at commit `43ce60a6`.
Do not expect the earlier 8/8 result to hold after increasing clearance.

The latest recorded layout moves obstacle 7 to `(15,8,N)` and keeps obstacle 8
at `(15,15,W)`. The live 40 cm API, called from the Pi, returned **8/8** in
**23.6 seconds**, with 54 command/pose entries ending in FIN. This calculates
a route only; physical clearance still needs the arena trial.

23 Algo/CV tests, 7 laptop launcher tests and 6 Pi launcher tests passed. The
Pi launcher tests also passed on the actual Pi. Only the matching launcher and
its test changed on the Pi; the running robot controller was left in place.
The laptop services were refreshed and their health/status verified from the
Pi. Its previous run had finished before that refresh.

Pi backups and validation records are in
`/home/nat/mdp-backups/task1-clearance40-2026-10-07/`; local records are in
`output/clearance40-2026-10-07/`. Saved capture/montage files remain in place.

After the laptop planner reloads, resend the map and check coverage before
Start. A prepared route already held by the Pi does not change automatically.
The Pi startup check now expects `turn_clearance_cm: 40` on its next launch.

[Planner rollback/backups](TASK1_ROLLBACK_2026-10-07.md)
[Current photo retries and finish behavior](TASK1_PHOTO_STOP_2026-10-07.md)
