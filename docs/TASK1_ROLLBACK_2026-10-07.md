# Task 1 rollback record: restored 10 cm planner and reduced turn clearance

**Later update:** the active turn clearance is back to 40 cm at Jacob's request.
See [current clearance trial](TASK1_CLEARANCE40_2026-10-07.md). The 30 cm results
below describe the earlier rollback, not the currently loaded configuration.

Jacob requested switching back from the 5 cm trial and reducing obstacle
clearance while keeping arena-edge margins. The active configuration is now
`restored-speed50-grid10`: motor speed 50, 20 × 20 cells at 10 cm, the same tablet
coordinates, and the exact resolved turn geometry/padding used before the 5 cm
trial (original base with both speed50-v2 overlays). There are no profile switches.

The previous planner, helper and command-pose schedule were restored from
`37367242`; the 5 cm trial is preserved at `4d2bea0f` and in
`output/rollback-grid5-2026-10-07/grid5-trial-source-4d2bea0f.zip`.
Older unused 5 cm coordinate/scheduling modules and tests are under
`Algo/archive/grid5-trial/`. Folder cleanup, MAP/READY/BEGIN, target chat updates,
matched-ACK tablet poses, the 180-second planning timeout, saved CV montages and
the working Ctrl-C shutdown fix remain in place. CV/STM firmware are unchanged.

## Clearance

- Robot-centre arena limits remain tablet cells **1–18 inclusive**, both axes.
- Straight positions retain the old check: reject when `|dx| + |dy| < 40 cm`
  AND `max(|dx|, |dy|) < 30 cm` relative to any obstacle centre.
- Turn start/end positions now require `|dx| + |dy| >= 30 cm`, reduced from 40 cm.
  This is Manhattan distance, not a 30 cm circular radius or a swept-turn check.
- `Algo/consts.py` contains the single active `TURN_CLEARANCE_CM = 30` setting.

Obstacle 7's `(15,12,S)` viewing pose was legal for straight travel but forbidden
as a turn endpoint at 40 cm; its other view poses were blocked by nearby obstacle 8.
The exact pre-trial planner also returned 7/8 on this map, so rollback alone did
not resolve the omission. Reducing turn clearance returned **8/8** in about
**14 seconds** in the isolated comparison. This is route-generation evidence;
physical clearance and turn accuracy need an arena test with the reduced buffer.

## Use

Laptop services must be restarted to load this source. From the repository root:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
```

On the Pi after stopping other controllers/serial monitors:

```bash
cd /home/nat/SC2079-MDP/rpi
python3 -u task1_main.py
```

The laptop status must say profile `restored-speed50-grid10`, speed50, grid10,
input10, arena20 and turn_clearance_cm30. The Pi checks this before opening
hardware and rejects route responses with the wrong grid. Reconnect Android,
resend the map, confirm **8/8**, wait for READY, then Start. No Android reflash
is needed for this rollback; its map/protocol remain at 10 cm.

Images remain in `output/cv-trial/`; successful `/stitch` saves a montage there.
Pi raw frames remain in `/home/nat/SC2079-MDP/rpi/captures/`.

## Verification and backup

23 Algo/CV tests and 7 laptop launcher tests passed locally. The Pi suite has
59 tests; its Linux interrupt regression is skipped on Windows and executed
on the Pi during deployment. The map regression checks all eight SNAP IDs,
command/pose alignment and the planned straight/turn edges. Edge-margin tests
confirm cells 0/19 remain forbidden and 1/18 remain valid.

The matching Pi rollback was installed and all 59 tests passed on the Pi.
Live validation from the Pi confirmed speed50, grid10, arena20, turn clearance30
and an 8/8 route in **14.3 seconds**, with 54 aligned entries ending in FIN.
No motor commands were issued during installation/validation.

The Pi backup/staging directory is
`/home/nat/mdp-backups/task1-rollback-grid10-2026-10-07/`. It preserves the replaced
5 cm launcher/controller/parser/tests, source hashes, staged checks and result
records. Local records are in `output/rollback-grid5-2026-10-07/`. The larger
pre-trial backups under `task1-grid5-2026-10-07` and `task1-update-2026-10-07-7d75a8d3`
remain intact. The laptop services were restarted into the restored profile. The Pi controller
was left stopped for the user to start. No Android reflash is required.
