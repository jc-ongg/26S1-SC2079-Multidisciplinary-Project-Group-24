# Speed-50 / 5 cm Task 1 test setup â€” 2026-10-07

Jacob confirmed that the supplied turn measurements were taken at speed **50**
and requested a **5 cm test setup with backup**. This is the single active movement
configuration. The Android map stays 10 cm; converting to a 40 Ã— 40 internal grid
preserves the full 2 m arena. Existing updated Android APK protocol remains compatible.

## Turn values

| Turn | Bare measured forward/back displacement | Bare lateral displacement | Padding | Planned endpoint |
| --- | --- | --- | --- | --- |
| FL | 17 cm forward | 30 cm left | 3 cm forward before | 20 / 30 cm |
| FR | 22 cm forward | 35 cm right | 3 cm forward before | 25 / 35 cm |
| BL | 30 cm backward | 15 cm left | None | 30 / 15 cm |
| BR | 35 cm backward | 23 cm right | 2 cm reverse after | 35 / 25 cm |

Padding is physical centimetres and is never multiplied by grid scaling.
Consecutive straights combine or cancel, while turns/SNAPs preserve boundaries.
The 90-degree STM commands and firmware remain unchanged. For example, 20 cm
forward followed by FR emits one 23 cm straight and a `right,90,forward,0` turn,
with speed 50 supplied by the Pi. The guide's speed20 and zero-angle examples
were corrected to match the actual interfaces. [Supplied guide](algo_and_turn_compensation_guide.md)
is preserved as a historical input, not the authoritative startup procedure.

## Startup and test

1. Stop other robot controllers and serial monitors. The Pi's KC monitor is a
   separate serial owner; it must not run alongside Task 1.
2. Start the laptop launcher from the repository root:

   ```powershell
   .\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe
   ```

3. Check `http://127.0.0.1:8000/status`: profile `turn-guide-speed50-grid5`, speed50,
   grid5, input10, arena40. CV health is `http://127.0.0.1:8001/health`.
4. In SSH on the Pi:

   ```bash
   cd /home/nat/SC2079-MDP/rpi
   LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
   ```

   Replace that address if the laptop's hotspot IP changes. The launcher checks
   the laptop profile before opening Bluetooth/serial. Route planning may take
   up to 180 seconds to return; the connection timeout is 3 seconds. The separate
   motor-completion watchdog remains 60 seconds.
5. Connect Android. Set the physical robot and tablet to the same start pose.
   Verify obstacle coordinates/facing, then **Send Map**. Check the target count
   and any omitted obstacle IDs, wait for **READY**, then press **Start**.
6. Begin with a clear single-target layout. Check all four compensated turns
   separately and a short route. Record measured displacement, heading and
   overshoot before using a full obstacle layout. Software checks do not certify
   physical accuracy; small pads may behave differently under battery/floor changes.
7. The tablet icon changes after matching motor completion, using rounded 10 cm
   planned positions. Exact 5 cm coordinates are retained for replanning. There
   is no continuous physical position sensor/odometry feed.
8. After FIN, open `http://127.0.0.1:8001/stitch`. It saves a montage in
   `output/cv-trial/stitched-<run hash>.jpg`. Originals/boxed captures remain there;
   Pi raw captures go to `/home/nat/SC2079-MDP/rpi/captures/`.

The professor allows route calculation before Start. The 180-second request
allowance does not extend the official two-minute preparation period; prep
overtime still consumes Task 1 runtime. No global six-minute auto-stop was added.

## Software validation

28 Algo/CV tests and 7 laptop-launcher tests passed locally. The 59-test Pi suite
passed on the Pi, including a real interrupted reconnect wait and worker/Manager
shutdown regression with all hardware mocked. Windows skips that Linux test.
Tests cover all turns in all headings against the supplied measurements,
5 cm distances, merge barriers, endpoint alignment, directed-path cost agreement
with independent A*, maximum target coverage, unreachable-edge rejection,
precise revisits, MAP/READY/BEGIN and matching completion handling.

Local planner probes: one target about 0.13 s; five targets about 0.20 s; the
last tablet eight-obstacle layout about 0.27 s with **7/8 targets**. Obstacle 7
has no reachable visit under the current clearance/turn model. A synthetic crowded
layout yields 4/8. These are software route results, not proof that a physical
layout is impossible. The Pi reports partial coverage before Start.

The planner now computes directed paths once per source and chooses view poses
and order in one dynamic program, avoiding repeated searches and thousands of
TSP solves. CV weights, confidence threshold and STM firmware were not changed.
Android installation and a physical movement trial still need verification.

## Backups and rollback

Before the 5 cm changes, all Task 1 source changes were saved in Git commit
**`37367242`**, retaining the old 10 cm planner and profile switches. A separate
source/reference ZIP is `output/repo-review-2026-10-07/pre-grid5-37367242.zip`.
The earlier uncommitted-file archive, binary patch, hash manifest and original
runtime status are in the same local output directory. Generated output is not
committed to GitHub.

To recover a source snapshot, export commit `37367242` into a separate directory
and use its paired Algo/Pi files. The backup runtime status records that the old
server ran original/speed50 with both speed50-v2 overlays. Do not mix that old
planner with the new launcher's grid/profile guard. Avoid resetting the shared
repository or deploying the entire archive over newer teammate files.

The Pi backup is `/home/nat/mdp-backups/task1-grid5-2026-10-07/`: `before-source.tar.gz`,
`before-manifest.json`, `capture-manifest.json`, staged packages, test logs and
`deployment-result.json`. Local copies are in `output/repo-review-2026-10-07/`.
The result manifest records every moved file, so old paths can be restored
without overwriting unrelated files. The earlier Task 1 deployment backup is
still `/home/nat/mdp-backups/task1-update-2026-10-07-7d75a8d3/`. Historical source moves preserve contents
rather than deleting them. Current Task 1 is `task1_main.py` â†’ `task1_controller.py`;
old A5/sensor scripts and Task 2 variants are separated from those entry points.
