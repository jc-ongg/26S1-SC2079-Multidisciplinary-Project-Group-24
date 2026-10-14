# Kia Chun — native STM32 Task 2

Standalone STM32CubeIDE project for STM32F407VETx. STM32 owns the route and sensor decisions; the included Pi bridge handles arrow recognition and heartbeats. Existing group STM32 firmware is not replaced.

## Current firmware

Latest trial: `firmware/KC_Task2_Parking_Entry_Geometry_Trial.hex`, already built with STM32CubeProgrammer-compatible Intel HEX output. **Parking remains unresolved: Kia Chun reports that the carpark still has an issue after this trial. This is work in progress for review, not a validated parking fix.** No new trace or precise latest contact location has been supplied yet; see `KNOWN_ISSUES.md`.

Includes the earlier O1 acceptance/fine-positioning work. Host regression tests and ARM Release compilation passed; they do not establish physical clearance. Earlier firmware had successful bypass/parking runs, but that does not validate this trial.

Included fixes:
- O1 verified acceptance30–34cm, nominal32cm; compensate the heading-zero side-lane straight by accepted gap minus32cm before the inward turn (whole-centimetre rounding).
- O1 RIGHT extra clearance,26cm inward diagonal and reverse finishing turn; separately calibrated LEFT route.
- Minimum2cm positioning corrections, speed12 for corrections up to4cm; cached O2 arrow selects LEFT26±1.5cm / RIGHT28±2cm directly.
- Brake settling and actual completed straight encoder travel; early vision overlaps approach; O2 IR edge searches and RIGHT back-clearance advance.
- Parking entry trial: checks sampled whole-footprint paths against O1, conservative carpark wall envelope and O2 plane; selects an earlier centring start and checks any reposition along the known return lane. Rechecks actual reposition encoder endpoint. Diagonal speed22 replaces100; invalid geometry stops without a blind fallback. Final US/IR parking confirmation remains.
- Carpark assumptions60x50cm, robot23x18.8cm, initial pose fully inside on approximately central heading. Estimated turn sweeps with5cm inflated body are not measured trajectories. The final alignment turn can overlap entrance depth while remaining within the modelled opening; this is not a promise of completed alignment wholly outside the mouth.

See `CHANGELOG.md` for detailed design history and `PHYSICAL_TESTS.md` for earlier test records. Turn translations and O1 endpoint anchors remain calibration estimates. O2 width is found using IR edges; the current approach/reverse geometry assumes at least60cm O1-rear to O2-front spacing. Robot length23cm, width18.8cm; calibration is specific to this robot. Turns and O1 bypass run at50; selected long transit commands at100. Do not transfer these values to a different chassis without calibration.

## Edit and rebuild

1. In STM32CubeIDE, choose File → Import → General → Existing Projects into Workspace.
2. Select this directory and import `KC_STM32_Task2_Trial`.
3. Select the **Release** build configuration and build. The IDE generates its own makefiles under `Release/`.
4. Flash the newly generated HEX. Rebuilding does not update the named checked-in HEX automatically: copy your result to a new descriptive filename and update checksums when publishing it.

Main edit locations:
- `Core/Inc/kc_task2.h`: native state machine, ranges, route and recovery logic.
- `Core/Src/kc_task2_platform.inc`: integration with motors, encoders and gyro.
- `Core/Inc/kc_stop_settle.h`: completed-motion braking/settling.
- `Core/Inc/kc_entry_geometry.h`: new sampled parking entry planner and geometry assumptions.
- `Core/Inc/kc_forward_profile.h`: straight speed profile.
- `Core/Src/main.c`: full integrated firmware; includes the adapter. Copying main.c alone is insufficient.

## Run on the Pi

Install the bridge dependency with your usual Python environment (`python3 -m pip install pyserial`). Run the v11i image API on the laptop at port8001; model weights are not included here. Ensure no other program owns `/dev/ttyACM0` or the camera. Keep the robot still during gyro startup.

From a clone of this repository on the Pi:

```sh
cd STM32/KC_STM32_Task2_Trial/pi
python3 KC_task2_stm32_bridge.py --execute --stage o1 --port /dev/ttyACM0 --laptop-ip YOUR_LAPTOP_IP
```

Replace `YOUR_LAPTOP_IP`. After validating both O1 directions, use `--stage full` for the complete route. Other options: `o2-prepare`, `o2-bypass`. **Every selection starts from the original parking/start pose; none resumes a previous stage.** Ctrl+C requests STOP; heartbeat loss stops the native run. The old HTML monitor is not the launcher for this firmware. Logs and camera photos are written in the working directory. Acceptance remains50% for LEFT/RIGHT recognition. This included bridge requires **Protocol2** (`BUILD/2/0`), matching the checked-in HEX files. A separately installed Protocol4 controlled-motion bridge cannot run these builds; do not bypass the identity check. On Kia Chun's Pi, the matching copy is under `KC_STM32_Task2_Trial/protocol2_compatible/`.

## Validation

From this project directory, with a host C compiler:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g test_task2.c -lm -o /tmp/kc_task2_test
/tmp/kc_task2_test
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined -g -I Core/Inc test_platform_settle.c -lm -o /tmp/kc_platform_test
/tmp/kc_platform_test
python3 -m unittest discover -s pi -p test_bridge.py
```

Current tests cover all four arrow combinations, sensor/heartbeat/motion guards, braking adapter, parking recovery, O1 acceptance boundaries and actual-gap compensation, plus reported unsafe entry rejection, reposition path checks, no-path/unknown geometry, actual reposition revalidation and diagonal travel limits. Latest ARM size: text78700,data544,bss29248 bytes. Existing legacy compile warnings remain. Host tests cannot prove swept wheel clearance or parking accuracy.

First physical check: O1 LEFT and RIGHT from original start, including accepted gaps near30/34cm, checking the full rear-wheel sweep, centred endpoint and O2 clearance. Then full run.

## Rollback and integrity

`firmware/KC_Task2_O1_Acceptance_Compensation_Fix.hex` is the preceding build without the parking entry planner. `firmware/KC_Task2_Fine_Positioning_Fix.hex` predates wider O1 acceptance. `firmware/KC_OLED_Flicker_Fix.hex` restores the earlier OLED-fixed legacy firmware; use its original Pi monitor rather than this native bridge. No board has been flashed during this publication.

`SHA256SUMS.txt` records current native source, bridge and included HEX files. Verify with `shasum -a 256 -c SHA256SUMS.txt` (macOS) or `sha256sum -c SHA256SUMS.txt` (Linux).
