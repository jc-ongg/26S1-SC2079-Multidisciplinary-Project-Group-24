# Smooth O1 with reverse finish (Claude, 2026-10-15)

Hexes in `Release/` (v1 and v2 are in `old_builds/`):
- `KC_Task2_Claude_SmoothO1_v3.hex`: smooth O1, O2 face speed 16, side IR every 100 ms (same O2 as `SafeO1_v5`)
- `KC_Task2_Claude_SmoothO1_FastO2_v3.hex`: smooth O1, O2 face speed 24, side IR every 50 ms (same O2 as `SafeO1_FastO2_v5`)

## v3: fixed reverse radii (after 2 o1 runs and 4 full runs on v2)

v2 ended within 0.05 to 1.0 cm of centre on LEFT arrows. On one RIGHT arrow it ended 2.4 cm off centre and 3.4 cm past its planned spot. Reverse-arc chords give right lock 30.2 to 30.8 cm (4 runs) and left lock 23.4 to 23.6 cm (2 runs). These did not follow the forward-turn radius scale measured in the same runs (0.93 to 1.07). v3 therefore uses fixed reverse radii of 30.4 and 23.5 cm. `F_RF_RADIUS` is still logged but no longer applied. The LEFT end target is 7 cm (was 8).

v3 simulation at a 60 cm gap, O2 100 cm wide, robot-like sweep:
- LEFT: 2.8 cm from O1, 5.0 cm from O2
- RIGHT: 5.6 cm from O1, 8.9 cm from O2

31,080 of 31,104 cases keep 3 cm or more. The stress sweep has no collisions. On the robot, the first v3 O1 RIGHT run ended 1.0 cm off centre (v2 was 2.4).

All 4 v2 full runs did the smooth O1 cleanly:
- Run 3 stopped at O2 with `ERROR_VISION`: no arrow read at 27 cm.
- The other 3 drove into the carpark centred, 11.5 cm from the back wall, then stopped with `ERROR_PARK_WALLS`: the side IR did not confirm two walls 41 to 59 cm apart in total. The parking code is unchanged from SafeO1.

The same happened on the first v3 full run (O1 RIGHT, 129 cm away; O1 ended 1.0 cm off centre). The side IR at the end read left 43.5 to 44.9 cm and right 68.8 to 104 cm. In a 60 cm carpark each side should read about 20 cm. The test area's carpark side walls were improvised from textbooks that day, which the side IR does not see as two walls ~20 cm away, so `ERROR_PARK_WALLS` here is the test setup, not the firmware. With proper walls, the earlier SafeO1 parks passed this check (side readings adding up to 48 to 51 cm).

## v2: fixes from the first robot runs (3 LEFT runs, O1 ~62 cm, 2026-10-15)

v1 cleared O1 every time but ended ~25 cm right of centre, beside O2.

- **Brake.** The brake on the diagonal did not brake. The motor timers' outputs are active-low, so `motorStop()` sets both driver inputs low and the wheels coast. The car rolled 26 cm, then another 7 cm while the lock was being set. The brake modes now hold both inputs high (`motorForwardA(0)`/`motorForwardB(0)`), which is a short brake. After stopping, if the stop point is more than 1.5 cm off, the car makes up to 2 short straight moves at PWM 1600 to correct it (`F_RF_NUDGE/<cm>/<n>`, mode 14).
- **Reverse radius.** A 71° reverse on the right lock measured R = 30.8 cm, so reverse radii are now the rolling ones (24.3 / 30.5).
- **Turn-in.** From the lane, the robot's heading came round ~7 cm later than modelled. The simulation now has a turn-in delay fitted to the 3 runs (forward 34.5 cm for 55°, as on the robot), and the firmware predicts it (`KCF_RF_TURNIN_DEAD`).
- **Steering start.** The swerve starts 12 cm earlier (46 / 47 cm from O1), so a slower servo still settles in the lane before O1. The car must start at least 49 cm from O1.
- **Simulation.** The steering lag sweep is now centred on the robot's measured `F_RAMP` (24.2 to 24.8).

v2 simulation, 31,104 cases per sweep, 60 cm gap, O2 100 cm wide:

| Sweep | O1 arrow | Closest to O1 | Closest to O2 |
|---|---|---|---|
| Robot-like (`ROBOTLIKE=1`): radius ±3%, steering ±8%, turn-in ±1 cm, 3 brake strengths | LEFT | 3.7 cm | 3.4 cm |
| Robot-like | RIGHT | 5.6 cm | 7.7 cm |
| Stress: radius ±8%, steering ±20%, turn-in ±2 cm | LEFT | 2.7 cm | 1.3 cm |
| Stress | RIGHT | 3.8 cm | 3.9 cm |

The stress sweep has no collisions.

At the 2026-10-15 robot setup (O1 63 cm away), the nominal runs are:
- LEFT: 7.0 cm from O1, 6.7 cm from O2, stopped 6.4 s after START
- RIGHT: 8.7 cm from O1, 10.2 cm from O2, stopped 6.1 s after START

The fallback is still `KC_Task2_Claude_SafeO1_FastO2_v5.hex` (or `SafeO1_v5`). The return and parking code is the same in all four.

## What it does

The approach and swerve out are the v5 ones, except that steering starts 8 cm earlier (42 cm from O1 on LEFT, 43 cm on RIGHT). The car then holds a 28 cm lane beside O1. After that:

1. **Turn in.** It turns in on full lock at the point where its prediction lands the rear bumper 7.5 cm (LEFT) or 10.5 cm (RIGHT) past O1's back face. The prediction starts from the curvature the gyro measures, because the lane hold is still steering.
2. **Inward angle.** It straightens at 58 to 70° (LEFT) or 57 to 69° (RIGHT). The exact angle is chosen on the fly to land on that target. During this turn it measures its own lock radius from the gyro (`F_RF_RADIUS`).
3. **Diagonal.** It drives straight on the diagonal and brakes where the brake slide plus the reverse arc will end on the centre line.
4. **Reverse.** It sets the same lock while stopped (300 ms), then reverses on it back to heading 0, slowing down for the last 20°.
5. It brakes and hands the pose to the O2 code. Stage `o1` stops here.

At a 60 cm gap it ends with the front about 25 to 33 cm from O2, which is the distance the O2 code wants.

## Simulation (`test_fast_o1.c`, not a robot test)

The simulation ran 10,368 cases:
- both arrows
- O1 at 60 or 150 cm
- O1 to O2 gap of 60, 75, 100 or 150 cm
- O2 30 or 100 cm wide
- lock radius ±8%, reverse radius a further ±5%
- servo swing time ×0.8 to ×1.25
- three brake strengths
- start ±2 cm off centre
- arrow early or late

All pass. At a 60 cm gap, the worst case is:
- LEFT: 4.0 cm from O1, 4.0 cm from O2
- RIGHT: 6.7 cm from O1, 6.8 cm from O2

It always ends within 2.4 cm of the centre line and 2.7° of straight. With O1 100 cm away it stops about 6.3 s (LEFT) or 5.9 s (RIGHT) after START. The simulated steering is linear in curvature, which matches the yaw trace from the servo test (the midpoint was reached ~55% of the way through a left-to-right flip).

## New log lines

- `F_RF_XEND/<predicted end>/<target>`: turn-in decision (rear-axle cm)
- `F_RF_TURN/<predicted end>/<diagonal heading>`: straighten decision
- `F_RF_RADIUS/<scale>/<reverse radius cm>`: measured lock radius (1.00 = as calibrated)
- `F_RF_BRAKE/<predicted end>/<speed>`: diagonal brake point
- `F_MODE` 9 diagonal, 10 brake, 11 set lock, 12 reverse, 13 brake, 8 done
- `ERROR_F_REVERSE`: the reverse arc ran too long or for more than 4 s. `ERROR_F_DIAGONAL`: the diagonal went past 50 cm. `ERROR_F_OFF_CENTRE`: O1 ended more than 8 cm off the centre line (v2). All three stop the car.

The car must start at least 51 cm from O1 (`ERROR_START_TOO_CLOSE`).

# Fast O1 build (Claude, 2026-10-14)

Copy of `KC_STM32_Task2_Trial` (Long_Return_Fix) with a new O1 section. O2, the return and parking are unchanged. Your original folder is untouched.

Firmware to flash: `Release/KC_Task2_Fast_O1_v5.hex`. Don't use the older builds.
Pi files: `pi/` (same bridge, three changes, listed below)

## What changed

O1 no longer stops at 32 cm, and it no longer runs seven separate turn and straight commands with a stop after each one.

1. At the start the robot takes one ultrasonic reading of O1 and asks the Pi for the arrow straight away.
2. It drives at PWM 4500 with gyro heading hold and lane hold, then slows to PWM 3550 for the last 30 cm before the swerve. If the arrow still isn't known 15 cm before the swerve point, it stops and waits for it.
3. When the front is 30 cm from O1, it starts the swerve. The steering switches come from encoder and gyro odometry, not fixed angles:
   - Full lock toward the arrow side, until the model predicts that switching now lands the rear axle 28 cm off the centre line.
   - Opposite lock until the heading is nearly straight, then hold that 28 cm lane beside O1.
   - Once the rear axle is 3 cm past O1's front face, turn back. Switch when the prediction says the car lands on the centre line, then hold the centre line.
4. It stops, sets the pose for the existing O2 code, and continues as before. Stage `o1` stops here.

The car measures its own servo swing on every swerve. It times the yaw rate passing between the two lock rates and uses that measurement for the turn back. The log line is `!KC_T2/F_RAMP/<cm>/<speed>;`.

Lock turns copy `motorTurn`: the outer wheel drives and the inner wheel is braked. In a lock turn the braked wheel skids, so odometry uses the driven wheel plus the gyro (`KC_HALF_TRACK_CM`, an estimate of 8 cm).

## Files changed

- `Core/Inc/kc_fast_o1.h`: new. The fast O1 phases, the prediction model and the lane hold.
- `Core/Inc/kc_task2.h`: fast phases added to the state machine. `KC_FAST_O1 0` restores the old O1.
- `Core/Src/kc_task2_platform.inc`: odometry, continuous drive (writes the servo register directly, because `setServoAngle` blocks for 100 ms), and the new `drive` hook.
- `Core/Src/main.c`: `TASK2KC` accepts stage 4 (servo test).
- `pi/KC_task2_stm32_bridge.py`:
  - keeps the camera open with Picamera2 (falls back to `rpicam-still`)
  - keeps classifying frames while the robot drives; accepts two agreeing frames ≥ 50%, or one ≥ 75%
  - new `--stage servo-test`
- `test_fast_o1.c`: host simulation (see below).

## Servo test results (robot, 2026-10-14, two runs)

- PWM 3550 ("speed 50") is 58.5 cm/s on a straight and 42 to 48 cm/s in a lock turn.
- Rolling lock radii are 24.3 cm left and 30.5 cm right. The standstill measurements gave 22.2 and 28.8.
- The yaw rate takes about 0.46 s to go from full left to full right, which is about 22 cm of travel. The firmware now uses these values. If the robot can't start, v2 and later print which start check failed.

## First O1 run (robot, arrow RIGHT, O1 61 cm away, v3)

The camera read RIGHT at 0.81 confidence on its first frame while the car was moving. From START to stopped took 4.1 s. The measured steering lag hit the old 25 cm cap at about 52 cm/s, so the car swung out to a 32.5 cm lane. Braking hard with the wheels steered left it 4.2° off. v4 allows up to 40 cm of lag, starts from 26 cm, and coasts to a stop with steering held. The simulation's switch points matched this run to within about 4 cm.

## O1 runs on v4 (robot, 2 RIGHT + 2 LEFT, O1 about 60 cm away)

All four passed O1 without contact, in 4.5 to 4.8 s from START to stopped. The arrow was read while moving or before START every time. The RIGHT swerve came close to O1's near corner.

- Steering lag depends on direction: right lock to left takes about 30.4 cm, left to right about 22.7 cm. v4 applied the first swing's value to the return swing, which goes the other way.
- Holding the centre line at the end was too weak with equal power on both rear wheels. The car stopped 3 to 6 cm off the line and 3.4 to 5.9° crooked.

v5 changes:
- separate lag values for each swing direction
- inner wheel slowed in proportion to the steering during straight and line-hold driving
- the swerve starts 34 cm (LEFT) or 35 cm (RIGHT) from O1, instead of 30

## Simulation results (not a robot test)

`cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc test_fast_o1.c -lm -o /tmp/t && /tmp/t`

The simulation covers 324 cases: both arrows, O1 at 60, 100 or 150 cm, turn radius off by ±8%, a left-to-right steering lag of 17, 22.7 or 28 cm (right-to-left scaled the same way), the robot starting 2 cm off centre, and the arrow arriving early or late. All 324 cases finish within 5 cm and 4° of the centre line. The closest any case gets to O1 is 6.8 cm. The simulation's switch points matched the real v3 and v4 runs to within about 4 cm.

With O1 100 cm away and the arrow known early, the car stops on the centre line about 4.6 s after START. It finishes with its front about 78 cm past O1's back face.

## Known limit: O1 to O2 gap under 90 cm

The swerve ends with the front about 78 cm past O1's back face. At 60 or 70 cm the simulation hits O2. **Test only with O2 at least 90 cm behind O1, or with O2 removed.** The fix is to estimate O2's distance from the arrow's size in the camera image while the car is beside O1. That needs a calibration from your Pi logs and is the next step.

## Test order

On the laptop, start the arrow API (`./KC_start_task2_arrow_api.sh` in `MDP ChatGPT`). Copy `pi/` to the Pi, flash `KC_Task2_Fast_O1.hex`, and keep the robot still during gyro calibration.

1. Servo test, with about 1.5 m of clear floor ahead and 40 cm to the left:
   `python3 KC_task2_stm32_bridge.py --execute --stage servo-test --port /dev/ttyACM0 --laptop-ip 192.168.50.218`
   It drives 25 cm, turns left to 35°, flips to full right and stops. Send me the printed `F_SPEED`, `F_FLIP`, `F_YAW` and `F_FLIP_DIST` lines.
2. O1 only, with O2 at least 90 cm behind O1 (or removed). Test both arrows, and O1 at 60 cm and at 150 cm:
   `python3 KC_task2_stm32_bridge.py --execute --stage o1 --port /dev/ttyACM0 --laptop-ip 192.168.50.218`
   Check the closest point to O1 and where it stops. Send me the `F_` lines and the `KC_native_task2_*.jsonl` log.
3. Only then try `--stage full`.

Ctrl+C sends STOP. Rollback: flash `KC_STM32_Task2_Trial/Release/KC_Task2_KiaChun_Long_Return_Fix.hex` and use the old `pi/` bridge.
