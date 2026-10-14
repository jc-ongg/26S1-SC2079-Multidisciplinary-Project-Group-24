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
