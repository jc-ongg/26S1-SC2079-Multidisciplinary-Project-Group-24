# Smooth O1 with reverse finish (Claude, 2026-10-15)

**Assessment build (tentative, 2026-10-16): `Release/KC_Task2_Claude_SmoothO1_FastO2_v19b.hex`** (SHA-256 `e256482562b4b927f23278e8bf77f8ecbc26f6012c96ad26dedce55b987996ac`). Run it with the teammate's Pi-driven return:

```
python3 KC_task2_stm32_bridge.py --execute --stage full-pi-return --port /dev/ttyACM0 --laptop-ip 192.168.50.218
```

Robot, 2026-10-16: 22 of 24 full runs parked, 22.8-28.9 s from START to parked (O1 + O2 on the STM32 14.8-20.6 s, Pi return 5.8-10.7 s), both O2 arrows, O1 60-150 cm away, O1-O2 gaps 60-150 cm, several O2 lengths. `pi/` holds the Pi files copied from the robot (`~/KC_STM32_Task2_Fast`) on 2026-10-16 after those runs:

- `KC_task2_stm32_bridge.py`: the teammate's fast-carpark bridge (commit b94472f) with one vision change. A low-confidence arrow no longer ends the run; at a stop, two agreeing frames at 35% or more decide it.
- `KC_task2_pi_return.py`: the teammate's newer Pi return (needs numpy), much changed from b94472f.
- `KC_task2_return_test.py` and `test_pi_return.py`: the teammate's return tests. `test_pi_return.py` passes (144 of 144 simulated runs parked).
- `KC_stm32_link.py`, `KC_range_calib.py` and `test_bridge.py` were already the same as the robot's.

One `test_bridge.py` test fails: it still expects a RuntimeError on a low-confidence arrow, which the bridge no longer raises. On the Pi, the bridge and the Pi return were last edited partway through the evening's runs (19 run logs are newer), so the earliest of the 24 runs may have used an earlier edit.

`laptop/` holds the arrow server the bridge sends its photos to (`--laptop-ip`, port 8001), as run on the laptop for those runs: `KC_start_task2_arrow_api.sh` and `KC_task2_arrow_api/` (`main.py`, `model_loader.py`, `test_api.py`, README). The model is not in the repo: it is a third-party checkpoint whose upstream licence governs redistribution. Download `runs/detect/train/weights/best.pt` from https://github.com/PranavU-Coder/arrow_detection at commit `335c0dc275769edd902c5bb41a99992fa12fff9c` and save it as `laptop/KC_task2_arrow_api/models/best_yolo11m_arrows.pt`. `main.py` refuses to start unless its SHA-256 is `5098030850b53b7a49ae7ef6ce3942e25d4863f5308dd71c814c7cd4330fd821`. The start script defaults to a Python path on Kia Chun's laptop; elsewhere set `KC_TASK2_PYTHON` to a Python with PyTorch 2.6 or later, Ultralytics (YOLO11), FastAPI, uvicorn, python-multipart and Pillow. `test_api.py`'s replay test needs saved O2 photos that are not in the repo; the selector tests run without them.

The two runs that did not park:
- O1 at 114 cm: the first O2 echo after the O1 part read 151.8 cm (the far wall; O2 was ~88 cm away). The 100 cm approach at speed 100 stopped about 4 cm short of O2 on the close-echo guard, then the STM32 reset.
- A narrow O2 with the arrow RIGHT: the back-face search found the face, stopped on its edge, and the standing check flickered for 2 s (`ERROR_BACK_NOT_ACQUIRED`).

Fixes for both are being tested separately (v19c, not on GitHub yet).

Older builds: `KC_Task2_Claude_SmoothO1_FastO2_v17.hex` (rolling U-turn), `..._v16c.hex`, `..._v15.hex`, `..._v13.hex` (robot-tested, firmware return).

## v19b: wider O2 distance band

v19 on the robot (3 runs with the Pi return, the ~150 cm O2): O1 RIGHT + O2 LEFT parked twice in 25.8-25.9 s (native O1+O2 15.1 s; O2's arrow came back during the O1 brake both times). O1 RIGHT + O2 RIGHT stopped with `ERROR_POSITION_LIMIT`: in front of O2 the ultrasonic read 33.3, 21.3, 33.1, 24.6 and 31.5 cm while the face positions those imply stayed within about +-2 cm of 143, and the 26-31 RIGHT band made it creep back and forth five times.

- Both arrows now accept up to 34 cm (LEFT 24.5-34, RIGHT 26-34). The lower limits stay: they keep the corner of the first 90 off the face. At 34 the side IR reads about 20 cm along the face.
- Corrections aim at 30, mid-band, so a few cm of echo noise after a move stays inside it. That run's first reading (33.3) is now accepted with no correction.

## v19: quicker O2 scan, faster face search on a narrow O2

v18 on the robot with the Pi return (4 runs, O2 LEFT, O2 about 20-25 cm wide): native O1+O2 16.9-18.4 s, the rolling U-turn in 2.2-2.5 s every time.

- **O2 scan.** The fast O1 ends at a standstill about 28-31 cm from O2. Each first 90 needs a range: LEFT 90 (15 cm forward) gives side-IR ~12-19 from 24.5 to 32 cm; RIGHT 90 (20.5 cm forward) the calibrated 26-30, now 26-31. Before the arrow is known, a range that suits either arrow is accepted (`o2_ranged` records which); if the arrow then needs the other, O2_TARGET corrects toward 28. The old 26.5-29.5 (no arrow) and 24.5-28.5 (LEFT) bands made the robot creep 2-3 cm (0.6-0.8 s each, twice in one run). The three v18 runs stopped at 31.1, 29.4 and 29.7 cm: all three now turn at once for their LEFT arrow (about 0.6-1.3 s saved).
- **Face search on a narrow O2.** After the first 90 the robot is past O2's end. Three open side-IR readings (0.15 s) now start the reverse search at once (was a 0.4 s wait), at speed 40 (was 16: 2.0-2.4 s for 24-28 cm). Log: `FACE_SEARCH/<IR>/<x>`. The rest is unchanged: the IR finds the face, the robot drives on until it loses it and rolls into the U-turn. About 1.5 s saved.
- **O2 photo during the O1 brake.** The O2 arrow is requested (`CAPTURE_EARLY/2`) when the fast O1's reverse finish starts braking: the robot already faces O2 within ~2 degrees and is almost stopped. That is ~0.5 s before O2_RANGE used to ask; in the v18 runs the reply took ~0.3 s after the request and the first O2 turn waited for it. O2_RANGE keeps this request, and the Pi still switches to its stationary rule (one photo at 50%) when the usual `CAPTURE/2` follows. Not sent for an O1-only run (cap 1).
- `test_task2.c` replays the v18 scan ranges with LEFT and RIGHT arrows and the faster search; `test_fast_o1.c` checks that the O2 request comes before `F_O1_DONE` for both O1 arrows.

## v18: fast-carpark's return and O2-end fixes

The `fast-carpark` branch (Tan Jou Yuan, commit b94472f on top of v11) adds a Pi-driven return and two firmware fixes.

- **Pi return, unchanged.** `pi/KC_task2_stm32_bridge.py`, `pi/KC_task2_pi_return.py` and `pi/test_pi_return.py` are copied byte for byte. `--stage full-pi-return` runs O1 and O2 on the STM32 (cap 3: it stops after O2's return turn), reads O1's and O2's fronts, the final pose and the back face's end from the `!KC_T2` lines, and drives home with plain FWD/REVS/TURNL/TURNR commands: one straight at 100 past O1, then one turn and one straight into the carpark when that fits, else a 45 or 90 degree S entry, then an ultrasonic park. Needs numpy on the Pi. `test_pi_return.py`: 144 of 144 simulated runs park.
- **It reads v17/v18 logs correctly.** The new phases (36 U-turn, 37 back search) were added after the old ones, so the states it uses (4, 15, 16) keep their numbers. On a simulated v18 run (O2 RIGHT, rolling U-turn) its parser gave O1 front 71.5, O2 front 151.5, the pose after the return turn and the back face's end at the mirrored far end, all correct. Its `side2` flag looks for state 25 (the old 10 cm back clearance) and stays unset with the U-turn; nothing uses it.
- **O2's end test (firmware).** The end counts as passed when the side IR reads 15 cm more than the face it was following (30 to 60 cm), not a fixed 60. On their robot run the IR read 19-20 on the face, then 43-62 from an object about 50 cm beyond O2's end; the fixed 60 never held three times and the car drove 95 cm into it. Readings between the face and that level keep the count. The face distance comes from the acquisition readings and a running average along the face; the back face starts from its acquisition (standing or rolling).
- **Wall stop (firmware).** Two forward echoes within 20 cm while following a face end the run with `EDGE_WALL` and `ERROR_EDGE_WALL` instead of driving on.
- **Pi return fix after the first v18 run** (O1 RIGHT, O2 LEFT, narrow O2, lane 46.8): on the O1 pass at speed 100 one side-IR reading of 21.8 cm (O1 should have read about 36) moved x 14.6 cm toward O1. The slant planned from there ran the car's left side into the carpark's side wall at the opening. `pi/KC_task2_pi_return.py` now:
  - lets one reading move x by at most 6 cm (`O1_SINGLE_FIX_MAX`); a larger fix (up to 15) needs two readings in a row within 4 cm;
  - takes, of unconfirmed readings, the one nearest dead reckoning, and logs every reading in the O1 window (`O1 pass IR readings: [...]`);
  - keeps 3 cm extra from O1 even after a fix (`O1_FIXED_MARGIN`; 5 without one, as before). The replay of that layout passed O1's corner at 1.9 cm in the 45 deg S with an exact pose;
  - keeps the slant's body 4 cm from the opening's side walls (`SLANT_SIDE_MARGIN`, was 2).
  `pi/test_pi_return.py` requires 3 cm from O1 (was 2) and replays the robot layout with the stray reading. 144 of 144 layouts and 12 of 12 replays park; the closest O1 pass is 3.2 cm. The firmware is unchanged.
- **Not taken:** their back face at speed 100 with the overshoot reversed off. v13's back face (90 until 45 cm before the mirrored far end, then face speed) is robot-tested and needs no reverse.
- `test_task2.c` has their robot-run replay (ends on the third clear reading, into the rolling U-turn) and the wall stop.

## v17: no stop before or after the O2 U-turn

- **Rolling in.** When the front face ends (3 IR readings over 60 cm), the U-turn starts at face speed without braking: the pose is taken from the face move's travel, the legacy PID is released and the steering swings in from centre. `o2_end_x` gets the 10 cm the brake used to add, so the back face's fast part ends where it did before. Log: `UTURN_ROLL_IN/<speed>/<x>`.
- **Rolling out.** On the straight at 180 degrees the side IR is read on the move. Two readings 10 to 40 cm apart by at most 3 cm, heading within 3 degrees and at least 4 cm run: the pose comes from odometry (no brake, no skid) and the back face starts at once (`UTURN_ROLL_OUT/<back IR>/<speed>`, then `BACK_FAST` as before).
- **Fallbacks.** A reading under 10 cm, two open readings once the heading has settled (narrow O2), or 30 cm without the face: brake, then the standing check and the back-face search of v16c.
- **Hand-overs in the adapter.** Into the U-turn, `kc_platform_drive` stops calling the legacy PID, commits its `currentAngle` into `globalHeading` and keeps the wheels' PWM. Out of it, `kc_platform_start` commits the drive's angle before the PID's init zeroes it. `test_platform_settle.c` checks both.
- **Simulation** (`test_task2.c`, 162 cases per route and start): rolling U-turns take 2.0 s (RIGHT route) and 2.1 s (LEFT route) from the face end to the back face, against about 2.2 to 2.4 s plus a brake and a standing check before. Shifts are unchanged (55.0 to 58.1 and 56.0 to 66.5 cm). Measured from where the face end was detected, the outer front corner reaches 44 to 54 cm, about the same as braking first (the brake adds about 9 cm). With a slow servo at top speed, 2 of 162 RIGHT-route cases brake instead of rolling out.
- The test's back face now ends at the near end's mirror image (O2 centred), not a fixed 30 cm after the back-face move starts.
- `KC_O2_UTURN_ROLL 0` restores v16c's brake before and after.

## v16c: narrow O2 back face, start-up PING

v16 on the robot (O1 LEFT, O2 LEFT, O2 about 25 cm wide): the RIGHT U-turn worked (58.2 cm over, 0.7 degrees off, 3.5 s) but stopped at x 12.5. The continuous U-turn ends about 12 cm further along the back face than the legacy 180 (the steering centring at speed, then the brake slide). That was past this O2's far end, the side IR saw nothing for 2 s, and the run ended `ERROR_BACK_NOT_ACQUIRED`. The RIGHT route on v15 would do the same with an O2 this narrow.

- **Back-face search.** If the side IR shows nothing beside the robot for 0.5 s after the corner, it reverses at speed 16 along the back lane (`BACK_SEARCH/<IR>/<x>`, up to 45 cm). Two face readings (10 to 40 cm) brake it (`BACK_FOUND`), and the standing check and the back face continue as before. If the face never shows, the run ends `ERROR_BACK_NOT_ACQUIRED` (or `ERROR_BACK_SEARCH` on a guard). It runs at most once per run and never on an O2 wide enough for the IR to see the face where the corner ends.
- **Start-up PING.** The STM32 sends its start-up events before it marks the run active, and the Pi's first heartbeat can arrive in between: one `!<id>/ERROR/TASK2KC_TOKEN;` at 0.1 s (harmless, the next heartbeat 0.5 s later counts). A PING with the current run's token is now accepted at any time.
- `test_task2.c` replays the robot run: no face, reverse, face found after 10 cm, back face started. A second case reverses the full 45 cm and fails.

## v16: U-turn on both routes, faster return planner

v15 on the robot (O1 LEFT, O2 RIGHT, 2 runs, both parked): the U-turn shifted 59.9 and 59.1 cm (predicted 57.0 and 56.7), ended 0.3 and 1.0 degrees off 180, took 3.3 s, and the back IR read about 23 cm.

- **LEFT route.** The legacy RIGHT 180 (4.0 s, slow last 30 degrees) is now the same continuous U-turn, mirrored on right lock. Two right-lock quarters already move about 61 cm sideways, more than the legacy 180's 57 (`KCU_SHIFT_R`), so the straight between them is skipped unless the lock radius is small. Simulation, 162 cases: 55.6 to 66.9 cm sideways, within 2.5 degrees of 180, 2.5 s on average and 3.0 s at worst. The back face is about 5 to 9 cm further away than with the legacy 180 (legacy back IR was 14 to 17 cm). The outer front corner reaches about 47 cm past the corner start on the face side, about 4 cm further than the legacy 180.
- `KC_O2_UTURN` is now a bit mask: 1 for the RIGHT route, 2 for the LEFT route, 3 (default) for both, 0 for the legacy corners.
- **Return planner.** In the v15 runs the robot sat still for 4.4 s and 2.9 s after the return turn (`RETURN_CLEAR`) while the STM32 checked 720,000 and 448,000 footprint poses. The trim search asked the entry planner about 30 times with the same template. v16 caches each entry position's result per template and stops that inner search at the first 5 cm safe interval, since the trim only needs a yes or no. The plans are identical: 18,002 random comparisons against the v15 planner gave no difference, and both v15 runs give the same plan (287.50/46 and 200.50/26) from 19,222 and 18,880 checks. Expected wait: about 0.1 s.
- Nothing else on the return or park changed.

## v15: one continuous U-turn at O2's corner (RIGHT arrow on O2)

The RIGHT route used to go round O2's end with LEFT 90, 10 cm forward at speed 30, LEFT 90: three stops and about 5 s (the LEFT route's single 180 takes 4.0 s). The 10 cm is needed because two left-lock 90s move the robot only about 46 cm sideways, which put the back face 7.7 cm from the IR with the wheel nearly touching. A plain continuous LEFT 180 has the same problem, so v15 drives the same shape without stopping (`Core/Inc/kc_o2_uturn.h`):

1. Steer to left lock at standstill (300 ms, as the legacy turn does).
2. Left lock at the swerve PWM (3550) until the heading, plus what the steering still adds while it centres, reaches 90 degrees.
3. Straight, holding 90 degrees, until the model says the second half lands 56 cm sideways (31 + 10 + 15, the old path's shift; back IR about 17.7 cm). The model uses the measured servo lag (time based, as in the fast O1) and this run's lock radius from the gyro. In simulation this straight is 8 to 15 cm.
4. Left lock until the heading reaches 180 degrees the same way.
5. Straight, holding 180 degrees, then brake once within 2 degrees (or after 30 cm).
6. Pose from wheel and gyro odometry plus the brake skid estimate, as for the LEFT route's 180. Then the back face as before (stationary IR check, fast part, end check).

The legacy straights and turns steer to `desiredHeading`, which a continuous-drive turn does not update. A new hook (`KcT2Hooks.goal`, `kc_platform_goal`) sets it to the new nominal heading after the U-turn, so the back face and the return turn aim the right way.

Simulation (`test_task2.c`, 162 cases): lock radius ±8%, servo lag ±25%, swerve speed 45 to 58 cm/s, brake skid 0.5 to 1.5 times the O1 value, heading at the corner ±2 degrees. Every case landed 55.5 to 58.0 cm sideways, ended within 3.1 degrees of 180, and knew its pose within 2.6 cm. The pose error is the skid uncertainty and lies along the back face. The U-turn takes 2.3 s on average and 2.8 s at worst, plus the brake stop. The outside front corner reaches about 41 cm past the corner start on the face side, about 3 cm further than a legacy LEFT 90.

Log lines: `UTURN_START/56/<heading>`, `UTURN_90/<sideways>/<heading>`, `UTURN_STRAIGHT/<straight cm>/<predicted shift>`, `UTURN/<shift>/<straight cm>`, `UTURN_TIME/<ms>/<heading error>`. New errors: `ERROR_UTURN_TIMEOUT` (5 s), `ERROR_UTURN_HEADING` (heading outside -20 to 209 degrees), `ERROR_UTURN_DISTANCE` (170 cm). A heading more than 5 degrees off after the brake gives `ERROR_FINAL_HEADING`, as for the legacy turns.

Building with `-DKC_O2_UTURN=0` restores LEFT 90, 10 cm, LEFT 90. The LEFT route's 180 is unchanged.

Check on the robot: in `UTURN`, the shift should be 53 to 62. The back IR in the first `O2_BACK_ACQUIRE` readings should be about 15 to 20 cm. Below 12 cm, raise `KCU_SHIFT`.

## v14: faster return home, quicker O2 scan

Return home:
- On a lane more than 60 cm out from O1's side (after a wide O2) the side IR cannot see O1, so the slow O1 search is skipped. The robot drives straight to the planned entry start (`O1_POSE_ESTIMATE/<x>/<entry y>`) and the entry is planned from odometry, as v13 already did after a search that found nothing.
- Straights of 35 cm or more on the way home run at speed 100 instead of 22 or 12: clearing O2, the crossing and trim straights, the entry diagonal and the park approach legs. The forward PID still caps PWM by remaining distance (5000 under 50 cm, 3000 under 20 cm, 850 under 10 cm), so each move ends as before. Shorter moves keep their old speed, because the 850 tail makes speed 100 slower over short distances.

O2 scan:
- If O2 was already ranged for the arrow that is then read, the repeat range check is skipped (`O2_RANGE_KEPT`).
- The LEFT arrow's O2 range band is now 24.5 to 28.5 cm, so a stop at 27 to 28 cm needs no correction move.

Expected savings (estimates): about 1.5 s with no O1 search on wide lanes, about 1 s on the long return straights, about 0.5 s on the park approach, about 0.7 s when an O2 correction move is avoided and 0.3 s from the skipped check.

## v13: fast back face (O2 is centred on the O1-O2 line)

A v12 timing log (O1 RIGHT, O2 LEFT, 140 cm O2):
- smooth O1 5.8 s
- O2 approach and arrow 2.8 s
- 90° turn 1.7 s
- front face 1.5 s
- 180° turn 4.0 s
- **back face 4.1 s** (100 cm + stop + 60 cm)
- 90° turn 1.9 s

The pauses between moves are only 0.1 to 0.2 s. The turns' slow last 30° are left unchanged: the measured turn geometry depends on them.

- **Fast part of the back face.** The near end found on the front face (`o2_end_x`, robot pose at the corner stop) mirrors to the far end. The back face runs at `KC_O2_BACK_FAST` (90) until `KC_O2_BACK_SLOWDOWN` (45 cm: about 10 cm of near-end stop overshoot plus 35 cm to slow down) before it. It then switches to face speed 50 without stopping (`BACK_FAST/<cm>/<near end>`, then `BACK_FAST_END`), and the 3-reading end check runs as before.
- **Early end.** During the fast part, the first IR reading over 40 cm brakes at once (`BACK_FAST_BRAKE`) and the normal check continues at face speed. That covers an O2 that is not centred.
- **Narrow O2.** With less than 25 cm of fast part, the whole back face runs at face speed.
- **Speed switch without stopping.** `kc_platform_start` commits `currentAngle` to `globalHeading` before a new straight move starts while one is running; `motorPidForward`'s init zeroes it.
- **Expected saving:** about 1.5 to 2 s on a 140 cm O2.
- **Tests.** `test_task2.c` covers the fast start, the switch without braking, the early-end brake and the narrow-O2 case. `test_platform_settle.c` covers the heading commit.

## v12: faster O2 faces

- `KC_O2_FACE_SPEED` is 50 (about 58 cm/s; was 30, about 35 cm/s), front and back faces.
- The RIGHT route's 10 cm back clearance stays at speed 30 (`KC_O2_CLEAR_SPEED`): it is short and has to stop accurately.
- An engine stop from a straight move now adds the unseen rear-wheel skid to the distance, using the O1 calibration: `KCF_RF_SKID` (5 cm) × (speed / 57 cm/s)². That is about 5 cm from speed 50 and 1.9 cm from 30.
- An O2 end is now found about 18 cm past the edge (about 8 cm at speed 30): 3 IR readings plus brake and skid. That only adds clearance at the corner and moves the return lane about 10 cm further out, which the return planner covers.
- For a 140 cm O2 this saves roughly 2.5 s against v11.
- `test_platform_settle.c` checks the skid addition.

## v11: steady ultrasonic readings are accepted during ranging

v10 robot runs, 2026-10-15: 7 full runs, 6 parked, all with `COMPLETE` and within 1.2 cm of centre in the carpark.
- **O1 RIGHT + O2 LEFT, 2 runs:** both parked (140 cm O2; 90° crossing once, 45° trim once).
- **O1 LEFT + O2 LEFT, 1 run:** parked.
- **O1 RIGHT + O2 RIGHT, 4 runs:** 3 parked (140 cm O2 twice, and a narrower O2 that used the IR re-acquire reverse). One stopped with `ERROR_US_UNRELIABLE` at O2.
- **O1 LEFT + O2 RIGHT:** not yet run. After an encoder 55.5 cm approach, the ultrasonic read a steady 33 to 34 cm against 28.5 expected. Every echo failed the ±4 cm check for 3.5 s.

In `kc_us_group`, during O1/O2 range positioning only (`O1_RANGE`, `O2_RANGE`, `O2_TARGET`), 8 echoes agreeing within 2 cm and within 10 cm of the expected range are now accepted (`US_REANCHORED/<range>/<expected>`). Echoes further off are still rejected; the existing "stable wrong echo cannot authorize travel" test (63 against 30) still fails safe. `test_task2.c` replays the failed run.

## v10: faster O2

- **Face following** at speed 30 (was 16), side IR every 50 ms (was 100). `KC_O2_FACE_SPEED` and `KC_IR_PERIOD_MS` are in kc_task2.h. The 10 cm back clearance (RIGHT route) and the IR-recovery resume use the same speed.
- **Braked stop.** Engine-initiated stops (`kc_platform_stop`) short-brake while the robot is moving (both inputs high) and wait until the encoders are quiet (60 ms, at most 400 ms) before releasing. Before, `motorStop()` let the wheels coast, about 3.5 cm from speed 16 and about 12 cm from speed 30, and that coast never reached the pose.
- **One 180° turn on the LEFT route.** At O2's corner the LEFT route (O2 arrow LEFT) does one RIGHT 180 instead of RIGHT 90 + RIGHT 90 (`KC_O2_CORNER_180`). The wide right lock leaves about 2.5 cm more back-face gap than the pair. There is no measured fit for a continuous 180, so the pose comes from rear-axle wheel and gyro odometry (`TURN180/<dx>/<dy>`). The RIGHT route keeps LEFT 90 + 10 cm + LEFT 90: a left 180 would leave about 7.7 cm to the back face, where the wheel nearly touched.
- **Tests.** `test_task2.c` passes with the new defaults and with `-DKC_O2_CORNER_180=0 -DKC_O2_FACE_SPEED=16`. `test_platform_settle.c` checks the braked stop.

First robot run on v10, 2026-10-15: O1 RIGHT, O2 LEFT, O2 about 140 cm wide. It parked (`PARK_INSIDE`, `COMPLETE`, 1.2 cm off centre).
- Smooth O1: 0.01 cm off centre in 5.9 s.
- O2 corner: `TURN180` odometry gave 20.5 cm sideways and 56.8 cm forward.
- Back face: one `EDGE_CONTINUE`.
- Return: lane 92.6 cm out, `RETURN_LANE_CROSS`. `O1_IR_IGNORED/43.45/82.21` rejected the stray object again.
- Entry: two 90° turns with a 40 cm straight.

The other three arrow combinations are not yet tested on v10.

Hexes in `Release/` (v1 to v8 are in `old_builds/`):
- `KC_Task2_Claude_SmoothO1_v9.hex`: smooth O1, O2 face speed 16, side IR every 100 ms (same O2 as `SafeO1_v5`)
- `KC_Task2_Claude_SmoothO1_FastO2_v9.hex`: smooth O1, O2 face speed 24, side IR every 50 ms (same O2 as `SafeO1_FastO2_v5`). Follows O2's faces ~50% faster; worth it for a wide O2.

## v9: no false O1 fix from a wide lane

On a v8 full run (O2 about 135 cm, return lane 100 cm out, `RETURN_LANE_CROSS` planned), the side IR saw an object 36 cm away during the O1 search. O1 was really about 85 cm away. `O1_EDGE` took it for O1 and moved the pose to 46 cm out, then the 45° entry drove into the carpark wall.

- **O1 sighting check.** `O1_EDGE` now accepts an IR reading as O1 only within 15 cm of where the odometry lane puts O1 (`|x| - 10.4`). Otherwise it logs `O1_IR_IGNORED/<IR>/<expected>` once and keeps the lane. From a lane beyond IR range nothing is accepted. `test_task2.c` reproduces the run (it fails on v8), and the wide-O2 full courses now include the stray 36 cm object.
- **Safer face extension.** `EDGE_CONTINUE` extends a face-following move by 60 cm only while the IR sees the face right now (≤ 40 cm), or by 10 cm to confirm an end already being detected. If the end was missed, it stops rather than drive toward the arena wall 50 cm past O2.
- **Start tip.** `ERROR_F_O1_CLOSE` at START (O1 144 cm away, two echoes under 24 cm) means something close was in front of the ultrasonic. Keep the space in front clear.

## v8: return from a very wide O2 (90 deg crossing)

On v7, both robot runs with a 140 cm O2 went round O2 (`EDGE_CONTINUE` worked), then stopped with `ERROR_RETURN_LANE_PATH` on lanes 94.5 and 85.3 cm out. At 45° that much sideways travel needs about 150 cm of length, and the layout had about 110 cm.

The entry planner can now use the legacy 90° turns:
- `kce_template_turn(..., 90)` uses LEFT 15.0 cm fwd / 31.0 cm side and RIGHT 20.5 / 38.0, the same values `kc_pose_finish` uses.
- The middle leg becomes a straight across.
- The same footprint checks apply against O1, O2, the carpark walls and the opening.
- `KCE_MAX_POSES` is 320, because a 90° plan needs 308 sample poses.

When nothing at 45° fits, `RETURN_CLEAR` tries, in order:
1. `RETURN_LANE_CROSS`: keep the lane past O1, then cross with turn, straight, turn in front of the carpark. `REAR_CLEAR` plans it; `ENTRY_TURN/90/<x>`.
2. `RETURN_LANE_CROSS_TRIM`: cross between O2 and O1 onto a 30, 35 or 40 cm lane, or 22 to 30 cm with no straight. The normal 45° entry follows.

Results:
- **Planner coverage** (`RETURN_CLEAR` order, lanes 46 to 120 cm, 20 layouts): 1,518 of 1,520 plan. The exceptions are lane -76 with O1 at 71.5 or 81.8 and a 60 cm gap, which stop safely.
- **Both failed robot runs:** they now plan (-94.5 as a cross-trim, +85.3 as a cross).
- **Full-course simulation** (`test_task2.c`, 140 cm O2, all four arrow combinations): every run parks. At 60 and 81 cm gaps all use the crossing; at 150 cm the normal 45° trim fits.
- **Regression test** for a very wide lane: the run now crosses instead of stopping. A lane beyond the crossing's reach (200 cm out) still stops with `ERROR_RETURN_LANE_PATH`.

Not robot-tested yet. The 90° turns are the legacy gyro-terminated turns used around O2.

## v7: wide O2

Two v6 full runs (O1 RIGHT, O2 LEFT, O2 about 140 cm wide) stopped with `ERROR_NO_EDGE`. The back leg starts about 10 cm inside O2's end, and its face-following move is capped at 100 cm. The car followed the back face for the full 100 cm and was still beside it.

- **Longer face following.** If a face-following move (front or back of O2) ends while the face has been seen, the car follows it for up to 2 more 60 cm moves (`EDGE_CONTINUE/<n>/<IR cm>`). That covers about 220 cm. `test_task2.c` has a test for it.
- **Return limit, not fixed.** A 140 cm O2 brings the car home about 85 to 90 cm off centre. In the 2026-10-15 test layout (O1 about 70 cm from the start, O1 to O2 gap about 80 cm), no safe way into the carpark fits. The planner refuses and the car stops behind O2 with `ERROR_RETURN_LANE_PATH`. Return planning works in every tested layout up to about 74 cm off centre (O2 up to about 120 cm), and in 8 of 12 layouts at 85 to 100 cm. The partial-trim fallback now also tries 50, 60 and 70 cm targets, which only helps beyond about 115 cm. Supporting 140 cm O2 in short layouts needs a new return manoeuvre, such as a 90° crossing between O1 and the carpark.

## v6: brake skid in the O1 pose, gyro bus recovery

- **Brake skid.** Tape measurements on 4 v4/v5 O1 runs disagreed with the log. RIGHT arrows logged 0.1 to 0.7 cm off centre but ended 3.5 to 5 cm left. LEFT arrows logged 1.2 to 1.5 cm left but ended 2.5 to 4 cm right. The short brake locks the rear wheels, and the car skids on about 5 cm along the diagonal after the encoders stop. The firmware now brakes `KCF_RF_SKID` (5 cm) earlier and adds the skid to its pose once stopped (`F_RF_SKID/<cm>/<y>`). `test_fast_o1.c` simulates the skid (true pose moves, odometry does not). Without the model it reproduces the robot's 4.5 cm error; with it the true end is within 0.2 cm nominally.
- **Gyro recovery.** The gyro (ICM20948 on I2C2) sometimes stops answering until reset (`ERROR_NOT_STATIONARY_READY/…/<ms since last sample>/1`). After 10 failed reads in a row, the IMU task now frees the bus (9 SCL pulses + STOP on PB10/PB11), resets I2C2 and wakes the IMU again. `IMU_RECOVERIES/<n>` at START shows if this happened. A run already stops (300 ms stale check) if the gyro drops out mid-run.
- `test_platform_settle.c` compiles again and checks the fast-O1 drive modes: brake = both inputs high, reverse lock turns, reverse straight.

v6 simulation:
- Robot-like sweep, skid 4 to 6 cm: 93,216 of 93,312 cases keep 3 cm or more. The worst case is 2.7 cm from O1 (LEFT), and none touch. End within 2.2 cm of centre and 1.3°.
- Stress sweep, skid 2.5 to 7.5 cm: no collisions.

## v5: parking without the side-wall check, O1 lateral trim

- **Parking.** `KC_PARK_REQUIRE_WALLS` (kc_task2.h) is 0 in this build. The car parks once the front ultrasonic reads 10 to 20 cm, after the existing stopped recheck, without needing both side IR walls. If it is closer than 10 cm, the one bounded retreat (at most 10 cm, back along its own entry path) no longer needs the walls either. `PARK_INSIDE/<left IR>/<right IR>` still logs the side readings. Set it to 1 to restore the wall check. `test_task2.c` covers both settings.
- **Lateral trim.** `KCF_RF_Y_TRIM_L` / `KCF_RF_Y_TRIM_R` (kc_fast_o1.h), in cm, with + meaning end further left. A v4 RIGHT run logged 0.08 cm off centre but was ~5 cm left by eye. The odometry cannot see that error (likely skid in the diagonal brake and wheel slip starting the reverse), so the trims are set from tape measurements. Both are 0 in v5. The pose handed to the O2 code includes the trim.

## v4: end-of-O1 centring calibrated from 7 robot runs

On v2/v3 the O1 finish was within 0.05 to 2.4 cm of centre, and it always stopped 0.6 to 1.5° short of straight (LEFT negative, RIGHT positive).

- **Diagonal brake slide.** The robot slid 4.3 to 4.75 cm from 57 to 58 cm/s (0.076 to 0.083 s). The firmware assumed 0.06 s, so it stopped about 1 cm long. `KCF_RF_SLIDE_S` is now 0.078. The correction move now triggers at 1.0 cm (was 1.5).
- **Reverse brake.** After braking the reverse, the car rotated another 1.6 to 2.1° every time. The fixed extra 1° lead made it stop short, so it is now 0 and the speed-based lead uses its own measured value (`KCF_RF_REV_SLIDE_S` 0.063 s).

v4 robot-like simulation, 31,104 cases, all passing:
- End off centre: median 0.35 to 0.5 cm, worst 1.3 cm (v3: worst 1.7)
- End heading: median 0.5°, worst 1.3° (v3: worst 3.5°)
- Closest to O1 / O2: LEFT 3.0 / 5.1 cm, RIGHT 5.9 / 9.3 cm

The stress sweep has no collisions.

The car centres on the line it starts on. Line it up with O1 and O2 at the start: 1° crooked puts it 1 to 2 cm off O2's centre, which the firmware cannot see.

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
