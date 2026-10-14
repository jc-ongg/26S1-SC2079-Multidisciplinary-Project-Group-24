# Latest shared build

Flash **Release/KC_Task2_KiaChun_Long_Return_Fix.hex** (already built). Startup marker: RETURN_LANE_TRIAL/2/22. Use `pi/KC_task2_stm32_bridge.py` with the included `KC_stm32_link.py`; this is protocol2.

Latest user transcript:13 completed attempts out of17; all four arrow combinations completed. See analysis/physical_runs_2026_10_14 and KNOWN_ISSUES.md for the four remaining error outcomes.

# Kia Chun — native STM32 Task 2 trial

This is an isolated copy of the OLED-fixed firmware. The original project in Downloads and the existing Pi runner are unchanged. Firmware has been built but has NOT been flashed or physically tested.

## Files

- `Release/KC_Task2_STM32_Trial.hex`: built trial firmware to flash.
- `Core/Src/main.c`: integrated firmware.
- `Core/Inc/kc_task2.h`: Task 2 movement/sensor state machine.
- `Core/Src/kc_task2_platform.inc`: adapter to existing motor, gyro and encoder routines.
- `Core/Src/main_before_TASK2KC.c.txt`: original source backup.
- `rollback/KC_OLED_Flicker_Fix.hex`: previous OLED firmware for rollback.
- `SHA256SUMS.txt`: checksums of source, bridge and firmware.
- `pi/KC_task2_stm32_bridge.py`: camera/heartbeat bridge; no Pi route decisions.

## First test

Flash the trial HEX, restart the board and keep the robot still during gyro calibration. Start the existing v11i image API on the laptop (port 8001). Stop the old robot monitor using Ctrl+C so only this bridge owns the serial port. The old HTML Run Task2 button does not start this native trial.

Place the robot at the ORIGINAL starting position facing O1, with a clear course and physical access to stop it. On the Pi:

```sh
cd ~/SC2079-MDP/rpi/task2/KC_STM32_Task2_Trial
python3 KC_task2_stm32_bridge.py --execute --stage o1 --port /dev/ttyACM0 --laptop-ip 192.168.50.218
```

This approaches O1, reads its arrow, bypasses it, returns toward the centreline and stops. Test both O1 arrows before extending the run. Ctrl+C sends STOP. Bridge loss stops native Task 2 after two seconds without a heartbeat.

Additional selections are `--stage o2-prepare`, `--stage o2-bypass`, and `--stage full`. EVERY selection starts at the original starting position, not the preceding stage's stopped pose. Full runs include the return and parking. Omitting `--execute` only prints a description and opens no serial connection.

## Control ownership

STM32 controls approach positioning, turns, IR acquisition and edge detection, return and parking. The Pi responds to camera requests using one left/right detection at >=50% confidence and supplies heartbeats. Capture/recognition failures stop the native run; no direction is guessed.

US is used for stopped approach positioning at O1/O2, monitored close range during approach, and final parking. Bypasses and lateral IR edge searches do not use US as a movement veto. Fresh samples have acquisition sequence numbers and timestamps, so repeated transmission of one sample cannot count as multiple readings. Stable wrong echoes are rejected against the prior range plus actual encoder travel. Persistent unknown range stops the trial rather than extending movement blindly.

The new command is `MOTOR/TASK2KC/50/cap`, separate from the unchanged legacy `MOTOR/TASK2`. Firmware identity is queried before starting. Manual STOP preempts the native run. Other movement commands are rejected while it is active. Motion failure, stale sensors, missing heartbeat and time limits stop the run.

## Calibration and limits

Robot dimensions: 23 cm long, 18.8 cm wide; IR is 18 cm ahead of the rear and approximately 4 cm inside the outer wheel. US is assumed at the front, 11.5 cm from the centre. O1 is 10x10 cm; O2 depth is 10 cm and width is detected, not configured. Approach model assumes at least 60 cm between O1 rear and O2 front.

Turn endpoints use the supplied speed-50 estimates: indoor 45-degree measurements and outdoor 90-degree measurements. These mixed-surface estimates are provisional. Gyro measures heading; straight encoder feedback measures travel including early sensor stops. Turn translation remains an estimate, not a measured X/Y position or a proof of wheel clearance.

O1 uses separately calibrated left/right sequences and an empirical centred finish/rear-gap anchor (34 cm left, 32.75 cm right). This assumption needs physical checking with this firmware. O2 uses a common viewing range of 28 cm, then left 26 cm/right 28 cm before turning. Reverse is limited by the minimum obstacle gap and robot length. No fixed O2 width is entered.

O2 edge searches require object acquisition before an object-to-clear transition. Initial clear readings trigger a bounded reverse search. Corner turns follow the confirmed edge without the former 21 cm lateral clearance advance. A new stationary acquisition is required before crossing O2's back. Persistent low IR readings stop; one bounded recovery is available during a search. These route details require new physical validation, especially the rear-wheel sweep and 50 cm boundary.

Return distance combines encoder travel and the initial US/obstacle geometry estimate. IR can correct the O1 landmark and lateral position. If O1 is beyond side-IR range, the firmware emits `O1_POSE_ESTIMATE` and uses the estimated pose; this is less certain than detecting O1. Final parking requires both side-wall IR readings and a valid front US reading. No guaranteed parking accuracy is claimed before physical tests.

## Validation and rollback

ARM Release build succeeded (text 73,996 bytes, data 544 bytes, bss 25,096 bytes). Existing legacy-source warnings remain. Portable C tests passed under address/undefined-behaviour sanitizers with warnings treated as errors: four arrow combinations, partial-run caps, stale/wrong sensor handling, gyro startup rejection, heartbeat loss, failed motion and interrupted travel. Python bridge tests passed for dry-run, 50% threshold, cancellation, identification, completion and STOP cleanup. These are simulations and mocks, not robot tests.

To revert, flash `rollback/KC_OLED_Flicker_Fix.hex` and use the previous Pi monitor/runner. The native bridge refuses to start on firmware without its native identity response. Original source hash is recorded in `BASELINE_SHA256.txt`.

## Encoder tolerance correction

The first native RIGHT tests stopped before the 7 cm straight segment because any negative encoder delta was rejected. Net rollback up to 0.25 cm is now treated as zero progress; signed encoder accumulation remains intact and larger negative travel still stops. Regression tests cover tiny negative startup travel, subsequent successful 7 cm completion, and excessive negative travel rejection. Flash `Release/KC_Task2_Encoder_Tolerance_Fix.hex` for this update. Previous HEX and affected sources are preserved alongside their originals with `.before_encoder_tolerance` suffixes. This change has not been physically tested.

## O1 approach recovery correction

Flash `Release/KC_Task2_O1_Approach_Recovery_Fix.hex` (includes encoder tolerance fix). O1 measured slightly below 30.5 cm after a forward approach can now reverse within the net encoder-measured distance just travelled, leaving a 1 cm margin. Initial placement too close still stops because rear clearance is unknown. Existing maximum four corrections remains. Regression covers the reported 30.08 cm after 26.36 cm forward case, 3 cm reverse and successful 33.28 cm recheck, and initial-too-close rejection. O2 preparation logs show one completed RIGHT preparation, two vision failures returning no arrow, and this O1 overshoot failure; no O2 bypass physical validation yet.

## Turn encoder check correction

Flash `Release/KC_Task2_Turn_Encoder_Fix.hex` (includes all preceding recovery fixes). Negative signed translation during TURNL/TURNR no longer triggers the straight-direction encoder check. Finite-value checking, motion failure, timeout and final gyro-heading checks remain. Regression verifies a RIGHT 90-degree turn with negative signed encoder travel can finish at the correct heading; a LEFT turn finishing 30 degrees short still fails. Forward/reverse negative-travel rejection remains tested. Original header and firmware are preserved with `.before_turn_encoder_fix` suffixes. The user's second O2 bypass log failed on the second RIGHT 90-degree turn; the preceding run completed. Physical wheel clearance is not inferable from these logs.

## Parallel recognition (native protocol version 2)

Flash `Release/KC_Task2_Parallel_Recognition.hex` and use the matching uploaded bridge. This includes all preceding fixes. Firmware identity is now BUILD/2/0; the new bridge refuses old firmware before sending a movement command.

After the first verified range at O1 or O2, STM32 emits CAPTURE_EARLY with the obstacle and run token. The Pi tries up to two captures during positioning. A >=50% result is retained through forward/reverse corrections; the robot must still reach and verify the target gap before turning. O2 recognition starts only after O1 bypass, facing O2, and may overlap reverse positioning when O2 is close.

At verified arrival STM32 emits CAPTURE. If an early result is already available it proceeds without another mandatory photo. Otherwise the same camera worker completes any in-flight attempt, then makes up to four fresh stationary attempts within a 20-second stationary budget. Early capture/API errors do not abort the approach; a persistent failure after stationary retries still stops. Heartbeats continue independently in the bridge's main loop. Early misses do not lower the 50% threshold. STOP and run-token/obstacle checks remain active.

C sanitizer tests pass all arrow combinations and early-result caching, target positioning before turn, and O1/O2 isolation. Five Python tests pass including early success and stationary retry after early misses. ARM build succeeded. Camera timing and moving-image recognition remain to be physically tested; a stop for distance verification still occurs, and recognition may finish after arrival.

Previous firmware/source/bridge are in `backup_parallel_vision/`. To roll back this change, restore its HEX and its Pi bridge together (protocol version 1).

## O2 RIGHT back-face clearance correction

Flash `Release/KC_Task2_O2_Right_Back_Clearance_Fix.hex`. It includes parallel recognition and all preceding fixes and uses the existing version-2 Pi bridge. Original header and firmware are in `backup_o2_right_back_clearance/`.

The failing RIGHT run had LEFT IR 7.7–7.8 cm after the second LEFT 90-degree turn, and the user confirmed the wheel was almost touching O2. A RIGHT-only FWD 10 cm at speed 16 is now inserted AFTER the first LEFT 90-degree turn (along O2's side, outward +Y) and BEFORE the second LEFT 90-degree turn (across the back). This is not an extra lateral edge-clearance advance. The LEFT route is unchanged.

Calculation: desired provisional IR gap 17.7 cm minus observed 7.7 cm = 10 cm translation. With IR 4 cm inward from the wheel, nominal wheel gap becomes 13.7 cm. Turn endpoints and sensor offsets are estimates; the change does not prove the swept wheel clearance. The post-turn 10–40 cm back-IR acquisition requirement remains, and the old 7.7 cm gap is still rejected. US does not veto this bypass segment. Encoder feedback records actual added travel and carries it into return distance estimation.

C sanitizer tests passed the four complete arrow combinations, RIGHT-only advance direction and measured pose, back acquisition at 17.7 cm, continued rejection at 7.7 cm, and unchanged LEFT turn selection. ARM Release build succeeded. No flash or physical movement was performed. First physical test: reset to original starting pose, set O2 RIGHT, run the existing `--stage o2-bypass` command, check clearance during the second LEFT turn and subsequent back crossing.

## Parking confirmation correction

Flash `Release/KC_Task2_Parking_Confirmation_Fix.hex`. Includes all previous fixes; Pi bridge version 2 is unchanged. Previous firmware/header are in `backup_park_confirmation/`.

User reports the second RIGHT/RIGHT full run parked nicely inside. It ended with ERROR_PARK_WALLS at US 12.61 cm. Saved telemetry shows left IR 39.4 cm, right IR 18.3 cm: sum 57.7 cm narrowly exceeded the old upper limit 57 cm. The upper sum limit is now 59 cm (lower remains 41); both individual readings must still be 10–40 cm. Require two fresh agreeing pairs (each sensor change <=3 cm), sampled at least 150 ms after entering stationary parking confirmation. If the US gap is already 10–20 cm, remain stopped for a bounded two-second confirmation instead of immediately failing; do not add forward travel to obtain a wall signature. Persistent missing walls or unreliable US still stop with an error.

Regression covers the exact reported 39.4/18.3/12.61 readings, no movement during confirmation, and missing-wall rejection after waiting. C sanitizer scenarios including all four arrow combinations and ARM Release build pass. Physical validation of this confirmation adjustment is pending.

## Separate forward-speed-100 trial

Flash `Release/KC_Task2_Forward100_Trial.hex` to select this trial. It includes all preceding fixes and uses the unchanged version-2 Pi bridge. The generic `KC_Task2_STM32_Trial.hex` and named Parking_Confirmation_Fix.hex remain the slower baseline; rebuilding current source generates the faster trial. Restore `backup_speed100_trial/` source and HEX together to revert.

Speed 100 applies to O1 bypass forward straights, the initial return straight, the centring diagonal, and forward approach distance commands >=40 cm. Turn commands remain speed50 and their calibration is unchanged. Short forward approach corrections remain22, reverse positioning16, IR edge searches16, return O1 landmark search12, O1 rear clearance16, O2 RIGHT back-clearance advance16, final parking12. Actual speeds are PWM command settings, not cm/s.

The forward controller previously capped final10cm power at850 PWM regardless of requested speed. Only for native speed100 forward commands, that cap becomes1500. Existing 20cm/50cm deceleration caps3000/5000 remain. Manual and legacy commands keep original slowdown850. A native flag is set per subcommand and cleared by native stop. Actual encoder distances, gyro checks, limits, heartbeat, close-IR and parking confirmation remain active.

C sanitizer tests pass all four route combinations and speed selection, deceleration boundaries and preserved manual-profile cap. ARM Release build succeeded. Physical speed100 behaviour is unvalidated: higher braking travel, centreline error and reduced O1 clearance are possible. First test from original starting pose: same --stage o2-bypass command. Compare clearance and O1 centring before --stage full. No Pi upload or movement was performed.

## Fast transit with calibrated O1 profile

Use `Release/KC_Task2_Fast_Transit_O1_Calibrated.hex` instead of Forward100_Trial.hex. The speed100 trial physically contacted O2 immediately after O1 bypass, before scanning/turning at O2, as clarified by the user. The three O1 RIGHT straight distances recorded about 9.43/16.24/10.34 cm for commands7/14/8. Restoring all O1 bypass commands to speed50 also restores the original850 PWM final10cm cap through the native subcommand flag. Distances and turns are unchanged. The slower baseline had physically cleared and centred O1.

Speed100 remains for long (>=40 cm) approach commands, the initial return straight and the centring diagonal. All IR edge/acquisition searches, O2 RIGHT10cm clearance advance, reverse corrections and parking retain their existing low speeds. Camera overlap remains active. No Pi update required. The generic slower baseline HEX is preserved. Regression covers restored O1 speed50, retained fast transit and four complete simulated routes; ARM build passes. Physical validation of this combination is pending. First retest from original pose is --stage o2-bypass; full return speed100 has not been physically validated.

## O2 RIGHT correction band

Flash `Release/KC_Task2_O2_Position_Band_Fix.hex`, based on Fast_Transit_O1_Calibrated. O1 stays speed50, fast transit remains100, all prior fixes retained, Pi unchanged. Previous header/HEX in `backup_o2_position_band/`.

The second uploaded run500904804 alternated O2 gap29.17→25.18→29.43→25.83→29.29 and exhausted four corrections. A ±1cm acceptance band was narrower than the minimum3cm command plus observed braking. Keep nominal RIGHT target28cm but accept26–30cm (±2), avoiding unnecessary reversals for1cm deviations. LEFT target26±1.5 and common viewing28±1.5 are unchanged. RIGHT-specific10cm back-clearance advance, IR proximity limits, encoder/gyro guards and correction cap remain. The wider band is a trial; final physical clearance must still be checked.

Regression covers exact29.17cm acceptance with no movement, and25.18cm followed by a3cm reverse and29.43cm acceptance without a second correction. All four full simulated arrow combinations pass under sanitizers, and ARM Release build passes. Physical retest pending: original start, same --stage o2-bypass.

## O1 RIGHT clearance and bounded parking retreat — NEW TRAJECTORY TRIAL

Flash `Release/KC_Task2_O1_Right_Clearance_Park_Recovery_Trial.hex`. No Pi bridge change. Previous header, platform adapter and O2_Position_Band_Fix.hex preserved in `backup_o1_clearance_park_reverse/`. This is a new O1 trajectory, not a physically validated replacement.

User confirmed rear-wheel contact happened during INITIAL O1 RIGHT bypass in run348108482. Add6cm to the straight BEFORE the inward LEFT45 (14→20). Simply retaining the previous final forward turn would add6cm toward O2 and risk the earlier O2 contact. Instead use the supplied indoor speed50 reverse-LEFT45 translation: backward12.5cm, left1.5cm, clockwise heading45. At an inward heading of+45deg this contributes global(+7.778,-9.899), replacing the old final forward-RIGHT45 global(-2.121,+23.335). Increase inward diagonal8→22cm, contributing another(-9.899,+9.899), to compensate lateral difference. Resulting net endpoint change is X0,Y-17.335cm versus the former route, while the inward turn starts6cm farther beyond O1. The empirical finish rear-gap anchor is adjusted32.75→15.415cm by that calculated difference; it has NOT been physically measured on this new route.

RIGHT sequence, every command speed50: TURNR45, FWD7, TURNL45, FWD20, TURNL45, FWD22, REVL45. Finishes facing original forward direction. LEFT route unchanged. Turns use existing gyro targets/status/timeouts; signed encoder translation is not used to reject reverse turns. The pose model now includes reverse-LEFT45; no other reverse-turn calibration added. True wheel sweep and endpoint errors remain unmeasured; this geometry does not prove collision freedom.

FIRST TEST from original start with O1 RIGHT: use --stage o1. Check rear-wheel clearance during inward LEFT45, final reverse-LEFT45, centreline alignment and front O2 clearance. Do not proceed to full until physically checked. All four simulated route combinations pass with this sequence; simulation is not a physical sweep test.

Parking retreat: if3fresh stable US readings give4–10cm front gap, confirm2fresh agreeing side-wall pairs after150ms settling. Retreat eligibility requires both sideIR10–40cm and sum41–61cm (allows previously observed20.4/39.5 pair); final parking confirmation retains sum41–59cm. Calculate reverse=ceil(12-frontgap), maximum10cm, speed12. Require reverse+1cm <=50cm park depth-23cm robot length-frontgap-7cm rear margin. Permit ONE retreat per run, then remeasure using actual encoder travel to anchor expected US. Unknown side corridor, excessively close front, inadequate rear budget or repeated correction stops. No additional reversing merely to satisfy wall confirmation.

Regression includes RIGHT sequence selection, reverse-LEFT heading/endpoint model, exact5.54cm parking case→7cm reverse→12.64cm verified gap, no retreat with missing side wall, and no second retreat. C tests passed under address/undefined-behaviour sanitizers with warnings treated as errors. ARM Release build succeeded. Original firmware sources and calibration measurements are retained.

## O1 RIGHT centring and dynamic parking — current trial

Flash `Release/KC_Task2_O1_Centred_Dynamic_Parking_Trial.hex`. It includes all previous changes, keeps calibrated O1 bypass speed50 and existing faster transit, and requires NO Pi bridge update. Previous header, adapter, tests and clearance/recovery HEX preserved in `backup_o1_centre_dynamic_parking/`.

O1 RIGHT inward diagonal22→26cm before reverse-LEFT45. Calculation:4*sin45=2.828cm left and4*cos45=2.828cm toward O2. Added6cm pre-inward clearance and the reverse finishing turn remain. RIGHT finish Y anchor updated15.415→18.243cm rear-to-O1-rear estimate; X still explicitly reset to calibrated centreline assumption. This is a correction based on the user's measured3cm right finish, not proof of centring. LEFT route unchanged. First physical test is --stage o1, O1 RIGHT, original start, O2 at minimum spacing; check whole swept path and endpoint. Only then full run.

Parking revalidation: one stopped retry per run. If an expected-range check cannot obtain a valid group within800ms (or initial US remains unreliable3500ms), brake and collect NEW US/IR generations. During the retry, bypass the stale encoder anchor only for confirmation:3fresh stable US readings10–20cm, two agreeing fresh side-wall pairs and heading within5deg. A stable far echo cannot authorize a new drive during this retry. Persistent invalid/unknown depth becomes ERROR_PARK_DEPTH_UNCONFIRMED after the3500ms retry window. It remains stopped; no inferred success from side walls alone. Sensor staleness and missing side walls retain stop checks.

Long final approach: remove fixed80cm rejection. Initial verified ranges remain bounded by existing2–180cm validity filter. Approach in <=40cm encoder commands, speed22 when front gap>50cm and12 otherwise, with existing deceleration and confirmed close-front stopping. Recheck each segment against actual encoder travel before another command. Maximum8forward segments and150cm cumulative actual forward travel, plus existing per-motion/run/heartbeat limits. Existing one-time geometry-bounded reverse for4–10cm front gap remains.

Regression tests cover the reported11.94cm vs18.55cm anchor mismatch, invalid US with valid side walls (no movement/no completion), stable far echoes during revalidation (no movement), missing side-wall rejection, exact90.17cm segmented approach40+38 with actual encoder travel, cumulative distance/segment limits, the26cm O1 command, all four complete simulated routes, and previous bounded reverse cases. C tests pass with address/undefined-behaviour sanitizers and -Wall -Wextra -Werror. ARM Release build succeeded. Physical validation is pending. No firmware flash or robot movement was performed.

## Straight completion braking/settling correction

Flash `Release/KC_Task2_Brake_Settle_Fix.hex`. Includes the O1 RIGHT26cm centring change, dynamic parking and every preceding current-trial correction. No Pi bridge update. Older firmware/header/platform/test copies in `backup_straight_settle/`.

Attachmentd2118a80-49d0-4e53-bf3b-af4397319e4d repeats earlier five-run logs then adds O1-only run775486754 after Pi SSH reconnect. That new run is O1 LEFT. Long FWD94 recorded96.12cm, stationary group30.18cm requested REVS3, then ERROR_ENCODER. Retrieved telemetry shows US29.1→26.7→26.2cm during reverse startup, consistent with continued approach motion. The code's signed reverse encoder check should not be relaxed to hide2.53cm of opposite-direction travel.

For speed100 commands restore the original final10cm PWM cap850 (previous speed trial1500); earlier20/50cm caps3000/5000 and requested100 for longer transit remain. After FWD/REVS motor primitives finish, native adapter retains brake and continues sampling actual encoder travel under that SAME command. It does not re-call a completed PID. Completion is delayed until both counters remain unchanged100ms, with800ms maximum. Continued movement yields ERROR_BRAKE_NOT_SETTLED and motor-error stop. Wait is nonblocking, preserving Pi heartbeats and STOP handling. Turns retain their own existing settle/completion logic.

All portable route/regression scenarios pass under ASan/UBSan and -Wall -Wextra -Werror. A new test_platform_settle.c includes the production adapter with mocked HAL and verifies completed PID is called only once, subsequent coasting counts remain included in pose, next phase waits100ms quiet, and native abort works. Quiet-window tests cover moving counters, timeout, timer/counter wrap. ARM Release build succeeded. Physical validation pending. Logs now emit PROFILE/26.00/850.00 and ENCODER_FINISH/travel/status to help distinguish this build and inspect completed distances.

Retest --stage o1 from original position; include the long LEFT approach that failed, then O1 RIGHT to physically validate26cm centring/clearance. No successful new RIGHT26cm physical test is present in the latest attachment. Do not confuse repeated old logs with new firmware results.

## Fine positioning and cached-arrow handover

Flash `Release/KC_Task2_Fine_Positioning_Fix.hex`. Includes brake settling, RIGHT26cm centring/reverse finish, all clearance and parking changes. Pi bridge unchanged. Previous header/tests/HEX are in `backup_fine_positioning/`.

Latest attachmentbc1cfb56-87a0-47d5-b540-22bf71d87227 contains successful LEFT and RIGHT O1-only runs with PROFILE26/850, then O1 correction oscillation, NOT_READY, O2 vision failure, and O2 common-view positioning oscillation despite cached LEFT recognition.

Corrections now use minimum2cm instead of3cm, speed12 for <=4cm corrections. One-centimetre commands remain avoided because the original forward primitive's target-minus1cm stopping threshold completes them immediately. Two-centimetre commands have a nonzero threshold. Existing100ms encoder-quiet braking and actual-distance US anchors remain; acceptance bands, maximum4corrections and proximity guards are unchanged. RIGHT nominal28cm accepted26–30, LEFT nominal26±1.5, O1 nominal32±1.5.

At O2, once an early arrow is cached, common-view positioning uses that arrow's final turn target (LEFT26/RIGHT28), rather than continuing to chase28cm and then repositioning again forLEFT. It still requires fresh verified US; cached vision cannot bypass positioning or final checks. FINE_POSITION/2.00/26.00 identifies the new configuration in logs. Camera confidence50%, capture retry policy and startup gyro readiness remain unchanged.

C sanitizer regression passes the exact O1 cases30.05→2cm reverse→32.70 and33.55→2cm forward→30.85, plus O2 LEFT at25.43 accepted without motion. Four complete simulated arrow combinations and the real native-adapter braking tests pass. ARM Release build succeeded. Physical validation pending, particularly2cm correction movement and O1 RIGHT centring/clearance. No flashing or movement performed.


## O1 acceptance and actual-gap compensation — 2026-10-14

Flash `Release/KC_Task2_O1_Acceptance_Compensation_Fix.hex`. This includes all preceding fine-positioning, braking, clearance and parking fixes. Pi bridge unchanged. Previous source/header, tests, README, checksums and fine-positioning HEX are preserved in `backup_o1_acceptance/`.

O1 nominal gap remains32cm; accept30–34cm inclusive after the existing fresh three-reading verification and encoder-based expected-range check. Readings outside this band still use bounded fine positioning. Keep the verified accepted gap for the bypass; do not change it from moving readings or vision replies. New log marker `O1_ACCEPTANCE/30.00/34.00`; `O1_GAP` records accepted gap and difference from32.

Compensate at the heading-zero forward side-lane straight BEFORE the inward turn: round(base + accepted_gap -32) whole centimetres. LEFT base9 becomes7–11; RIGHT base20 becomes18–22. This offsets longitudinal starting-gap variation without changing diagonal distances or lateral centring. Integer rounding leaves at most0.5cm longitudinal model residual. RIGHT's additional6cm pre-inward clearance and26cm diagonal/reverse finish remain. `O1_LANE_ADJUST` records accepted gap and commanded lane distance. Missing/out-of-band accepted gap fails before this command. Turn calibration, wheel slip and physical swept clearance remain empirical limitations; this is not an absolute positioning guarantee.

Regression checks cover reported30.05/33.55/33.80 readings accepted without correction, inclusive30/34 boundaries, both lane compensation directions and nominal32, maximum0.5cm model residual, outside-band correction, missing-gap rejection, unknown initial rear clearance, all four full simulated arrow combinations, and production adapter braking. ASan/UBSan checks and ARM Release build passed (text76164, data544, bss25136). No robot movement or flash performed.

Physical test: original start, O2 at minimum intended spacing; run --stage o1 for LEFT and RIGHT, check rear wheel clearance for the entire inward turn, actual endpoint offset and O2 front clearance. Include accepted gaps near both30 and34 if possible. Then full run. New acceptance/compensation is not yet physically validated.


## Parking entry geometry trial — 2026-10-14

User reported side-wall contact during the diagonal and another contact on entry in full LEFT/LEFT run567849394. Return missed the O1 IR landmark and used estimated X56.11. Old fixed rear-clear poseY57.90 plus RIGHT45/F58/LEFT45 advanced77.84cm before the first parking US check (3.17cm). Software endpoint X near0 did not establish a collision-free path.

New build `Release/KC_Task2_Parking_Entry_Geometry_Trial.hex` includes all earlier O1/fine-positioning fixes; protocol remains2, use the uploaded `protocol2_compatible` bridge. No Pi bridge/API changes required. Previous source/tests/HEX/checksums in `backup_parking_entry_geometry/`.

Replace unconditional rear-clear advance with a sampled whole-footprint entry plan. Retain calibrated45degree turn pair; choose an earlier start on the return lane, repositioning only along that lane at12 if needed. Recorded return-lane start limits reversal. The complete reposition path is checked too. For the reported trace the chosen estimated startY97.50 replaces oldY57.90; from O1 search stopY73.61 this requests approximately24cm reverse. The lateral diagonal remains58cm but starts earlier and runs22, not100. Its actual travel may not exceed the checked command+2cm allowance. Recheck the full route at actual reposition encoder endpoint before starting turns. Entry heading tolerance tightened to2degrees for straight/reposition and completed turns, without rejecting the changing heading DURING turns.

Geometry inputs: robot outer23x18.8cm, O1 10x10 and initial front-US position anchor, carpark inside60x50cm centred on initial X0. Starting robot assumed fully inside facing out; unknown rear gap0..27cm generates a conservative wall envelope: nearest front/back wallY=-11.5cm and side-wall mouthY<=38.5cm. Side walls assumed10cm thick. Body inflated5cm, two illustrative turn paths checked (linear centre/yaw and heading-tangent cubic fit), plus diagonal overrun0/2cm. Each turn20samples and straight16samples. Search integerY candidates; use middle of widest>=5cm feasible interval. No reducing margins or blindly falling back. No model path gives ERROR_ENTRY_NO_PATH before turn; actual reposition mismatch gives ERROR_ENTRY_RECHECK. Template scratch is static4KB, not on2KB motor stack. New events PARK_ENTRY/5/22 and ENTRY_PLAN/startY/diagonal.

These are planning margins and calibration assumptions, NOT a physical swept-path guarantee. True intermediate turn positions remain unmeasured; pose may be wrong when O1 is not acquired. The full path is checked against walls, but the final alignment turn may overlap the entrance depth while the body is already within the central opening; this is not a promise that every turn completes entirely outside the mouth. If the actual carpark/start differs from these assumptions, do not treat the plan as valid. No sensor policy is weakened and final front-US/IR confirmation remains.

Regression covers reported unsafeY57.90 rejection, feasible earlier start, known return-corridor bounds, impossible/NaN geometry, reversal throughO1 rejection, encoder recheck/overrun rejection, diagonal22, all four full simulated arrow combinations and production braking adapter. Host C ASan/UBSan -Wall -Wextra -Werror passed; ARM Release build passed. New geometry has not been physically validated or flashed by the agent. Initial physical check must inspect the reposition and full entry sweep; Ctrl+C remains STOP.

Separate unresolved risk: this run measured only2.90cm toO2 after O1 bypass. This parking-only change does not alter the O1 route or resolve that earlier proximity. It must not be represented as fixing both issues.


## Heading feedback correction — 2026-10-14

Built `Release/KC_Task2_Heading_Feedback_Fix.hex`, protocol2, unchanged camera API/bridge. Fixes native adapter double-counting retained currentAngle after a straight/reverse commits it into globalHeading. Match SENSOR/OLED: globalHeading while stopped, globalHeading+currentAngle while tracking. Retain2degree entry limit; log HEADING_ORIGIN, HEADING_FEEDBACK/1/2 and ENTRY_HEADING actual/target on rejection. Production adapter tests reproduce stale-angle false rejection; live integration remains. Host tests/ARM build pass. Old files in backup_heading_feedback. This does not resolve or validate physical side-wall clearance. See analysis/parking_collision/heading_feedback_findings.md for both new run traces and the unresolved mismatch between first physical collision report and its logged motion sequence. No flash/motion performed.

## Kia Chun return lane trial (2026-10-14)

Flash `Release/KC_Task2_KiaChun_Return_Lane_Trial.hex` for the separate friend-inspired return trial. It clears O2, preflights an inward correction for lanes wider than45cm, then uses a shorter final parking diagonal. Existing measured45-degree turns, front US anchors, encoder travel and final parking confirmation remain. No Pi/API update; use protocol2_compatible bridge. O1 bypass unchanged. Startup event RETURN_LANE_TRIAL/1/22 identifies this build.

Host tests and ARM build pass; physical clearance is unvalidated. Sampled turn sweeps are illustrative, not a guarantee. See analysis/friend_Task2_return/return_lane_trial.md. Previous source and Heading_Feedback_Fix HEX are preserved in backup_friend_return. No flash/motion was performed.

## Long return window fix (2026-10-14)

Run428994270 with150cm O1–O2 spacing falsely stopped at ERROR_RETURN_LANE_PATH. Parking preflight searched only40cm ahead of the lane correction; it now searches relative to O1 and checks the full connecting straight. Flash `Release/KC_Task2_KiaChun_Long_Return_Fix.hex`; startup RETURN_LANE_TRIAL/2/22. Protocol2 bridge unchanged. Exact failed geometry and all four arrow combinations at60/150cm pass host tests; ARM build passes. Physical parking remains unvalidated. Rollback in backup_long_return_window. Details: analysis/friend_Task2_return/long_return_window_fix.md.
