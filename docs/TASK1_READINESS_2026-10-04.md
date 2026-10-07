# Task 1 readiness audit - 2026-10-04

**2026-10-07 follow-up:** see [current readiness and assessment run guide](TASK1_RUN_GUIDE_2026-10-07.md).
The user confirmed professor approval for calculation before Start. A crowded
eight-obstacle planner crash was fixed locally; four Algo configurations passed
43 tests each. Subsequently the four Pi runtime files were deployed and verified;
see [Pi deployment](RPI_DEPLOYMENT_2026-10-07.md). Tablet installation and physical
checks remain unverified. The original audit/deployment plan below is historical.

Software changes are prepared on `codex/task1-readiness`. The physical Pi is
unavailable. Nothing here certifies the deployed Pi, flashed STM firmware,
Bluetooth delivery, battery, calibration, or an actual assessment run.

## Requested behavior

1. Place the robot and obstacles, then press **Send Map**. Android sends a
   complete map with a new request ID to the Pi. The Pi requests the laptop
   Algo route immediately and stores the commands without moving.
2. **Task 1 Start stays disabled** until the Pi confirms that this exact map's
   route is ready. Editing/resetting/loading a map, using manual controls, or
   reconnecting Bluetooth requires sending the map again. A stale response,
   HTTP failure, invalid route, or duplicate Start cannot release movement.
3. Start sends only the execution signal; it does not recompute the route.
   This implements the requested preparation workflow. On 2026-10-07 the user
   confirmed that the professor allows pre-Start calculation during preparation.
4. TARGET frames update obstacle IDs on the map and add a timestamped Chat
   entry such as `Target found: obstacle 3, image ID 11`. History survives tab
   changes/view recreation during the app session, is limited to 500 entries,
   and can be cleared. It is not persisted after the app process exits.
5. Each actual motor command carries its own endpoint, or no endpoint while a
   compound turn is unfinished. Matching STM completion updates the tablet.
   Padding cannot consume a future planner pose. Fully cancelled moves have
   no motor ACK; a later SNAP/FIN safely supplies their endpoint.
6. Connected manual controls also wait for completion before moving the icon.
   The local representation of a manual turn is an orientation change, not a
   measurement of its physical arc. Offline controls remain a simulator.
7. `STATUS,finished` stops the Task 1 timer automatically after map updates.
   The recorded finish time survives switching away from the timer tab.
   Manual Stop sends RESULT and prevents further Pi dispatch; an in-flight
   motor command may finish. It is not an emergency motor stop or a qualifying
   automatic finish. The app timer begins at Start; it does not measure/add
   preparation overtime, which the official supervisor timing must include.

## Assessment requirements and current evidence

Sources: `Rules for the Task 1 and Task 2 (1).pdf` (identical to the original
copy), `Android Remote Controller checklist (1).pdf`, the 2026-2027 assessment
checklist/briefing, and the Android/RPi/algorithm briefing references.

| Requirement | Evidence / remaining check |
| --- | --- |
| Quarantine laptop, tablet and robot before layout release | Operational requirement; prepare everything before quarantine. |
| Two-minute preparation; overtime consumes the six-minute run allowance | Rules Task 1.3/1.5 and FAQ 8. Coordinates/facing are entered on Android; calibration stays in the yellow carpark. |
| Correct image IDs on the Android obstacle map in real time | Existing TARGET handler retained; complete-frame parsing and numeric-ID tests pass. Verify the tablet physically after deployment. |
| Robot stops automatically within six minutes | FIN waits for preceding motor completion and synchronous recognition. Actual stopping and duration require a physical run. No new six-minute motor cutoff is implemented. |
| Tiled camera images with bounding boxes and background | Trial CV `/stitch` produces a JPEG; inspected the real-model test montage. Updated Pi tags captures with the map request ID; updated trial CV shows only that run and retains one representative per character. |
| Save final Android map and tiled captures | Take screenshots, ensure the supervisor photographs the map, and email both screenshots as required by Task 1.11. Chat is supplementary evidence. |
| Up to eight obstacles and representative symbol coverage | The local model exposes all numeric target IDs 11-40, plus NA. Four saved-image checks are not a test of all 30 symbols or the real lighting/background. |
| Only one official RETRY across Task 1/2 | First run is void if RETRY is used; equipment stays quarantined without robot/algorithm-setting changes. The controller's automatic revisit within a run is separate. |

## Local validation

- Android debug build and JVM tests, including readiness correlation, errors,
  fragmented TARGET/ROBOT frames, readable target history and view replay.
- Curated Pi suite: mocked Bluetooth, serial, HTTP and camera. Covers readiness
  before Start, stale requests, errors, motor completion ordering, padding,
  repeated runs, CV locking, manual stop, revisits and worker shutdown.
- Algo tests with original, forward50-only, reverse50-only and combined50
  configurations. Original recorded-map motor commands remain unchanged.
  Real Flask route responses are validated with the same parser used by Pi.
- Laptop launcher tests. `--check` found neither service healthy; no laptop
  service was started or stopped by this audit.
- Loaded the actual `Algo/best.pt` and used the trial CV Flask test client:

| Saved capture | Result | Confidence |
| --- | --- | --- |
| right_arrow_38.jpg | RIGHT / 38 | 0.427 |
| letter_S_28.jpg | S / 28 | 0.952 |
| number_2_12.jpg | 2 / 12 | 0.922 |
| bullseye.jpg | NA (ignored) | 0.000 |

Artifacts: `output/task1-audit-2026-10-04/recognition-results.json` and
`audit-stitch.jpg` in the same folder. Right-arrow confidence is relatively
low in this saved image; test distance, focus and lighting on the actual robot.
The original `best_v11i_m.onnx` is absent here; this verifies the temporary
best.pt backend, not equivalence to the original ONNX service.

## Pending coordinated deployment

**Do not install only the new Android APK and expect the current Pi to work.**
The old Pi waits until BEGIN to plan and does not send PLAN READY, so the new
app correctly leaves Task 1 Start disabled until the matching Pi update exists.

Laptop: deploy/use updated `Algo/main.py` plus new `Algo/execution_plan.py`,
and updated `Algo/cv_trial_server.py`, then restart Algo/CV through the normal
laptop launcher. The updated Pi requires
`command_poses` in the API response. Keep the selected calibration profile and
speed consistent; this change does not alter motor geometry or speed.
The run-specific montage is implemented in the temporary CV backend. If the
original ONNX backend is restored later, verify its current-run filtering;
its historical `/stitch` implementation does not provide this grouping.

Pi runtime files to compare and patch when access returns:

- `final_week_8_backup.py`
- `communication/android_protocol.py`
- `communication/android.py`
- **new** `path_plan.py`

Use `task1_main.py` as the entry point. Existing launcher/settings/profile
changes from earlier work may also be pending on the actual Pi; inspect them
against its real source instead of assuming this snapshot is an exact mirror.
Back up the live files, stop the existing controller, compare each file, review
the narrow change, run the curated tests, and verify only one parent controller
owns serial/Bluetooth. Do not bulk-copy the entire RPi folder or reflash STM as
part of this update. New shutdown source terminates and joins all five workers;
the currently deployed shutdown may still leave workers alive.

The completion watchdog is now 60 s instead of 15 s to accommodate compressed
long straights. Verify the longest real movement fits this bound; the watchdog
must never advance the next command after a missing ACK.

## First physical checks after deployment

1. Confirm the actual laptop hotspot/Wi-Fi address in Pi settings. The audit
   laptop was not on the expected 192.168.50.x hotspot. Algo is port 8000;
   the trial CV server is port 8001. Confirm `/status`, `/health` and a camera
   capture before quarantine. Have the tablet app and laptop services ready.
2. Send a simple map. Check the Pi calls `/path` immediately, the robot stays
   stationary during preparation, and Start becomes enabled only after READY.
   Edit a face/coordinate and ensure Start disables until Send Map is repeated.
3. Start once. Confirm it executes the cached route without another `/path`
   request. Check tablet positions after a straight, turn and turn padding.
   These are planner estimates; early firmware DONE, slipping or calibration
   error would still need hardware investigation.
4. Confirm each recognised image ID appears on its correct obstacle and in
   Chat, including when Chat was not open. Switch tabs during the run and
   confirm the timer has the final duration after automatic completion.
5. Verify FIN stops the robot without manual intervention, the current-run
   tiled display contains the scored captures, and both screenshots are saved.
6. Test a failed detection/revisit: successful obstacles remain collision
   obstacles (SKIP/8) and are not photographed again. Check the revisit starts
   from the correct acknowledged pose. This logic is tested only in software.
7. Test a second map/run, a ready-state Bluetooth reconnect, and shutdown/
   restart. Do not assume recovery from a Bluetooth drop during a moving run
   is qualified; verify it physically. Confirm there are no orphan workers.
8. Run a representative four-to-eight-obstacle arena at the selected speed and
   verify physical turn clearance, final heading, camera distance, battery and
   the six-minute limit. Current speed50 turn overlays remain provisional.

## Reference files

The added Android checklist/briefing directly supports C.1-C.10. The RPi and
image-recognition guides describe interfaces, multitasking and inference.
The algorithm briefing and Reeds-Shepp paper explain planning approaches;
they do not override assessment timing. The STM manuals, board schematic,
motor data sheets, flashing guide, care notes and robot tests support physical
assembly/diagnostics. The multithreading and SC2104 exercise PDFs are reference
material. Their presence does not prove the corresponding hardware works.

The supplied PDFs do not explicitly discuss pre-Start algorithm computation;
professor approval was confirmed by the user on 2026-10-07. The duplicate rules
PDFs have identical contents.
