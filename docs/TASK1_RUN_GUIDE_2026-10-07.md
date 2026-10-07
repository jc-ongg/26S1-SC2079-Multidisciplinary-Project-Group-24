# Task 1 readiness and assessment run guide - 2026-10-07

**Superseded:** use the [restored 10 cm planner with 30 cm turn clearance](TASK1_ROLLBACK_2026-10-07.md) for the current setup. The 5 cm trial is backed up.

**Current movement setup:** [speed50 / 5 cm test setup](TASK1_GRID5_TEST.md) supersedes the
old profile commands and planning benchmarks in this historical audit.

**Pi deployment update:** the four runtime files were installed and verified
on 2026-10-07; all 57 offline Pi tests passed on the actual Pi, and its parser
accepted a real laptop route. The controller was left stopped. See
[deployment record](RPI_DEPLOYMENT_2026-10-07.md). Tablet installation and a
physical run remain unverified. The initial audit findings below still apply.

## Readiness decision

The requested normal-run features have passed software checks, and the Pi
runtime update is installed. The system is ready for integration testing with
the matching Android build. It is not yet verified ready for the assessment:
tablet installation is unverified and the full updated system has not completed
an arena run. The original audit preceded the subsequent requested Pi deployment.

On 2026-10-07 the user confirmed that the professor allows route calculation
before Start, during preparation. Treat that as the professor clarification
reported by the user; the supplied rules do not explicitly discuss computation.

The initial audit did not deploy anything, start laptop servers, connect to a
Pi, or change STM firmware. Its only new runtime change was a focused Algo
planner fix. The later Pi deployment was authorized separately by the user.

## Requirements and code evidence

| Requirement | Current implementation | Remaining verification |
| --- | --- | --- |
| Quarantine laptop, tablet and robot before the layout is released | Operational responsibility, not app functionality | Get all setup and testing finished before quarantine. |
| Enter robot pose, obstacle coordinates and image-facing sides on Android | Touch placement, editing/dragging, facing selection and complete MAP transmission exist | Check all official coordinates and sides; check app coordinates use the same origin as the arena. |
| Prepare within two minutes; overtime uses the six-minute run allowance | MAP now plans before Start; matching READY enables Start | Rehearse map entry plus planning together. App timer does not add preparation overtime. |
| Start only after supervisor approval, then autonomous operation | BEGIN executes the prepared route; motor dispatch waits for matching STM completion | Check robot remains stationary until Start. No manual movement/map edits/restarts during the official run. |
| Correct numeric image ID on the correct Android obstacle, in real time | Pi sends TARGET; Android updates the map and adds a Chat entry | Test actual Bluetooth and images. Chat/laptop recognition alone cannot substitute for the Android map. |
| Automatic stop within the official time allowance | FIN waits for motor completion and recognition, then reports finished; app timer stops | There is no overall six-minute cutoff. Seven capture attempts and one revisit pass can extend a run. Prove the complete sequence fits the allowance, or implement and test a time-budget strategy before relying on it. |
| RAW camera images with bounding boxes, one representative per recognized character, tiled in one window | Updated trial CV retains the background, draws boxes/IDs, and groups captures by map/run ID | Open the current-run montage on the laptop after the run and check it agrees with the tablet. Pi's stitch request does not open a browser window. |
| Save and email final map and camera-display screenshots | No automatic submission | Ensure supervisor photographs the map, save both screenshots, and email them after the run. |
| Up to eight obstacles from the 30-image pool | IDs 11-40 supported; local five/eight-obstacle route probes described below | Test recognition across the image pool and variable backgrounds. A ready route can omit targets; inspect coverage in rehearsals. |

Task 1 requires starting inside the yellow carpark and stopping automatically.
The return-to-carpark requirement appears in Task 2; do not add it to Task 1
based on the Task 2 FAQ.

Rules FAQ 3/4 awards 10 points per correct image ID on the correct obstacle and
penalizes each wrong ID by 10 points. Timing separates equal recognition scores.
Failing to display real-time image IDs on Android, or failing to display the
camera images at the end, prevents qualification. Manual stopping makes a run
incomplete under Task 1.10/FAQ 15.

The Android C.1-C.10 functionality is present in source: text exchange, connection
GUI, controls, selective status, map, obstacle placement/dragging, facing,
connection-loss handling, TARGET and ROBOT. This source review does not certify
physical demonstrations or supervisor signatures on the separate checklist.

## New findings from this audit

1. **Planner crash fixed locally.** One crowded eight-obstacle layout caused
   an HTTP 500 while assembling a candidate path. Original-profile candidate
   costs can assume a reverse edge exists even when A* cannot drive that edge.
   The planner now rejects a candidate with a missing actual segment and only
   accepts its score after every segment succeeds. Geometry, speed and movement
   constants are unchanged. A regression test exercises the failing layout.
   Include `Algo/algo/algo.py` in the laptop changes you use next time.
2. **Ready is not a guarantee of visiting every obstacle.** The crowded layout
   now returns a Pi-validated route visiting 4/8 targets instead of crashing.
   This does not prove the other targets are physically impossible; the planner
   searches a bounded set of alternatives. A separate perimeter layout visits
   8/8. During practice inspect the generated SNAP obstacle IDs, not only READY.
3. **The time budget is not enforced globally.** The 60-second ACK watchdog
   pauses further dispatch after a missing completion; it is neither a six-minute
   assessment cutoff nor an emergency motor stop. The app's manual Stop also
   allows an already-sent motor command to finish. These are not substitutes for
   demonstrating automatic stopping within the allowed time.
4. **Bluetooth delivery across a moving-run disconnect is not guaranteed.**
   Pi's sender removes a message from its queue before sending; an OSError can
   lose that TARGET/status. Reconnection does not replay an authoritative target
   snapshot, and Android returns to listening rather than automatically dialing
   the Pi. Verify pre-Start reconnection and actual link stability. Do not claim
   successful mid-run recovery from the source alone.
5. **Camera evidence has operational limits.** Trial CV's run grouping is in
   memory and `/stitch` includes captures from the last 20 minutes. Save the
   montage promptly; restarting CV loses the grouping. The original ONNX weights
   remain absent here, and its historical montage code does not have this run
   filtering. Use a backend you have tested before quarantine.

## Local checks and their limits

On 2026-10-07, 43 Algo/CV tests per configuration passed for original,
forward50-only, reverse50-only and combined50. Inapplicable profile tests are
skipped (20/19/17/13 respectively). Recorded baseline motor-command responses
still pass. No live server or hardware is involved in these tests.

The new original-profile Flask probes were accepted by the actual Pi plan
validator and ended in FIN:

| Layout | Targets included | Planning time on this laptop |
| --- | --- | --- |
| Five-obstacle example from rules | 5/5 | About 2 s |
| Crowded synthetic eight-obstacle layout | 4/8 | About 15 s |
| Perimeter synthetic eight-obstacle layout | 8/8 | About 48 s |

Some other checks ran concurrently, so these times are observations, not
isolated benchmarks or guaranteed bounds. They exclude physical movement,
capture and inference. Pi's planning request times out after 60 seconds and
Android's route-confirmation wait after 70 seconds. Send Map early and rehearse
the selected profile on representative layouts.

The existing 2026-10-04 validation remains: 57 curated offline Pi tests and
9 laptop-launcher tests passed; Android debug assembly and 8 JVM tests passed.
Android/Pi runtime source was not changed in this audit, so it was not rebuilt
or retested solely to repeat those checks. The saved APK checksum still agrees
with that validation. Earlier best.pt checks recognized RIGHT/38, S/28 and 2/12,
and ignored the bullseye. They do not establish all-30-class field accuracy.

The read-only laptop check on 2026-10-07 reported neither Algo8000 nor CV8001
healthy. It did not start or stop either service. Do not assume the servers are
running because their source files are on this laptop.

Artifacts: `output/task1-audit-2026-10-07/planning-probe.json` and
`algo-tests-*.log`. Prior build/model results remain under
`output/task1-audit-2026-10-04/`.

## Before assessment day

1. Deploy the matched Android, Algo/CV and Pi updates. The Pi portion is now
   complete and laptop services were verified; tablet installation remains
   unverified. For any later Pi update, compare/back up the real
   Pi before patching its four pending runtime files: `final_week_8_backup.py`,
   `communication/android_protocol.py`, `communication/android.py`, and new
   `path_plan.py`. Inspect existing `task1_main.py`, settings and profile support
   too. The Pi is not a Git checkout; pushing to GitHub does not update it.
   Install Android through Android Studio Run or the saved debug APK. Restart
   both laptop services to load changes. No STM reflash is part of these patches.
2. Choose and physically validate the motor speed/calibration configuration.
   Default launch uses original turns at speed50. The speed50-v2 overlays are
   provisional; preserve a configuration that works on the actual robot.
3. Run a simple one-obstacle integration test, followed by representative
   four-to-eight-obstacle runs. Check MAP prepares immediately without motion,
   READY corresponds to the current map, Start does not recalculate, IDs arrive
   on the correct obstacles, the tablet does not move early, and FIN stops both
   robot execution and the timer.
4. Test a missed detection/revisit, a second run after resetting, reconnecting
   before Start, and shutdown/restart with only one controller parent. Confirm
   these do not leave stale targets or independent controllers on the links.
5. Measure the whole official-style sequence: map entry plus planning within
   two minutes, then motion/captures/revisit/display within the remaining six
   minutes. Keep a margin; a single optimistic run is not a timing guarantee.
6. Practise saving the Android map and the camera montage. Keep the supervisor's
   recipient address and the professor's preparation clarification accessible.
   Agree who handles the tablet, robot/layout checks, and laptop/evidence.

## What to bring and prepare

- Robot with the working Pi/STM setup, camera securely mounted/focused, charged
  battery, and working Pi-to-STM USB/serial and power connections.
- Laptop with charger/battery, local model/dependencies, tested services and
  calibration options, Wi-Fi configured for the Pi hotspot, and the montage
  URL bookmarked. Task execution should not depend on downloading anything.
- Tablet with the matched app installed, charged battery, Bluetooth permissions
  granted, and the Pi already paired. Practise its screenshot shortcut.
- Charging/data cables and permitted spare power/equipment. Complete any
  replacements before quarantine, subject to supervisor instructions.
- Saved launch commands, this checklist, supervisor contact details, and the
  professor's clarification. Use permitted notes; do not assume equipment can
  be accessed after quarantine.

## Normal startup before quarantine

Keep laptop Wi-Fi on the Pi hotspot; use its actual Wi-Fi address in the Pi's
API settings. The laptop's internet/tether address is not the server address
reachable by the Pi. Check connectivity from the Pi, not only laptop localhost.

From the Windows repository root, after stopping any older laptop servers:

```powershell
.\Algo\.venv-cv-win\Scripts\python.exe -u RPi-services/laptop_start.py --python .\Algo\.venv-cv-win\Scripts\python.exe --cv-backend trial
```

This uses the standard speed50 / 5 cm test configuration. There are no movement
profile switches. See [current setup and backups](TASK1_GRID5_TEST.md). Check `http://127.0.0.1:8000/status` for the loaded configuration
and `http://127.0.0.1:8001/health` for CV. Keep the terminal open.

On the Pi, after stopping the existing controller and confirming there is one
parent owning the links:

```bash
cd ~/SC2079-MDP/rpi
python3 -u task1_main.py
```

Ensure deployed settings use the laptop's current Wi-Fi IP. `LAPTOP_IP=...`
works only after the updated settings support is present. Do not use `--manual`
or the sensor/Task2 helper `final_week_8.py` for the assessment run. The new
speed50 overlays do not require the old `--calibrated` speed25 option.

Connect the tablet, perform a permitted pre-quarantine smoke test, then reset
its timer and map so old recognition labels are absent. Prevent laptop sleep
and tablet screen timeout for the assessment. Follow the supervisor's power
and quarantine instructions; do not rely on being allowed to restart/setup
equipment while it is quarantined.

## What is likely to happen during Task 1

1. **Quarantine.** Put laptop, tablet and robot in the designated area before
   layout release. Team order is random. Do not modify settings while quarantined.
2. **Your turn is called.** Retrieve equipment and begin the supervised two-minute
   preparation. Put the whole robot in the yellow carpark. Confirm the tablet
   and laptop/Pi services have their working connections.
3. **Enter the official layout.** Place the robot's true centre coordinate and
   heading in Android. Enter every obstacle's x/y and image-facing side. The
   given face describes the side of the obstacle with the image, not the
   direction the robot should face. Up=N, right=E, down=S, left=W on the arena
   coordinate system. Verify physical obstacles/images against the official
   layout; the rules say coordinate checks do not require physical measuring.
4. **Send Map early.** Pi asks Algo for the route and remains stationary. Wait for
   Route ready and enabled Task 1 Start. Calculation during preparation is
   allowed according to the professor clarification you reported. If you change
   an obstacle/pose, move manually, or reconnect, send the final map again.
   Any calibration during preparation must stay wholly inside the carpark;
   leaving it during calibration disqualifies that run (FAQ 9).
5. **Ready and approval.** Tell the supervisor you are ready. Check the final map,
   that Task 1 is selected, and that its Start is enabled. Press Start only after
   approval. Place the tablet so the supervisor can see the virtual map and IDs.
6. **Autonomous run.** Do not touch other equipment or controls. Pi sends one
   movement at a time, waits for matching completion, reports the estimated
   position, captures at each SNAP, and updates the target ID on Android.
   Pauses during capture/retry are expected. Chat logging is supplementary;
   leave the map visible. Failed detections can trigger one automatic revisit
   pass within this same run; this is separate from the official RETRY chance.
7. **Automatic finish.** After final motor completion and recognition, FIN clears
   execution and sends finished; Android stops the timer. The supervisor checks
   the final displayed IDs and automatic stopping. Task 1 does not require a
   return to the carpark. Do not press manual Stop to make a completed run look
   automatic. If intervention is necessary, the run is incomplete.
8. **Show and save evidence after the run.** Keep the final Android map visible
   and ensure the supervisor photographs it. On the laptop open/refresh
   `http://127.0.0.1:8001/stitch`; show the tiled camera backgrounds, boxes and
   image IDs, and check they match the tablet. Save a tablet-map screenshot and
   laptop-montage screenshot promptly, then email both to the supervisor. Do not
   reset the app, exit CV, or clear outputs before saving them.
9. **RETRY decision, if needed.** There is only one official RETRY shared between
   Task 1 and Task 2. Choosing it voids the first run's score. Task 1 retry is
   after all teams' first attempts and uses a new layout; keep the equipment
   quarantined throughout, with no robot or algorithm-setting modifications.

## Timing reminder

The first two minutes are preparation. Preparation beyond that consumes the
six-minute allowance. Remaining execution allowance is:

`360 seconds - max(0, preparation seconds - 120 seconds)`

For example, three minutes of preparation leaves five minutes to finish.
Android starts its timer when Start is pressed and does not account for this
overtime. Follow the supervisor's official timing rather than treating six
minutes on the app timer as automatically valid.

## Sources

- `Rules for the Task 1 and Task 2 (1).pdf`: general rules; Task 1.1-11;
  FAQ 3/4/8/9/12-17. The other rules copy has identical contents.
- `MDP briefing.pdf`: Task 1 image pool and task assessment slides 15-17.
- `MDP assessment and system checklist (2).pdf`: 2026-2027 assessment;
  mobile robot/path-planning/Android module checklist pages 6-10.
- `Android Remote Controller checklist (1).pdf`: C.1-C.10.
- Professor clarification reported by the user in this chat on 2026-10-07.

The assessment PDFs specify requirements. Repository documentation and source
provide implementation evidence; neither replaces a physical run or supervisor
instructions.
