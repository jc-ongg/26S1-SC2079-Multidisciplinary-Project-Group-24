# Task 1 improvement backlog - 7 October 2026

This is a record of proposals and user preferences. None of the changes below
has been implemented by creating this note.

## Current baseline and agreed scope

- Working source baseline: c4e4aad9, speed input 50, 10 cm planner and tablet
  cells, 20 x 20 arena, and 40 cm Manhattan turn-start/end clearance.
- The optimized planner caches directed paths/neighbours and uses one grouped
  subset dynamic program for camera-position selection and visit order.
- Keep the working 10 cm setup recoverable when trying a finer planner.
- The team wants 1 cm cells for internal Algo path calculation only.
  Keep the tablet map at 10 cm. This preference does not authorize a migration.
- Latest trial plan: on 8 October 2026, try a 5 cm internal planner first;
  consider 1 cm after acceptable planning time, coverage and physical movement.
  No grid migration is requested for today. Keep speed input 50, the 10 cm
  tablet map and physical clearance settings consistent for comparisons.
- Proposed chat log: one task1_latest.txt, overwritten at each new Task 1
  attempt; include preparation by starting the file at Send Map. Append
  continuously and finalize it on an ending. Provide Export/Share on Android.
- User asked for proposals first, without runtime/source changes.
- User prefers small focused verification rather than long unit-test suites.
- The Pi is currently unavailable to the user. Verify deployed source and
  flashed STM firmware when it is accessible; do not assume local STM source
  is the flashed build.

## Priority proposals

| Priority | Proposal | Components | Evidence and intended result |
| --- | --- | --- | --- |
| 1 | Select the intended target when several symbols are visible | Algo/CV, Pi | Pi drops the SNAP left/centre/right hint; CV chooses the largest box. An eight-target row produced paired duplicate readings. Pass viewing intent and distinguish adjacent detections; return ambiguous/missing intended detections as failures. |
| 2 | Verify and unify reverse-turn execution and calibration | STM32, Algo | Local REVL/REVR routines contain internal straight segments, fixed motor values and 87-degree internal targets. They bypass forward-turn heading bookkeeping. First identify the flashed build, then measure the complete emitted commands. |
| 3 | Make short straight compensation commands reliable | STM32, Algo | Local forward stopping uses target minus 1 cm; a 1 cm forward can complete without powering motors. Saved plans contain 1-2 cm commands. Validate short-command response before tuning fine pads. |
| 4 | Check the full swept robot body and compensation trajectory | Algo | Current turn legality checks only start/end centre positions. Model intermediate turn poses, physical footprint, arena edges and the final padded/cancelled command trajectory. |
| 5 | Confirm physical Stop and detect stalled movement | Android, Pi, STM32 | Task 1 manual Stop and Ctrl+C currently do not request STM STOP; ACK faults stop dispatch only. Add confirmed Stop, ABORTED handling and encoder-progress watchdogs. Local STM source supports preemption. |
| 6 | Unify every run-ending path | Pi, Android, CV for evidence | Normal completion, partial results, exhausted/unavailable revisit, manual Stop and faults should share idempotent cleanup while retaining distinct outcome and motion-confirmation state. |
| 7 | Restore state after Bluetooth reconnection | Pi, Android | Reconnect currently sends connection/mode messages without a complete pose/results/terminal-state snapshot. Replay authoritative state so a lost finish message cannot leave the tablet timer running indefinitely. |
| 8 | Persist and export the current-run TXT transcript | Android; Pi for additional structured run data | Current chat is in-memory with a 500-entry limit. Record run/map/settings, messages, timestamps, target findings, errors and a final summary continuously. Overwrite only for a new attempt. |

## Other improvements to consider

- Route costs based on measured execution time: current ordinary turn cost is
  about 31 versus 1 for a 10 cm straight. Measure complete turn/padding time
  and use consistent units when optimizing running time.
- Publish measured gyro heading; add encoder-travel telemetry for position
  estimation. Odometry still needs calibration and does not establish exact
  physical X/Y. Keep one serial reader.
- Persist image metadata per run and obstacle. Current CV stores an in-memory
  capture index keyed by recognized image ID, so duplicate IDs overwrite a
  tile and CV restart loses the index. Keep originals, accepted/rejected
  detections, confidence, intended obstacle, pose and annotations.
- Monitor Pi worker failures and STM telemetry freshness. Enforce one serial
  owner across Task 1, manual mode and KC monitor. Task 2 already has useful
  typed telemetry, freshness and ABORTED/STOP handling to reference.
- Improve camera resolution/exposure and useful second-attempt behaviour.
  Retain the requested one initial photo plus one retry per visit.
- Consolidate configuration and include calibration/firmware identifiers or
  hashes in status/startup checks. Current profile checks do not validate
  the exact padding or flashed firmware.
- Handle invalid maps and protocol values with explicit preparation errors
  instead of unexpected exceptions/worker exits.
- Reconcile Android Start acceptance with Pi state; expose failures clearly
  and give pending manual commands a bounded recovery path.
- Align viewer hosts/ports with current Algo 8000 and CV 8001. Consolidate
  legacy inference entry points: Algo /infer still has an index-based class
  mapping while active best.pt labels are numeric image IDs.
- Keep simulator curves labelled as approximations; eventually display the
  same measured movement primitives used for collision checks.

## Proposed clean-end behaviour

- Finished: normal automatic completion with all requested targets recognized.
- Finished with missing targets: valid partial route or recognition/revisit
  attempts exhausted, followed by normal automatic stopping.
- Stopped manually: end execution and report whether STM Stop was confirmed.
- Fault: preserve the fault reason and distinguish confirmed stationary state
  from an unconfirmed motor state; never report a failed move as completed.
- Connection lost: display unknown/disconnected state, then reconcile with
  the Pi. Bluetooth loss alone does not prove the robot stopped.
- Finalize timing once, invalidate/clear the old execution route, publish
  recognized/missing targets, finish the transcript, and save image evidence.
- Preparation failure before Start is a separate outcome and must not invent
  a running timer. A prepared route is not an executed run.

## 1 cm internal-planner proposal

The 2 m arena would require approximately 200 x 200 internal positions rather
than 20 x 20. Including four headings, the state space grows by roughly
100 times; runtime growth must be measured and is not necessarily 100 times.

Potential benefits are finer turn endpoints, better placement of scan poses
and less straight padding to force measured turns onto 10 cm multiples.
It does not fix heading error, wheel slip, incorrect starting placement or
unreliable small motor commands. The 0.5 cm measurements remain fractional
even with 1 cm cells.

A migration needs more than changing GRID_CELL_CM:

1. Define one physical coordinate origin and convert tablet cell-centres
   consistently for robot and obstacle inputs.
2. Scale arena dimensions, footprint, wall margins, obstacle size and scan
   distances in physical centimetres. Keep current clearances in centimetres.
3. Represent measured motion primitives in internal units. Keep padding in
   physical centimetres; current helper/execution padding scales with the
   straight conversion multiplier and must be decoupled.
4. Scale A* step costs and its heuristic consistently. Keeping a turn cost
   of 31 with a 1 cm Manhattan heuristic can overestimate the cost of a turn
   spanning 60-70 cm, so simple grid rescaling is insufficient.
5. Update Pi profile checks and plan validation to accept internal 1 cm
   coordinates. Retain exact internal pose for revisits; convert separately
   for the unchanged 10 cm tablet protocol/display.
6. Compress consecutive straight steps into meaningful physical commands.
   A 1 cm planning grid must not imply issuing every movement as a 1 cm
   motor command.
7. Consider coarse planning with fine refinement near turns/scan positions
   to preserve the recent planning speed. Compare coverage, planning time
   and movement repeatability against the recoverable 10 cm baseline.

Likely components: Algo plus Pi; Android changes are needed only if the
display/intermediate pose protocol is extended. Fine grid alone does not
require STM flashing, but reliable motion/Stop changes would.

## Image evidence and retrieval

- Laptop originals, annotations and saved montage destination: output/cv-trial/.
- Pi original capture destination: /home/nat/SC2079-MDP/rpi/captures/.
- The laptop folder was checked during this discussion and was empty.
  The cause was not established; Pi copies have not been checked while offline.
- There is no seven-image cap. NA detections add no accepted tile; duplicate
  recognized image IDs currently share one tile.
- Task 1 rule 7 requires representative camera images with background and
  bounding boxes tiled in one window; FAQ 16 requires the end-of-run display.
  Rule 11 requires emailing the tablet-map and camera-display screenshots.
  Source: Rules for the Task 1 and Task 2.pdf, pages 2, 3 and 8.

## Suggested implementation sequence

Target association, firmware/short-command verification, swept collision
checking, confirmed Stop, then common endings/reconnect/TXT logging. Treat
the 1 cm planner as a separate controlled experiment with the baseline intact.

## Next physical trial - 8 October 2026

1. Preserve the current working 10 cm planner as a recoverable baseline.
2. Prepare a 5 cm internal grid with a 40 x 40 representation of the same
   2 m arena. Keep tablet input at 10 cm, speed input 50, and physical
   obstacle/wall clearances consistent. Update Algo/Pi conversions together.
3. Compare planning time and target coverage on the same layouts. Check
   complete compensated turn endpoints, body clearance and a short physical
   route before an eight-target run. Use focused verification.
4. If the 5 cm version is acceptable, consider a 1 cm internal-grid trial,
   checking search costs/heuristic, command compression and preserved margins.
5. Evaluate the adjacent-target recognition issue separately; changing grid
   resolution alone does not establish which detected symbol belongs to an
   obstacle. Do not automatically replace the known-working configuration.
