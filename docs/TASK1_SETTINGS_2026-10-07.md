# Current Task 1 settings — 7 October 2026

One active setup: speed 50, 90° turns, 10 cm planner/tablet cells, and 40 cm
Manhattan obstacle clearance for turn starts/ends. Robot-centre arena limits
remain cells 1–18. The profile identifier remains restored-speed50-grid10 so
the existing Pi startup check stays compatible.

Jacob confirmed these are bare STM 90° turns at speed 50, without padding:

| Turn | Measured longitudinal shift | Measured lateral shift | Straight before | Straight after | Planner endpoint |
| --- | --- | --- | --- | --- | --- |
| FL | Forward 15 cm | Left 31 cm | Forward 5 cm | Forward 9 cm | Forward 20 / left 40 cm |
| FR | Forward 20.5 cm | Right 38 cm | Forward 10 cm | Forward 2 cm | Forward 30 / right 40 cm |
| BL | Back 29.5 cm | Left 13 cm | Reverse 1 cm | Reverse 7 cm | Back 30 / left 20 cm |
| BR | Back 36 cm | Right 21 cm | Reverse 4 cm | Reverse 9 cm | Back 40 / right 30 cm |

The after-turn straight follows the robot's new heading. Values are physical
whole centimetres sent to STM. Exact FR/BL before-padding would be 9.5/0.5 cm;
integer commands round these upward to 10/1 cm. Predicted FR and BL longitudinal
displacements are therefore 30.5 cm, while the planner uses 30 cm. This 0.5 cm
residual needs an arena check.

Algo/consts.py now has FL/FR/BL/BR geometry (longitudinal,lateral) of (2,4),
(3,4), (3,2), (4,3) cells, and before/after padding (5,9), (10,2), (1,7), (4,9) cm.
Forward/reverse straight conversion remains 10 cm per cell. Straight clearance
rejects positions when Manhattan distance is below 40 cm AND maximum axis
distance is below 30 cm. Turn clearance requires Manhattan distance at least
40 cm. The 20×20 arena uses centre coordinates 1–18 inclusive.

The planner caches legal neighbours and directed paths by coordinates/heading,
including failures. Map edits clear these caches. Directed reachability excludes
impossible viewing poses while retaining all physical obstacles. One grouped
subset dynamic program chooses camera poses and visit order: maximize targets
covered, then minimize travel cost and camera penalties. It replaces thousands
of repeated route-order calls. Photo-only states generate SNAP without a
fabricated backward move, preserve separate target IDs at a repeated pose,
and keep following command poses aligned.

Live checks from the Pi before loading the new turn padding:

| Recent layout | Earlier observed time | Optimized live time | Coverage |
| --- | --- | --- | --- |
| Obstacle 5 at (16,10,W) | About 103 s | 1.05 s | 7/8; obstacle 5 remains unreachable |
| Obstacle 5 moved to (16,12,W) | About 49 s | 1.50 s | 8/8 |

These compare earlier observed runs with the new server, rather than controlled
repeated benchmarks. Physical run time and movement accuracy are separate.

Flow remains Send Map → READY → Start. Valid partial routes enable Start and
skip omitted targets. Map edits invalidate readiness. As updated on 8 October,
photo policy is up to three photos per visit, stopping at the first accepted
result, and at most one automatic revisit pass for visited targets whose
recognition failed: maximum six photos per revisited
obstacle. Targets omitted by planning do not enter the revisit list. Missing
images or an unavailable revisit still finish normally and stop the timer;
an actual motor fault halts dispatch.

Timeouts: Algo connect/read 3/180 s; Android READY wait 190 s in the updated
APK; motor completion watchdog 60 s; cold rpicam-still operation 10 s; image API
connect/read 3/10 s. The tablet timer starts at Start and continues through
capture/revisit planning. Official preparation overrun is not added automatically.

Laptop endpoints remain Algo8000 and CV8001. The last verified laptop hotspot
address is 192.168.50.181; the Pi is 192.168.50.5. CV now uses only
RPi-services/image-api/best_v11i_m.onnx with confidence 0.25.
Laptop images/montages stay in RPi-services/image-api/own_results/ and Pi originals
in /home/nat/SC2079-MDP/rpi/captures/.
Fast capture keeps Picamera2 open between scans; rpicam-still remains the fallback.
Deploy camera.py with task1_controller.py and recognition.py for this update.

Planner and padding changes need only a laptop service refresh. The earlier
Android 190-second wait fix still requires installing its updated APK if the
tablet has the old build. Stop other programs using STM serial before Task 1.

After loading the new padding, the live recent maps returned 7/8 in 0.97 s and
8/8 in 1.38 s. Their final commands and command poses matched the current padding.
The Pi's startup profile check accepted the server at speed 50.

Verification used focused map/command checks, small cache and optimizer
reference checks, three existing restored-grid tests (1.4 s), and two fast
optimizer/cache regressions (0.024 s), rather than the full suites. No motor
commands were sent. Backups/records are in
output/planner-performance-2026-10-07/ and output/turn-readings-2026-10-07/.
Git commit 0f1d01ce preserves the old planner and padding. The senior ZIP remains
a reference; its collision checker, turn calibration and workers were not copied.
