# Raspberry Pi Task 1

Run **task1_main.py**, importing task1_controller.py. The matching laptop profile
is **restored-speed50-grid10**: speed50, 10 cm planner/tablet map, arena20,
turn_clearance_cm40. The launcher checks these before opening hardware.

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use the laptop's actual Wi-Fi address. Settings support LAPTOP_IP, CV_IP,
ALGO_PORT, IMAGE_API_PORT and STM_SERIAL_PORT. Defaults are ports8000/8001
and the existing serial by-id device. Dependencies are requirements.txt;
Bluetooth uses stdlib sockets. The camera is kept running for the whole run via
Picamera2 (camera.py); rpicam-still is only the fallback if it cannot open.

MAP prepares the route before Start; matching READY enables BEGIN. Planning
requests use (3,180) seconds for connection/read; the separate motor-completion
watchdog stays60 seconds. The Pi rejects mismatched grid responses. Locations
advance only after matching ACK. Turn geometry uses the 10 cm map; the laptop
applies the latest measured speed-50 padding. Revisit coordinates remain on
the same 10 cm grid. The Ctrl-C shutdown fix remains installed.

Camera frames, including retries, go into captures/. Laptop raw images, result
JSON, annotations and montages stay in RPi-services/image-api/own_results/.
Task 1 checks that the laptop is running best_v11i_m.onnx before using STM.
The CV upload includes run_id, target_obstacle_id, camera alignment and attempt.
Accepted IDs are normalised by recognition.py, sent as TARGET frames and cached
for Bluetooth reconnect replay. Already recognised obstacles are not recaptured.
Other robot controllers/serial monitors must
be stopped before Task1. --manual is a separate arrow test; tools/bt_echo.py is
a Bluetooth diagnostic. run_week8.sh launches the same Task1 program.

archive/ holds old A5/sensor experiments and numbered Task2 versions. The
preserved KC monitor and helpers are in task2/; its motion logic is unchanged.
A Git push does not deploy this non-Git Pi directory.
Deploy task1_controller.py, recognition.py and camera.py together for this update.
System Python needs Picamera2 for fast capture; rpicam-still remains the fallback.
Stop the camera preview before starting Task 1.

[ONNX handoff deployment](../docs/TASK1_ONNX_HANDOFF_2026-10-08.md)

```bash
python3 -B tests/test_offline.py
```

Tests mock hardware; Linux also exercises real Ctrl-C/worker/Manager shutdown.
[Current setup and rollback backups](../docs/TASK1_ROLLBACK_2026-10-07.md)
[Current measured turn settings and planner performance](../docs/TASK1_SETTINGS_2026-10-07.md)

Photo policy: one photo per visit with no immediate retry, and at most one
revisit pass (two photos total per revisited obstacle).
Partial recognition and an
unavailable revisit now send finished normally. See [photo/stop update](../docs/TASK1_PHOTO_STOP_2026-10-07.md).
