# Raspberry Pi Task 1

Run **task1_main.py**, importing task1_controller.py. The matching laptop profile
is **restored-speed50-grid10**: speed50, 10 cm planner/tablet map, arena20,
turn_clearance_cm30. The launcher checks these before opening hardware.

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

Use the laptop's actual Wi-Fi address. Settings support LAPTOP_IP, CV_IP,
ALGO_PORT, IMAGE_API_PORT and STM_SERIAL_PORT. Defaults are ports8000/8001
and the existing serial by-id device. Dependencies are requirements.txt;
Bluetooth uses stdlib sockets and the camera uses OS rpicam-still.

MAP prepares the route before Start; matching READY enables BEGIN. Planning
requests use (3,180) seconds for connection/read; the separate motor-completion
watchdog stays60 seconds. The Pi rejects mismatched grid responses. Locations
advance only after matching ACK. Turn geometry/padding and the 10 cm revisit
coordinates match the pre-trial setup. The Ctrl-C shutdown fix remains installed.

Camera frames, including retries, go into captures/. Laptop images and saved
montages stay in output/cv-trial/. Other robot controllers/serial monitors must
be stopped before Task1. --manual is a separate arrow test; tools/bt_echo.py is
a Bluetooth diagnostic. run_week8.sh launches the same Task1 program.

archive/ holds old A5/sensor experiments and numbered Task2 versions. The
preserved KC monitor and helpers are in task2/; its motion logic is unchanged.
A Git push does not deploy this non-Git Pi directory.

```bash
python3 -B tests/test_offline.py
```

Tests mock hardware; Linux also exercises real Ctrl-C/worker/Manager shutdown.
[Current setup and rollback backups](../docs/TASK1_ROLLBACK_2026-10-07.md)
