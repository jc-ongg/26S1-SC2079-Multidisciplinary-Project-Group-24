# Raspberry Pi Task 1

Run **`task1_main.py`**, which imports `task1_controller.py`. One active setup:
speed 50 and the `turn-guide-speed50-grid5` laptop profile. The launcher verifies
speed, profile, 5 cm planner, 10 cm input map and 40-cell arena before hardware
opens. The old `--calibrated` option has been removed.

```bash
cd /home/nat/SC2079-MDP/rpi
LAPTOP_IP=192.168.50.181 python3 -u task1_main.py
```

`settings.py` supports `LAPTOP_IP`, `CV_IP`, `ALGO_PORT`, `IMAGE_API_PORT` and
`STM_SERIAL_PORT`. Use the laptop's Wi-Fi address. Defaults are ports 8000/8001
and the existing serial by-id device. Current dependencies are `requirements.txt`;
Bluetooth uses stdlib sockets and the camera uses OS `rpicam-still`.

- MAP requests a route during preparation; matching READY enables BEGIN.
- Route requests allow 3 seconds to connect and 180 seconds to read a result.
- A motor command still needs its matching completion within 60 seconds.
- Turn padding and straight-command merging are handled by Algo; the STM
  firmware is unchanged. Tablet location advances after command completion.
- Precise 5 cm poses survive coarse tablet display rounding for revisits.
- Route coverage and omitted obstacle numbers are reported before Start.
- Camera captures, including retry attempts, go into `captures/`.

`task1_main.py --manual` is for arrow testing. `tools/bt_echo.py` is a Bluetooth
diagnostic. Neither should run alongside Task 1. `run_week8.sh` is a compatibility
wrapper for the same Task 1 launcher.

`archive/` preserves older A5/sensor/calibration experiments and numbered Task 2
runners. `task2/` contains the latest recovered KC monitor and its helpers; run
`python3 task2/KC_robot_monitor.py` for that separate serial owner. Its motion
logic is unchanged and was not validated by the Task 1 cleanup. A Git push
does not deploy this non-Git Pi directory.

```bash
python3 -B tests/test_offline.py
```

These tests mock serial, Bluetooth, camera and HTTP. They do not verify physical
accuracy. See [current setup and backup record](../docs/TASK1_GRID5_TEST.md).
