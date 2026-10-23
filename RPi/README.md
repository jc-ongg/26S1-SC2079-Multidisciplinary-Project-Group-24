# Raspberry Pi integration

[← Project overview](../README.md)

The Pi connects the tablet, camera, laptop services and STM32. Deploy the selected task directory's contents together.

| Directory | Entry point | Responsibility |
| --- | --- | --- |
| `Task1/` | `task1_main.py` | Prepare the map, execute the queued route, capture images and return results. |
| `Task2/` | `KC_task2_stm32_bridge.py` | Handle camera requests/heartbeats, then run Pi-driven return and parking. |

## Prerequisites

- Python 3 and the selected task's `requirements.txt`.
- Camera access through system Picamera2/libcamera, or the `rpicam-still` fallback.
- USB serial access to STM32; Bluetooth RFCOMM support for the Task 1 tablet link.
- A network route to the laptop's planner/recognition services.

Only one controller should own STM32 serial or the camera. Start the laptop services first.

## Run Task 1

Deploy `RPi/Task1` to the Pi. From that deployed directory:

```bash
python3 -m pip install -r requirements.txt
LAPTOP_IP=<laptop-wifi-ip> python3 -u task1_main.py
```

The launcher verifies the laptop movement profile before opening hardware. It expects speed 50, a 10 cm grid and 40 cm turn clearance. The recognition health check expects the Task 1 ONNX backend.

Environment overrides in `Task1/settings.py`:

| Variable | Purpose | Default |
| --- | --- | --- |
| `LAPTOP_IP` | Planner host. | `192.168.50.181` |
| `CV_IP` | Recognition host. | Same as `LAPTOP_IP`. |
| `ALGO_PORT` | Planner HTTP port. | `8000` |
| `IMAGE_API_PORT` | Recognition HTTP port. | `8001` |
| `STM_SERIAL_PORT` | STM32 device. | Existing USB serial by-id path. |

Serial uses **115200 baud**. Override the device/address for the current machine rather than editing source.

### Preparation and Start

1. Connect Android, enter the robot pose/obstacles and send the map.
2. The Pi requests the route, validates it and queues its motor schedule.
3. The matching READY reply enables Start. Android allows up to 190 seconds for preparation.
4. Start executes that prepared request. Map edits/reconnection invalidate prior preparation.

Saved tablet layouts are JSON; Bluetooth uses framed text:

```text
Tablet → Pi: //MAP,example123/ROBOT,1,1,N/OBSTACLE,1,5,7,E//
Pi → Tablet: //PLAN,example123,READY//
Tablet → Pi: //BEGIN,1,example123//
```

The ID must match the latest submission. READY confirms preparation and does not contain the route. See [the tablet protocol](../Android/BLUETOOTH_PROTOCOL.md).

The camera remains warm between captures, and recognition retries stop on success. Tablet pose updates follow matching motor acknowledgements; they represent planned execution poses rather than continuous measured odometry.

## Run Task 2

Deploy `RPi/Task2`, flash matching firmware and start the laptop arrow API. From the deployed directory:

```bash
python3 -m pip install -r requirements.txt
python3 KC_task2_stm32_bridge.py --execute --stage full-pi-return --port /dev/ttyACM0 --laptop-ip <laptop-wifi-ip>
```

STM32 handles O1/O2. The bridge supplies arrows and heartbeats, then `KC_task2_pi_return.py` uses telemetry to plan return, carpark entry and parking. NumPy is required.

Omit `--execute` for a description without opening serial. Other selections are defined in `STAGES` and command-line help. Each stage starts from the original starting pose rather than continuing a previous stage's stopped pose.

| File | Purpose |
| --- | --- |
| `KC_task2_stm32_bridge.py` | Camera worker, arrow confirmation, heartbeat and native-run coordination. |
| `KC_task2_pi_return.py` | Return geometry, carpark entry and parking. |
| `KC_stm32_link.py` | Shared STM32/vision link utilities. |
| `KC_o1_range_watch.py`, `KC_range_calib.py` | Range observation/calibration tools. |

The bridge retries uncertain/unavailable recognition. Its stationary policy can accept repeated agreeing weaker detections; the API's nominal 50% threshold alone does not describe the complete confirmation policy.

## Offline tests

From the repository root, run `python -B RPi/Task1/tests/test_offline.py`. This curated suite blocks real serial construction. The Linux/Pi interrupt regression is skipped on Windows.

From `RPi/Task2`:

```bash
python -B -m unittest test_bridge -v
python -B test_pi_return.py
```

Return tests simulate 144 combinations of layout, pose error, echo angle and carpark depth. They do not replace physical clearance checks.
