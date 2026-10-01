i# MDP Group 24 — Run Book

Everything needed to bring the system up from cold. Last verified working: 2026-09-11.

## Addresses

| Thing | Address | Notes |
|---|---|---|
| Pi (hotspot `MDP-group-24`, pw `123455`) | `192.168.50.5` | user `nat`, pw `123456` |
| Mac on that hotspot | `192.168.50.147` | DHCP — **re-check each session** |
| Camera stream (Pi) | `:5000` | `/health`, `/video_feed` |
| Image API (Mac) | `:8000` | `/health`, `/predict` |
| Viewer (Mac) | `:5173` | browser UI |

Mac IP changes between sessions. Check with `ipconfig getifaddr en0` and update
`~/SC2079-MDP/rpi/settings.py` on the Pi if it moved, or image rec silently fails.

## Keeping internet while on the Pi hotspot

The Mac has one WiFi radio: on the Pi hotspot it has no internet. Plug the iPhone
in by USB with Personal Hotspot on — internet comes over the cable, WiFi talks to
the Pi. Confirm `iPhone USB` sits above `Wi-Fi` in
System Settings -> Network -> "..." -> Set Service Order.

The Mac tends to roam back to campus WiFi. If SSH or the stream dies for no
reason, check `ipconfig getifaddr en0` — it should be `192.168.50.x`.

---

## Architecture (as of 2026-09-17)

The Algo server runs on the **Mac**, not the Pi (the Pi is too slow for YOLO).
It is the Group-24 Flask app in `26S1-.../Algo`, and it serves *both* path
planning and image recognition, so the old `image-api` is no longer needed.

```
Tablet  --Bluetooth SPP ch1-->  Pi  --WiFi-->  Mac :8000  (Algo: /path + /infer)
                                 |
                                 +--serial /dev/ttyACM0-->  STM32 (MDP_test.hex)
```

### Ports on the Mac — both inference models stay available

```
:8000  Algo (Flask)   path planning /path + /status   (also has /infer, best.pt)
:8001  image-api      /predict + /health              (best_v11i_m.onnx, 31 classes)
:5173  viewer         proxies /stream -> Pi, /api -> API_HOST
```

**Image recognition uses image-api on :8001** (`best_v11i_m.onnx`) — the Algo
server does path planning only.

The Algo repo's own `best.pt` read arrows and digits less reliably and is
**disabled**: moved to `disabled-models/best.pt.unused`, and `main.py` now loads
it lazily, so the server starts fine without it and `/infer` answers 503 with a
pointer to :8001 instead of crashing. Restore by moving the file back — no code
change needed.

Note they are genuinely different networks, not two exports of one:

| | `best_v11i_m.onnx` (in use) | `best.pt` (disabled) |
|---|---|---|
| type | YOLOv8m-**seg** (`task='segment'`) | detect |
| trained | 2024-09-20 | with the 2026 Algo repo |
| labels | `A`,`BULLSEYE`,`CIRCLE`,`LEFT`,`UP`… | pre-mapped ids `20`,`21`,`NA`,`22`… |
| has `stop` | **no** | yes (id 40) |
| has `CIRCLE` | yes | no |

The onnx is a **segmentation** export: `YOLO(MODEL_NAME, task='segment')` in
image-api is required — without the explicit task ultralytics yields junk class
indices and confidences above 1.0. Do not "simplify" that line.

**Check the official image list against the onnx label set.** It has no `stop`
class, so if the assessment includes a stop sign this model cannot detect it at
any threshold — that needs a retrain, not a config change.

Start the viewer against image-api:

```bash
cd ~/docs/Y4S1/mdp/final/mdp-transfer/viewer
PI_HOST=http://192.168.50.5:5000 API_HOST=http://localhost:8001 npm run dev
```

```bash
cd ~/docs/Y4S1/mdp/final/mdp-transfer/image-api
IMAGE_API_PORT=8001 ./venv/bin/python main.py
```

The two APIs are *not* interchangeable — endpoint, upload field and response
shape all differ:

| | image-api :8001 | Algo :8000 |
|---|---|---|
| endpoint | `/predict` | `/infer` |
| field | `image` | `file` |
| reply | `{image_id, conf, xyxy}` / `{'image_id':'NA'}` / **404** | `{ok, n, detections:[…]}` |

`rpi/settings.py` therefore has two pairs: `API_IP`/`API_PORT` (algo, 8000) and
`API_IP_IMAGE`/`API_PORT_IMAGE` (image-api, 8001).

Compare the models on a live frame with `rpi/compare_models.py`.

### Start the Algo server (Mac)

```bash
cd ~/docs/Y4S1/mdp/final/26S1-SC2079-Multidisciplinary-Project-Group-24-main/Algo
uv venv --python 3.12 .venv                       # first time only
uv pip install --python .venv/bin/python -r requirements.txt
./.venv/bin/python main.py                        # binds 0.0.0.0:8000
```

`main.py` has `run_turtle = False`, so it runs Flask headless. First `/infer`
call takes ~12s (model load); warm calls are ~190-890ms depending on image size.

Endpoints: `GET /status`, `POST /path` (text/plain), `POST /infer`
(multipart, field name **`file`** — `image` returns 400).

`/path` payload — `convert_input()` reads fields 2, 3 and 5 of the comma split
and only accepts spelled-out directions:

```
STATUS,OBSTACLE_LIST,<retrying>,<rx>;<ry>;<NORTH|EAST|SOUTH|WEST>,false,<id>;<x>;<y>;<DIR> | ...,true,true
```

Response: `data.stm_commands`, e.g.
`['right,90,forward,0', 'center,0,forward,80', 'SNAP1_L', 'FIN']`.
Note SNAP carries a direction suffix (`SNAP1_L`) — strip it for the obstacle id.

**`videostream/main.py` and Week 8 cannot both run** — Picamera2 holds the
sensor exclusively and `rpicam-still` then fails with "Device or resource busy".
Stop the stream before a run. Never `kill -9` it while it holds the camera: that
wedges the V4L2 driver ("Operation already in progress") and only a reboot
clears it. Use plain `kill <pid>` and let it release.

## Launch sequence

### 1. Mac — image API (terminal 1)

```bash
cd ~/docs/Y4S1/mdp/final/mdp-transfer/image-api
./venv/bin/python main.py            # NOTE: venv python, not plain `python`
```

### 2. Mac — viewer (terminal 2)

```bash
cd ~/docs/Y4S1/mdp/final/mdp-transfer/viewer
PI_HOST=http://192.168.50.5:5000 npm run dev
```

Open http://localhost:5173

### 3. Pi — camera stream (terminal 3)

```bash
ssh nat@192.168.50.5
cd ~/SC2079-MDP/videostream && python3 main.py
```

### 4. Pi — main program (terminal 4)

```bash
ssh nat@192.168.50.5
cd ~/SC2079-MDP/rpi && python3 final_week_8.py
```

No `sudo` and no `venv` — the venv is empty; system python3 has everything.
It prints `Awaiting Bluetooth connection on RFCOMM CHANNEL 1` and blocks. That is
it working.

### 5. Tablet

Pair with `MDPGrp24` in Android Bluetooth settings (one time), then connect
from the MDP app. The Pi advertises **standard SPP** (UUID
`00001101-0000-1000-8000-00805F9B34FB`) on **RFCOMM channel 1**.

### Two-way Bluetooth test (the send-and-echo deliverable)

Instead of step 4, to prove messages travel both ways:

```bash
ssh nat@192.168.50.5
cd ~/SC2079-MDP/rpi && python3 bt_echo.py
```

Connect from the tablet, then type at the `send>` prompt. Anything typed goes to
the tablet; anything the tablet sends back prints as `<-- FROM TABLET`.
Messages are newline-terminated JSON: `{"cat": "info", "value": "hello"}`.
Type `status:running` to set a category. `quit` to exit.

If the app hardcodes the old custom UUID `94f39d29-7d6d-437d-973b-fba39e49d4ee`
it will not find the service — either change the app to the standard SPP UUID or
have it connect to channel 1 directly.

---

## Verify

```bash
curl http://192.168.50.5:5000/health     # camera  -> {"status":"healthy"}
curl http://localhost:8000/health        # image API -> healthy, 31 classes
curl http://localhost:5173/stream/health # viewer can reach the Pi
ssh nat@192.168.50.5 'curl -s http://192.168.50.147:8000/health'   # Pi can reach Mac
```

---

## Known state / gotchas

- **STM32 is connected (as of 2026-09-17).** The adapter is a **CH34x**
  (`1a86:55d4`, QinHeng), not the Silicon Labs CP2102 the original notes assumed.
  It enumerates as `/dev/ttyACM0` via
  `/dev/serial/by-id/usb-1a86_USB_Single_Serial_5A6C068380-if00` at 115200.
  Verified openable with pyserial; `nat` is in `dialout` so no chmod is needed.

  **The by-id name embeds the adapter's serial number, so it differs per physical
  board.** `settings.py` had `...5A6C068378...` (a different unit) which silently
  fails to resolve. If `STMLink.connect()` errors, run `ls /dev/serial/by-id/` and
  paste the real name into `settings.py`. Backup at `settings.py.bak-8378`.

- **The STM does not answer over the board's USB-C port.** The firmware
  (`robot/mdp-grp-16`) serves **USART3 on header pins PC10 (TX) / PC11 (RX)** at
  115200 8N1 — see `mdp-grp-16.ioc` and `main.c:781`. The USB-C connector is not
  wired to USART3, so plugging Pi->USB-C enumerates a CH343 bridge and then sits
  silent forever. Wire a USB-TTL adapter to the header instead:
  `adapter TX -> PC11`, `adapter RX -> PC10`, `GND -> GND`.
  Verified 2026-09-17: board powered (LEDs lit), every baud 9600-230400 silent,
  no ACK, robot did not move.

- **THE STM PROTOCOL IS NOT WHAT `rpi/communication/stm32.py` SENDS.** The board
  runs `MDP_test.hex` (Downloads), *not* the `robot/mdp-grp-16` source. Decoded
  from the hex's string table and verified live on 2026-09-17:

  ```
  send:   <id>/<COMPONENT>/<COMMAND>/<p1>/<p2>;      <- terminator is ';' NOT '\n'
  reply:  !<id>/DONE;                                 <- async, arrives on completion
          !<id>/ERROR/INVALID_SPEED;
          !<id>/ERROR/UNKNOWN_MOTOR_CMD;
          !<id>/ERROR/INVALID_COMPONENT;
          !<id>/ERROR/INVALID_COMMAND_ID;
          !<id>/ERROR/QUEUE_FULL;
  ```

  Components: `MOTOR`, `GENERAL`. Motor commands: `FWD REVS REVL REVR STOP
  TURNL TURNR TURN90L TURN90R TASK2 PWMTURNL PWMTURNR`. General: `CAPTURE DONE`.
  Parser is `sscanf("%d/%40[^/]/%40[^/]/%d/%d")`. Speed is range-checked (9999
  rejected, 50 accepted). Boot banner is `!READY;`.

  **A newline-terminated command is silently ignored** — that alone cost an hour
  of debugging. Verified: `1/MOTOR/STOP/0/0;` -> `!1/DONE;` in 0.3s, while
  `1/MOTOR/STOP/0/0\n` -> silence for 5s.

  `DONE` is asynchronous and identified by `<id>`, so the sender must track ids
  rather than assuming the next reply belongs to the last command.

  Old-style 4/5-byte commands (`FW010`, `STOP`) get no reply at all.

- **Two latent bugs to fix before the STM will ever ACK**, both in
  `Core/Src/communication.c` vs `rpi/communication/stm32.py`:
  1. Firmware does `HAL_UART_Receive_IT(..., COMMAND_MAX_LENGTH)` with
     `COMMAND_MAX_LENGTH 5` (`ring_buffer.h:15`), so it blocks until **exactly 5
     bytes** arrive. A 4-char command like `STOP` never fires the RX interrupt.
     Pad commands to 5 bytes.
  2. Firmware replies with a bare `"ACK"` and **no newline**
     (`Communication_Send_Ack`), but `STMLink.recv()` calls `readline()`, which
     blocks until a newline or timeout. Use `read(3)` or have the firmware append
     `\n`.

- **pybluez does not work on this Pi and cannot be made to.** Python 3.13 removed
  distutils, the libbluetooth headers are absent, and the Pi has no internet in
  hotspot mode. `communication/android.py` was rewritten to use the standard
  library `socket.AF_BLUETOOTH` plus `sdptool` for the SPP record. The original is
  kept at `communication/android.py.pybluez-backup`.

- **Bluetooth needs `bluetoothd --compat`**, set permanently via
  `/etc/systemd/system/bluetooth.service.d/override.conf`. That same override has
  an `ExecStartPost` that chmods `/var/run/sdp` on every start, so no manual
  `chmod` is needed after reboots any more.

- **BlueZ hides the adapter after 180s by default**, which is why the tablet
  stopped seeing the Pi. `DiscoverableTimeout = 0` in `/etc/bluetooth/main.conf`
  plus the same call inside `android.py` keeps it visible permanently. Adapter
  alias is now `MDPGrp24` (was `raspberrypi`) and class is `0x1f00` so Android
  does not filter it out.

- **The `0x1f00` class does NOT survive a reboot** — it comes back as `0x400000`.
  The alias `MDPGrp24` does persist. Re-set the class after every boot:
  `echo 123456 | sudo -S hciconfig hci0 class 0x1f00`. Only matters for a tablet
  that still has to *discover* the Pi; an already-paired tablet connects fine
  without it.

- **Pi clock runs weeks behind** (no NTP on the hotspot), so timestamps in
  `/tmp/*.log` are not wall-clock time. Harmless, but don't trust them when
  correlating with Mac-side logs.

- **pyserial** is installed by unzipping the wheel into
  `~/.local/lib/python3.13/site-packages/` because system python has no pip and
  PEP 668 blocks installs. To add more pure-python packages, do the same.

- **Mac npm cache has root-owned files** from a past `sudo npm`. Fix once with
  `sudo chown -R $(id -u):$(id -g) ~/.npm`.

- **Vite was pinned to 5.4** (from 8) because Node 20.9 is too old for Vite 8.

- `settings.py` originally pointed image rec at `192.168.16.151:8001`; corrected to
  the Mac at port 8000. Backup at `settings.py.bak`.
