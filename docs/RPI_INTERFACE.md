# RPi Interface — connecting and sending commands

Everything needed to drive the robot from the Pi. All of it verified on
hardware 2026-09-17/18; where a thing is assumed rather than measured it says
so.

---

## 1. Getting on

| | |
|---|---|
| Pi | `192.168.50.5`, user `nat`, password `123456` |
| WiFi | `MDP-group-24` / `123455` |
| Laptop (algo + image-api) | `192.168.50.147` |

```bash
ssh nat@192.168.50.5
cd ~/SC2079-MDP/rpi
```

SSH keys are already installed for the usual laptop, so it may not ask for the
password at all.

**The laptop roams off this network constantly.** If SSH hangs or times out,
check `ipconfig getifaddr en0` — if it is not `192.168.50.x`, rejoin
`MDP-group-24` before assuming the Pi is down.

---

## 2. Sending a command to the STM

```python
from communication.stm32 import STMLink

link = STMLink()
link.connect()

link.send("center,0,forward,40")     # drive forward 40cm
print(link.recv())                   # -> "ACK !1/DONE;"

link.disconnect()
```

`send()` accepts either style and translates it:

| you write | goes on the wire |
|---|---|
| `center,0,forward,40` | `1/MOTOR/FWD/50/40;` |
| `center,0,reverse,25` | `2/MOTOR/REVS/50/25;` |
| `right,90,forward,0` | `3/MOTOR/TURNR/50/90;` |
| `left,45,forward,0` | `4/MOTOR/TURNL/50/45;` |
| `STOP` | `5/MOTOR/STOP/0/0;` |
| `FW010` (legacy) | `6/MOTOR/FWD/50/10;` |

`recv()` blocks until the move finishes and returns `ACK !<id>/DONE;`,
`ERROR ...`, or `None` on timeout.

### One-liner from the laptop

```bash
ssh nat@192.168.50.5 'cd ~/SC2079-MDP/rpi && python3 -c "
from communication.stm32 import STMLink
l=STMLink(); l.connect(); l.send(\"right,90,forward,0\"); print(l.recv()); l.disconnect()"'
```

---

## 3. The raw STM protocol

The board runs `MDP_test.hex`. If you bypass `STMLink`, this is the wire format:

```
send    <id>/<COMPONENT>/<COMMAND>/<p1>/<p2>;
reply   !<id>/DONE;
        !<id>/ERROR/INVALID_SPEED;  /UNKNOWN_MOTOR_CMD;  /INVALID_COMPONENT;
        !<id>/ERROR/INVALID_COMMAND_ID;  /QUEUE_FULL;
boot    !READY;
```

**The terminator is `;` — a newline-terminated command is silently ignored.**
That one detail cost a full afternoon. `1/MOTOR/STOP/0/0;` answers in 0.3s;
`1/MOTOR/STOP/0/0\n` gets nothing at all.

Components: `MOTOR`, `GENERAL`.
Motor commands: `FWD REVS REVL REVR STOP TURNL TURNR TURN90L TURN90R TASK2
PWMTURNL PWMTURNR`. General: `CAPTURE DONE`.
Speed (`p1`) is range-checked; 50 works, 9999 is rejected.

`DONE` is **asynchronous** and identified by `<id>` — the reply to command 3 may
arrive after telemetry and after other frames. Match on the id, do not assume
the next frame is yours.

### Do not use TURN90L / TURN90R

Measured against the board's own gyro:

```
TURN90R   -> heading moved   +0.0 deg     (does not register)
TURNR/90  -> heading moved  -90.3 deg     accurate
TURNR/30  -> heading moved  -30.7 deg     accurate
```

`STMLink` therefore routes every forward turn through `TURNL`/`TURNR` with an
explicit angle, including 90. Anything depending on orientation goes wrong with
the square-turn commands.

---

## 4. Telemetry

The board streams this unprompted at ~8 Hz, all the time:

```
!SENSOR/80.0/96.3/52.3/1/-41.2/0.00;
         f1   f2   f3  f4  f5    f6
```

| field | what it appears to be | confidence |
|---|---|---|
| f1, f2 | distance sensors; swing wildly with surroundings | low |
| **f3** | **forward range-finder, in cm** — tracked 102→92→81 across two 10cm moves | good |
| f4 | 0/1 flag, possibly obstacle-detected | guess |
| **f5** | **gyro heading in degrees** — 0.00 deg drift over 40s stationary | good |
| f6 | changes during turns, settles to 0 | unknown |

Nothing is labelled by the firmware. **Ask the STM team before relying on f1,
f2, f4 or f6.**

```python
link.read_sensors()   # -> ['80.0','96.3','52.3','1','-41.2','0.00']
link.heading()        # -> -41.2
link.heading_delta(a, b)   # signed shortest rotation, handles wrap
```

`recv()` skips telemetry automatically and keeps the newest in
`link.last_sensor`. **If you write your own reader, you must skip `!SENSOR`
frames yourself** — otherwise you will read telemetry, never see the ACK, and
the robot will appear to hang after one command.

---

## 5. Measured robot behaviour

| | value | how |
|---|---|---|
| Command units | ≈1 cm | f3 fell ~21 over two 10-unit moves |
| **Turn radius** | **≈30 cm** | 180 turn displaced 60cm; 90 turn displaced 30cm |
| Gyro drift | 0.00 deg / 40s | stationary measurement |
| Turn accuracy | good, **not repeatable** | right-180 gave 110 deg once, 172 deg next |

**The robot does not pivot on the spot.** A 90 turn also carries it ~30cm
sideways and ~30cm forward. Any manoeuvre that assumes point turns will miss.
Because repeatability is poor, prefer closed-loop correction over blind
sequences.

---

## 6. Android over Bluetooth

Pair with **`MDPGrp24`**, RFCOMM channel 1, standard SPP UUID
`00001101-0000-1000-8000-00805F9B34FB`.

Frames are `//payload//` with no newline (see
`MDP_Group24_Bluetooth_Protocol.md`).

```python
from communication.android import AndroidLink, AndroidMessage
a = AndroidLink(); a.connect()          # blocks until the tablet connects
a.send(AndroidMessage("status", "hello"))    # -> //STATUS,hello//
print(a.recv())    # -> '{"cat": "manual", "value": "FW010"}'
```

| RPi to tablet | frame |
|---|---|
| `AndroidMessage("location", {"x":8,"y":12,"d":0})` | `//ROBOT,8,12,N//` |
| `AndroidMessage("image-rec", {"obstacle_id":3,"image_id":11})` | `//TARGET,3,11//` |
| `AndroidMessage("status", "text")` | `//STATUS,text//` |

Inbound `//F// //B// //L// //R//` become `{"cat":"manual","value":"FW010"}` etc.
`//OBSTACLE,...//` and `//ROBOT,...//` only update state; `//BEGIN//` then
emits the whole map plus a start signal.

**One tap currently sends ~5 frames** (button repeat), so one press queues five
moves. Android-side fix.

**A test chat is easier than writing code:**

```bash
ssh nat@192.168.50.5 'cd ~/SC2079-MDP/rpi && python3 -u bt_echo.py'
```

---

## 7. The two HTTP APIs on the laptop

```
:8000  Algo       POST /path  (text/plain)   GET /status
:8001  image-api  POST /predict (field "image")   GET /health
```

Image recognition is **image-api on 8001**. The Algo server's own `/infer` is
disabled — it used different weights that read arrows and digits worse.

```python
# path planning
payload = "STATUS,OBSTACLE_LIST,false,1;1;NORTH,false,1;5;7;EAST,true,true"
r = requests.post("http://192.168.50.147:8000/path", data=payload,
                  headers={"Content-Type": "text/plain"})
cmds = r.json()["data"]["stm_commands"]
# -> ['right,90,forward,0', 'center,0,forward,80', 'SNAP1_L', 'FIN']

# recognition
r = requests.post("http://192.168.50.147:8001/predict",
                  files={"image": open("snap.jpg", "rb")})
# -> {"image_id":"A","conf":0.9,"xyxy":[...],"bullseye":{...}}
#    {"image_id":"NA","bullseye":null}   or HTTP 404 when nothing is found
```

Directions in the algo payload must be spelled out: `NORTH EAST SOUTH WEST`.
`SNAP` commands carry a direction suffix (`SNAP1_L`) — strip it for the id.

The bullseye is reported **separately** from `image_id`, never as the target,
so a blank obstacle face cannot be mistaken for a real symbol. `bullseye`
present with `image_id: "NA"` means "obstacle, wrong face"; `bullseye: null`
means nothing in view.

---

## 8. Things that will bite you

- **Camera is exclusive.** `videostream/main.py` holds it via Picamera2, so
  `rpicam-still` fails with "Device or resource busy". Stop the stream before
  any capture.
- **Never `kill -9` the stream** while it holds the camera — it wedges the V4L2
  driver and only a reboot clears it. `kill -TERM`, wait, then `kill -INT`
  (uvicorn needs the second signal; the first only starts a graceful shutdown
  that never finishes).
- **`libcamera-still` does not exist** on this OS. It is `rpicam-still`.
- **Bluetooth class resets every boot.** Re-run
  `echo 123456 | sudo -S hciconfig hci0 class 0x1f00` or the tablet may not see
  the Pi.
- **`pkill -f <pattern>` over SSH kills your own shell**, because the pattern
  appears in its command line. Kill by PID.
- Only one program may hold `/dev/ttyACM0` usefully at a time. Stop
  `final_week_8.py` before running a test script.
