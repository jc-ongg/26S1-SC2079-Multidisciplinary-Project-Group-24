#!/usr/bin/env python3
"""
Box Face Inspection & Circumnavigation Task
--------------------------------------------
Strictly adheres to STM32_README_Group24 protocol:
1. Baud rate: 115200 on /dev/ttyACM0 (USART3)
2. Framing: Commands start with ':' and end with ';' -> :<id>/MOTOR/<cmd>/<speed>/<val>;
   - Speeds (0-101) are the SPEED_FWD / SPEED_REVS / SPEED_TURN constants below;
     override per run with:  --fwd 50 --rev 28 --turn 21
   - STOP: 0
3. Target distance: 30 cm from box face (tolerance ±3.0 cm).
4. Circumnavigation: Uses single-command 180° turn to avoid compounded 90° turn errors.
5. Anti-ramming safety: IR sensors (left_ir, right_ir) stop robot if <= 18 cm.
6. Auto-detects laptop IP from SSH session or accepts command-line argument.
7. Startup Vision Health Check: Verifies YOLO API is alive before moving!
"""

import sys
import os
import time
import subprocess
import requests

# ================= Configuration =================
TARGET_DISTANCE_CM = 30.0      # Desired distance to box in cm (30 cm)
DISTANCE_TOLERANCE_CM = 3.0    # Acceptable tolerance (27.0 cm to 33.0 cm)
IR_EMERGENCY_STOP_CM = 18.0    # If either IR sensor reads <= 18 cm, emergency stop
SNAP_PATH = "/tmp/box_face_snap.jpg"
STREAM_URL = "http://localhost:5000/video_feed"   # Pi camera stream (videostream/main.py)
MAX_FACES = 4                  # A box has 4 faces

SPEED_FWD = 100
SPEED_REVS = 28
SPEED_TURN = 21

# ---- Task 2, obstacle 1 (python3 box_inspect_task.py --task2 [--obstacle 1|2|both]) ----
TASK2_STOP_DIST_CM = 24.0   # stop this far from obstacle 1, read the arrow, then start turning
TASK2_IR_STOP = False       # side-IR anti-ramming stop during the approach. Off: in the
                            # carpark the side IRs see the carpark walls and would trip it.
ARROW_MIN_CONF = 0.60       # ignore arrow reads below this confidence
ARROW_CONFIRMATIONS = 1     # same direction this many reads in a row before turning
ARROW_MAX_TRIES = 6         # captures before giving up (robot then stays put)

# ---- Return leg (--obstacle both only) ----
# x2 = ultrasonic reading right after the obstacle-1 sequence finishes, straight to
# obstacle 2's face. The return leg runs after obstacle 2's sequence, written for a LEFT
# arrow (RIGHT mirrors automatically, same as the obstacle sequences). Edit it freely.
# It is a function of x2, so you can use x2 directly in a step, e.g. ("forward", x2 + 41, 100).
# Same (method, value, speed) format as the other sequences, plus one special step:
#   ("approach_front", target_cm, 0) -> move_to_distance(): read the front ultrasonic once, drive
#                                       forward (or reverse if too close) to end up target_cm away
ROVER_LENGTH_CM = 23.0

# Ending: forward x2 puts the robot parallel to obstacle 1, a sideways distance
#   S = obstacle2_length / 2 + RETURN_D_CM   away from it. It then does a 45-degree S-curve to
# shift sideways by S: turn 45 (toward obstacle 1), drive the diagonal, turn 45 back, then
# approach_front to the stop distance. The turn direction follows obstacle 2's arrow
# (LEFT arrow -> turn_right 45, RIGHT arrow mirrors it).
RETURN_D_CM = 0.0               # <<< the extra cm "D" - EDIT THIS
RETURN_DIAG_ANGLE_DEG = 45
RETURN_DIAG_CORRECTION_CM = 0.0   # tune on the robot: + drives the diagonal longer, - shorter
RETURN_FINAL_STOP_CM = 15         # approach_front target at the end
OBS2_LENGTH_FALLBACK_CM = 60.0    # used (with a warning) if obstacle 2's length was not measured


def return_diagonal_cm(obstacle2_length_cm):
    """Diagonal drive length that shifts the robot sideways by L2/2 + D."""
    import math
    shift = obstacle2_length_cm / 2 + RETURN_D_CM
    diag = shift / math.sin(math.radians(RETURN_DIAG_ANGLE_DEG)) + RETURN_DIAG_CORRECTION_CM
    return max(0.0, diag)


def task2_return_left_sequence(x2, obstacle2_length_cm):
    return [
        ("forward", x2 + ROVER_LENGTH_CM + 22, 100),                              # level with obstacle 1, parallel
        ("turn_right", RETURN_DIAG_ANGLE_DEG, 100),        # toward obstacle 1's side
        ("forward", return_diagonal_cm(obstacle2_length_cm), 100),
        ("turn_left", 30, 100),         # straight again
        ("approach_front", RETURN_FINAL_STOP_CM, 0),       # back to the carpark
    ]


# IR-guided forward steps (fwd_until_<side>_clear / fwd_until_<side>_see).
# The STM's side IRs are analog and only calibrated for ~10-40 cm: 80.0 or a big number
# means "nothing there", so an obstacle is "seen" when the reading is <= IR_SEE_CM.
IR_SEE_CM = 40.0            # reading at or below this = obstacle beside the robot
IR_CONFIRM_SAMPLES = 1      # consecutive fresh IR samples (~100 ms apart) needed to call a change.
                            # 1 = stop on the first clear reading (least overshoot); 2 = debounced (+~0.1 s of travel)
IR_MAX_TRAVEL_CM = 120      # the long forward is this far; if it ends with no trigger the step fails
IR_MAX_DRIVE_S = 25         # wall-clock backstop for the same thing

# Obstacle length from the IR "sees" -> "clear" time span (fwd_past_* steps, no firmware change).
# length = span_s * cm_per_s(speed) * IR_LENGTH_CAL. Measured on the team's robot: 22 -> 28,
# 30 -> 38, 50 -> 50 cm/s (22/30 are inferred). Slower = finer: one ~100 ms telemetry gap is
# ~2.8 cm at speed 22 vs ~5 cm at speed 50. Calibrate IR_LENGTH_CAL on an obstacle of known
# length: IR_LENGTH_CAL = real_length / printed_length.
CM_PER_S = {22: 28.0, 30: 38.0, 50: 50.0, 100: 80.0}
IR_LENGTH_CAL = 1.0

# Sequence for a LEFT arrow: (ProtocolSTMLink method, cm or degrees, speed 0-101).
# The RIGHT arrow runs the mirror image automatically (turn_left <-> turn_right,
# reverse_left <-> reverse_right). Edit this list to tune the manoeuvre.
TASK2_LEFT_SEQUENCE = [
    ("turn_left", 90, 100),
    ("forward", 5, 100),
    ("turn_right", 180, 100),
    #("reverse", 5, 100),
    ("turn_left", 90, 100),
    ("reverse", 15, 100),
]
# Obstacle 1, RIGHT arrow: its own sequence instead of the mirror, because the turning radius
# differs per side - after the 180 the robot needs a small nudge forward to end up centred
# on obstacle 2, then the final 90. Set to None to fall back to mirroring TASK2_LEFT_SEQUENCE.
TASK2_RIGHT_SEQUENCE = [
    ("forward", 3, 100),
    ("turn_right", 90, 100),
    ("reverse", 6, 100),
    #("forward", 7, 100),
    ("turn_left", 180, 100),
    ("reverse", 6, 100),
    ("turn_right", 90, 100),
    ("reverse", 15, 100),
]
MIRROR = {"turn_left": "turn_right", "turn_right": "turn_left",
          "reverse_left": "reverse_right", "reverse_right": "reverse_left",
          "fwd_until_right_clear": "fwd_until_left_clear", "fwd_until_left_clear": "fwd_until_right_clear",
          "fwd_until_right_see": "fwd_until_left_see", "fwd_until_left_see": "fwd_until_right_see",
          "fwd_past_right": "fwd_past_left", "fwd_past_left": "fwd_past_right"}

# ---- Task 2, obstacle 2: the same routine with its own stop distance and sequence ----
# Run it after obstacle 1 with  --obstacle both , or on its own with  --obstacle 2 .
# Starts as a copy of the obstacle 1 values - edit it independently.
TASK2_OBS2_STOP_DIST_CM = 20.0
TASK2_OBS2_LEFT_SEQUENCE = [
    ("turn_left", 90, 100),
    ("fwd_until_right_clear", -3, 50),   # along obstacle 2 until the RIGHT IR spikes to ~80 (end passed)
    ("turn_right", 180, 100), 
    ("fwd_until_right_clear",-5, 50),# U-turn to the far side
    #("fwd_past_right", 0, 100),          # back along it: RIGHT IR sees it, then spikes to ~80 again
    ("turn_right", 90, 100),
]
# A RIGHT arrow runs the mirror: turn_right 90, LEFT IR clear, turn_left 180, LEFT IR past, turn_left 90.


def resolve_laptop_ip():
    """
    Resolves the laptop's IP address:
    1. Command-line argument: python3 box_inspect_task.py 192.168.50.147
    2. SSH_CLIENT / SSH_CONNECTION environment variables (auto-detects laptop from SSH)
    3. Default fallback to 192.168.50.147
    """
    if len(sys.argv) > 1 and sys.argv[1].replace(".", "").isdigit():
        return sys.argv[1]

    ssh_client = os.environ.get("SSH_CLIENT", "")
    if ssh_client:
        return ssh_client.split()[0]

    ssh_conn = os.environ.get("SSH_CONNECTION", "")
    if ssh_conn:
        return ssh_conn.split()[0]

    return "192.168.50.147"


LAPTOP_IP = resolve_laptop_ip()
IMAGE_API_URLS = [
    (f"http://{LAPTOP_IP}:8001/predict", "image"),   # image-api (best_v11i_m.onnx) - tried first
    (f"http://{LAPTOP_IP}:8000/infer", "file"),      # Algo/main.py built-in YOLO (old best.pt) - fallback
]


class ProtocolSTMLink:
    """
    Implements the exact serial protocol specified in STM32_README_Group24.docx:
    - Commands: :<cmd_id>/MOTOR/<command>/<speed>/<value>;
    - Telemetry: !SENSOR/<left_ir>/<right_ir>/<ultrasonic>/<us_valid>/<heading>/<gyro_z>;
    - Acknowledgment: !<cmd_id>/DONE;, !<cmd_id>/ABORTED;, !<cmd_id>/ERROR/<reason>;
    """
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        if not os.path.exists(port):
            # the STM32 can re-enumerate as ttyACM1/2 after a USB glitch - use whichever exists
            import glob
            found = sorted(glob.glob("/dev/ttyACM*"))
            if found:
                print(f"[Serial] {port} not found - using {found[0]}")
                port = found[0]
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.next_cmd_id = 1
        self.latest_telemetry = None
        self.rx_buffer = ""
        self.last_obstacle_length_cm = None   # set by a fwd_past_* drive that saw the whole obstacle

    def connect(self):
        import serial
        self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
        time.sleep(0.4)
        print(f"[STMLink] Connected to {self.port} at {self.baudrate} baud.")
        
        print("[STMLink] Waiting for system telemetry / !READY; ...")
        t_start = time.time()
        while time.time() - t_start < 3.0:
            self.poll()
            if self.latest_telemetry is not None:
                print(f"[STMLink] Initial Telemetry received: {self.latest_telemetry}")
                break
            time.sleep(0.04)

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.stop()
            self.ser.close()
            print("[STMLink] Disconnected.")

    def poll(self):
        """
        Reads pending bytes from UART, parses ;-terminated messages,
        updates latest_telemetry, and returns any terminal ACK/DONE messages.
        """
        terminal_msg = None
        if not self.ser or not self.ser.is_open:
            return None

        while self.ser.in_waiting > 0:
            data = self.ser.read(self.ser.in_waiting)
            if not data:
                break
            self.rx_buffer += data.decode("utf-8", errors="ignore")

            while ";" in self.rx_buffer:
                raw_msg, self.rx_buffer = self.rx_buffer.split(";", 1)
                raw_msg = raw_msg.strip()
                if not raw_msg:
                    continue
                msg = raw_msg + ";"

                if msg == "!READY;":
                    print("[STM32 -> RPi] System Ready & Gyro Calibrated.")
                elif msg.startswith("!SENSOR/"):
                    self._parse_telemetry(msg)
                elif msg.startswith("!"):
                    terminal_msg = msg

        return terminal_msg

    def _parse_telemetry(self, msg):
        """
        Parses !SENSOR/<left_ir>/<right_ir>/<ultrasonic>/<us_valid>/<heading>/<gyro_z>;
        or, from the newer firmware, the same 6 fields plus <desired_heading>/<servo_pwm>;
        """
        payload = msg.rstrip(";").lstrip("!")
        parts = payload.split("/")
        if len(parts) in (7, 9) and parts[0] == "SENSOR":
            try:
                left_ir = float(parts[1])
                right_ir = float(parts[2])
                ultrasonic = float(parts[3])
                us_valid = int(parts[4])
                heading = float(parts[5])
                gyro_z = float(parts[6])
                desired_heading = float(parts[7]) if len(parts) == 9 else None
                servo_pwm = int(parts[8]) if len(parts) == 9 else None

                self.latest_telemetry = {
                    "left_ir_cm": left_ir,
                    "right_ir_cm": right_ir,
                    "ultrasonic_cm": ultrasonic if us_valid == 1 else None,
                    "ultrasonic_valid": bool(us_valid),
                    "raw_ultrasonic": ultrasonic,
                    "heading_deg": heading,
                    "gyro_z_dps": gyro_z,
                    "desired_heading_deg": desired_heading,
                    "servo_pwm": servo_pwm,
                    "timestamp": time.time()
                }
            except ValueError:
                pass

    def send_nowait(self, command, speed, value):
        """Sends a motor command WITHOUT waiting for its DONE; returns its cmd id.
        A later command preempts it (the STM aborts it and starts the new one)."""
        cmd_id = self.next_cmd_id
        self.next_cmd_id += 1
        packet = f":{cmd_id}/MOTOR/{command}/{speed}/{value};"
        self.ser.write(packet.encode("utf-8"))
        print(f"[RPi -> STM32] {packet}  (not waiting)")
        return cmd_id

    def send_cmd(self, command, speed, value, timeout=15.0):
        """
        Sends formatted command :<cmd_id>/MOTOR/<command>/<speed>/<value>;
        and waits for !<cmd_id>/DONE; (or ABORTED / ERROR).
        """
        cmd_id = self.next_cmd_id
        self.next_cmd_id += 1

        packet = f":{cmd_id}/MOTOR/{command}/{speed}/{value};"
        self.ser.write(packet.encode("utf-8"))
        print(f"[RPi -> STM32] {packet}")

        calc_timeout = max(timeout, value * 0.15)
        start_time = time.time()
        while time.time() - start_time < calc_timeout:
            msg = self.poll()
            if msg:
                body = msg.lstrip("!").rstrip(";")
                parts = body.split("/")
                try:
                    resp_id = int(parts[0])
                    if resp_id == cmd_id:
                        status = parts[1]
                        print(f"[STM32 -> RPi] {msg} (Status: {status})")
                        if status == "DONE":
                            return True
                        elif status in ("ABORTED", "ERROR"):
                            print(f"[WARN] Command {cmd_id} terminated with {status}")
                            return False
                except (ValueError, IndexError):
                    pass
            time.sleep(0.02)

        print(f"[WARN] Command {cmd_id} timed out after {calc_timeout:.1f}s.")
        return False

    # speed=None means "use the current SPEED_* constant" so the --fwd/--rev/--turn
    # flags (or an edit to the constants) always take effect.
    def forward(self, cm, speed=None):
        return self.send_cmd("FWD", SPEED_FWD if speed is None else speed, int(cm))

    def reverse(self, cm, speed=None):
        return self.send_cmd("REVS", SPEED_REVS if speed is None else speed, int(cm))

    def turn_left(self, deg, speed=None):
        return self.send_cmd("TURNL", SPEED_TURN if speed is None else speed, int(deg))

    def turn_right(self, deg, speed=None):
        return self.send_cmd("TURNR", SPEED_TURN if speed is None else speed, int(deg))

    # Reverse-turn macros (STM REVL / REVR -> RTURN90L_TEST / RTURN90R_TEST).
    # The firmware runs a fixed routine with its own speeds and angle, so the
    # speed/deg sent here only satisfy the command parser and do not change it.
    def reverse_left(self, deg=90, speed=None):
        return self.send_cmd("REVL", SPEED_TURN if speed is None else speed, int(deg))

    def reverse_right(self, deg=90, speed=None):
        return self.send_cmd("REVR", SPEED_TURN if speed is None else speed, int(deg))

    # ---- IR-guided forward driving -------------------------------------------------
    def ir_sees_obstacle(self, side):
        """
        True if the `side` ("left"/"right") IR currently reads an obstacle (<= IR_SEE_CM).
        Majority of 3 fresh readings, so one noisy sample cannot decide. None = no telemetry.
        """
        key = "left_ir_cm" if side == "left" else "right_ir_cm"
        votes = []
        for _ in range(3):
            telem = get_current_sensors(self)
            if telem:
                votes.append(telem[key] <= IR_SEE_CM)
            time.sleep(0.6)
        if not votes:
            return None
        return sum(votes) * 2 > len(votes)

    def _drive_through_ir_states(self, side, targets, extra_cm, speed):
        """
        ONE continuous forward drive: sends a single long FWD, watches the `side` IR while
        the robot keeps moving, and once the IR has gone through `targets` in order
        (True = obstacle seen, False = clear) each for IR_CONFIRM_SAMPLES fresh samples in
        a row, cuts in with either the extra distance or a STOP. The STM preempts the long
        forward, so there is no start/stop between IR checks.
        """
        speed = SPEED_FWD if speed is None else speed
        key = "left_ir_cm" if side == "left" else "right_ir_cm"
        long_id = self.send_nowait("FWD", speed, IR_MAX_TRAVEL_CM)
        started = time.time()
        idx, streak, last_ts = 0, 0, None
        event_ts = []                       # telemetry timestamp of each state change
        while idx < len(targets):
            msg = self.poll()
            if msg:
                parts = msg.lstrip("!").rstrip(";").split("/")
                if parts[0] == str(long_id):        # the long forward ended by itself
                    print(f"[IR] Drove {IR_MAX_TRAVEL_CM} cm (or hit an error: {msg}) with no "
                          f"{side} IR trigger - giving up.")
                    self.stop()
                    return False
            telem = self.latest_telemetry
            if telem and telem["timestamp"] != last_ts:      # a fresh sample
                last_ts = telem["timestamp"]
                seen = telem[key] <= IR_SEE_CM
                streak = streak + 1 if seen == targets[idx] else 0
                if streak >= IR_CONFIRM_SAMPLES:
                    print(f"[IR] {side} IR {'sees the obstacle' if targets[idx] else 'is clear'} "
                          f"({telem[key]:.1f} cm) after {time.time() - started:.1f}s")
                    event_ts.append(telem["timestamp"])
                    idx, streak = idx + 1, 0
            if time.time() - started > IR_MAX_DRIVE_S:
                print(f"[IR] No {side} IR trigger in {IR_MAX_DRIVE_S}s - stopping.")
                self.stop()
                return False
            time.sleep(0.01)
        if targets == [True, False] and len(event_ts) == 2:
            span = event_ts[1] - event_ts[0]
            self.last_obstacle_length_cm = span * cm_per_s(speed) * IR_LENGTH_CAL
            print(f"[IR] {side} IR saw the obstacle for {span:.2f}s at speed {speed} "
                  f"-> length ~ {self.last_obstacle_length_cm:.1f} cm "
                  f"(IR_LENGTH_CAL={IR_LENGTH_CAL})")
        if extra_cm > 0:
            print(f"[IR] driving {extra_cm} cm more.")
            return self.forward(extra_cm, speed=speed)   # preempts the long forward
        if extra_cm < 0:
            print(f"[IR] backing up {-extra_cm} cm to cancel the overshoot.")
            return self.reverse(-extra_cm, speed=speed)  # preempts the long forward
        return self.stop()

    def forward_until_ir(self, side, until, extra_cm, speed=None):
        """
        Continuous forward drive until the `side` IR changes state, then `extra_cm` more.
          until="clear": go while the obstacle is beside the robot; stop when it is gone
                         (the robot has passed the end of the obstacle).
          until="see"  : go until an obstacle appears beside the robot.
        If the sensor is already in the wanted state at the start, it only drives the
        extra distance. Fails (stops, returns False) if nothing triggers within
        IR_MAX_TRAVEL_CM / IR_MAX_DRIVE_S.
        """
        want_seen = (until == "see")
        seen = self.ir_sees_obstacle(side)
        if seen is None:
            print(f"[IR] No telemetry - cannot read the {side} IR, not moving.")
            return False
        print(f"[IR] {side} IR sees obstacle: {seen}; driving until it "
              f"{'sees one' if want_seen else 'clears'}.")
        if seen == want_seen:
            return self.forward(extra_cm, speed=speed) if extra_cm > 0 else True
        return self._drive_through_ir_states(side, [want_seen], extra_cm, speed)

    # Sequence-friendly names: (name, extra_cm, speed). The *_right_* steps watch the
    # RIGHT IR (obstacle on the robot's right, i.e. after turning left); the mirror
    # for a right arrow swaps them to the LEFT IR automatically.
    def fwd_until_right_clear(self, extra_cm=0, speed=None):
        return self.forward_until_ir("right", "clear", extra_cm, speed)

    def fwd_until_left_clear(self, extra_cm=0, speed=None):
        return self.forward_until_ir("left", "clear", extra_cm, speed)

    def fwd_until_right_see(self, extra_cm=0, speed=None):
        return self.forward_until_ir("right", "see", extra_cm, speed)

    def fwd_until_left_see(self, extra_cm=0, speed=None):
        return self.forward_until_ir("left", "see", extra_cm, speed)

    def forward_past_ir(self, side, extra_cm=0, speed=None):
        """
        Drives along an obstacle and past its far end in ONE continuous motion: if the
        `side` IR is not seeing the obstacle yet, it waits to see it, then waits for it to
        clear (the spike back to ~80), then `extra_cm` more. Use it when the robot starts
        BEFORE the obstacle, e.g. after a U-turn, where "until clear" would stop at once.
        """
        seen = self.ir_sees_obstacle(side)
        if seen is None:
            print(f"[IR] No telemetry - cannot read the {side} IR, not moving.")
            return False
        print(f"[IR] {side} IR sees obstacle: {seen}; driving past the obstacle.")
        targets = [False] if seen else [True, False]
        return self._drive_through_ir_states(side, targets, extra_cm, speed)

    def fwd_past_right(self, extra_cm=0, speed=None):
        return self.forward_past_ir("right", extra_cm, speed)

    def fwd_past_left(self, extra_cm=0, speed=None):
        return self.forward_past_ir("left", extra_cm, speed)

    def stop(self):
        return self.send_cmd("STOP", 0, 0, timeout=1.0)


def cm_per_s(speed):
    """Straight-line speed in cm/s for a motor speed, interpolated from CM_PER_S."""
    pts = sorted(CM_PER_S.items())
    if speed <= pts[0][0]:
        return pts[0][1]
    for (s0, v0), (s1, v1) in zip(pts, pts[1:]):
        if speed <= s1:
            return v0 + (v1 - v0) * (speed - s0) / (s1 - s0)
    return pts[-1][1]


def get_current_sensors(link, retries=10):
    """
    Polls link and returns the most up-to-date telemetry dict.
    """
    for _ in range(retries):
        link.poll()
        if link.latest_telemetry:
            age = time.time() - link.latest_telemetry["timestamp"]
            if age < 0.5:
                return link.latest_telemetry
        time.sleep(0.04)
    return link.latest_telemetry


def read_ultrasonic_cm(link, retries=20):
    """
    Polls telemetry for one valid (us_valid) ultrasonic reading, retrying briefly
    since the sensor can report no-echo on any given sample. None if it never gets
    a valid reading within `retries`.
    """
    for _ in range(retries):
        telem = get_current_sensors(link)
        if telem and telem["ultrasonic_valid"]:
            return telem["ultrasonic_cm"]
        time.sleep(0.1)
    return None


def read_ultrasonic_median(link, samples=5, settle_s=0.3, max_wait_s=3.0):
    """
    Median of `samples` FRESH valid ultrasonic readings (a new telemetry timestamp each time,
    ~100 ms apart, so ~0.5 s total) after a short settle pause. Needs at least 3 valid
    readings, otherwise falls back to a single read_ultrasonic_cm(). None if no echo at all.
    """
    time.sleep(settle_s)
    readings, last_ts = [], None
    deadline = time.time() + max_wait_s
    while len(readings) < samples and time.time() < deadline:
        link.poll()
        telem = link.latest_telemetry
        if telem and telem["timestamp"] != last_ts:
            last_ts = telem["timestamp"]
            if telem["ultrasonic_valid"]:
                readings.append(telem["ultrasonic_cm"])
        time.sleep(0.01)
    if len(readings) < 3:
        print(f"[Ultrasonic] only {len(readings)} valid sample(s) - falling back to a single read.")
        return read_ultrasonic_cm(link)
    readings.sort()
    median = readings[len(readings) // 2]
    print(f"[Ultrasonic] {len(readings)} samples {readings} -> median {median:.1f} cm")
    return median


def approach_box(link, target_dist=TARGET_DISTANCE_CM, ir_stop=True):
    """
    Drives the robot towards the box until distance is ~target_dist cm (30 cm).
    Uses Ultrasonic when us_valid == 1.
    If ultrasonic has no echo (us_valid == 0), checks IR sensors for proximity.
    ir_stop=False skips the side-IR emergency stop (used by Task 2).
    """
    print(f"\n--- [Phase 1: Approaching box to ~{target_dist:.1f} cm] ---")

    while True:
        telem = get_current_sensors(link)
        if not telem:
            print("[Approach] No telemetry received from STM32 yet. Waiting...")
            time.sleep(0.2)
            continue

        us_valid = telem["ultrasonic_valid"]
        us_dist = telem["ultrasonic_cm"]
        left_ir = telem["left_ir_cm"]
        right_ir = telem["right_ir_cm"]

        print(f"[Telemetry] US: {us_dist if us_valid else 'NO ECHO'} cm | IR_L: {left_ir:.1f} cm | IR_R: {right_ir:.1f} cm")

        # Emergency Anti-Ramming Check using IR sensors
        min_ir = min(left_ir, right_ir)
        if ir_stop and min_ir <= IR_EMERGENCY_STOP_CM:
            print(f"⚠️ [SAFETY STOP] IR sensor detected obstacle too close ({min_ir:.1f} cm <= {IR_EMERGENCY_STOP_CM} cm)!")
            link.stop()
            print("[Approach] Backing up 5 cm to maintain safe distance...")
            link.reverse(5)
            break

        # Check Ultrasonic Reading
        if not us_valid or us_dist is None:
            print("⚠️ [WARN] Ultrasonic has NO ECHO (us_valid=0)!")
            if min_ir <= 35.0:
                print(f"[Approach] IR sensors confirm box is in range ({min_ir:.1f} cm). Stopping here for capture.")
                link.stop()
                break
            else:
                print("[Approach] Creeping forward carefully 5 cm...")
                link.forward(5)
                time.sleep(0.2)
                continue

        # Ultrasonic is valid!
        error = us_dist - target_dist
        print(f"[Approach] Distance to box: {us_dist:.1f} cm (Target: {target_dist:.1f} cm, Error: {error:+.1f} cm)")

        if abs(error) <= DISTANCE_TOLERANCE_CM:
            print(f"[Approach] SUCCESS! Robot within {target_dist} ± {DISTANCE_TOLERANCE_CM} cm.")
            link.stop()
            break
        elif error > DISTANCE_TOLERANCE_CM:
            step = int(min(error * 0.7, 15))
            if step < 5:
                step = 5
            print(f"[Approach] Advancing {step} cm at tested speed {SPEED_FWD}...")
            link.forward(step)
        else:
            step = int(min(abs(error), 10))
            if step < 5:
                step = 5
            print(f"[Approach] Too close! Reversing {step} cm at tested speed {SPEED_REVS}...")
            link.reverse(step)

        time.sleep(0.2)


def move_to_distance(link, target_dist, speed=None):
    """
    One-shot positioning: reads the front ultrasonic once, then drives (reading - target_dist)
    cm forward, or reverses (target_dist - reading) cm if it is already closer than target_dist
    (e.g. reads 5, target 20 -> reverse 15). Falls back to approach_box() if there is no echo.
    """
    print(f"\n--- [Position: ultrasonic one-shot move to {target_dist:.1f} cm] ---")
    dist = read_ultrasonic_cm(link)
    if dist is None:
        print("[Position] No valid ultrasonic reading - falling back to approach_box().")
        approach_box(link, target_dist=target_dist, ir_stop=TASK2_IR_STOP)
        return True
    delta = dist - target_dist
    print(f"[Position] Ultrasonic {dist:.1f} cm, target {target_dist:.1f} cm -> move {delta:+.1f} cm")
    if abs(delta) <= DISTANCE_TOLERANCE_CM:
        print("[Position] Already within tolerance - not moving.")
        return True
    if delta > 0:
        ok = link.forward(delta, speed=speed)
    else:
        ok = link.reverse(-delta, speed=speed)
    time.sleep(0.2)
    final = read_ultrasonic_cm(link)
    print(f"[Position] Now {final:.1f} cm from the obstacle." if final is not None
          else "[Position] No ultrasonic reading after the move.")
    return ok


def check_vision_server():
    """
    Checks that a laptop vision server is reachable before starting the mission:
    image-api on :8001 (best_v11i_m.onnx) first, then the old Algo server on :8000.
    """
    for url, label in ((f"http://{LAPTOP_IP}:8001/health", "image-api (best_v11i_m.onnx)"),
                       (f"http://{LAPTOP_IP}:8000/status", "Algo YOLO (old best.pt)")):
        print(f"\n[Vision Check] Testing {label} at {url} ...")
        try:
            if requests.get(url, timeout=2.5).status_code == 200:
                print(f"✅ [Vision Check] Connection OK! Using {label}.")
                return True
        except Exception as e:
            print(f"❌ [Vision Check] {label} not reachable ({e})")
    print(f"   Laptop IP detected: {LAPTOP_IP}")
    print("   Troubleshooting:")
    print("   1. Is image-api running on the laptop (python main.py in image-api/)?")
    print("   2. Is your laptop connected to the robot Wi-Fi 'MDP-group-24'?")
    print("   3. Run with explicit IP: python3 box_inspect_task.py <your_laptop_ip>")
    return False


def capture_and_predict():
    """
    Captures photo using rpicam-still and queries the YOLO image API on laptop.
    Returns (image_id, confidence, is_target_found)
    """
    print("\n--- [Phase 2: Camera Capture & Recognition] ---")

    # 1. Capture snapshot with rpicam-still
    try:
        cmd = f"rpicam-still -t 500 -o {SNAP_PATH} --width 640 --height 480 -n --immediate"
        subprocess.run(cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print(f"[Camera] Saved photo to {SNAP_PATH}")
    except Exception as e:
        print(f"[Camera Error] Failed to capture picture: {e}")
        return "NA", 0.0, False

    # 2. Query YOLO Laptop API
    last_error = ""
    for api_url, field_name in IMAGE_API_URLS:
        try:
            with open(SNAP_PATH, "rb") as f:
                resp = requests.post(api_url, files={field_name: f}, timeout=5.0)

            if resp.status_code == 200:
                data = resp.json()

                # Format 1: Algo/main.py detections list
                if "detections" in data:
                    dets = data.get("detections", [])
                    if len(dets) > 0:
                        top = dets[0]
                        target_id = str(top.get("cls_name", top.get("cls_id", ""))).strip()
                        conf = float(top.get("conf", 0.0))
                        print(f"[Vision Result from {api_url}] Detected: '{target_id}', confidence: {conf:.2f}")
                        if target_id and target_id != "NA":
                            return target_id, conf, True
                    else:
                        print(f"[Vision from {api_url}] No valid target detected on this face (bullseye ignored).")
                        return "NA", 0.0, False

                # Format 2: Standalone image-api dict
                image_id = str(data.get("image_id", "NA")).strip()
                conf = float(data.get("conf", 0.0))
                bullseye = data.get("bullseye")
                print(f"[Vision Result from {api_url}] image_id: '{image_id}', confidence: {conf:.2f}, bullseye: {bullseye}")

                if image_id != "NA" and image_id != "None" and image_id != "":
                    return image_id, conf, True
                else:
                    return "NA", conf, False

        except Exception as e:
            last_error = str(e)
            continue

    print(f"[Vision Error] Failed to contact YOLO API ({last_error}).")
    return "NA", 0.0, False


def grab_frame_from_stream(path=SNAP_PATH, url=STREAM_URL, timeout=4.0):
    """
    Saves one complete JPEG from the Pi's camera stream (videostream/main.py) to `path`.
    The stream owns the camera while it runs, so rpicam-still cannot be used then.
    Returns True on success, False if the stream is not up / gave no frame in time.
    """
    try:
        deadline = time.time() + timeout
        with requests.get(url, stream=True, timeout=timeout) as r:
            buf = b""
            for chunk in r.iter_content(chunk_size=4096):
                buf += chunk
                start = buf.find(b"\xff\xd8")
                end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                if start != -1 and end != -1:
                    with open(path, "wb") as f:
                        f.write(buf[start:end + 2])
                    return True
                if time.time() > deadline:
                    break
    except requests.RequestException:
        pass
    return False


def capture_snapshot():
    """Takes a photo into SNAP_PATH: from the running stream if there is one, else rpicam-still."""
    if grab_frame_from_stream():
        return True
    try:
        cmd = f"rpicam-still -t 500 -o {SNAP_PATH} --width 640 --height 480 -n --immediate"
        subprocess.run(cmd, shell=True, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True
    except Exception as e:
        print(f"[Arrow] Camera capture failed: {e} (no stream on :5000 and the camera is busy)")
        return False


def read_arrow_once():
    """
    Captures one photo and asks the vision API whether it shows a LEFT or RIGHT arrow.
    Returns (direction, confidence): direction is "left", "right", or None if no arrow
    was seen (or the capture / API failed). Every detection is checked, not just the
    first, so a bullseye or other symbol in frame cannot hide the arrow.
    """
    if not capture_snapshot():
        return None, 0.0

    for api_url, field_name in IMAGE_API_URLS:
        try:
            with open(SNAP_PATH, "rb") as f:
                resp = requests.post(api_url, files={field_name: f}, timeout=5.0)
            if resp.status_code != 200:
                continue
            data = resp.json()
            # Algo/main.py returns a list; image-api returns a single dict.
            if "detections" in data:
                found = [(str(d.get("cls_name", "")), float(d.get("conf", 0.0)))
                         for d in data["detections"]]
            else:
                found = [(str(data.get("image_id", "")), float(data.get("conf", 0.0)))]
            best = (None, 0.0)
            for name, conf in found:
                low = name.strip().lower()
                if low in ("left", "right") and conf > best[1]:
                    best = (low, conf)
            return best
        except Exception:
            continue
    print("[Arrow] Could not reach the vision API.")
    return None, 0.0


def confirm_arrow():
    """
    Reads the arrow until ARROW_CONFIRMATIONS consecutive reads agree on the same
    direction at >= ARROW_MIN_CONF. Returns "left" / "right", or None if it never
    settles within ARROW_MAX_TRIES (a wrong turn is a disqualification, so the
    robot must not guess).
    """
    streak_dir, streak = None, 0
    for attempt in range(1, ARROW_MAX_TRIES + 1):
        direction, conf = read_arrow_once()
        ok = direction is not None and conf >= ARROW_MIN_CONF
        print(f"[Arrow] try {attempt}/{ARROW_MAX_TRIES}: {direction or 'no arrow'} "
              f"conf {conf:.2f} -> {'accepted' if ok else 'ignored'}")
        if not ok:
            streak_dir, streak = None, 0
            continue
        if direction == streak_dir:
            streak += 1
        else:
            streak_dir, streak = direction, 1
        if streak >= ARROW_CONFIRMATIONS:
            return streak_dir
    return None


def mirror_sequence(sequence):
    """Left-arrow sequence -> right-arrow sequence (swap left/right turns)."""
    return [(MIRROR.get(method, method), value, speed) for method, value, speed in sequence]


def run_task2_obstacle(link, number, read_only=False):
    """
    One Task 2 obstacle: read the arrow, drive to its stop distance, then run the
    matching sequence (LEFT as written, RIGHT mirrored). `link` is None in read-only
    mode, where only the arrow is read and the robot is never touched.
    """
    if number == 1:
        stop_dist, left_sequence = TASK2_STOP_DIST_CM, TASK2_LEFT_SEQUENCE
    else:
        stop_dist, left_sequence = TASK2_OBS2_STOP_DIST_CM, TASK2_OBS2_LEFT_SEQUENCE

    print(f"\n---------- Task 2 - obstacle {number}: stop at {stop_dist} cm ----------")
    # Scan first from where the robot stands, then drive in to stop_dist. If the long-range
    # scan can't confirm an arrow, scan again after the approach (close-up) before giving up.
    direction = confirm_arrow()
    if link:
        if not move_to_distance(link, stop_dist):
            print(f"[Task2] Obstacle {number}: positioning move failed - stopping.")
            link.stop()
            return False
        if direction is None:
            print(f"[Task2] Obstacle {number}: no arrow from afar - rescanning up close.")
            direction = confirm_arrow()
    if direction is None:
        print(f"\n[Task2] Obstacle {number}: arrow not confirmed - robot stays where it is.")
        return False

    if direction == "left":
        sequence = left_sequence
    elif number == 1 and TASK2_RIGHT_SEQUENCE is not None:
        sequence = TASK2_RIGHT_SEQUENCE
    else:
        sequence = mirror_sequence(left_sequence)
    print(f"\n[Task2] Obstacle {number}: arrow = {direction.upper()} -> sequence:")
    for method, value, speed in sequence:
        print(f"          {method}({value}, speed={speed})")
    if read_only:
        print("[Task2] --read-only: not moving.")
        return sequence

    for method, value, speed in sequence:
        print(f"[Task2] {method}({value}, speed={speed})")
        if not getattr(link, method)(value, speed=speed):
            print(f"[Task2] {method} failed - stopping.")
            link.stop()
            return False
    print(f"\n[Task2] Obstacle {number} sequence complete.")
    if number == 2 and link.last_obstacle_length_cm is not None:
        print(f"[Task2] Obstacle 2 length (IR span x speed): ~{link.last_obstacle_length_cm:.1f} cm")
    return sequence


def run_task2(obstacles=(1,), read_only=False):
    """
    Runs the given Task 2 obstacles in order over one serial connection, stopping at
    the first that fails. read_only=True never opens the serial port.
    """
    print("==========================================================")
    print(f"   Task 2 - obstacle(s) {', '.join(map(str, obstacles))} (arrow -> go around it)")
    print(f"   need {ARROW_CONFIRMATIONS}x arrow >= {ARROW_MIN_CONF} | Laptop YOLO: {LAPTOP_IP}")
    print("==========================================================")

    if not check_vision_server():
        print("[Task2] Vision server unreachable - not moving. Start the Algo server and retry.")
        return False

    link = None
    if not read_only:
        link = ProtocolSTMLink()
        try:
            link.connect()
        except Exception as e:
            print(f"[FATAL] Could not connect to STM32 serial: {e}")
            return False

    x2 = None
    try:
        sequence = None
        for i, number in enumerate(obstacles):
            if i:
                time.sleep(0.3)
            sequence = run_task2_obstacle(link, number, read_only)
            if not sequence:
                return False
            if link and number == 1 and 2 in obstacles:
                x2 = read_ultrasonic_median(link)
                print(f"[Task2] x2 (post-obstacle-1 -> obstacle 2 face): "
                      f"{x2:.1f} cm" if x2 is not None else
                      "[Task2] x2: no valid ultrasonic reading after obstacle 1.")

        if 2 in obstacles and 1 in obstacles:
            if x2 is None and link:
                print("[Task2] No x2 reading - cannot plan the return leg. Stopping.")
                link.stop()
                return False
            mirrored = sequence != TASK2_OBS2_LEFT_SEQUENCE   # RIGHT arrow was read
            l2 = link.last_obstacle_length_cm if link else None
            if l2 is None:
                l2 = OBS2_LENGTH_FALLBACK_CM
                print(f"[Task2] WARNING: obstacle 2 length not measured - using fallback "
                      f"{l2:.0f} cm. Use a fwd_past_* step in the obstacle 2 sequence to measure it.")
            else:
                print(f"[Task2] Obstacle 2 length for the return leg: {l2:.1f} cm")
            left_return = task2_return_left_sequence(x2 if x2 is not None else 0.0, l2)
            return_sequence = mirror_sequence(left_return) if mirrored else left_return
            print(f"\n[Task2] Return leg (x2={x2}):")
            for method, value, speed in return_sequence:
                print(f"          {method}({value}, speed={speed})")
            if link and not run_return_sequence(link, return_sequence):
                return False
        return True
    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted with Ctrl+C. Stopping robot...")
        if link:
            link.stop()
        return False
    finally:
        if link:
            link.disconnect()


def run_return_sequence(link, sequence):
    """Runs the return-leg sequence, resolving the approach_front step."""
    for method, value, speed in sequence:
        print(f"[Task2] {method}({value}, speed={speed})")
        if method == "approach_front":
            ok = move_to_distance(link, value)
        else:
            ok = getattr(link, method)(value, speed=speed)
        if not ok:
            print(f"[Task2] {method} failed - stopping.")
            link.stop()
            return False
        time.sleep(0.2)
    return True


def circumnavigate_to_next_face(link):
    """
    Navigates around the corner to face the adjacent face of the box.
    Uses a single-command 180° turn to eliminate cumulative turning errors.
    """
    print("\n--- [Phase 3: Circumnavigating to next face of the box] ---")

    # Step 1: Back up 15 cm for clearance
    print("[Move] Step 1: Reversing 15 cm for clearance...")
    link.reverse(23)
    time.sleep(0.3)

    # Step 2: Turn right 90 deg away from box face
    print("[Move] Step 2: Right turn 90 deg...")
    link.turn_right(90)
    time.sleep(0.3)

    # Step 3: Advance 20 cm along flank
    print("[Move] Step 3: Driving forward along flank (20 cm)...")
    link.forward(20)
    time.sleep(0.3)

    # Step 4: Turn left 180 deg in ONE single command!
    print("[Move] Step 4: Single-command Left Turn 180 deg (sweeping around corner)...")
    link.turn_left(180)
    time.sleep(0.3)

    print("[Move] Completed corner orbit. Ready to approach next face to ~30 cm.")


def apply_speed_flags(argv):
    """
    Optional overrides of the speed constants for this run (each 0-101; the STM
    multiplies by 71 and rejects anything above 7199):
        python3 box_inspect_task.py --fwd 50 --rev 28 --turn 21
    Returns False (after printing why) if a flag is malformed.
    """
    global SPEED_FWD, SPEED_REVS, SPEED_TURN
    names = {"--fwd": "SPEED_FWD", "--rev": "SPEED_REVS", "--turn": "SPEED_TURN"}
    i = 1
    while i < len(argv):
        if argv[i] in names:
            val = argv[i + 1] if i + 1 < len(argv) else ""
            if not val.isdigit() or not 0 <= int(val) <= 101:
                print(f"[ARGS] {argv[i]} needs a whole number 0-101, got '{val}'")
                return False
            globals()[names[argv[i]]] = int(val)
            i += 2
        else:
            i += 1
    return True


def main():
    if not apply_speed_flags(sys.argv):
        return

    if "--task2" in sys.argv:
        which = "1"
        if "--obstacle" in sys.argv:
            i = sys.argv.index("--obstacle")
            which = sys.argv[i + 1] if i + 1 < len(sys.argv) else ""
        plans = {"1": (1,), "2": (2,), "both": (1, 2)}
        if which not in plans:
            print(f"[ARGS] --obstacle must be 1, 2 or both, got '{which}'")
            return
        run_task2(plans[which], read_only="--read-only" in sys.argv)
        return

    print("==========================================================")
    print("    Single Box Face Inspection & Circumnavigation Task    ")
    print(f"     Target Dist: {TARGET_DISTANCE_CM} cm | Laptop YOLO: {LAPTOP_IP}")
    print("==========================================================")

    # Check Vision Server connection before starting
    vision_ok = check_vision_server()
    if not vision_ok:
        ans = input("\n[PROMPT] Vision server was not reachable. Continue anyway? (y/n): ")
        if ans.strip().lower() != 'y':
            print("Aborted. Please start the vision server and ensure Wi-Fi is connected.")
            return

    link = ProtocolSTMLink()
    try:
        link.connect()
    except Exception as e:
        print(f"[FATAL] Could not connect to STM32 serial: {e}")
        return

    face_index = 1

    try:
        while face_index <= MAX_FACES:
            print(f"\n==================== INSPECTING FACE {face_index}/{MAX_FACES} ====================")

            # 1. Approach to ~30 cm
            approach_box(link, target_dist=TARGET_DISTANCE_CM)

            # 2. Capture and inspect face
            image_id, conf, is_target = capture_and_predict()

            if is_target:
                print(f"\n🎉 [SUCCESS] TARGET IDENTIFIED ON FACE {face_index}!")
                print(f"            Target ID : {image_id}")
                print(f"            Confidence: {conf:.2f}")

                link.stop()
                break

            # 3. If not target and more faces remain, orbit to next face
            if face_index < MAX_FACES:
                print(f"[Info] Face {face_index} is not the target symbol. Moving to next face...")
                circumnavigate_to_next_face(link)
                face_index += 1
            else:
                print("\n[Finished] All 4 faces inspected. No symbol identified.")
                break

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted with Ctrl+C. Stopping robot...")
        link.stop()
    finally:
        link.disconnect()
        print("[Done] Script finished and serial disconnected.")


if __name__ == "__main__":
    main()
