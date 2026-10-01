#!/usr/bin/env python3
"""
Task 2 — Fastest Car (RPi high-level controller)
Group 24

IMPORTANT ARCHITECTURE
----------------------
This script owns the Task 2 state machine on the Raspberry Pi.
It deliberately DOES NOT use the legacy STM32 TASK2 command/state machine.
The STM32 is treated only as a low-level controller:
    FWD, REVS, TURNL, TURNR, STOP + sensor telemetry.

The serial protocol implemented here matches the latest Group 24 STM32 main.c:
    RPi -> STM32: :<id>/MOTOR/<cmd>/<speed>/<value>;
    STM32 -> RPi: !READY;
                   !SENSOR/<LIR>/<RIR>/<US>/<VALID>/<HEADING>/<GYRO>;
                   !<id>/DONE;
                   !<id>/ABORTED;
                   !<id>/ERROR/<reason>;

Task 2 high-level route (v1):
1. Approach obstacle 1 using front ultrasonic.
2. Capture image and classify LEFT/RIGHT on the laptop.
3. Slalom around obstacle 1 and return to centreline.
4. Approach obstacle 2 using front ultrasonic.
5. Capture image and classify LEFT/RIGHT.
6. Move to the requested side corridor, pass obstacle 2 using the inner IR,
   cross behind it, return south on the opposite corridor.
7. Use the inner IR to pass obstacle 2, obstacle 1, then detect the carpark side.
8. Shift inward and enter the carpark with an ultrasonic wall guard.

This is a first hardware-testable RPi implementation. The values under
"TUNABLE GEOMETRY" are intentionally grouped at the top so physical testing can
refine them without rewriting the state machine.
"""

from __future__ import annotations

import argparse
import glob
import math
import os
import signal
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

try:
    import requests
except ImportError:
    requests = None

try:
    import serial
except ImportError:
    serial = None


# ---------------------------------------------------------------------------
# CONNECTION / PROTOCOL
# ---------------------------------------------------------------------------
BAUD_RATE = 115200
DEFAULT_LAPTOP_IP = "192.168.50.147"

# Latest STM32-tested command values from the STM32 README.
STRAIGHT_SPEED = 28
TURN_SPEED = 21

# A slightly slower value for sensor-triggered runs, where we need time to stop.
SENSOR_DRIVE_SPEED = 22

COMMAND_TIMEOUT_S = 20.0
TURN_TIMEOUT_S = 12.0


# ---------------------------------------------------------------------------
# TASK 2 SENSOR TARGETS
# ---------------------------------------------------------------------------
OBSTACLE_CAPTURE_US_CM = 25.0
# We issue STOP a little before 25 cm to allow command/vehicle stopping latency.
ULTRASONIC_STOP_LEAD_CM = 8.0
ULTRASONIC_CONFIRM_SAMPLES = 2

IR_OBJECT_CM = 30.0
# Hysteresis: after seeing an object, require a slightly larger distance before
# declaring that its far edge has passed.
IR_CLEAR_CM = 35.0
IR_CONFIRM_SAMPLES = 2


# ---------------------------------------------------------------------------
# TUNABLE GEOMETRY — start here after the first physical test
# ---------------------------------------------------------------------------
# Obstacle 1 slalom, directly from the supplied Task 2 run guide.
OBS1_TURN_BACKOFF_CM = 5
OBS1_DIAGONAL_CM = 28
OBS1_STRAIGHT_PAST_CM = 25
OBS1_ANGLE_DEG = 45

# Obstacle 2 guide target: corridor centre at approximately +/-42 cm and 84 cm
# across the back. We first back away from the 25 cm capture point so the
# diagonal side-shift does not cut the front corner of a wide obstacle.
OBS2_CORRIDOR_OFFSET_CM = 42
OBS2_BACKOFF_CM = 35
OBS2_SHIFT_ANGLE_DEG = 45
OBS2_DIAGONAL_CM = round(OBS2_CORRIDOR_OFFSET_CM / math.sin(math.radians(45)))  # ~59
OBS2_CROSS_BEHIND_CM = 84

# Once the IR sees the far edge of obstacle 2, add enough room for the following
# 90-degree turn. This is intentionally conservative for v1.
OBS2_BEHIND_CLEARANCE_CM = 30
RETURN_EDGE_CLEARANCE_CM = 10

# Carpark values from the supplied Task 2 run guide.
CARPARK_INWARD_SHIFT_CM = 22
CARPARK_SHIFT_DIAGONAL_CM = round(CARPARK_INWARD_SHIFT_CM / math.sin(math.radians(45)))  # ~31
CARPARK_PULL_IN_CM = 35
# Front ultrasonic points at the back wall once the robot faces into the lot.
CARPARK_WALL_STOP_CM = 10.0

# Sensor-driven commands are intentionally given a large encoder target; the
# RPi pre-empts them with STOP when the relevant sensor condition is reached.
SENSOR_SEARCH_MAX_CM = 200


# ---------------------------------------------------------------------------
# DATA TYPES
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class SensorFrame:
    left_ir_cm: float
    right_ir_cm: float
    ultrasonic_cm: float
    ultrasonic_valid: bool
    heading_deg: float
    gyro_z_dps: float
    received_at: float


@dataclass(frozen=True)
class CommandResult:
    cmd_id: int
    status: str  # DONE / ABORTED / ERROR
    reason: Optional[str] = None


class Task2Error(RuntimeError):
    pass


# ---------------------------------------------------------------------------
# STM32 LINK
# ---------------------------------------------------------------------------
class STM32Link:
    """Semicolon-framed, telemetry-aware STM32 serial link."""

    def __init__(self, port: Optional[str] = None, baud: int = BAUD_RATE):
        if serial is None:
            raise Task2Error("pyserial is not installed. Run: pip3 install pyserial")

        self.port = port or self._auto_port()
        self.baud = baud
        self.ser = None

        self._stop_reader = threading.Event()
        self._reader_thread: Optional[threading.Thread] = None
        self._rx_buffer = bytearray()
        self._write_lock = threading.Lock()
        self._cv = threading.Condition()

        self._ready = False
        self._sensor_seq = 0
        self._last_sensor: Optional[SensorFrame] = None
        self._results: Dict[int, CommandResult] = {}
        self._next_cmd_id = 1

    @staticmethod
    def _auto_port() -> str:
        candidates = sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*"))
        if not candidates:
            raise Task2Error("No STM32 serial device found under /dev/ttyACM* or /dev/ttyUSB*")
        return candidates[0]

    def connect(self, ready_timeout: float = 12.0) -> None:
        print(f"[STM] Opening {self.port} @ {self.baud}")
        self.ser = serial.Serial(
            self.port,
            self.baud,
            timeout=0.05,
            write_timeout=1.0,
        )
        self._stop_reader.clear()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # If the board booted before we opened the port, !READY may already be
        # gone. The latest STM32 only starts SENSOR telemetry after !READY, so a
        # valid SENSOR frame also proves the board is ready.
        deadline = time.monotonic() + ready_timeout
        with self._cv:
            while not self._ready:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Task2Error(
                        "STM32 did not become ready. Keep the robot still during gyro calibration "
                        "and check /dev/ttyACM0."
                    )
                self._cv.wait(timeout=min(remaining, 0.5))
        print("[STM] Ready")

    def disconnect(self) -> None:
        self._stop_reader.set()
        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None

    def _reader_loop(self) -> None:
        while not self._stop_reader.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
            except Exception as exc:
                if not self._stop_reader.is_set():
                    print(f"[STM] Serial read error: {exc}", file=sys.stderr)
                return

            if not chunk:
                continue

            self._rx_buffer.extend(chunk)
            while b";" in self._rx_buffer:
                raw, _, rest = self._rx_buffer.partition(b";")
                self._rx_buffer = bytearray(rest)
                frame = raw.decode("utf-8", errors="ignore").strip()
                if frame:
                    self._handle_frame(frame + ";")

    def _handle_frame(self, frame: str) -> None:
        if frame == "!READY;":
            with self._cv:
                self._ready = True
                self._cv.notify_all()
            return

        if frame.startswith("!SENSOR/"):
            body = frame[:-1]
            parts = body.split("/")
            if len(parts) != 7:
                return
            try:
                sensor = SensorFrame(
                    left_ir_cm=float(parts[1]),
                    right_ir_cm=float(parts[2]),
                    ultrasonic_cm=float(parts[3]),
                    ultrasonic_valid=(int(parts[4]) == 1),
                    heading_deg=float(parts[5]),
                    gyro_z_dps=float(parts[6]),
                    received_at=time.monotonic(),
                )
            except ValueError:
                return

            with self._cv:
                self._last_sensor = sensor
                self._sensor_seq += 1
                # Telemetry starts only after READY in the latest firmware.
                self._ready = True
                self._cv.notify_all()
            return

        if not frame.startswith("!"):
            return

        body = frame[1:-1]
        parts = body.split("/")
        if len(parts) < 2:
            return
        try:
            cmd_id = int(parts[0])
        except ValueError:
            return

        status = parts[1]
        if status not in {"DONE", "ABORTED", "ERROR"}:
            return
        reason = parts[2] if status == "ERROR" and len(parts) >= 3 else None
        result = CommandResult(cmd_id=cmd_id, status=status, reason=reason)
        with self._cv:
            self._results[cmd_id] = result
            self._cv.notify_all()

    def _new_cmd_id(self) -> int:
        with self._cv:
            cmd_id = self._next_cmd_id
            self._next_cmd_id += 1
            return cmd_id

    def send_motor_async(self, command: str, speed: int, value: int) -> int:
        if self.ser is None:
            raise Task2Error("STM32 serial link is not connected")
        cmd_id = self._new_cmd_id()
        # ':' is required by HAL_UART_RxCpltCallback to reset/start the command
        # buffer. ';' is the command terminator.
        packet = f":{cmd_id}/MOTOR/{command}/{int(speed)}/{int(value)};"
        with self._write_lock:
            self.ser.write(packet.encode("ascii"))
            self.ser.flush()
        print(f"[STM ->] {packet}")
        return cmd_id

    def result_now(self, cmd_id: int) -> Optional[CommandResult]:
        with self._cv:
            return self._results.get(cmd_id)

    def wait_result(self, cmd_id: int, timeout: float = COMMAND_TIMEOUT_S) -> CommandResult:
        deadline = time.monotonic() + timeout
        with self._cv:
            while cmd_id not in self._results:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Task2Error(f"Timed out waiting for command {cmd_id}")
                self._cv.wait(timeout=min(remaining, 0.25))
            result = self._results[cmd_id]

        suffix = f"/{result.reason}" if result.reason else ""
        print(f"[STM <-] !{result.cmd_id}/{result.status}{suffix};")
        return result

    def move(self, command: str, speed: int, value: int, timeout: Optional[float] = None) -> None:
        cmd_id = self.send_motor_async(command, speed, value)
        result = self.wait_result(
            cmd_id,
            timeout=timeout if timeout is not None else (TURN_TIMEOUT_S if command.startswith("TURN") else COMMAND_TIMEOUT_S),
        )
        if result.status != "DONE":
            raise Task2Error(
                f"Movement {command} failed: {result.status}"
                + (f"/{result.reason}" if result.reason else "")
            )

    def stop(self, timeout: float = 5.0) -> None:
        cmd_id = self.send_motor_async("STOP", 0, 0)
        result = self.wait_result(cmd_id, timeout=timeout)
        if result.status != "DONE":
            raise Task2Error(f"STOP did not complete: {result}")

    def sensor_snapshot(self) -> Tuple[int, Optional[SensorFrame]]:
        with self._cv:
            return self._sensor_seq, self._last_sensor

    def wait_next_sensor(self, after_seq: int, timeout: float = 1.0) -> Tuple[int, SensorFrame]:
        deadline = time.monotonic() + timeout
        with self._cv:
            while self._sensor_seq <= after_seq or self._last_sensor is None:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Task2Error("Timed out waiting for STM32 sensor telemetry")
                self._cv.wait(timeout=min(remaining, 0.25))
            return self._sensor_seq, self._last_sensor

    def wait_valid_ultrasonic(self, timeout: float = 3.0) -> SensorFrame:
        seq, sensor = self.sensor_snapshot()
        if sensor and sensor.ultrasonic_valid:
            return sensor
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            seq, sensor = self.wait_next_sensor(seq, timeout=min(1.0, deadline - time.monotonic()))
            if sensor.ultrasonic_valid:
                return sensor
        raise Task2Error("No valid ultrasonic reading received")


# ---------------------------------------------------------------------------
# LAPTOP VISION
# ---------------------------------------------------------------------------
class VisionClient:
    """Supports both Group-24 vision layouts found in the supplied repo/docs."""

    def __init__(self, laptop_ip: str = DEFAULT_LAPTOP_IP, timeout: float = 7.0):
        if requests is None:
            raise Task2Error("requests is not installed. Run: pip3 install requests")
        self.timeout = timeout
        # Current task instructions point to Algo/main.py:8000 /infer. The repo
        # also contains the older/better image-api:8001 /predict. Try 8000 first
        # and fall back automatically so Task 2 does not depend on one branch's
        # service layout.
        self.backends = [
            ("algo", f"http://{laptop_ip}:8000/infer"),
            ("image-api", f"http://{laptop_ip}:8001/predict"),
        ]

    def detect_arrow(self, image_path: Path, attempts: int = 3) -> str:
        last_error = None
        for attempt in range(1, attempts + 1):
            print(f"[VISION] Arrow inference attempt {attempt}/{attempts}")
            for kind, url in self.backends:
                try:
                    arrow, conf = self._query(kind, url, image_path)
                    if arrow in {"left", "right"}:
                        print(f"[VISION] {kind}: {arrow.upper()} (conf={conf:.3f})")
                        return arrow
                except Exception as exc:
                    last_error = exc
                    print(f"[VISION] {kind} unavailable/no arrow: {exc}")
            if attempt < attempts:
                time.sleep(0.25)

        raise Task2Error(f"Could not classify LEFT/RIGHT arrow. Last error: {last_error}")

    def _query(self, kind: str, url: str, image_path: Path) -> Tuple[Optional[str], float]:
        with image_path.open("rb") as f:
            if kind == "algo":
                response = requests.post(
                    url,
                    files={"file": (image_path.name, f, "image/jpeg")},
                    timeout=self.timeout,
                )
            else:
                response = requests.post(
                    url,
                    files={"image": (image_path.name, f, "image/jpeg")},
                    timeout=self.timeout,
                )

        if response.status_code >= 400:
            raise Task2Error(f"HTTP {response.status_code}: {response.text[:180]}")
        data = response.json()

        if kind == "algo":
            best_arrow = None
            best_conf = -1.0
            for det in data.get("detections", []):
                label = str(det.get("cls_name", "")).strip().lower()
                if label in {"left", "right"}:
                    conf = float(det.get("conf", 0.0))
                    if conf > best_conf:
                        best_arrow = label
                        best_conf = conf
            return best_arrow, max(best_conf, 0.0)

        image_id = str(data.get("image_id", "")).strip().lower()
        if image_id in {"left", "right"}:
            return image_id, float(data.get("conf", 0.0))
        return None, 0.0


def _terminate_videostream_processes() -> None:
    """SIGTERM only actual videostream processes; never pkill -f the SSH shell."""
    try:
        ps = subprocess.run(
            ["ps", "-eo", "pid=,args="],
            text=True,
            capture_output=True,
            check=True,
        )
    except Exception:
        return

    me = os.getpid()
    for line in ps.stdout.splitlines():
        if "videostream" not in line.lower():
            continue
        try:
            pid_str, args = line.strip().split(None, 1)
            pid = int(pid_str)
        except Exception:
            continue
        if pid == me:
            continue
        print(f"[CAMERA] Stopping camera holder PID {pid}: {args}")
        try:
            os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError):
            pass
    time.sleep(1.0)


def capture_image(output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cmd = [
        "rpicam-still",
        "-t", "500",
        "-o", str(output_path),
        "--width", "640",
        "--height", "480",
        "-n",
        "--immediate",
    ]

    for attempt in (1, 2):
        print(f"[CAMERA] Capturing {output_path} (attempt {attempt}/2)")
        proc = subprocess.run(cmd, text=True, capture_output=True, timeout=12)
        if proc.returncode == 0 and output_path.exists() and output_path.stat().st_size > 0:
            return

        err = (proc.stderr or proc.stdout or "camera capture failed").strip()
        if attempt == 1 and ("busy" in err.lower() or "resource" in err.lower()):
            _terminate_videostream_processes()
            continue
        raise Task2Error(f"rpicam-still failed: {err[-500:]}")

    raise Task2Error("Camera capture failed")


# ---------------------------------------------------------------------------
# TASK 2 MOTION HELPERS
# ---------------------------------------------------------------------------
def opposite(side: str) -> str:
    if side == "left":
        return "right"
    if side == "right":
        return "left"
    raise ValueError(side)


def turn(link: STM32Link, side: str, angle_deg: int) -> None:
    cmd = "TURNL" if side == "left" else "TURNR"
    print(f"[MOVE] turn {side} {angle_deg}°")
    link.move(cmd, TURN_SPEED, angle_deg, timeout=TURN_TIMEOUT_S)


def forward(link: STM32Link, cm: int, speed: int = STRAIGHT_SPEED) -> None:
    if cm <= 0:
        return
    print(f"[MOVE] forward {cm} cm")
    link.move("FWD", speed, cm)


def reverse(link: STM32Link, cm: int, speed: int = STRAIGHT_SPEED) -> None:
    if cm <= 0:
        return
    print(f"[MOVE] reverse {cm} cm")
    link.move("REVS", speed, cm)


def _preempt_with_stop(link: STM32Link, active_cmd_id: int) -> None:
    stop_id = link.send_motor_async("STOP", 0, 0)
    stop_result = link.wait_result(stop_id, timeout=5.0)
    if stop_result.status != "DONE":
        raise Task2Error(f"Sensor-triggered STOP failed: {stop_result}")
    # The active command should become ABORTED. We don't block for it because
    # the STOP DONE is the safety-critical terminal response; its ABORTED frame
    # may already have arrived and is retained by STM32Link.
    active_result = link.result_now(active_cmd_id)
    if active_result:
        print(f"[MOVE] pre-empted command {active_cmd_id}: {active_result.status}")


def approach_until_ultrasonic(
    link: STM32Link,
    target_cm: float = OBSTACLE_CAPTURE_US_CM,
    max_travel_cm: int = SENSOR_SEARCH_MAX_CM,
) -> float:
    """Drive forward until the front ultrasonic reaches the capture distance."""
    initial = link.wait_valid_ultrasonic()
    print(f"[US] initial {initial.ultrasonic_cm:.1f} cm, target {target_cm:.1f} cm")

    trigger_cm = target_cm + ULTRASONIC_STOP_LEAD_CM
    if initial.ultrasonic_cm <= trigger_cm:
        print("[US] already at/inside stopping threshold")
        return initial.ultrasonic_cm

    cmd_id = link.send_motor_async("FWD", SENSOR_DRIVE_SPEED, max_travel_cm)
    seq, _ = link.sensor_snapshot()
    hits = 0
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        result = link.result_now(cmd_id)
        if result is not None:
            if result.status == "DONE":
                raise Task2Error(
                    f"FWD reached its {max_travel_cm} cm safety limit before ultrasonic reached {target_cm} cm"
                )
            if result.status == "ERROR":
                raise Task2Error(f"Ultrasonic approach movement failed: {result.reason}")

        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        if not sensor.ultrasonic_valid:
            continue

        print(f"[US] {sensor.ultrasonic_cm:5.1f} cm", end="\r", flush=True)
        if sensor.ultrasonic_cm <= trigger_cm:
            hits += 1
        else:
            hits = 0

        if hits >= ULTRASONIC_CONFIRM_SAMPLES:
            print()
            print(f"[US] threshold reached at {sensor.ultrasonic_cm:.1f} cm -> STOP")
            _preempt_with_stop(link, cmd_id)
            time.sleep(0.15)
            final = link.wait_valid_ultrasonic(timeout=2.0)
            print(f"[US] stopped at ~{final.ultrasonic_cm:.1f} cm")
            return final.ultrasonic_cm

    raise Task2Error("Ultrasonic approach timed out")


def _ir_value(sensor: SensorFrame, side: str) -> float:
    return sensor.left_ir_cm if side == "left" else sensor.right_ir_cm


def drive_until_ir_passed(
    link: STM32Link,
    sensor_side: str,
    after_clear_cm: int,
    label: str,
    max_travel_cm: int = SENSOR_SEARCH_MAX_CM,
) -> None:
    """Drive until selected IR sees OBJECT and then CLEAR, then add clearance."""
    seq, current = link.sensor_snapshot()
    if current is None:
        seq, current = link.wait_next_sensor(seq, timeout=2.0)

    seen_object = _ir_value(current, sensor_side) <= IR_OBJECT_CM
    object_hits = IR_CONFIRM_SAMPLES if seen_object else 0
    clear_hits = 0

    print(
        f"[IR-{sensor_side.upper()}] pass {label}; start={_ir_value(current, sensor_side):.1f} cm, "
        f"seen_object={seen_object}"
    )

    cmd_id = link.send_motor_async("FWD", SENSOR_DRIVE_SPEED, max_travel_cm)
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        result = link.result_now(cmd_id)
        if result is not None:
            if result.status == "DONE":
                raise Task2Error(f"IR search limit reached before passing {label}")
            if result.status == "ERROR":
                raise Task2Error(f"IR pass movement failed: {result.reason}")

        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        dist = _ir_value(sensor, sensor_side)
        print(f"[IR-{sensor_side[0].upper()}] {dist:5.1f} cm | {label}", end="\r", flush=True)

        if not seen_object:
            if dist <= IR_OBJECT_CM:
                object_hits += 1
            else:
                object_hits = 0
            if object_hits >= IR_CONFIRM_SAMPLES:
                seen_object = True
                clear_hits = 0
                print(f"\n[IR] {label}: object acquired")
        else:
            if dist >= IR_CLEAR_CM:
                clear_hits += 1
            else:
                clear_hits = 0
            if clear_hits >= IR_CONFIRM_SAMPLES:
                print(f"\n[IR] {label}: far edge cleared -> STOP")
                _preempt_with_stop(link, cmd_id)
                if after_clear_cm > 0:
                    forward(link, after_clear_cm, speed=SENSOR_DRIVE_SPEED)
                return

    raise Task2Error(f"Timed out waiting to pass {label}")


def drive_until_ir_detected(
    link: STM32Link,
    sensor_side: str,
    label: str,
    max_travel_cm: int = SENSOR_SEARCH_MAX_CM,
) -> None:
    """Drive until selected IR sees an object, then STOP."""
    seq, current = link.sensor_snapshot()
    if current is None:
        seq, current = link.wait_next_sensor(seq, timeout=2.0)

    hits = 0
    cmd_id = link.send_motor_async("FWD", SENSOR_DRIVE_SPEED, max_travel_cm)
    deadline = time.monotonic() + COMMAND_TIMEOUT_S
    print(f"[IR-{sensor_side.upper()}] searching for {label}")

    while time.monotonic() < deadline:
        result = link.result_now(cmd_id)
        if result is not None:
            if result.status == "DONE":
                raise Task2Error(f"IR search limit reached before detecting {label}")
            if result.status == "ERROR":
                raise Task2Error(f"IR detect movement failed: {result.reason}")

        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        dist = _ir_value(sensor, sensor_side)
        print(f"[IR-{sensor_side[0].upper()}] {dist:5.1f} cm | search {label}", end="\r", flush=True)
        if dist <= IR_OBJECT_CM:
            hits += 1
        else:
            hits = 0
        if hits >= IR_CONFIRM_SAMPLES:
            print(f"\n[IR] {label} detected -> STOP")
            _preempt_with_stop(link, cmd_id)
            return

    raise Task2Error(f"Timed out searching for {label}")


def drive_with_ultrasonic_guard(
    link: STM32Link,
    max_forward_cm: int,
    wall_stop_cm: float,
) -> None:
    """Move into carpark, but pre-empt immediately if rear wall gets close."""
    cmd_id = link.send_motor_async("FWD", SENSOR_DRIVE_SPEED, max_forward_cm)
    seq, _ = link.sensor_snapshot()
    hits = 0
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        result = link.result_now(cmd_id)
        if result is not None:
            if result.status == "DONE":
                print(f"[PARK] completed planned {max_forward_cm} cm pull-in")
                return
            if result.status == "ERROR":
                raise Task2Error(f"Parking movement failed: {result.reason}")

        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        if not sensor.ultrasonic_valid:
            continue
        print(f"[PARK US] {sensor.ultrasonic_cm:5.1f} cm", end="\r", flush=True)
        if sensor.ultrasonic_cm <= wall_stop_cm + 2.0:
            hits += 1
        else:
            hits = 0
        if hits >= 1:
            print(f"\n[PARK] rear wall guard at {sensor.ultrasonic_cm:.1f} cm -> STOP")
            _preempt_with_stop(link, cmd_id)
            return

    raise Task2Error("Parking pull-in timed out")


# ---------------------------------------------------------------------------
# TASK 2 STATE MACHINE — RPi OWNED
# ---------------------------------------------------------------------------
class Task2Runner:
    def __init__(self, link: STM32Link, vision: VisionClient, capture_dir: Path):
        self.link = link
        self.vision = vision
        self.capture_dir = capture_dir
        self.capture_dir.mkdir(parents=True, exist_ok=True)

    def _capture_arrow(self, obstacle_no: int) -> str:
        path = self.capture_dir / f"task2_obstacle_{obstacle_no}.jpg"
        capture_image(path)
        return self.vision.detect_arrow(path)

    def obstacle1_bypass(self, arrow: str) -> None:
        print(f"\n=== OBSTACLE 1 BYPASS: {arrow.upper()} ===")
        outward = arrow
        inward = opposite(arrow)

        # Give the front-right/front-left corner more clearance before the
        # first steering manoeuvre. This was added after the first hardware
        # test, where the car stopped too close to obstacle 1.
        print(f"[MOVE] reverse {OBS1_TURN_BACKOFF_CM} cm before first turn")
        reverse(self.link, OBS1_TURN_BACKOFF_CM, speed=SENSOR_DRIVE_SPEED)

        # 45 out, 28 diagonal, 45 back to north, 25 past box. Then turn
        # inward immediately; the following 28 cm diagonal is what physically
        # returns the car to the centreline.
        turn(self.link, outward, OBS1_ANGLE_DEG)
        forward(self.link, OBS1_DIAGONAL_CM)
        turn(self.link, inward, OBS1_ANGLE_DEG)
        forward(self.link, OBS1_STRAIGHT_PAST_CM)
        turn(self.link, inward, OBS1_ANGLE_DEG)
        forward(self.link, OBS1_DIAGONAL_CM)
        turn(self.link, outward, OBS1_ANGLE_DEG)

    def shift_to_obstacle2_corridor(self, arrow: str) -> str:
        """Return outgoing corridor name: 'left' or 'right'."""
        print(f"\n=== SHIFT TO OBSTACLE 2 {arrow.upper()} CORRIDOR ===")
        # At the image-capture point the front is only ~25 cm from obstacle 2.
        # Back off first, then make a 45-degree translation to +/-42 cm.
        reverse(self.link, OBS2_BACKOFF_CM, speed=SENSOR_DRIVE_SPEED)
        turn(self.link, arrow, OBS2_SHIFT_ANGLE_DEG)
        forward(self.link, OBS2_DIAGONAL_CM, speed=SENSOR_DRIVE_SPEED)
        turn(self.link, opposite(arrow), OBS2_SHIFT_ANGLE_DEG)
        return arrow

    @staticmethod
    def inner_sensor(corridor: str, travel_heading: str) -> str:
        """Which robot-side IR points toward the centreline obstacle."""
        if travel_heading == "north":
            return "right" if corridor == "left" else "left"
        if travel_heading == "south":
            return "left" if corridor == "left" else "right"
        raise ValueError(travel_heading)

    def obstacle2_loop_and_return(self, arrow2: str) -> None:
        outgoing = self.shift_to_obstacle2_corridor(arrow2)
        outgoing_ir = self.inner_sensor(outgoing, "north")

        # Pass the variable-width obstacle using the IR edge instead of assuming
        # its north/south size.
        drive_until_ir_passed(
            self.link,
            outgoing_ir,
            after_clear_cm=OBS2_BEHIND_CLEARANCE_CM,
            label="obstacle 2 northbound",
        )

        # Loop behind the obstacle: turn toward the centre, traverse 84 cm from
        # one +/-42 cm corridor to the other, then turn again to face south.
        across_turn = opposite(outgoing)
        turn(self.link, across_turn, 90)
        forward(self.link, OBS2_CROSS_BEHIND_CM)
        turn(self.link, across_turn, 90)

        returning = opposite(outgoing)
        return_ir = self.inner_sensor(returning, "south")
        print(f"\n=== RETURN SOUTH ON {returning.upper()} CORRIDOR ===")

        # First object southbound is obstacle 2 itself.
        drive_until_ir_passed(
            self.link,
            return_ir,
            after_clear_cm=RETURN_EDGE_CLEARANCE_CM,
            label="obstacle 2 southbound",
        )

        # The next object is obstacle 1. This naturally handles the 60-150 cm
        # variable spacing because we wait for the IR transition rather than
        # hard-coding the gap.
        drive_until_ir_passed(
            self.link,
            return_ir,
            after_clear_cm=RETURN_EDGE_CLEARANCE_CM,
            label="obstacle 1 southbound",
        )

        # Continue until the inward-looking IR sees the carpark side structure.
        drive_until_ir_detected(
            self.link,
            return_ir,
            label="carpark side",
        )

        print("\n=== CARPARK ALIGNMENT ===")
        # Facing south: from the right/east corridor, robot-right points inward;
        # from the left/west corridor, robot-left points inward. Therefore the
        # inward turn has the same name as the returning corridor.
        inward_turn = returning
        turn(self.link, inward_turn, 45)
        forward(self.link, CARPARK_SHIFT_DIAGONAL_CM, speed=SENSOR_DRIVE_SPEED)
        turn(self.link, opposite(inward_turn), 45)

        # Pull into the 50 cm deep carpark. The fixed 35 cm plan is retained,
        # with front ultrasonic acting as a hard wall guard.
        drive_with_ultrasonic_guard(
            self.link,
            max_forward_cm=CARPARK_PULL_IN_CM,
            wall_stop_cm=CARPARK_WALL_STOP_CM,
        )
        self.link.stop()

    def run(self, stage: str = "full") -> None:
        print("\n================ TASK 2: FASTEST CAR ================")

        sensor = self.link.wait_valid_ultrasonic(timeout=3.0)
        print(
            f"[START] LIR={sensor.left_ir_cm:.1f} RIR={sensor.right_ir_cm:.1f} "
            f"US={sensor.ultrasonic_cm:.1f} heading={sensor.heading_deg:.1f}"
        )

        if stage == "check":
            arrow = self._capture_arrow(0)
            print(f"[CHECK] vision sees {arrow.upper()}")
            return

        print("\n=== APPROACH OBSTACLE 1 ===")
        approach_until_ultrasonic(self.link, OBSTACLE_CAPTURE_US_CM)
        arrow1 = self._capture_arrow(1)
        print(f"[TASK2] obstacle 1 arrow = {arrow1.upper()}")
        self.obstacle1_bypass(arrow1)

        if stage == "obs1":
            self.link.stop()
            print("[TASK2] Stage obs1 complete; stopped after obstacle 1 bypass.")
            return

        print("\n=== APPROACH OBSTACLE 2 ===")
        approach_until_ultrasonic(self.link, OBSTACLE_CAPTURE_US_CM)
        arrow2 = self._capture_arrow(2)
        print(f"[TASK2] obstacle 2 arrow = {arrow2.upper()}")

        if stage == "detect2":
            self.link.stop()
            print("[TASK2] Stage detect2 complete; stopped at obstacle 2 capture point.")
            return

        self.obstacle2_loop_and_return(arrow2)
        print("\n================ TASK 2 COMPLETE ================")


# ---------------------------------------------------------------------------
# CLI / MAIN
# ---------------------------------------------------------------------------
def print_plan() -> None:
    print("Task 2 v1 plan:")
    print(f"  obstacle capture target: {OBSTACLE_CAPTURE_US_CM:.0f} cm")
    print(
        f"  obs1: reverse {OBS1_TURN_BACKOFF_CM} cm -> 45°, {OBS1_DIAGONAL_CM} cm, "
        f"45°, {OBS1_STRAIGHT_PAST_CM} cm, 45°, {OBS1_DIAGONAL_CM} cm, 45°"
    )
    print(
        f"  obs2: backoff {OBS2_BACKOFF_CM} cm -> shift ~{OBS2_CORRIDOR_OFFSET_CM} cm -> "
        f"IR pass -> {OBS2_CROSS_BEHIND_CM} cm behind-cross"
    )
    print(
        f"  return: IR-pass obs2 -> IR-pass obs1 -> detect carpark -> "
        f"shift {CARPARK_INWARD_SHIFT_CM} cm -> pull in {CARPARK_PULL_IN_CM} cm with US guard"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RPi controller for MDP Task 2 Fastest Car")
    parser.add_argument("--port", default=None, help="STM32 serial port; auto-detect by default")
    parser.add_argument("--laptop-ip", default=DEFAULT_LAPTOP_IP, help="Laptop running vision service")
    parser.add_argument(
        "--stage",
        choices=["check", "obs1", "detect2", "full"],
        default="full",
        help="Run a partial hardware stage for calibration/testing",
    )
    parser.add_argument("--dry-run", action="store_true", help="Print the route/constants without moving")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print_plan()
    if args.dry_run:
        return 0

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    capture_dir = Path.home() / "SC2079-MDP" / "task2_captures" / timestamp

    link = STM32Link(port=args.port)
    vision = VisionClient(laptop_ip=args.laptop_ip)
    runner = Task2Runner(link, vision, capture_dir)

    try:
        link.connect()
        runner.run(stage=args.stage)
        return 0
    except KeyboardInterrupt:
        print("\n[EMERGENCY] Ctrl+C -> STOP")
        try:
            link.stop(timeout=3.0)
        except Exception as exc:
            print(f"[EMERGENCY] STOP acknowledgement failed: {exc}", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"\n[TASK2 ERROR] {exc}", file=sys.stderr)
        try:
            link.stop(timeout=3.0)
        except Exception:
            pass
        return 1
    finally:
        link.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
