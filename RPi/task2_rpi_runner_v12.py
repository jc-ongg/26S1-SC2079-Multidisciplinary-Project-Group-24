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

Task 2 high-level route (v9 calibration):
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
DEFAULT_LAPTOP_IP = "192.168.50.218"

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

# From physical testing, the car overshoots the ultrasonic trigger by about 5 cm.
O1_CAPTURE_US_CM = 65.0
O2_CAPTURE_US_CM = 35.0
ULTRASONIC_STOP_LEAD_CM = 5.0
ULTRASONIC_CONFIRM_SAMPLES = 2

# Side-facing IR thresholds. The IRs are now intended to be mounted roughly
# 90 degrees left/right, near the rear half of the car.
IR_OBJECT_CM = 30.0
IR_CLEAR_CM = 35.0
IR_CONFIRM_SAMPLES = 2
IR_DANGER_CM = 7.0

# O1-specific thresholds from the 2026-10-01 physical test.
# On the O1 inside side, the IR was stable at ~38-42 cm while seeing O1,
# then jumped to ~86-90 cm immediately after the rear edge. The old global
# 30/35 cm thresholds therefore missed a real OBJECT->CLEAR transition.
O1_IR_OBJECT_CM = 50.0
O1_IR_CLEAR_CM = 70.0


# ---------------------------------------------------------------------------
# MEASURED ROBOT GEOMETRY / TUNABLE ROUTE
# ---------------------------------------------------------------------------

CAR_LENGTH_CM = 23.0
CAR_WIDTH_CM = 18.8

# Measured from the side-IR position to the rear-most point of the car.
IR_TO_REAR_CM = 18.0
IR_EDGE_MARGIN_CM = 5.0
IR_EDGE_EXTRA_CM = round(IR_TO_REAR_CM + IR_EDGE_MARGIN_CM)  # 23 cm

# 90-degree calibration supplied from floor testing at speed input 100.
# These are swept-footprint measurements, not centre-point displacement.
TURN90_SPEED = 100
TURN90_MAX_X_CM = 13.0
TURN90_TOTAL_Y_CM = 42.7

# Keep 45-degree turns at the previously working turn speed.
TURN45_SPEED = TURN_SPEED

# Obstacle 1: leave O1 in a side lane instead of re-centring in the minimum
# 60 cm O1->O2 gap. Scan farther away, make the already-proven 45/28/45
# lane change, then use the inside side-IR to stop as soon as O1's rear edge
# passes the IR. Because IR_TO_REAR=18 cm on a 23 cm car, the IR is about
# 5 cm behind the front; at CLEAR the front is therefore only ~5 cm past O1.
# O1 is fixed 10 x 10 cm. Physical v8 testing showed that after the proven
# 45deg -> 28 cm -> 45deg lane change, another 10 cm forward was exactly
# where the side IR cleared O1's rear edge. For Fastest Car we now hard-code
# 12 cm (10 cm measured + 2 cm margin) and do NOT use IR to finish O1.
OBS1_TURN_BACKOFF_CM = 0
OBS1_OUT_DIAGONAL_CM = 28
OBS1_SIDE_FORWARD_CM = 12
OBS1_ANGLE_DEG = 45

# Legacy O1 IR helper constants are kept only so the old calibration helper
# functions remain importable; v9 O1 does NOT use IR for route decisions.
OBS1_IR_CHUNK_CM = 2
OBS1_OBJECT_SEARCH_MAX_CM = 35
OBS1_CLEAR_SEARCH_MAX_CM = 20

# O2 arrow is centred on the obstacle, but we only need its LEFT/RIGHT label.
# From the O1 side lane, try a coarse camera search at 0°, 15°, then 25° inward.
# Stop immediately on the FIRST reliable detection; do not waste time sweeping
# the whole visibility window. Then undo the accumulated angle so the robot is
# straight in the original side lane again.
O2_SWEEP_ANGLES_DEG = (0, 15, 25)
O2_AIM_TURN_SPEED = TURN45_SPEED

# After restoring the straight side-lane heading, reverse a small calibrated
# amount to create extra room before the requested 90° turn around O2.
O2_TURN_BACKOFF_CM = 5

# O2 is variable-width. Do NOT use the old fixed 42/84 cm corridor values.
# We find its side edge with the inside IR, count actual commanded travel in
# small chunks, and mirror that same translation behind O2.
O2_SIDE_CHUNK_CM = 5
O2_SIDE_SEARCH_MAX_CM = 100
O2_PASS_SEARCH_MAX_CM = 80

# Parking: acquire BOTH carpark side walls with the side IRs, then use the
# front ultrasonic to stop safely before the back wall.
CARPARK_SEARCH_MAX_CM = 170
CARPARK_PULL_IN_MAX_CM = 40
CARPARK_WALL_STOP_CM = 18.0

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

    def try_detect_arrow(self, image_path: Path, attempts: int = 1) -> Tuple[Optional[str], float]:
        """Best-effort arrow detection used during the O2 camera sweep.

        Unlike detect_arrow(), this does not raise when no arrow is visible.
        It returns (None, 0.0) so the RPi can turn another small step and retry.
        """
        best_arrow = None
        best_conf = 0.0
        for attempt in range(1, attempts + 1):
            print(f"[VISION] sweep inference attempt {attempt}/{attempts}")
            for kind, url in self.backends:
                try:
                    arrow, conf = self._query(kind, url, image_path)
                    if arrow in {"left", "right"} and conf >= best_conf:
                        best_arrow = arrow
                        best_conf = conf
                except Exception as exc:
                    print(f"[VISION] {kind} unavailable/no arrow: {exc}")
            if best_arrow is not None:
                print(f"[VISION] sweep sees {best_arrow.upper()} (conf={best_conf:.3f})")
                return best_arrow, best_conf
            if attempt < attempts:
                time.sleep(0.15)
        print("[VISION] sweep: no arrow")
        return None, 0.0

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


def turn(link: STM32Link, side: str, angle_deg: int, speed: Optional[int] = None) -> None:
    cmd = "TURNL" if side == "left" else "TURNR"
    if speed is None:
        speed = TURN90_SPEED if angle_deg >= 90 else TURN45_SPEED
    print(f"[MOVE] turn {side} {angle_deg}° @ speed {speed}")
    link.move(cmd, speed, angle_deg, timeout=TURN_TIMEOUT_S)


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
    target_cm: float = O1_CAPTURE_US_CM,
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


def drive_until_ir_passed_chunked(
    link: STM32Link,
    sensor_side: str,
    label: str,
    chunk_cm: int = OBS1_IR_CHUNK_CM,
    object_search_max_cm: int = OBS1_OBJECT_SEARCH_MAX_CM,
    clear_search_max_cm: int = OBS1_CLEAR_SEARCH_MAX_CM,
) -> int:
    """Safely detect CLEAR -> OBJECT -> CLEAR using short encoder chunks.

    Returns total commanded forward distance from the start of the search.
    If O1 is not acquired within object_search_max_cm, stop and fail rather
    than driving blindly toward O2.
    """
    total = 0

    def stable_state():
        vals = _read_ir_samples(link, sensor_side, samples=IR_CONFIRM_SAMPLES)
        print(f"[IR-{sensor_side[0].upper()}] samples={','.join(f'{v:.1f}' for v in vals)} cm")
        # O1 uses thresholds calibrated from the real side-lane test.
        # Keep the global 30/35 thresholds unchanged for later O2 calibration.
        is_object = all(v <= O1_IR_OBJECT_CM for v in vals)
        is_clear = all(v >= O1_IR_CLEAR_CM for v in vals)
        return is_object, is_clear

    is_object, _ = stable_state()
    seen_object = is_object
    if seen_object:
        print(f"[IR] {label}: object already acquired at search start")
    else:
        print(f"[IR] {label}: searching for OBJECT in {chunk_cm} cm chunks")
        while total < object_search_max_cm:
            forward(link, chunk_cm, speed=SENSOR_DRIVE_SPEED)
            total += chunk_cm
            is_object, _ = stable_state()
            if is_object:
                seen_object = True
                print(f"[IR] {label}: OBJECT acquired after {total} cm")
                break

    if not seen_object:
        link.stop()
        raise Task2Error(
            f"{label}: IR never acquired O1 within {object_search_max_cm} cm; "
            "stopped instead of continuing toward O2"
        )

    after_object = 0
    print(f"[IR] {label}: searching for rear-edge CLEAR")
    while after_object < clear_search_max_cm:
        forward(link, chunk_cm, speed=SENSOR_DRIVE_SPEED)
        total += chunk_cm
        after_object += chunk_cm
        _, is_clear = stable_state()
        if is_clear:
            print(
                f"[IR] {label}: CLEAR after {after_object} cm past acquisition; "
                f"total side-lane travel={total} cm"
            )
            return total

    link.stop()
    raise Task2Error(
        f"{label}: O1 did not clear within {clear_search_max_cm} cm after acquisition"
    )


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


def _read_ir_samples(link: STM32Link, sensor_side: str, samples: int = 2):
    """Collect fresh IR samples after a short movement."""
    seq, _ = link.sensor_snapshot()
    values = []
    for _ in range(samples):
        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        values.append(_ir_value(sensor, sensor_side))
    return values


def move_side_until_ir_edge_counted(
    link: STM32Link,
    sensor_side: str,
    label: str,
    chunk_cm: int = O2_SIDE_CHUNK_CM,
    max_travel_cm: int = O2_SIDE_SEARCH_MAX_CM,
) -> int:
    """
    Move across O2's FRONT in short encoder-controlled chunks.

    The selected inside IR must first see O2 (OBJECT) and then become CLEAR.
    Returns the sum of commanded chunks up to the detected lateral edge.
    """
    travelled = 0
    seen_object = False
    object_hits = 0
    clear_hits = 0

    _, current = link.sensor_snapshot()
    if current is not None:
        dist = _ir_value(current, sensor_side)
        if dist <= IR_OBJECT_CM:
            seen_object = True
            object_hits = IR_CONFIRM_SAMPLES

    print(f"[IR-{sensor_side.upper()}] locating lateral edge of {label}")

    while travelled < max_travel_cm:
        step = min(chunk_cm, max_travel_cm - travelled)
        forward(link, step, speed=SENSOR_DRIVE_SPEED)
        travelled += step

        values = _read_ir_samples(link, sensor_side, samples=IR_CONFIRM_SAMPLES)
        print(
            f"[IR-{sensor_side[0].upper()}] {values[-1]:5.1f} cm | "
            f"{label} | lateral={travelled} cm"
        )

        for dist in values:
            if not seen_object:
                if dist <= IR_OBJECT_CM:
                    object_hits += 1
                else:
                    object_hits = 0
                if object_hits >= IR_CONFIRM_SAMPLES:
                    seen_object = True
                    clear_hits = 0
                    print(f"[IR] {label}: object acquired")
            else:
                if dist >= IR_CLEAR_CM:
                    clear_hits += 1
                else:
                    clear_hits = 0
                if clear_hits >= IR_CONFIRM_SAMPLES:
                    print(f"[IR] {label}: lateral edge cleared at {travelled} cm")
                    return travelled

    if not seen_object:
        raise Task2Error(f"{label}: inside IR never acquired obstacle")
    raise Task2Error(f"{label}: lateral edge not cleared within {max_travel_cm} cm")


def drive_until_both_ir_detected(
    link: STM32Link,
    label: str,
    max_travel_cm: int = CARPARK_SEARCH_MAX_CM,
) -> None:
    """Drive until both side-facing IRs stably see the carpark side walls."""
    seq, _ = link.sensor_snapshot()
    left_hits = 0
    right_hits = 0

    print(f"[PARK] searching for {label} with both side IRs")
    cmd_id = link.send_motor_async("FWD", SENSOR_DRIVE_SPEED, max_travel_cm)
    deadline = time.monotonic() + COMMAND_TIMEOUT_S

    while time.monotonic() < deadline:
        result = link.result_now(cmd_id)
        if result is not None:
            if result.status == "DONE":
                raise Task2Error(
                    f"Reached {max_travel_cm} cm before detecting both carpark walls"
                )
            if result.status == "ERROR":
                raise Task2Error(f"Carpark search movement failed: {result.reason}")

        seq, sensor = link.wait_next_sensor(seq, timeout=1.0)
        left = sensor.left_ir_cm
        right = sensor.right_ir_cm
        print(f"[PARK IR] L={left:5.1f} R={right:5.1f}", end="\r", flush=True)

        if left <= IR_DANGER_CM or right <= IR_DANGER_CM:
            print("\n[PARK] side IR danger threshold -> STOP")
            _preempt_with_stop(link, cmd_id)
            raise Task2Error(
                f"Parking alignment unsafe: LIR={left:.1f} RIR={right:.1f}"
            )

        left_hits = left_hits + 1 if left <= IR_OBJECT_CM else 0
        right_hits = right_hits + 1 if right <= IR_OBJECT_CM else 0

        if left_hits >= IR_CONFIRM_SAMPLES and right_hits >= IR_CONFIRM_SAMPLES:
            print("\n[PARK] both carpark side walls acquired -> STOP")
            _preempt_with_stop(link, cmd_id)
            return

    raise Task2Error("Timed out searching for carpark side walls")


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

    def _capture_arrow_optional(self, name: str) -> Tuple[Optional[str], float]:
        path = self.capture_dir / f"{name}.jpg"
        capture_image(path)
        return self.vision.try_detect_arrow(path, attempts=1)

    def obstacle1_bypass(self, arrow: str) -> None:
        """Hard-coded compact O1 side exit; no IR decision logic.

        v8 physical calibration:
          45 out -> 28 cm -> 45 straight -> 10 cm
        put the side IR exactly at O1's rear-edge CLEAR transition.
        O1 is fixed at 10 x 10 cm, so v10 uses 12 cm to add ~2 cm margin.
        The robot deliberately stays in the side lane; it does NOT re-centre
        in the minimum 60 cm O1->O2 gap.
        """
        print(f"\n=== OBSTACLE 1 HARDCODED SIDE EXIT: {arrow.upper()} ===")
        outward = arrow
        inward = opposite(arrow)

        turn(self.link, outward, OBS1_ANGLE_DEG, speed=TURN45_SPEED)
        forward(self.link, OBS1_OUT_DIAGONAL_CM)
        turn(self.link, inward, OBS1_ANGLE_DEG, speed=TURN45_SPEED)
        forward(self.link, OBS1_SIDE_FORWARD_CM, speed=SENSOR_DRIVE_SPEED)
        self.link.stop()

        _, sensor = self.link.sensor_snapshot()
        if sensor is not None:
            print(
                f"[O1 EXIT] hardcoded {OBS1_SIDE_FORWARD_CM} cm side-lane pass; "
                f"LIR={sensor.left_ir_cm:.1f} RIR={sensor.right_ir_cm:.1f} "
                f"US={sensor.ultrasonic_cm:.1f} valid={sensor.ultrasonic_valid}"
            )

    def aim_and_capture_obstacle2(self, o1_side: str) -> Tuple[str, int]:
        """Coarse camera-only O2 search from the O1 side lane.

        We no longer drive toward O2 just because the camera found its centred
        arrow.  The sweep is ONLY for classifying O2 LEFT/RIGHT.

        Search headings relative to the straight side-lane pose:
            0° -> 15° inward -> 25° inward

        Stop on the FIRST reliable LEFT/RIGHT detection, remember the accumulated
        search angle, then undo exactly that angle to restore the original course
        heading.  No forward approach is performed here.

        Returns (arrow2, detected_angle_deg).
        """
        inward = opposite(o1_side)
        outward = o1_side

        print(
            f"\n=== O2 COARSE CAMERA SEARCH FROM {o1_side.upper()} SIDE LANE "
            f"(inward={inward.upper()}) ==="
        )

        current_angle = 0
        detected_arrow = None
        detected_conf = 0.0
        detected_angle = 0

        for idx, target_angle in enumerate(O2_SWEEP_ANGLES_DEG):
            delta = target_angle - current_angle
            if delta > 0:
                print(
                    f"[O2 SEARCH] rotate inward {delta}° "
                    f"({current_angle}° -> {target_angle}°)"
                )
                turn(self.link, inward, delta, speed=O2_AIM_TURN_SPEED)
                current_angle = target_angle

            arrow, conf = self._capture_arrow_optional(
                f"task2_o2_search_{idx:02d}_{current_angle:02d}deg"
            )

            if arrow in {"left", "right"}:
                detected_arrow = arrow
                detected_conf = conf
                detected_angle = current_angle
                print(
                    f"[O2 SEARCH] angle={current_angle:2d}° -> "
                    f"{arrow.upper()} conf={conf:.3f}; FIRST detection accepted"
                )
                break

            print(f"[O2 SEARCH] angle={current_angle:2d}° -> no arrow")

        if detected_arrow is None:
            if current_angle > 0:
                print(f"[O2 SEARCH] no detection -> restore heading by {current_angle}°")
                turn(self.link, outward, current_angle, speed=O2_AIM_TURN_SPEED)
            self.link.stop()
            raise Task2Error(
                f"O2 arrow not detected at coarse search angles "
                f"{list(O2_SWEEP_ANGLES_DEG)}"
            )

        # Return to the exact straight side-lane heading before O2 manoeuvring.
        if detected_angle > 0:
            print(
                f"[O2 SEARCH] stored {detected_arrow.upper()} at {detected_angle}°; "
                f"restore straight heading by turning {outward.upper()} {detected_angle}°"
            )
            turn(self.link, outward, detected_angle, speed=O2_AIM_TURN_SPEED)

        self.link.stop()
        _, sensor = self.link.sensor_snapshot()
        if sensor is not None:
            print(
                f"[O2 SEARCH] straight again; stored={detected_arrow.upper()} "
                f"conf={detected_conf:.3f}; "
                f"LIR={sensor.left_ir_cm:.1f} RIR={sensor.right_ir_cm:.1f} "
                f"US={sensor.ultrasonic_cm:.1f} valid={sensor.ultrasonic_valid}"
            )

        return detected_arrow, detected_angle

    def obstacle2_loop_and_return(self, arrow2: str) -> int:
        """Dynamic variable-width O2 bypass using side-facing IR edge detection."""
        print(f"\n=== OBSTACLE 2 DYNAMIC BYPASS: {arrow2.upper()} ===")

        outward = arrow2
        inward = opposite(arrow2)
        inside_ir = "left" if arrow2 == "right" else "right"

        # Face sideways across O2's front.
        turn(self.link, outward, 90, speed=TURN90_SPEED)

        # Move laterally until the inside IR passes O2's requested edge.
        side_to_edge_cm = move_side_until_ir_edge_counted(
            self.link,
            inside_ir,
            label="obstacle 2 front edge",
        )

        # The side IR is 18 cm ahead of the rear in this direction of travel.
        forward(self.link, IR_EDGE_EXTRA_CM, speed=SENSOR_DRIVE_SPEED)
        side_total_cm = side_to_edge_cm + IR_EDGE_EXTRA_CM
        print(f"[O2] total outward translation command = {side_total_cm} cm")

        # Face north again and pass the 10 cm obstacle depth.
        turn(self.link, inward, 90, speed=TURN90_SPEED)
        drive_until_ir_passed(
            self.link,
            inside_ir,
            after_clear_cm=IR_EDGE_EXTRA_CM,
            label="obstacle 2 longitudinal pass",
            max_travel_cm=O2_PASS_SEARCH_MAX_CM,
        )

        # Behind O2: turn toward centre, mirror the same translation, then turn
        # again to face home. With symmetric 90-degree arcs this returns close
        # to the original centreline without using the old fixed 84 cm value.
        turn(self.link, inward, 90, speed=TURN90_SPEED)
        forward(self.link, side_total_cm, speed=SENSOR_DRIVE_SPEED)
        turn(self.link, inward, 90, speed=TURN90_SPEED)

        print("[O2] returned approximately to centreline; heading home")
        return side_total_cm

    def return_past_o1(self, arrow1: str) -> None:
        print("\n=== RETURN TO OBSTACLE 1 ===")
        approach_until_ultrasonic(self.link, O1_CAPTURE_US_CM)

        # Facing home, the same physical side is the opposite robot-relative side.
        return_side = opposite(arrow1)
        print(
            f"[RETURN] outbound O1={arrow1.upper()} -> "
            f"return bypass={return_side.upper()}"
        )
        self.obstacle1_bypass(return_side)

    def park(self) -> None:
        print("\n=== RETURN TO CARPARK ===")

        # Drive until both side-facing IRs see the carpark walls.
        drive_until_both_ir_detected(
            self.link,
            label="carpark side walls",
            max_travel_cm=CARPARK_SEARCH_MAX_CM,
        )

        print("\n=== FINAL PARKING PULL-IN ===")
        drive_with_ultrasonic_guard(
            self.link,
            max_forward_cm=CARPARK_PULL_IN_MAX_CM,
            wall_stop_cm=CARPARK_WALL_STOP_CM,
        )
        self.link.stop()

        sensor = self.link.wait_valid_ultrasonic(timeout=2.0)
        print(
            f"[PARK] final US={sensor.ultrasonic_cm:.1f} cm "
            f"LIR={sensor.left_ir_cm:.1f} RIR={sensor.right_ir_cm:.1f}"
        )

    def _approach_o2_safely(self) -> float:
        sensor = self.link.wait_valid_ultrasonic(timeout=2.0)
        print(f"[O2 CHECK] current front US={sensor.ultrasonic_cm:.1f} cm")

        trigger = O2_CAPTURE_US_CM + ULTRASONIC_STOP_LEAD_CM
        if sensor.ultrasonic_cm <= trigger:
            if sensor.ultrasonic_cm < 15.0:
                raise Task2Error(
                    f"Too close to O2 to start 90-degree manoeuvre: "
                    f"{sensor.ultrasonic_cm:.1f} cm"
                )
            print("[O2 CHECK] already within capture zone; no extra forward command")
            return sensor.ultrasonic_cm

        return approach_until_ultrasonic(self.link, O2_CAPTURE_US_CM)

    def run(self, stage: str = "full") -> None:
        print("\n================ TASK 2: FASTEST CAR v11 ================")

        sensor = self.link.wait_valid_ultrasonic(timeout=3.0)
        print(
            f"[START] LIR={sensor.left_ir_cm:.1f} RIR={sensor.right_ir_cm:.1f} "
            f"US={sensor.ultrasonic_cm:.1f} heading={sensor.heading_deg:.1f}"
        )

        if stage == "check":
            arrow = self._capture_arrow(0)
            print(f"[CHECK] vision sees {arrow.upper()}")
            return

        if stage == "obs2-only":
            print("\n=== O2-ONLY TEST: place robot facing O2 on centreline ===")
            self._approach_o2_safely()
            arrow2 = self._capture_arrow(2)
            print(f"[TASK2] obstacle 2 arrow = {arrow2.upper()}")
            self.obstacle2_loop_and_return(arrow2)
            self.link.stop()
            print("[TASK2] Stage obs2-only complete.")
            return

        if stage == "park":
            print("\n=== PARK-ONLY TEST ===")
            self.park()
            print("[TASK2] Stage park complete.")
            return

        # ---------- O1 ----------
        print("\n=== APPROACH OBSTACLE 1 ===")
        approach_until_ultrasonic(self.link, O1_CAPTURE_US_CM)
        arrow1 = self._capture_arrow(1)
        print(f"[TASK2] obstacle 1 arrow = {arrow1.upper()}")
        self.obstacle1_bypass(arrow1)

        if stage == "obs1":
            self.link.stop()
            print("[TASK2] Stage obs1 complete; stopped after O1 bypass.")
            return

        # ---------- O2 COARSE CAMERA DETECTION ----------
        # Stay in the O1 side lane. Try 0°, 15°, then 25° inward and accept
        # the FIRST reliable LEFT/RIGHT detection. Restore straight heading;
        # do not approach O2 diagonally.
        arrow2, aim_angle = self.aim_and_capture_obstacle2(arrow1)

        if stage == "detect2":
            self.link.stop()
            print(
                f"[TASK2] Stage detect2 complete; O2={arrow2.upper()}, "
                f"detected_at={aim_angle}°. Course heading restored. "
                "No reverse or O2 bypass executed."
            )
            return

        if stage == "o2turn":
            # User-calibrated next step:
            #   1) camera search only to learn LEFT/RIGHT
            #   2) restore the straight O1-side-lane heading
            #   3) reverse 5 cm to create turning room
            #   4) TURN 90° toward O2 arrow at speed 100
            #   5) STOP immediately and inspect the inside side-IR reading
            print(
                f"\n=== O2 FIRST 90° TEST: {arrow2.upper()} @ speed {TURN90_SPEED} ==="
            )
            print(f"[O2 TURN] reverse {O2_TURN_BACKOFF_CM} cm before 90° turn")
            reverse(self.link, O2_TURN_BACKOFF_CM, speed=SENSOR_DRIVE_SPEED)

            _, before = self.link.sensor_snapshot()
            if before is not None:
                print(
                    f"[O2 TURN] pre-turn LIR={before.left_ir_cm:.1f} "
                    f"RIR={before.right_ir_cm:.1f} "
                    f"US={before.ultrasonic_cm:.1f} valid={before.ultrasonic_valid}; "
                    f"calibrated 90° @100 max-X~{TURN90_MAX_X_CM:.1f} cm, "
                    f"total-Y~{TURN90_TOTAL_Y_CM:.1f} cm"
                )

            turn(self.link, arrow2, 90, speed=TURN90_SPEED)
            self.link.stop()
            time.sleep(0.2)
            _, after = self.link.sensor_snapshot()
            if after is not None:
                inside_ir = "right" if arrow2 == "left" else "left"
                inside_val = _ir_value(after, inside_ir)
                print(
                    f"[O2 TURN] post-turn LIR={after.left_ir_cm:.1f} "
                    f"RIR={after.right_ir_cm:.1f} "
                    f"US={after.ultrasonic_cm:.1f} valid={after.ultrasonic_valid} "
                    f"heading={after.heading_deg:.1f}"
                )
                print(
                    f"[O2 TURN] inside IR={inside_ir.upper()} reads "
                    f"{inside_val:.1f} cm"
                )
            print(
                "[TASK2] Stage o2turn complete. STOPPED immediately after "
                "reverse + first 90° turn. No side translation executed."
            )
            return

        # Keep the complete O2 bypass gated until the first 90-degree turn is
        # physically validated at the minimum O1->O2 spacing.
        raise Task2Error(
            "v12 safety gate: validate --stage o2turn first. "
            "The remaining O2 edge-follow / return manoeuvre is intentionally gated."
        )


# ---------------------------------------------------------------------------
# CLI / MAIN
# ---------------------------------------------------------------------------
def print_plan() -> None:
    print("Task 2 v12 plan:")
    print(
        f"  robot: {CAR_LENGTH_CM:.1f} x {CAR_WIDTH_CM:.1f} cm, "
        f"IR->rear={IR_TO_REAR_CM:.1f} cm"
    )
    print(
        f"  90° calibration: speed={TURN90_SPEED}, "
        f"max-X~{TURN90_MAX_X_CM:.1f} cm, total-Y~{TURN90_TOTAL_Y_CM:.1f} cm"
    )
    print(
        f"  O1: target {O1_CAPTURE_US_CM:.0f} cm -> 45°, {OBS1_OUT_DIAGONAL_CM} cm -> "
        f"45° straight -> hardcoded {OBS1_SIDE_FORWARD_CM} cm -> STOP in side lane"
    )
    print(
        f"  O2 search: coarse angles {list(O2_SWEEP_ANGLES_DEG)} -> FIRST LEFT/RIGHT detection -> undo angle"
    )
    print(
        f"  o2turn test: straighten -> reverse {O2_TURN_BACKOFF_CM} cm -> TURN 90° toward stored arrow @ {TURN90_SPEED} -> STOP"
    )
    print(
        f"  parking: both side IRs acquire carpark -> US wall guard "
        f"{CARPARK_WALL_STOP_CM:.0f} cm"
    )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RPi controller for MDP Task 2 Fastest Car")
    parser.add_argument("--port", default=None, help="STM32 serial port; auto-detect by default")
    parser.add_argument("--laptop-ip", default=DEFAULT_LAPTOP_IP, help="Laptop running vision service")
    parser.add_argument(
        "--stage",
        choices=["check", "obs1", "detect2", "o2turn", "obs2-only", "obs2", "return", "park", "full"],
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
