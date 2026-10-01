import re
import threading
import time
from typing import Optional

import serial

from communication.link import Link
from settings import SERIAL_PORT, BAUD_RATE

# The flashed firmware (MDP_test.hex) speaks:
#     send   <id>/<COMPONENT>/<COMMAND>/<p1>/<p2>;
#     reply  !<id>/DONE;   or   !<id>/ERROR/<REASON>;
# The trailing ';' is the frame terminator. A newline-terminated command is
# silently dropped -- the parser is sscanf("%d/%40[^/]/%40[^/]/%d/%d") and it
# never sees a complete frame without the ';'.
TERMINATOR = b";"

# Speed passed as p1 for every motion command. The firmware range-checks this
# and answers !<id>/ERROR/INVALID_SPEED; if it is out of range (9999 is
# rejected, 50 is accepted).
DEFAULT_SPEED = 50

# How long to wait for a !<id>/DONE; frame. This must exceed the longest single
# movement, because the firmware only answers once the move has finished --
# measured at 3.1s for FWD/50/10, so a 2s timeout silently returns None
# mid-move and the caller mistakes a working command for a dead link.
READ_TIMEOUT = 30

# Legacy RPi command prefix -> firmware motor command.
#   FW/BW  straight moves, distance carried in the numeric suffix
#   FL/FR  forward 90-degree turns
#   BL/BR  reverse turns
#   TL/TR  steer
#
# FL/FR go through the parameterised TURNL/TURNR for the same reason as
# ALGO_TURN below -- TURN90L/TURN90R do not move the reported heading.
COMMAND_MAP = {
    "FW": "FWD",
    "FS": "FWD",
    "BW": "REVS",
    "BS": "REVS",
    "FL": "TURNL",
    "FR": "TURNR",
    "BL": "REVL",
    "BR": "REVR",
    "TL": "TURNL",
    "TR": "TURNR",
}
_LEGACY_TURNS = {"FL", "FR", "BL", "BR", "TL", "TR"}

_LEGACY_RE = re.compile(r"^([A-Z]{2})(-{2}|\d+)$")

# The Algo server's /path endpoint returns stm_commands in a different shape:
#     ['center,0,forward,120', 'right,90,forward,0', 'SNAP1', 'FIN']
# i.e. <steering>,<angle>,<direction>,<distance>. Straight moves carry the
# distance in the 4th field; turns carry the angle in the 2nd and 0 distance.
_ALGO_RE = re.compile(
    r"^(center|left|right)\s*,\s*(\d+)\s*,\s*(forward|reverse)\s*,\s*(\d+)$",
    re.IGNORECASE)

# (steering, direction) -> firmware command.
#
# Forward turns are NOT mapped here: they always go to the parameterised
# TURNL/TURNR with an explicit angle. Measured against the STM's own gyro
# heading:
#
#     TURN90R    -> heading changed   +0.0 deg
#     TURN90L    -> heading changed   +0.0 deg
#     TURNR/90   -> heading changed  -90.3 deg
#     TURNL/90   -> heading changed  +90.7 deg
#     TURNR/30   -> heading changed  -30.7 deg
#
# The square-turn commands never move the reported heading, so anything that
# depends on the robot's orientation silently goes wrong with them.
ALGO_MAP = {
    ("center", "forward"): "FWD",
    ("center", "reverse"): "REVS",
    ("left", "reverse"): "REVL",
    ("right", "reverse"): "REVR",
}

# Forward turns, any angle.
ALGO_TURN = {"left": "TURNL", "right": "TURNR"}


class STMLink(Link):
    """Serial link to the STM32 running MDP_test.hex.

    The public interface is unchanged from the original implementation --
    ``send(str)`` and ``recv() -> str`` -- so ``final_week_8.py`` keeps working.
    What changed is the wire format underneath.

    ``recv()`` still returns a string starting with ``"ACK"`` when a command
    completes, because ``recv_stm()`` in final_week_8.py tests
    ``message.startswith("ACK")`` to release the movement lock. The real reply
    is appended after it so the logs stay honest, e.g. ``"ACK !3/DONE;"``.

    ``DONE`` replies are asynchronous: the firmware queues commands and answers
    when each one finishes, identified by its id. Callers that need to match a
    specific command should read ``last_done_id``.
    """

    def __init__(self):
        super().__init__()
        self.serial_link = None
        self.speed = DEFAULT_SPEED
        self.last_done_id: Optional[int] = None
        self.last_error: Optional[str] = None
        # Newest !SENSOR/... telemetry; the firmware streams it at ~8 Hz.
        self.last_sensor = None
        self._id = 0
        self._id_lock = threading.Lock()

    def connect(self):
        """Open the serial port and drain anything left in the buffer."""
        self.serial_link = serial.Serial(
            SERIAL_PORT, BAUD_RATE, timeout=READ_TIMEOUT)
        self.serial_link.reset_input_buffer()
        self.serial_link.reset_output_buffer()
        self.logger.info(
            f"Connected to STM32 on {SERIAL_PORT} at {BAUD_RATE} baud")

    def disconnect(self):
        """Close the serial link."""
        if self.serial_link is not None:
            self.serial_link.close()
            self.serial_link = None
        self.logger.info("Disconnected from STM32")

    def _next_id(self) -> int:
        """Command ids must be unique so async DONE replies can be matched."""
        with self._id_lock:
            self._id += 1
            return self._id

    def translate(self, command: str) -> str:
        """Turn an RPi command into a firmware frame.

        ``FW010``                 -> ``7/MOTOR/FWD/50/10;``
        ``center,0,forward,120``  -> ``8/MOTOR/FWD/50/120;``
        ``right,90,forward,0``    -> ``9/MOTOR/TURNR/50/90;``
        ``STOP``                  -> ``10/MOTOR/STOP/0/0;``

        A command already in ``<component>/<cmd>/...`` form is passed through
        with an id and terminator attached, so raw frames can still be sent.
        Note this allocates an id, so do not call it just to print a frame and
        then call send() -- the id will not match the reply.
        """
        raw = command.strip().rstrip(";")
        cmd_id = self._next_id()

        # Already a native frame: COMPONENT/COMMAND/p1/p2
        if "/" in raw:
            parts = raw.split("/")
            if parts[0].isdigit():      # caller supplied its own id, drop it
                parts = parts[1:]
            while len(parts) < 4:
                parts.append("0")
            return f"{cmd_id}/{'/'.join(parts[:4])}{TERMINATOR.decode()}"

        # Algo /path format: "center,0,forward,120" / "right,90,forward,0"
        algo = _ALGO_RE.match(raw)
        if algo:
            steering, angle, direction, distance = algo.groups()
            steering, direction = steering.lower(), direction.lower()
            angle = int(angle)

            if steering == "center":
                return (f"{cmd_id}/MOTOR/{ALGO_MAP[('center', direction)]}"
                        f"/{self.speed}/{int(distance)};")

            if direction == "forward":
                # Always parameterised -- see ALGO_MAP for why TURN90x is not
                # used even when the angle is exactly 90.
                return (f"{cmd_id}/MOTOR/{ALGO_TURN[steering]}"
                        f"/{self.speed}/{angle};")

            # Reverse turns have no parameterised equivalent; REVL/REVR are
            # fixed manoeuvres and the angle is passed through for whatever
            # use the firmware makes of it.
            return (f"{cmd_id}/MOTOR/{ALGO_MAP[(steering, 'reverse')]}"
                    f"/{self.speed}/{angle};")

        upper = raw.upper()

        if upper.startswith("STOP"):
            return f"{cmd_id}/MOTOR/STOP/0/0;"
        if upper.startswith("CAPTURE") or upper.startswith("SNAP"):
            return f"{cmd_id}/GENERAL/CAPTURE/0/0;"
        if upper.startswith("DONE") or upper.startswith("FIN"):
            return f"{cmd_id}/GENERAL/DONE/0/0;"

        match = _LEGACY_RE.match(upper)
        if not match:
            raise ValueError(f"Cannot translate command for STM32: {command!r}")

        prefix, amount = match.groups()
        if prefix not in COMMAND_MAP:
            raise ValueError(f"Unknown STM32 command prefix: {prefix!r}")

        # "FW--" means "move indefinitely" in the old manual mode; the firmware
        # has no such concept, so treat it as a large distance.
        value = 255 if amount == "--" else int(amount)
        # Legacy turns (FL00, FR00 -- what the tablet's L/R buttons send) carry
        # no angle. TURNL/TURNR take p2 as the angle, so 0 would mean "turn
        # 0 degrees"; treat a missing angle as a quarter turn.
        if prefix in _LEGACY_TURNS and value == 0:
            value = 90
        return f"{cmd_id}/MOTOR/{COMMAND_MAP[prefix]}/{self.speed}/{value};"

    def send(self, message: str) -> None:
        """Translate and write one command frame."""
        frame = self.translate(message)
        self.serial_link.write(frame.encode("utf-8"))
        self.serial_link.flush()
        self.logger.debug(f"Sent to STM32: {frame} (from {message!r})")

    def recv(self) -> Optional[str]:
        """Block until a command completion arrives.

        The firmware streams ``!SENSOR/...;`` telemetry at roughly 8 Hz whether
        or not anything was asked of it, and those frames interleave with the
        reply we actually want:

            !SENSOR/80.0/84.8/52.3/1/-41.2/0.00;
            !99/DONE;

        So a single read almost always returns telemetry. Returning that would
        be read as "not an ACK" by recv_stm, the movement lock would never be
        released, and the robot would stop after one command. Sensor frames are
        therefore stashed in ``last_sensor`` and skipped, and this keeps reading
        until a completion or the deadline.

        Returns a string beginning with ``ACK`` on completion, ``ERROR ...`` on
        a rejected or abandoned command, ``READY`` on the boot banner, or
        ``None`` on timeout.
        """
        deadline = time.monotonic() + READ_TIMEOUT
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                self.logger.warning(
                    "STM32: no completion within %ss (telemetry seen: %s)",
                    READ_TIMEOUT, self.last_sensor is not None)
                return None

            raw = self.serial_link.read_until(TERMINATOR)
            if not raw:
                continue

            message = raw.decode("utf-8", errors="replace").strip()
            body = message.lstrip("!").rstrip(";")
            parts = body.split("/")
            if not parts:
                continue

            tag = parts[0].upper()

            # Telemetry: keep the newest and go back to waiting.
            if tag == "SENSOR":
                self.last_sensor = parts[1:]
                continue

            self.logger.debug(f"Received from STM32: {message}")

            # !READY; -- boot banner, not a command completion
            if tag == "READY":
                self.logger.info("STM32 reported READY (booted)")
                return "READY"

            if len(parts) >= 2 and parts[1].upper() == "DONE":
                try:
                    self.last_done_id = int(parts[0])
                except ValueError:
                    self.last_done_id = None
                return f"ACK {message}"

            if len(parts) >= 3 and parts[1].upper() == "ERROR":
                self.last_error = "/".join(parts[2:])
                self.logger.error(f"STM32 rejected command: {message}")
                return f"ERROR {message}"

            return message

    # --- telemetry ------------------------------------------------------------
    #
    # The firmware's own format string (STM32/Core/Src/main.c):
    #     "!SENSOR/%.1f/%.1f/%.1f/%u/%.1f/%.2f;"
    #      leftIr, rightIr, usDistance, usValid, heading, gyroZ
    #
    # The helpers below reset the input buffer and read directly, so use them
    # only from standalone scripts -- never while another process is reading
    # the same port (final_week_8.py's recv_stm), or the two will steal each
    # other's frames.

    def read_sensors(self, timeout: float = 3.0):
        """Block for a fresh !SENSOR frame and return its six fields."""
        if self.serial_link is None:
            return None
        deadline = time.monotonic() + timeout
        self.serial_link.reset_input_buffer()
        while time.monotonic() < deadline:
            raw = self.serial_link.read_until(TERMINATOR)
            if not raw:
                continue
            parts = (raw.decode("utf-8", errors="replace")
                        .strip().lstrip("!").rstrip(";").split("/"))
            if parts and parts[0].upper() == "SENSOR" and len(parts) >= 7:
                self.last_sensor = parts[1:]
                return self.last_sensor
        return None

    def heading(self, timeout: float = 3.0) -> Optional[float]:
        """Current gyro heading in degrees, or None if telemetry stalls."""
        fields = self.read_sensors(timeout)
        if not fields or len(fields) < 5:
            return None
        try:
            return float(fields[4])
        except ValueError:
            return None

    @staticmethod
    def heading_delta(before: float, after: float) -> float:
        """Signed shortest rotation from one heading to another."""
        d = after - before
        while d > 180:
            d -= 360
        while d < -180:
            d += 360
        return d

    def ultrasonic_cm(self):
        """Forward ultrasonic range in cm, or None when the reading is invalid.

        Field 4 is the firmware's own validity flag (usValid); without an echo
        the distance field is meaningless.
        """
        s = self.read_sensors()
        if not s or len(s) < 4:
            return None
        try:
            if int(float(s[3])) != 1:
                return None
            v = float(s[2])
        except ValueError:
            return None
        return v if 2.0 < v < 400.0 else None

    def ir_cm(self):
        """(left, right) IR readings."""
        s = self.read_sensors()
        if not s or len(s) < 2:
            return (None, None)
        try:
            return (float(s[0]), float(s[1]))
        except ValueError:
            return (None, None)

    def turn_by(self, degrees: float, right: bool, tolerance: float = 6.0,
                attempts: int = 4, logger=None):
        """Turn a real number of degrees, verified against the gyro.

        The firmware gives up on a turn it cannot finish and answers
        ``!<id>/ERROR/TURN_TIMEOUT;`` having already rotated part of the way,
        so this measures the heading, turns, and re-issues whatever is left
        until it is within tolerance. Returns (achieved_degrees, ok).

        Standalone scripts only (see the telemetry note above): inside
        final_week_8.py the recv_stm process owns the port, and reading here
        too fails with "multiple access on port".
        """
        def say(m):
            (logger or self.logger.debug)(m)

        start = self.heading()
        if start is None:
            say("turn_by: no heading available, cannot verify")
            self.send(f"{'right' if right else 'left'},{int(degrees)},forward,0")
            return None, self.recv() is not None

        want = -abs(degrees) if right else abs(degrees)
        for attempt in range(1, attempts + 1):
            now = self.heading()
            done = self.heading_delta(start, now) if now is not None else 0.0
            remaining = want - done
            if abs(remaining) <= tolerance:
                say(f"turn_by: {done:+.1f} of {want:+.1f} deg - good")
                return done, True

            step = int(round(min(170, max(5, abs(remaining)))))
            self.send(f"{'right' if remaining < 0 else 'left'},{step},forward,0")
            reply = self.recv()
            after = self.heading()
            achieved = self.heading_delta(start, after) if after is not None else done
            if reply and "TURN_TIMEOUT" in reply:
                say(f"turn_by: attempt {attempt} timed out at {achieved:+.1f} deg, retrying")
            else:
                say(f"turn_by: attempt {attempt} -> {achieved:+.1f} of {want:+.1f} deg")

        final = self.heading()
        done = self.heading_delta(start, final) if final is not None else 0.0
        ok = abs(want - done) <= tolerance * 2
        say(f"turn_by: finished at {done:+.1f} of {want:+.1f} deg (ok={ok})")
        return done, ok
