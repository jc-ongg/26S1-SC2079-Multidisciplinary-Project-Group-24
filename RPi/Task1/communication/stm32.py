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
#   TL/TR  continuous steer
COMMAND_MAP = {
    "FW": "FWD",
    "FS": "FWD",
    "BW": "REVS",
    "BS": "REVS",
    # FL/FR are 90-degree turns; route them through the parameterised command
    # for the same reason as ALGO_MAP -- TURN90x does not move the heading.
    "FL": "TURNL",
    "FR": "TURNR",
    "BL": "REVL",
    "BR": "REVR",
    "TL": "TURNL",
    "TR": "TURNR",
}

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
# heading on 2026-09-18:
#
#     TURN90R    -> heading changed   +0.0 deg
#     TURN90L    -> heading changed   +0.0 deg
#     TURNR/90   -> heading changed  -90.3 deg
#     TURNL/90   -> heading changed  +90.7 deg
#     TURNR/30   -> heading changed  -30.7 deg
#
# The square-turn commands never move the reported heading, so anything that
# depends on the robot's orientation -- a planned path, or centring on a marker
# -- silently goes wrong with them. The parameterised command is accurate to
# about a degree, so it is used for every forward turn including 90.
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

    def __init__(self, log_telemetry: bool = True):
        super().__init__()
        self.log_telemetry = log_telemetry
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
        """Turn a legacy RPi command into a firmware frame.

        ``FW010`` -> ``7/MOTOR/FWD/50/10;``
        ``STOP``  -> ``8/MOTOR/STOP/0/0;``

        A command already in ``<component>/<cmd>/...`` form is passed through
        with an id and terminator attached, so raw frames can still be sent.
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
        distance = 255 if amount == "--" else int(amount)
        return f"{cmd_id}/MOTOR/{COMMAND_MAP[prefix]}/{self.speed}/{distance};"

    def send(self, message: str) -> int:
        """Translate and write one command frame."""
        frame = self.translate(message)
        self.serial_link.write(frame.encode("utf-8"))
        self.serial_link.flush()
        self.logger.debug(f"Sent to STM32: {frame} (from {message!r})")
        return int(frame.split("/", 1)[0])

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

        Returns a string beginning with ``ACK`` on completion so the existing
        movement-lock logic keeps working, ``ERROR ...`` on a rejected command,
        or ``None`` on timeout so callers can retry rather than treat a stall as
        an acknowledgement.
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

            # Telemetry: keep the newest and go back to waiting. Fields are not
            # labelled by the firmware, so they are exposed as-is.
            if tag == "SENSOR":
                self.last_sensor = parts[1:]
                if self.log_telemetry:
                    self.logger.debug(f"STM32 telemetry: {parts[1:]}")
                continue

            self.logger.debug(f"Received from STM32: {message}")

            # !READY; -- boot banner, not a command completion. Skip it like
            # telemetry: the caller is waiting for an ACK, and returning READY
            # here makes the first command after a power-cycle appear to fail.
            if tag == "READY":
                self.logger.info("STM32 reported READY (booted)")
                continue

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


    def read_sensors(self, timeout: float = 3.0):
        """Block for a fresh !SENSOR frame and return its six fields.

        Telemetry streams at ~8 Hz, so this normally returns in well under a
        second. Fields 1-3 are distance sensors and swing with whatever is
        nearby; field 5 is the gyro heading, which measured 0.00 degrees of
        drift over 40 seconds stationary and is the only one worth steering on.
        """
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
            if parts and parts[0].upper() == "SENSOR" and len(parts) >= 6:
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


    def turn_by(self, degrees: float, right: bool, tolerance: float = 6.0,
                attempts: int = 4, logger=None):
        """Turn a real number of degrees, verified against the gyro.

        A single turn command is not enough. The firmware gives up on a turn it
        cannot finish and answers ``!<id>/ERROR/TURN_TIMEOUT;`` having already
        rotated part of the way -- so a caller that only checks "did I get an
        error" is left believing nothing happened, while a caller that ignores
        the reply believes it turned the full amount. Neither is true, and that
        is what made commanded 180s measure 110 degrees one run and 172 the
        next.

        So: measure the heading, turn, measure again, and re-issue whatever is
        left until it is within tolerance or the attempts run out.

        Returns (achieved_degrees, ok).
        """
        def say(m):
            if logger:
                logger(m)
            else:
                self.logger.debug(m)

        start = self.heading()
        if start is None:
            # No telemetry: fall back to firing and hoping, but say so.
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
                say(f"turn_by: attempt {attempt} timed out after "
                    f"{self.heading_delta(start, after):+.1f} deg, retrying the rest")
            elif reply is None:
                say(f"turn_by: attempt {attempt} got no reply")
            else:
                say(f"turn_by: attempt {attempt} -> {achieved:+.1f} of {want:+.1f} deg")

        final = self.heading()
        done = self.heading_delta(start, final) if final is not None else 0.0
        ok = abs(want - done) <= tolerance * 2
        say(f"turn_by: finished at {done:+.1f} of {want:+.1f} deg (ok={ok})")
        return done, ok

    def ultrasonic_cm(self):
        """Forward ultrasonic range in cm, or None when the reading is invalid.

        Field 4 of the telemetry is the firmware's own validity flag
        (``usValid``). Ignoring it is why a reading of 82cm once sat next to a
        camera measurement of 57cm -- the echo had not come back and the value
        was meaningless.
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
        """(left, right) IR readings. Named per the firmware's own sprintf."""
        s = self.read_sensors()
        if not s or len(s) < 2:
            return (None, None)
        try:
            return (float(s[0]), float(s[1]))
        except ValueError:
            return (None, None)

    def sensors(self):
        """Most recent telemetry as a list of strings, or None.

        The firmware sends six unlabelled fields; observed behaviour suggests
        distances and a heading, but nothing is documented, so no meaning is
        imposed here. Ask the STM team before relying on any single field.
        """
        return self.last_sensor
