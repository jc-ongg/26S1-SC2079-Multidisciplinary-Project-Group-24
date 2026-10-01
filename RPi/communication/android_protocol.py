"""Framing and translation for the MDP Group 24 Bluetooth protocol.

Pure logic, no sockets -- see MDP_Group24_Bluetooth_Protocol.md.

Wire format is ``//<payload>//``. Bluetooth is a byte stream, so one read may
carry half a frame or several at once (``//STATUS,Moving////ROBOT,5,8,E//``),
which is why FrameBuffer keeps leftovers between reads instead of treating each
read as one message.

The rest of final_week_8.py speaks the older ``{"cat": ..., "value": ...}``
dicts, so this module translates in both directions and that code is left alone.
"""
import json
from collections import deque

# Protocol uses single-letter compass points; the algorithm and the RPi's
# internal state use the 0/2/4/6 encoding from the Algo server's face_map.
DIR_TO_LETTER = {0: "N", 2: "E", 4: "S", 6: "W",
                 1: "N", 3: "E", 5: "S", 7: "W"}
LETTER_TO_DIR = {"N": 0, "E": 2, "S": 4, "W": 6}
LETTER_TO_NAME = {"N": "NORTH", "E": "EAST", "S": "SOUTH", "W": "WEST"}

# Android's manual-drive buttons -> legacy STM command strings.
MANUAL_MOVES = {"F": "FW010", "B": "BW010", "L": "FL090", "R": "FR090"}


class FrameBuffer:
    """Accumulates bytes and yields complete ``//...//`` payloads."""

    def __init__(self):
        self._buf = ""

    def feed(self, chunk):
        """Add received text and return the complete payloads now available."""
        if chunk:
            self._buf += chunk
        out = []
        while True:
            start = self._buf.find("//")
            if start == -1:
                break
            end = self._buf.find("//", start + 2)
            if end == -1:
                # Partial frame -- keep it (and drop anything before it, which
                # cannot become a valid frame any more).
                self._buf = self._buf[start:]
                break
            payload = self._buf[start + 2:end]
            self._buf = self._buf[end + 2:]
            if payload:
                out.append(payload)
        return out

    @property
    def pending(self):
        return self._buf


def frame(payload):
    """Wrap a payload for sending."""
    return f"//{payload}//"


class AndroidState:
    """Tracks the map Android has described so far.

    Obstacles arrive one ``//OBSTACLE,...//`` at a time, but the RPi wants the
    whole list when the run starts, so they are accumulated here and emitted on
    ``//BEGIN//``.
    """

    def __init__(self):
        self.robot = {"x": 1, "y": 1, "d": 0}
        self.obstacles = {}
        self.outbox = deque()

    # ---------------------------------------------------------------- inbound
    def handle(self, payload):
        """Translate one payload into zero or more internal message dicts."""
        parts = [p.strip() for p in payload.split(",")]
        head = parts[0].upper()

        if head.startswith("MAP"):
            return self._handle_map(payload)

        if head == "ROBOT" and len(parts) >= 4:
            if parts[3] == "-1":
                return []                      # removal, nothing to run
            self.robot = {"x": int(parts[1]), "y": int(parts[2]),
                          "d": LETTER_TO_DIR.get(parts[3].upper(), 0)}
            return []

        if head == "OBSTACLE" and len(parts) >= 5:
            oid = int(parts[1])
            if parts[4] == "-1":
                self.obstacles.pop(oid, None)
                return []
            self.obstacles[oid] = {"id": oid, "x": int(parts[2]),
                                   "y": int(parts[3]),
                                   "d": LETTER_TO_DIR.get(parts[4].upper(), 0)}
            return []

        if head == "BEGIN":
            # The run starts: hand over the accumulated map, then the go signal.
            return [
                {"cat": "obstacles",
                 "value": {"obstacles": list(self.obstacles.values()),
                           "mode": "0",
                           "robot": dict(self.robot)}},
                {"cat": "control", "value": "start"},
            ]

        if head in MANUAL_MOVES:
            return [{"cat": "manual", "value": MANUAL_MOVES[head]}]

        if head == "RESULT":
            return [{"cat": "result", "value": parts[1] if len(parts) > 1 else ""}]

        # Anything else (chat text etc.) is passed through as info rather than
        # dropped, so it shows up in the logs instead of vanishing.
        return [{"cat": "info", "value": payload}]

    def _handle_map(self, payload):
        """``//MAP/ROBOT,3,3,N/OBSTACLE,1,5,7,E//`` is an authoritative snapshot."""
        self.obstacles = {}
        self.robot = {"x": 1, "y": 1, "d": 0}
        for section in payload.split("/")[1:]:
            section = section.strip()
            if not section:
                continue
            bits = [b.strip() for b in section.split(",")]
            kind = bits[0].upper()
            if kind == "ROBOT" and len(bits) >= 4 and bits[3] != "-1":
                self.robot = {"x": int(bits[1]), "y": int(bits[2]),
                              "d": LETTER_TO_DIR.get(bits[3].upper(), 0)}
            elif kind == "OBSTACLE" and len(bits) >= 5 and bits[4] != "-1":
                oid = int(bits[1])
                self.obstacles[oid] = {"id": oid, "x": int(bits[2]),
                                       "y": int(bits[3]),
                                       "d": LETTER_TO_DIR.get(bits[4].upper(), 0)}
        return []


# ---------------------------------------------------------------------- outbound
def to_frame(cat, value):
    """Translate an internal message into a protocol frame.

    Returns None when there is nothing meaningful to send, so callers can skip
    rather than emit an empty frame.
    """
    if cat == "location" and isinstance(value, dict):
        letter = DIR_TO_LETTER.get(value.get("d", 0), "N")
        return frame(f"ROBOT,{value.get('x')},{value.get('y')},{letter}")

    if cat == "image-rec" and isinstance(value, dict):
        obstacle_id = value.get("obstacle_id")
        image_id = value.get("image_id")
        if obstacle_id is None or image_id in (None, "NA"):
            return None
        return frame(f"TARGET,{obstacle_id},{image_id}")

    if cat in ("info", "error", "status", "mode"):
        text = value if isinstance(value, str) else json.dumps(value)
        # Commas are the field separator, so keep the status on one field.
        return frame(f"STATUS,{text}")

    return frame(f"STATUS,{value}")
