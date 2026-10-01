"""Offline checks for the RPi Task 1 code: serial, Bluetooth and HTTP mocked."""
import json, logging, sys, threading, types

import os
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

# --- stubs for things that only exist on the Pi -----------------------------
logger_mod = types.ModuleType("logger")
logger_mod.prepare_logger = lambda: logging.getLogger("t")
sys.modules["logger"] = logger_mod

class FakeSerial:
    """Feeds scripted bytes to read_until; records writes."""
    def __init__(self, *a, **k):
        self.buf = b""; self.written = []
    def feed(self, b): self.buf += b
    def read_until(self, term=b";"):
        i = self.buf.find(term)
        if i == -1:
            out, self.buf = self.buf, b""
            return out
        out, self.buf = self.buf[:i + 1], self.buf[i + 1:]
        return out
    def write(self, b): self.written.append(b.decode())
    def flush(self): pass
    def reset_input_buffer(self): pass
    def reset_output_buffer(self): pass
    def close(self): pass
serial_mod = types.ModuleType("serial"); serial_mod.Serial = FakeSerial
sys.modules["serial"] = serial_mod

fails = 0
def check(label, got, want):
    global fails
    ok = got == want
    fails += not ok
    print(f"  {'OK  ' if ok else 'FAIL'} {label}" + ("" if ok else f"\n       got  {got!r}\n       want {want!r}"))

# ---------------------------------------------------------------- stm32
print("== stm32.translate ==")
from communication.stm32 import STMLink, READ_TIMEOUT
l = STMLink()
for cmd, want in [
    ("center,0,forward,120", "MOTOR/FWD/50/120;"),
    ("center,0,reverse,37",  "MOTOR/REVS/50/37;"),
    ("right,90,forward,0",   "MOTOR/TURNR/50/90;"),
    ("left,90,forward,0",    "MOTOR/TURNL/50/90;"),
    ("left,45,forward,0",    "MOTOR/TURNL/50/45;"),
    ("right,90,reverse,0",   "MOTOR/REVR/50/90;"),
    ("left,90,reverse,0",    "MOTOR/REVL/50/90;"),
    ("FW010",                "MOTOR/FWD/50/10;"),
    ("BW010",                "MOTOR/REVS/50/10;"),
    ("FL00",                 "MOTOR/TURNL/50/90;"),   # tablet L button
    ("FR00",                 "MOTOR/TURNR/50/90;"),   # tablet R button
    ("STOP",                 "MOTOR/STOP/0/0;"),
]:
    check(f"{cmd:22} -> {want}", l.translate(cmd).split("/", 1)[1], want)
ids = [int(l.translate("STOP").split("/")[0]) for _ in range(3)]
check("ids increase by one", ids[1] - ids[0] == 1 and ids[2] - ids[1] == 1, True)
try:
    l.translate("NONSENSE"); check("unknown command raises", False, True)
except ValueError:
    check("unknown command raises ValueError", True, True)

print("== stm32.recv skips telemetry ==")
l.serial_link = FakeSerial()
l.serial_link.feed(b"!SENSOR/80.0/96.3/52.3/1/-41.2/0.00;!SENSOR/80.0/96.0/52.0/1/-41.2/0.00;!7/DONE;")
check("DONE after 2 SENSOR frames", l.recv(), "ACK !7/DONE;")
check("last_done_id", l.last_done_id, 7)
check("telemetry stashed", l.last_sensor[4], "-41.2")
l.serial_link.feed(b"!SENSOR/1/2/3/1/0/0;!9/ERROR/TURN_TIMEOUT;")
check("ERROR surfaced", l.recv(), "ERROR !9/ERROR/TURN_TIMEOUT;")
l.serial_link.feed(b"!READY;")
check("boot banner", l.recv(), "READY")
check("heading_delta wraps", (STMLink.heading_delta(350, 10), STMLink.heading_delta(-41, 49)), (20, 90))

# ---------------------------------------------------------------- android protocol
print("== android_protocol ==")
from communication.android_protocol import FrameBuffer, AndroidState, to_frame
fb = FrameBuffer()
check("split frame, part 1", fb.feed("//OBSTA"), [])
check("split frame, part 2", fb.feed("CLE,1,5,7,E////BEGIN,1//"), ["OBSTACLE,1,5,7,E", "BEGIN,1"])
st = AndroidState()
for p in ["MAP", "ROBOT,10,2,N", "OBSTACLE,1,10,15,E", "OBSTACLE,1,10,15,S", "OBSTACLE,2,18,10,W"]:
    st.handle(p)
msgs = st.handle("BEGIN,1")
check("BEGIN,1 -> obstacles then start", [m["cat"] for m in msgs], ["obstacles", "control"])
check("re-faced obstacle kept latest", msgs[0]["value"]["obstacles"][0], {"id": 1, "x": 10, "y": 15, "d": 4})
check("robot pose carried", msgs[0]["value"]["robot"], {"x": 10, "y": 2, "d": 0})
st.handle("MAP")
check("Reset Map (//MAP//) wipes obstacles", st.obstacles, {})
check("L button", st.handle("L"), [{"cat": "manual", "value": "FL00"}])
check("RESULT,<task>,<time>", st.handle("RESULT,1,00:10.93"),
      [{"cat": "result", "value": {"task": "1", "time": "00:10.93"}}])
check("TARGET numeric", to_frame("image-rec", {"obstacle_id": "1", "image_id": "17"}), "//TARGET,1,17//")
check("TARGET NA suppressed", to_frame("image-rec", {"obstacle_id": "1", "image_id": "NA"}), None)
check("ROBOT frame", to_frame("location", {"x": 10, "y": 13, "d": 0}), "//ROBOT,10,13,N//")

# ---------------------------------------------------------------- final_week_8
print("== final_week_8 ==")
import final_week_8 as fw
from final_week_8 import RaspberryPi

class Q:
    def __init__(self): self.items = []
    def put(self, x): self.items.append(x)
    def get(self): return self.items.pop(0)
    def empty(self): return not self.items
class Lock:
    def __init__(self): self.n = 0
    def release(self): self.n += 1
    def acquire(self, timeout=None): return True

def bare_pi():
    pi = object.__new__(RaspberryPi)
    pi.logger = logging.getLogger("t")
    pi.android_queue, pi.command_queue, pi.path_queue, pi.rpi_action_queue = Q(), Q(), Q(), Q()
    pi.movement_lock = Lock()
    pi.obstacles, pi.current_location = {}, {}
    pi.success_obstacles, pi.failed_obstacles = [], []
    pi.retry_done = threading.Event()
    return pi

# request_algo: payload format + stm_commands queued
sent = {}
class Resp:
    status_code = 200
    def json(self):
        return {"data": {"stm_commands": ["center,0,forward,130", "SNAP1", "FIN"],
                         "path": [{"x": 10, "y": 1, "d": 0}, {"x": 10, "y": 14, "d": 0}]},
                "error": None}
def fake_post(url, data=None, headers=None, timeout=None, files=None):
    sent["url"], sent["data"] = url, data
    return Resp()
fw.requests.post = fake_post
pi = bare_pi()
pi.request_algo({"obstacles": [{"id": 1, "x": 9, "y": 19, "d": 4},
                               {"id": 2, "x": 18, "y": 10, "d": 6}]},
                robot_x=9, robot_y=1, robot_dir=0)
check("algo url", sent["url"], "http://192.168.50.147:8000/path")
check("algo payload", sent["data"],
      "STATUS,OBSTACLE_LIST,false,9;1;NORTH,false,1;9;19;SOUTH | 2;18;10;WEST,true,true")
check("stm_commands queued", pi.command_queue.items, ["center,0,forward,130", "SNAP1", "FIN"])
check("path queued (minus start)", pi.path_queue.items, [{"x": 10, "y": 14, "d": 0}])

# FIN retry: failed ids are strings, obstacles keyed by int -> must not crash
pi = bare_pi()
pi.obstacles = {1: {"id": 1, "x": 9, "y": 19, "d": 4}, 2: {"id": 2, "x": 18, "y": 10, "d": 6}}
pi.failed_obstacles = ["2", "2"]
pi.current_location = {"x": 6, "y": 11, "d": 6}
calls = []
pi.request_algo = lambda data, x, y, d, retrying=False: calls.append((data, x, y, d, retrying))
check("retry requested", pi.retry_failed_obstacles(), True)
check("retry only failed obstacle, from current pose",
      calls[0], ({"obstacles": [{"id": 2, "x": 18, "y": 10, "d": 6}], "mode": "0"}, 6, 11, 6, True))
check("retry happens once", pi.retry_failed_obstacles(), False)

# snap_and_rec: 404 = no objects -> retried as NA, not a fatal error
pi = bare_pi()
fw.os.system = lambda cmd: 0
import builtins, io
real_open = builtins.open
builtins.open = lambda *a, **k: io.BytesIO(b"jpg")
seq = iter([404, 200])
class R2:
    def __init__(self, code): self.status_code = code
    content = json.dumps({"image_id": "7", "obstacle_id": "17", "conf": 0.9}).encode()
fw.requests.post = lambda url, files=None: R2(next(seq))
pi.snap_and_rec("1")
builtins.open = real_open
target = [m for m in pi.android_queue.items if m.cat == "image-rec"]
check("404 retried, then TARGET for obstacle 1 id 17",
      [(m.value["obstacle_id"], m.value["image_id"]) for m in target], [("1", "17")])
check("movement lock released once", pi.movement_lock.n, 1)

print()
print("ALL PASS" if not fails else f"{fails} FAILURE(S)")
sys.exit(1 if fails else 0)
