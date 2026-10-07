#!/usr/bin/env python3
"""Checklist A.5 -- circle the obstacle without ever losing sight of it.

    python3 a5_track.py               # run
    python3 a5_track.py --calibrate   # report what the camera and sonar see
    python3 a5_track.py --dry-run     # decide, never move

Earlier versions used big manoeuvres -- reverse, 90, 180 -- computed from the
turn radius. They kept failing for two reasons that are not going away:

  * the robot arcs ~30cm through a 90 turn, so every turn is also a large
    translation, and
  * the turns are not repeatable: a commanded right-180 measured 110 degrees
    once and 172 the next time.

So nothing here is commanded blind. Every move is small, the camera is checked
straight afterwards, and if the marker has gone the move is undone before
anything else is tried. The obstacle stays in frame the whole way round, which
is the only thing that makes an unrepeatable drivetrain workable.

The sonar (telemetry field 3, a forward range-finder in cm) is used for range
instead of the marker's apparent size: it keeps reading when the marker drifts
to the edge of frame, and it does not care how big the marker is.
"""
import argparse
import os
import subprocess
import sys
import time

import requests

from settings import API_IP_IMAGE, API_PORT_IMAGE
from communication.stm32 import STMLink

IMAGE_API = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
SNAP = "/tmp/a5_track.jpg"
FRAME_W, FRAME_H = 640, 480
CAPTURE_MS = 250

# --- geometry ---------------------------------------------------------------
MARKER_CM = 10.0        # obstacle face is 10x10
FOV_HALF_TAN = 0.6      # tan(31 deg): ~62 degree horizontal view
TURN_RADIUS_CM = 30.0   # measured; the robot arcs, it does not pivot

# --- what "in position to read" means ---------------------------------------
READ_MIN_CM, READ_MAX_CM = 22.0, 38.0
CENTER_TOL = 0.14       # marker may sit this far off centre, as frame fraction

# --- step sizes -------------------------------------------------------------
# Small enough that one step can never sweep the marker out of view: at 40cm the
# frame spans 48cm, and a 15 degree turn displaces 2*30*sin(7.5) = 7.8cm.
NUDGE_DEG = 15
ARC_DEG = 25            # turn away from the marker to travel around it
ARC_CM = 12             # then this far along
STEP_CM = 8

MIN_CONF = 0.50
ORBIT_TARGET_DEG = 90   # a quarter turn moves us to the next face
ORBIT_TOL_DEG = 15
MAX_ACTIONS = 60
FACES_TO_TRY = 4


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# --- perception -------------------------------------------------------------
def capture():
    if os.path.exists(SNAP):
        os.remove(SNAP)
    p = subprocess.run(
        ["rpicam-still", "-e", "jpg", "-n", "-t", str(CAPTURE_MS),
         "--width", str(FRAME_W), "--height", str(FRAME_H), "-q", "80",
         "-o", SNAP],
        capture_output=True, text=True, timeout=30)
    if not os.path.exists(SNAP) or os.path.getsize(SNAP) == 0:
        log(f"  capture failed: {p.stderr.strip()[-140:]}")
        return False
    return True


def look():
    """Returns (target, marker) -- marker is {offset, size, conf} or None."""
    if not capture():
        return None, None
    with open(SNAP, "rb") as f:
        try:
            r = requests.post(IMAGE_API,
                              files={"image": ("s.jpg", f, "image/jpeg")},
                              timeout=60)
        except requests.RequestException as e:
            log(f"  image-api unreachable: {e}")
            return None, None
    if r.status_code == 404:          # image-api's "nothing found"
        return None, None
    if r.status_code != 200:
        log(f"  image-api http {r.status_code}")
        return None, None

    body = r.json()
    target = None
    iid = body.get("image_id")
    if iid and iid != "NA" and float(body.get("conf", 0)) >= MIN_CONF:
        target = {"image_id": iid, "conf": body.get("conf")}

    marker = None
    b = body.get("bullseye")
    if b and b.get("xyxy"):
        x1, _, x2, _ = b["xyxy"]
        marker = {
            "offset": ((x1 + x2) / 2 - FRAME_W / 2) / FRAME_W,
            "size": abs(x2 - x1) / FRAME_W,
            "conf": b.get("conf", 0),
        }
    return target, marker


class Tracker:
    """Moves the robot while keeping the obstacle in frame.

    Every move goes through `attempt`, which checks the camera afterwards and
    reverses the move if the marker was lost. That is what lets this work with
    turns that are neither repeatable nor on-the-spot.
    """

    def __init__(self, link, dry_run=False):
        self.link = link
        self.dry_run = dry_run
        self.marker = None
        self.target = None

    # -- primitives ---------------------------------------------------------
    def _send(self, command, why):
        if self.dry_run:
            log(f"  WOULD {command:<24} {why}")
            return True
        log(f"  {command:<24} {why}")
        self.link.send(command)
        reply = self.link.recv()
        if reply is None:
            log("    !! no reply from STM")
            return False
        if reply.startswith("ERROR"):
            log(f"    !! {reply}")
            return False
        return True

    def turn(self, deg, right, why=""):
        return self._send(f"{'right' if right else 'left'},{deg},forward,0", why)

    def forward(self, cm, why=""):
        return self._send(f"center,0,forward,{int(cm)}", why)

    def back(self, cm, why=""):
        return self._send(f"center,0,reverse,{int(cm)}", why)

    def sonar_cm(self):
        """Forward range from telemetry field 3, in cm."""
        if self.dry_run:
            return None
        s = self.link.read_sensors()
        if not s or len(s) < 3:
            return None
        try:
            v = float(s[2])
        except ValueError:
            return None
        # The sensor reports implausible values when it gets no echo.
        return v if 5.0 < v < 200.0 else None

    def observe(self):
        self.target, self.marker = look()
        return self.marker is not None

    # -- the safety net -----------------------------------------------------
    def attempt(self, do, undo, why):
        """Make a move; put it back if the obstacle left the frame.

        This is the whole idea. A move that loses the marker is not a disaster
        to be recovered from later by sweeping blindly -- it is undone straight
        away, while we still know exactly what caused it.
        """
        if not do():
            return False
        if self.dry_run:
            return True
        if self.observe():
            return True
        log(f"    lost the marker on '{why}' -- undoing")
        undo()
        self.observe()
        return False

    def nudge(self, right, why):
        return self.attempt(
            lambda: self.turn(NUDGE_DEG, right, why),
            lambda: self.turn(NUDGE_DEG, not right, "(undo)"),
            why)

    def creep(self, cm, why):
        return self.attempt(
            lambda: self.forward(cm, why),
            lambda: self.back(cm, "(undo)"),
            why)

    def retreat(self, cm, why):
        return self.attempt(
            lambda: self.back(cm, why),
            lambda: self.forward(cm, "(undo)"),
            why)

    def arc(self, right):
        """One step around the obstacle: turn off it, travel, turn back on.

        Split into three checked moves rather than one manoeuvre, so if any
        part of it loses the marker only that part is undone.
        """
        log("    arc step")
        if not self.attempt(lambda: self.turn(ARC_DEG, right, "(turn off the face)"),
                            lambda: self.turn(ARC_DEG, not right, "(undo)"),
                            "arc turn"):
            return False
        if not self.attempt(lambda: self.forward(ARC_CM, "(travel around)"),
                            lambda: self.back(ARC_CM, "(undo)"),
                            "arc travel"):
            return False
        # Turning back is what re-aims the camera; if it overshoots the centring
        # logic will fix it on the next pass.
        return self.attempt(lambda: self.turn(ARC_DEG, not right, "(turn back on)"),
                            lambda: self.turn(ARC_DEG, right, "(undo)"),
                            "arc re-aim")


def search(bot):
    faces = 1
    orbiting = False
    orbit_from = None
    lost = 0

    if not bot.observe():
        log("no obstacle in view to begin with -- point the robot at it")
        return None

    for _ in range(MAX_ACTIONS):
        if bot.target:
            log(f"  >>> FOUND {bot.target['image_id']} conf={bot.target['conf']}")
            bot._send("STOP", "(target found - halt)")
            return bot.target

        if bot.marker is None:
            lost += 1
            log(f"  marker not in frame ({lost})")
            if lost >= 4:
                log("  lost the obstacle completely")
                return None
            # Small alternating look-around; undone automatically if it fails.
            bot.nudge(lost % 2 == 1, "(look around)")
            continue
        lost = 0

        off = bot.marker["offset"]
        sonar = bot.sonar_cm()
        cam_cm = MARKER_CM / (2 * FOV_HALF_TAN * bot.marker["size"])
        # Prefer the camera: it measures the marker itself. The sonar is a
        # single forward beam and will happily range the wall behind the cube
        # when the marker is off to one side -- seen reading 82cm against the
        # camera's 57cm. It is a fallback, not the primary.
        dist = cam_cm if bot.marker else sonar
        src = "camera" if bot.marker else "sonar"
        if sonar and abs(sonar - cam_cm) > 20:
            log(f"    (sonar {sonar:.0f}cm disagrees with camera {cam_cm:.0f}cm "
                f"- probably ranging past the cube)")
        log(f"  marker conf={bot.marker['conf']:.2f} offset={off:+.3f} "
            f"{dist:.0f}cm ({src})" + ("  [orbiting]" if orbiting else ""))

        # 1. Keep it centred. Everything else assumes it is ahead of us.
        if abs(off) > CENTER_TOL:
            bot.nudge(off > 0, f"(centre, off by {off:+.2f})")
            continue

        # 2. Hold a sensible range.
        if dist > READ_MAX_CM:
            bot.creep(STEP_CM, f"(close in from {dist:.0f}cm)")
            continue
        if dist < READ_MIN_CM:
            bot.retreat(STEP_CM, f"(back off from {dist:.0f}cm)")
            continue

        # 3. Centred and in range.
        if orbiting:
            now = bot.link.heading() if not bot.dry_run else None
            turned = abs(bot.link.heading_delta(orbit_from, now)) if now is not None else 0
            log(f"    orbit {turned:.0f}/{ORBIT_TARGET_DEG} deg")
            if turned >= ORBIT_TARGET_DEG - ORBIT_TOL_DEG:
                faces += 1
                log(f"  arrived at face {faces}")
                orbiting = False
                continue
            if not bot.arc(True):
                log("    arc blocked; trying a smaller nudge")
                bot.nudge(True, "(edge around)")
            continue

        # Nothing here, so this face is blank -- start going round.
        log(f"  face {faces}: bullseye only, no symbol")
        if faces >= FACES_TO_TRY:
            log("  all four faces checked, no symbol found")
            return None
        orbiting = True
        orbit_from = bot.link.heading() if not bot.dry_run else 0.0
        log(f"  -- circling to face {faces + 1} --")

    log("  ran out of actions")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    link = STMLink()
    link.connect()
    bot = Tracker(link, dry_run=args.dry_run)

    try:
        if args.calibrate:
            bot.observe()
            if bot.marker:
                cam = MARKER_CM / (2 * FOV_HALF_TAN * bot.marker["size"])
                log(f"  marker  offset {bot.marker['offset']:+.3f}  "
                    f"conf {bot.marker['conf']:.2f}  camera says {cam:.0f}cm")
            else:
                log("  no marker in view")
            log(f"  sonar   {bot.sonar_cm()}")
            log(f"  heading {link.heading()}")
            log(f"  target  {bot.target}")
            return 0

        found = search(bot)
        if found:
            log(f"SUCCESS: {found['image_id']}")
            return 0
        log("FAILED: no target image found")
        return 1
    finally:
        if not args.dry_run:
            try:
                link.send("STOP")
                link.recv()
            except Exception:
                pass
        link.disconnect()


if __name__ == "__main__":
    sys.exit(main())
