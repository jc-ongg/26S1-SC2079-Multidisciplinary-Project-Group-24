#!/usr/bin/env python3
"""Checklist A.5 -- remember where the obstacle is, so losing sight is survivable.

    python3 a5_map.py              # run
    python3 a5_map.py --dry-run    # decide, never move
    python3 a5_map.py --probe      # report pose/marker/range and exit

Every earlier attempt failed the same way: the robot turned, the marker left the
frame, and from then on it was guessing. Sweeping blindly to re-acquire does not
work when a single turn also drags the robot 30cm sideways.

So this one keeps a map. Two things are tracked in a fixed world frame:

    the robot's pose    (x, y, heading)
    the obstacle        (x, y), averaged over every sighting

Heading comes from the gyro, which measured zero drift over 40 seconds, so it is
trusted absolutely. Position is dead-reckoned: straight moves are easy, and a
turn is integrated as a circular arc of TURN_RADIUS_CM, because this robot
cannot pivot -- a 90 degree turn carries it ~30cm forward and ~30cm sideways.

The payoff is that when the marker goes out of frame the robot is not lost. It
computes the bearing to the remembered obstacle and turns straight back to it,
and any sighting immediately re-anchors the estimate.
"""
import argparse
import math
import os
import subprocess
import sys
import time

import requests

from settings import API_IP_IMAGE, API_PORT_IMAGE
from communication.stm32 import STMLink

IMAGE_API = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}/predict"
SNAP = "/tmp/a5_map.jpg"
FRAME_W, FRAME_H = 640, 480

MARKER_CM = 10.0
FOV_DEG = 62.0
FOV_HALF_TAN = math.tan(math.radians(FOV_DEG / 2))
TURN_RADIUS_CM = 30.0

READ_CM = 30.0          # where we want to sit to read a face
READ_TOL = 8.0
VIEW_TOL_DEG = 12.0     # how far off-bearing before turning back to the cube

ORBIT_STEP_DEG = 45     # how far round the cube to move per leg
MIN_CONF = 0.50
MAX_ACTIONS = 40
FACES = 4


def log(m):
    print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def norm(a):
    """Wrap an angle to (-180, 180]."""
    while a > 180:
        a -= 360
    while a <= -180:
        a += 360
    return a


# --- perception -------------------------------------------------------------
def look():
    if os.path.exists(SNAP):
        os.remove(SNAP)
    subprocess.run(["rpicam-still", "-e", "jpg", "-n", "-t", "250",
                    "--width", str(FRAME_W), "--height", str(FRAME_H),
                    "-q", "80", "-o", SNAP], capture_output=True, timeout=30)
    if not os.path.exists(SNAP) or os.path.getsize(SNAP) == 0:
        return None, None
    with open(SNAP, "rb") as f:
        try:
            r = requests.post(IMAGE_API,
                              files={"image": ("s.jpg", f, "image/jpeg")},
                              timeout=60)
        except requests.RequestException as e:
            log(f"    image-api unreachable: {e}")
            return None, None
    if r.status_code not in (200, 404):
        return None, None
    if r.status_code == 404:
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
        size = abs(x2 - x1) / FRAME_W
        # Bearing: how far off the optical axis the marker sits. Positive
        # offset is to the right, which is a negative (clockwise) bearing.
        offset = ((x1 + x2) / 2 - FRAME_W / 2) / FRAME_W
        marker = {
            "bearing": -offset * FOV_DEG,
            "range": MARKER_CM / (2 * FOV_HALF_TAN * size) if size > 0 else None,
            "conf": b.get("conf", 0),
            "offset": offset,
        }
    return target, marker


# --- the map ----------------------------------------------------------------
class World:
    """Robot pose and obstacle position, in a fixed frame.

    The frame is arbitrary: it is pinned to wherever the robot starts, heading
    along +x. Only relative geometry matters.
    """

    def __init__(self, heading0):
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0          # our frame
        self._gyro0 = heading0      # gyro reading that corresponds to heading 0
        self.obstacle = None        # (x, y)
        self.sightings = 0

    # -- pose --------------------------------------------------------------
    def sync_heading(self, gyro):
        """Trust the gyro for orientation; it does not drift."""
        if gyro is not None:
            self.heading = norm(gyro - self._gyro0)

    def moved_straight(self, cm):
        t = math.radians(self.heading)
        self.x += cm * math.cos(t)
        self.y += cm * math.sin(t)

    def turned(self, deg, right, reverse=False):
        """Integrate a turn as a circular arc, not a pivot.

        Through a turn of phi at radius r the robot advances r*sin(phi) along
        its heading and slides r*(1-cos(phi)) to the side. Ignoring that is
        what made every earlier version lose the cube.
        """
        phi = math.radians(abs(deg))
        fwd = TURN_RADIUS_CM * math.sin(phi)
        side = TURN_RADIUS_CM * (1 - math.cos(phi))
        if reverse:
            fwd = -fwd
        t = math.radians(self.heading)
        sgn = -1.0 if right else 1.0
        # side is perpendicular to heading: +90 degrees is to the left
        self.x += fwd * math.cos(t) + sgn * side * math.cos(t - sgn * math.pi / 2)
        self.y += fwd * math.sin(t) + sgn * side * math.sin(t - sgn * math.pi / 2)

    # -- obstacle ----------------------------------------------------------
    def saw(self, bearing, rng):
        """Fold a sighting into the obstacle estimate."""
        if rng is None:
            return
        t = math.radians(self.heading + bearing)
        ox = self.x + rng * math.cos(t)
        oy = self.y + rng * math.sin(t)
        if self.obstacle is None:
            self.obstacle = (ox, oy)
        else:
            # Light smoothing: a sighting is worth more than the old estimate,
            # but one bad box should not throw the map away.
            px, py = self.obstacle
            self.obstacle = (0.4 * px + 0.6 * ox, 0.4 * py + 0.6 * oy)
        self.sightings += 1

    def bearing_to_obstacle(self):
        if self.obstacle is None:
            return None
        ox, oy = self.obstacle
        want = math.degrees(math.atan2(oy - self.y, ox - self.x))
        return norm(want - self.heading)

    def range_to_obstacle(self):
        if self.obstacle is None:
            return None
        ox, oy = self.obstacle
        return math.hypot(ox - self.x, oy - self.y)

    def __str__(self):
        o = ("obstacle (%.0f,%.0f)" % self.obstacle) if self.obstacle else "obstacle unknown"
        return "pose (%.0f,%.0f) hdg %+.0f | %s" % (self.x, self.y, self.heading, o)


class Robot:
    def __init__(self, link, world, dry_run=False):
        self.link = link
        self.w = world
        self.dry_run = dry_run

    def _do(self, command, why):
        if self.dry_run:
            log(f"  WOULD {command:<24} {why}")
            return True
        log(f"  {command:<24} {why}")
        self.link.send(command)
        reply = self.link.recv()
        if reply is None or reply.startswith("ERROR"):
            log(f"    !! {reply}")
            return False
        self.w.sync_heading(self.link.heading())
        return True

    def forward(self, cm, why=""):
        ok = self._do(f"center,0,forward,{int(cm)}", why)
        if ok:
            self.w.moved_straight(cm)
        return ok

    def back(self, cm, why=""):
        ok = self._do(f"center,0,reverse,{int(cm)}", why)
        if ok:
            self.w.moved_straight(-cm)
        return ok

    def turn(self, deg, right, why=""):
        """Gyro-verified turn.

        The firmware abandons turns it cannot finish (!ERROR/TURN_TIMEOUT;)
        having already rotated part way, so a fire-and-forget turn leaves the
        pose estimate wrong by tens of degrees. turn_by() re-issues the
        remainder until the gyro agrees, and returns what was actually
        achieved -- which is what the map is updated with.
        """
        deg = int(max(5, min(170, abs(deg))))
        if self.dry_run:
            log(f"  WOULD turn {'right' if right else 'left'} {deg:<13} {why}")
            self.w.turned(deg, right)
            self.w.heading = norm(self.w.heading + (-deg if right else deg))
            return True
        log(f"  turn {'right' if right else 'left'} {deg:<19} {why}")
        achieved, ok = self.link.turn_by(deg, right, logger=lambda m: log(f"    {m}"))
        # Integrate the arc for what actually happened, not what was asked.
        if achieved is not None:
            self.w.turned(abs(achieved), achieved < 0)
        else:
            self.w.turned(deg, right)
        self.w.sync_heading(self.link.heading())
        return ok

    def observe(self):
        target, marker = look()
        if marker:
            self.w.saw(marker["bearing"], marker["range"])
        return target, marker


def face_the_obstacle(bot):
    """Turn to point at the remembered obstacle. Returns True if we moved."""
    b = bot.w.bearing_to_obstacle()
    if b is None or abs(b) <= VIEW_TOL_DEG:
        return False
    log(f"    map says the cube is {b:+.0f} deg away - turning back to it")
    bot.turn(abs(b), b < 0, "(re-aim using the map)")
    return True


def run(bot):
    w = bot.w
    target, marker = bot.observe()
    if marker is None:
        log("no marker in view at the start - point the robot at the cube")
        return None
    log(f"  {w}")

    faces_read = 0
    orbit_done = 0.0

    for _ in range(MAX_ACTIONS):
        target, marker = bot.observe()

        if target:
            log(f"  >>> FOUND {target['image_id']} conf={target['conf']}")
            bot._do("STOP", "(halt)")
            return target

        if marker is None:
            # Not lost: the map still knows where it is.
            log(f"  marker out of frame - {w}")
            if not face_the_obstacle(bot):
                log("  map has no obstacle and nothing in view - stopping")
                return None
            continue

        rng = marker["range"]
        log(f"  marker conf={marker['conf']:.2f} bearing={marker['bearing']:+.0f} "
            f"range={rng:.0f}cm | {w}")

        # 1. Point at it.
        if abs(marker["bearing"]) > VIEW_TOL_DEG:
            b = marker["bearing"]
            bot.turn(abs(b), b < 0, f"(aim at the marker, {b:+.0f} deg)")
            continue

        # 2. Sit at reading range.
        if rng > READ_CM + READ_TOL:
            bot.forward(min(12, rng - READ_CM), f"(close in from {rng:.0f}cm)")
            continue
        if rng < READ_CM - READ_TOL:
            bot.back(min(12, READ_CM - rng), f"(back off from {rng:.0f}cm)")
            continue

        # 3. Aimed and in range, but no symbol: this face is blank.
        faces_read += 1
        log(f"  face {faces_read}: bullseye only, no symbol")
        if faces_read >= FACES:
            log("  every face checked, no symbol found")
            return None

        # Move a step around the cube. Back off first so the turn's 30cm arc
        # happens clear of the obstacle, then let the map re-aim us.
        log(f"  -- moving round to face {faces_read + 1} --")
        bot.back(25, "(open up before turning)")
        bot.turn(ORBIT_STEP_DEG, True, "(swing around the cube)")
        bot.forward(20, "(travel round)")
        face_the_obstacle(bot)

    log("  ran out of actions")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--probe", action="store_true")
    args = ap.parse_args()

    link = STMLink()
    link.connect()
    world = World(link.heading() or 0.0)
    bot = Robot(link, world, dry_run=args.dry_run)

    try:
        if args.probe:
            t, m = bot.observe()
            log(f"  {world}")
            log(f"  marker: {m}")
            log(f"  target: {t}")
            log(f"  bearing to obstacle: {world.bearing_to_obstacle()}")
            log(f"  range to obstacle  : {world.range_to_obstacle()}")
            return 0
        found = run(bot)
        log(f"  final: {world}  ({world.sightings} sightings)")
        if found:
            log(f"SUCCESS: {found['image_id']}")
            return 0
        log("FAILED: no target found")
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
