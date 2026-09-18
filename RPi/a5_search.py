#!/usr/bin/env python3
"""Checklist A.5 -- find the one valid image on a four-sided obstacle.

    python3 a5_search.py               # run it
    python3 a5_search.py --calibrate   # just report what the camera sees
    python3 a5_search.py --dry-run     # decide, but never move

Nothing here is a fixed route. The cube can be moved mid-run and the robot has
to notice and follow, so every movement is decided from the current frame:

    snap
      valid image        -> report it, done
      bullseye, off-centre -> turn to centre it
      bullseye, too small  -> drive closer (it moved away, or we started far)
      bullseye, close      -> this face is blank; go round to the next one
      nothing at all       -> sweep to re-acquire

Distance comes from the bullseye's apparent width: three of the four faces carry
one, so there is almost always something to measure against. A marker that
shrinks between frames means the obstacle was pulled away, and the same rule
that handles "started too far" handles that too -- no special case.

Everything below the constants is tuned by measurement, not guesswork: run
--calibrate at the distance you want and read BOX_FRAC off the output.
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
SNAP = "/tmp/a5_search.jpg"

# 640x480 is a native binned sensor mode at the full field of view, so it is the
# fastest thing the camera can produce and the geometry below still holds. A
# larger still buys nothing: inference runs at 640 anyway.
FRAME_W, FRAME_H = 640, 480

# Milliseconds rpicam-still spends settling exposure before the shot. The
# default 700+ dominates the loop; 250 still gives auto-exposure time to
# converge indoors while roughly halving the cost of a decision.
CAPTURE_MS = 250

# --- tuning -----------------------------------------------------------------
# Target stand-off is 30cm. The obstacle face is ~10cm wide and the camera sees
# ~62 degrees horizontally, so at 30cm the visible width is
#     2 * 30 * tan(31 deg) = 36cm
# and a 10cm marker fills 10/36 = 0.28 of the frame.
#
# 20cm (0.42) was tried first and is too tight: the view is only 24cm wide
# there, so the few centimetres the robot drifts during a turn can put the face
# outside the frame entirely. Confirm with --calibrate.
TARGET_BOX_FRAC = 0.28          # ~30cm
# Stop closing in once within this much of the target, to avoid hunting.
# Roughly +-3cm at this range.
BOX_FRAC_TOL = 0.04
# How far the marker may sit from the frame centre before we turn, as a
# fraction of frame width. Loose enough not to burn a move on a small error.
CENTER_TOL_FRAC = 0.12

STEP_NEAR = 10          # units per approach step when we are close
STEP_FAR = 30           # bigger stride when the marker is still small
FAR_BOX_FRAC = 0.20     # below this (~40cm) the obstacle is "far away"

# Orbit legs. Roughly (stand-off + half the cube), so a 20cm stand-off and a
# 10cm cube give ~25. They need not be exact: the search loop re-centres on the
# marker afterwards, and the heading check catches a bad rotation.
# The robot does NOT pivot on the spot: a 90-degree turn sweeps an arc that
# carries it about TURN_RADIUS_CM forward AND the same sideways. Measured at
# ~10cm on this robot. Three turns therefore supply 3 x that much lateral
# travel on their own, which is why the orbit legs are short -- and at a 30cm
# stand-off the second leg is not needed at all.
#
#     leg1 = half_cube + distance - turn_radius
#     leg2 = distance - 3 * turn_radius      (clamped at 0)
#
# Both are computed from the distance the camera actually measures at the time,
# so nothing here is tied to a particular starting position.
TURN_RADIUS_CM = 30.0
CUBE_HALF_CM = 5.0
MARKER_CM = 10.0
FOV_HALF_TAN = 0.6      # tan(31 deg); ~62 degree horizontal field of view

ORBIT_TOL_DEG = 12      # how close to a quarter turn counts as arrived

# Orbiting happens in swings, not one big manoeuvre. Each swing turns away from
# the cube, drives a short way, and then the main loop re-centres and re-ranges
# before the next one -- so an error in any single swing is corrected rather
# than accumulated. That is the whole reason for doing it incrementally.
#
# The swing has to be large: a small turn points the robot AT the cube and
# drives it closer, it does not carry it around. ~45 degrees is roughly
# tangential while still leaving the marker inside the frame.
ORBIT_SWING_DEG = 45
ORBIT_SWING_CM = 20
ORBIT_MAX_SWINGS = 8

# Back off to here before orbiting. The turn radius is ~30cm, so at a 30cm
# stand-off a single swing throws the cube clean out of a 36cm-wide view. At
# 60cm the view is ~72cm and the same swing keeps it comfortably in frame.
ORBIT_STANDOFF_FRAC = 0.14      # ~60cm

TURN_STEP_DEG = 20      # correction turn when the marker is off-centre
# Sweep pattern used when the marker is lost: (degrees, turn_right). Each entry
# undoes the last and extends past it, so the covered arc grows each time --
# +30, then -60 (net -30), then +90 (net +60), and so on.
SWEEP_PLAN = [(30, True), (60, False), (90, True), (120, False), (150, True)]

MIN_CONF = 0.50         # ignore weak detections
MAX_ACTIONS = 30        # hard stop so a bad run cannot loop forever
FACES_TO_TRY = 4


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


# --- perception -------------------------------------------------------------
def capture(path=SNAP):
    """Take one still. Nothing else may hold the camera (stop videostream)."""
    if os.path.exists(path):
        os.remove(path)
    p = subprocess.run(
        ["rpicam-still", "-e", "jpg", "-n", "-t", str(CAPTURE_MS),
         "--width", str(FRAME_W), "--height", str(FRAME_H),
         # Skip the file-exists dance and keep JPEG encoding cheap; quality 80
         # is far above what a 640px detection needs.
         "-q", "80", "-o", path],
        capture_output=True, text=True, timeout=30)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        log(f"  capture failed: {p.stderr.strip()[-160:]}")
        log("  (is videostream/main.py still holding the camera?)")
        return False
    return True


def look():
    """One capture + inference.

    Returns (target, bullseye) where each is None or a dict. The target is a
    real symbol; the bullseye is the obstacle marker, which image-api reports
    separately so it can never be mistaken for a target id.
    """
    if not capture():
        return None, None
    with open(SNAP, "rb") as f:
        try:
            r = requests.post(IMAGE_API,
                              files={"image": ("snap.jpg", f, "image/jpeg")},
                              timeout=90)
        except requests.RequestException as e:
            log(f"  image-api unreachable: {e}")
            return None, None

    # image-api answers 404 when it finds nothing at all -- not an error here.
    if r.status_code == 404:
        return None, None
    if r.status_code != 200:
        log(f"  image-api http {r.status_code}")
        return None, None

    body = r.json()
    bulls = body.get("bullseye")

    target = None
    image_id = body.get("image_id")
    if image_id and image_id != "NA" and float(body.get("conf", 0)) >= MIN_CONF:
        target = {"image_id": image_id, "conf": body.get("conf"),
                  "xyxy": body.get("xyxy")}
    return target, bulls


def distance_cm(box_frac):
    """Rough distance to the marker from its apparent width.

    A MARKER_CM-wide face fills the frame in proportion to how close it is:
    visible width at distance d is 2*d*tan(fov/2), so the fraction the marker
    occupies gives d back directly.
    """
    if box_frac <= 0:
        return None
    return MARKER_CM / (2.0 * FOV_HALF_TAN * box_frac)


def geometry(box):
    """Turn a bounding box into the two numbers the loop steers on.

    box_frac  -- width as a fraction of the frame; our proxy for distance
    offset    -- how far the box centre sits from the frame centre, signed,
                 as a fraction of frame width. Positive means it is to the
                 right, so the robot must turn right.
    """
    x1, _, x2, _ = box
    width = abs(x2 - x1)
    cx = (x1 + x2) / 2.0
    return width / FRAME_W, (cx - FRAME_W / 2.0) / FRAME_W


# --- movement ---------------------------------------------------------------
class Robot:
    def __init__(self, link, dry_run=False):
        self.link = link
        self.dry_run = dry_run

    def do(self, command, why=""):
        label = f"{command:<24} {why}"
        if self.dry_run:
            log(f"  WOULD {label}")
            return True
        log(f"  {label}")
        self.link.send(command)
        reply = self.link.recv()
        if reply is None:
            log("    !! no reply from STM (timed out)")
            return False
        if reply.startswith("ERROR"):
            log(f"    !! STM rejected: {reply}")
            return False
        return True

    def forward(self, units, why=""):
        return self.do(f"center,0,forward,{units}", why)

    def back(self, units, why=""):
        return self.do(f"center,0,reverse,{units}", why)

    def turn(self, deg, right, why=""):
        side = "right" if right else "left"
        return self.do(f"{side},{deg},forward,0", why)

    def next_face(self, distance):
        """Reverse, quarter turn one way, half turn the other.

        With a 30cm turn radius the robot cannot orbit tightly -- it has to use
        the radius rather than fight it. Reversing first, then turning 90 one
        way and 180 the OTHER way, sweeps it round the corner and leaves it
        facing back at the next face:

            reverse B   -> (0, -D-B)
            right 90    -> (r, -D-B+r)    facing away along the face
            left 180    -> (r, -D-B+3r)   facing back at the cube

        It comes level with the cube when -D-B+3r = 0, so the reverse is
        B = 3r - D, computed from the distance the camera measures right now.
        Turning the 180 the same way as the 90 instead lands it past the cube
        staring at empty floor.
        """
        d = distance if distance else 36.0
        back = int(round(3 * TURN_RADIUS_CM - d))
        log(f"    measured {d:.0f}cm -> reverse {back}cm, R90, L180")

        if back > 2:
            if not self.back(back, "(open up for the swing)"):
                return False
        ok = self.turn(90, True, "(quarter turn along the face)")
        return ok and self.turn(180, False, "(half turn back onto the next face)")

    def back_off_to(self, current_cm, wanted_cm):
        """Reverse until roughly `wanted_cm` from the marker."""
        gap = int(round(wanted_cm - current_cm))
        if gap <= 2:
            return True
        return self.back(gap, f"(open up to ~{wanted_cm:.0f}cm to orbit)")


# --- main loop --------------------------------------------------------------
def search(robot):
    """Perceive, decide, act -- until a symbol is found or every face is tried.

    Two modes. Normally the robot closes to reading range and reads the face.
    When a face turns out to be blank it switches to orbiting: repeated swings,
    each followed by re-centring, until the gyro says it has come a quarter of
    the way round. Then it reads again.

    Orbit progress is measured by heading, and only while the marker is centred
    -- with the cube centred in frame, the robot's own heading is a direct proxy
    for where it is standing around the cube.
    """
    faces_tried = 1          # the face we start in front of counts as the first
    misses = 0
    orbiting = False
    orbit_from = None
    swings = 0

    orbit_range = distance_cm(ORBIT_STANDOFF_FRAC)

    for _ in range(MAX_ACTIONS):
        target, bulls = look()

        if target:
            log(f"  >>> FOUND {target['image_id']} conf={target['conf']}")
            robot.do("STOP", "(target found - halt)")
            return target

        if not bulls:
            misses += 1
            log(f"  nothing in view ({misses})")
            if misses > len(SWEEP_PLAN):
                log("  giving up: cannot re-acquire the obstacle")
                return None
            deg, right = SWEEP_PLAN[misses - 1]
            robot.turn(deg, right, f"(sweep {misses}/{len(SWEEP_PLAN)})")
            continue

        misses = 0
        box_frac, offset = geometry(bulls["xyxy"])
        dist = distance_cm(box_frac)
        log(f"  bullseye conf={bulls['conf']:.2f} size={box_frac:.3f} "
            f"offset={offset:+.3f} ~{dist:.0f}cm"
            + ("  [orbiting]" if orbiting else ""))

        # Centre first, always: every other decision assumes the marker is
        # ahead, and orbit progress is only meaningful when it is.
        if abs(offset) > CENTER_TOL_FRAC:
            robot.turn(TURN_STEP_DEG, offset > 0, "(re-centre on the marker)")
            continue

        if orbiting:
            now = robot.link.heading() if not robot.dry_run else None
            if now is None:
                turned = 90.0            # no gyro: trust the swing count
                log("    (no heading; counting swings instead)")
                turned = swings * ORBIT_SWING_DEG
            else:
                turned = abs(robot.link.heading_delta(orbit_from, now))
            log(f"    orbit progress {turned:.0f} of 90 deg ({swings} swings)")

            if turned >= 90 - ORBIT_TOL_DEG:
                log(f"  arrived at face {faces_tried + 1}")
                faces_tried += 1
                orbiting = False
                swings = 0
                continue

            if swings >= 1:
                log("  !! still not round after the manoeuvre; stopping")
                return None
            swings += 1
            robot.next_face(dist)
            continue

        # Not orbiting: hold reading range.
        if box_frac < TARGET_BOX_FRAC - BOX_FRAC_TOL:
            stride = STEP_FAR if box_frac < FAR_BOX_FRAC else STEP_NEAR
            robot.forward(stride, f"(close in, {dist:.0f}cm -> 30cm)")
            continue
        if box_frac > TARGET_BOX_FRAC + BOX_FRAC_TOL * 2:
            robot.back(STEP_NEAR, f"(too close, {dist:.0f}cm)")
            continue

        # Centred, in range, no symbol: this face is blank.
        log(f"  face {faces_tried}: bullseye only, no symbol here")
        if faces_tried >= FACES_TO_TRY:
            log("  checked every face without finding a symbol")
            return None

        orbiting = True
        swings = 0
        orbit_from = robot.link.heading() if not robot.dry_run else 0.0
        log(f"  -- orbiting to the next face (from heading "
            f"{orbit_from if orbit_from is None else f'{orbit_from:+.1f}'}) --")
        continue

    log(f"  stopped after {MAX_ACTIONS} actions without finding a symbol")
    return None


def calibrate():
    """Report what the camera sees, so the constants can be measured."""
    log("calibration -- place the robot where you want it to stop reading")
    target, bulls = look()
    if bulls:
        box_frac, offset = geometry(bulls["xyxy"])
        log(f"  bullseye  conf={bulls['conf']:.2f}  xyxy={bulls['xyxy']}")
        log(f"  box_frac  = {box_frac:.3f}   <-- put this in TARGET_BOX_FRAC")
        log(f"  offset    = {offset:+.3f}    (0 = perfectly centred)")
    else:
        log("  no bullseye detected")
    if target:
        log(f"  target    {target['image_id']} conf={target['conf']}")
    else:
        log("  no target symbol detected")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibrate", action="store_true",
                    help="report what the camera sees and exit")
    ap.add_argument("--dry-run", action="store_true",
                    help="decide and log moves without sending them")
    ap.add_argument("--android", action="store_true",
                    help="report progress to the tablet (stop bt_echo first)")
    args = ap.parse_args()

    if args.calibrate:
        calibrate()
        return 0

    android = None
    if args.android:
        from communication.android import AndroidLink, AndroidMessage
        android = AndroidLink()
        log("waiting for the tablet...")
        android.connect()
        log("tablet connected")

    def tell(cat, value):
        if android:
            from communication.android import AndroidMessage
            try:
                android.send(AndroidMessage(cat, value))
            except OSError:
                pass

    link = STMLink()
    if not args.dry_run:
        link.connect()
        log("STM connected")

    robot = Robot(link, dry_run=args.dry_run)
    try:
        tell("status", "A5: searching for the target image")
        found = search(robot)
        if found:
            tell("image-rec", {"obstacle_id": "1", "image_id": found["image_id"]})
            tell("status", f"Found target {found['image_id']}")
            log(f"SUCCESS: {found['image_id']}")
            return 0
        tell("status", "A5: no target found")
        log("FAILED: no target image found")
        return 1
    finally:
        if not args.dry_run:
            link.disconnect()
        if android:
            android.disconnect()


if __name__ == "__main__":
    sys.exit(main())
