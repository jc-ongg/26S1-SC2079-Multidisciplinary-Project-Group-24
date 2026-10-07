#!/usr/bin/env python3
"""Checklist A.5, following the pattern the seniors had working.

    python3 a5_senior.py             # run
    python3 a5_senior.py --dry-run   # print the plan, move nothing

Their version, translated to this robot's command set:

    drive forward until the range-finder reads <= 40cm, then stop
    reverse 50                      <- all turning happens with room to spare
    turn left 90 (forward), forward 35
    per face:
        turn right 90 (forward), forward 30, turn right 90 (forward)
        photograph
        turn right 90 (REVERSE), forward 75

Two things in that which my own attempts got wrong:

  * every turn happens well back from the cube, not at reading distance. A
    turn on this robot arcs ~30cm, so turning up close always carries it past
    the obstacle -- which is exactly what kept happening.
  * the turns alternate forward and reverse. A reverse turn arcs the opposite
    way, so the sideways drift from the forward turns is cancelled rather than
    accumulated.

The approach phase is closed-loop on the range-finder, so the cube can be moved
and the robot will still stop the right distance from it. The circling legs are
the seniors' measured numbers; they assume their turn radius, so if this robot
differs the legs are what to adjust -- start with FACE_LEG and RETURN_LEG.
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
SNAP = "/tmp/a5_senior.jpg"

# --- approach ---------------------------------------------------------------
STOP_AT_CM = 40.0       # seniors' threshold
APPROACH_STEP = 10      # drive in this far between range checks
MAX_APPROACH_STEPS = 20

# --- the circling legs (seniors' numbers) -----------------------------------
BACK_OFF = 50
ENTRY_LEG = 35
FACE_LEG = 30
RETURN_LEG = 75

MIN_CONF = 0.50
FACES = 4


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def photograph():
    """Capture and classify. Returns (target, saw_bullseye)."""
    if os.path.exists(SNAP):
        os.remove(SNAP)
    subprocess.run(
        ["rpicam-still", "-e", "jpg", "-n", "-t", "250",
         "--width", "640", "--height", "480", "-q", "80", "-o", SNAP],
        capture_output=True, timeout=30)
    if not os.path.exists(SNAP) or os.path.getsize(SNAP) == 0:
        log("    capture failed (is videostream holding the camera?)")
        return None, False
    with open(SNAP, "rb") as f:
        try:
            r = requests.post(IMAGE_API,
                              files={"image": ("s.jpg", f, "image/jpeg")},
                              timeout=60)
        except requests.RequestException as e:
            log(f"    image-api unreachable: {e}")
            return None, False
    if r.status_code == 404:
        return None, False
    if r.status_code != 200:
        log(f"    image-api http {r.status_code}")
        return None, False
    body = r.json()
    saw_bullseye = bool(body.get("bullseye"))
    iid = body.get("image_id")
    if iid and iid != "NA" and float(body.get("conf", 0)) >= MIN_CONF:
        return {"image_id": iid, "conf": body.get("conf")}, saw_bullseye
    return None, saw_bullseye


class Robot:
    def __init__(self, link, dry_run=False):
        self.link = link
        self.dry_run = dry_run

    def cmd(self, command, why=""):
        if self.dry_run:
            log(f"  WOULD {command:<26} {why}")
            return True
        log(f"  {command:<26} {why}")
        self.link.send(command)
        reply = self.link.recv()
        if reply is None:
            log("    !! no reply from STM")
            return False
        if reply.startswith("ERROR"):
            log(f"    !! {reply}")
            return False
        return True

    def range_cm(self):
        """Forward range-finder (telemetry field 3), or None if implausible."""
        if self.dry_run:
            return None
        s = self.link.read_sensors()
        if not s or len(s) < 3:
            return None
        try:
            v = float(s[2])
        except ValueError:
            return None
        return v if 5.0 < v < 250.0 else None


def approach(bot):
    """Close in until the range-finder says we are at the stopping distance.

    Closed-loop, so moving the cube mid-approach just means more (or fewer)
    steps -- there is no fixed drive distance anywhere in here.
    """
    log(f"approach: driving in until <= {STOP_AT_CM:.0f}cm")
    for i in range(MAX_APPROACH_STEPS):
        d = bot.range_cm()
        if d is None:
            log("  no range reading; assuming we are close enough")
            return True
        log(f"  range {d:.0f}cm")
        if d <= STOP_AT_CM:
            bot.cmd("STOP", "(at stopping distance)")
            return True
        step = APPROACH_STEP if d - STOP_AT_CM > APPROACH_STEP else int(d - STOP_AT_CM)
        if step < 2:
            bot.cmd("STOP", "(close enough)")
            return True
        if not bot.cmd(f"center,0,forward,{step}", f"(close in from {d:.0f}cm)"):
            return False
    log("  gave up approaching")
    return False


def run(bot):
    if not approach(bot):
        return None

    log("setting up for the circuit")
    bot.cmd(f"center,0,reverse,{BACK_OFF}", "(back off to give the turns room)")
    bot.cmd("left,90,forward,0", "(turn to run along the first face)")
    bot.cmd(f"center,0,forward,{ENTRY_LEG}", "(move to the corner)")

    for face in range(1, FACES + 1):
        log(f"--- face {face} ---")
        bot.cmd("right,90,forward,0", "(turn across the corner)")
        bot.cmd(f"center,0,forward,{FACE_LEG}", "(come level with the face)")
        bot.cmd("right,90,forward,0", "(turn to look at it)")

        target, bullseye = photograph()
        log(f"    saw: target={target} bullseye={bullseye}")
        if target:
            log(f"  >>> FOUND {target['image_id']} conf={target['conf']}")
            bot.cmd("STOP", "(halt)")
            return target

        # Reverse turn here: it arcs the opposite way and cancels the drift the
        # two forward turns above introduced.
        bot.cmd("right,90,reverse,0", "(reverse turn back out)")
        bot.cmd(f"center,0,forward,{RETURN_LEG}", "(travel to the next corner)")

    log("  circled every face without finding a symbol")
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    link = STMLink()
    if not args.dry_run:
        link.connect()
        log("STM connected")
    bot = Robot(link, dry_run=args.dry_run)
    try:
        found = run(bot)
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
