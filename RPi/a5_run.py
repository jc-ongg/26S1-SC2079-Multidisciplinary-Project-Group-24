#!/usr/bin/env python3
"""Checklist A.5 -- navigate around an obstacle until a valid image is found.

    python3 a5_run.py              # drive the robot
    python3 a5_run.py --dry-run    # print the plan, move nothing
    python3 a5_run.py --android    # also report to the tablet (stop bt_echo first)

This is the one place where recognition actually steers the robot. The main
Week 8 path task never does: there the Algo server plans the whole route up
front and image results only go to Android. Here the result decides whether to
pivot to the next face.

    A5_Initial  ->  drive up, SNAP
      valid image?  -> report and stop
      nothing?      -> A5_Pivot -> SNAP -> repeat

The obstacle's other faces carry a bullseye, which the server's CLS_MAP
deliberately drops, so a bullseye face comes back with no detections at all --
indistinguishable from an empty frame, and in both cases the right move is to
pivot.
"""
import argparse
import json
import os
import subprocess
import sys
import time

import requests

from settings import API_IP, API_PORT, API_IP_IMAGE, API_PORT_IMAGE
from communication.stm32 import STMLink

BASE = f"http://{API_IP}:{API_PORT}"                   # Algo: path planning
IMAGE_BASE = f"http://{API_IP_IMAGE}:{API_PORT_IMAGE}"  # image-api: recognition
SNAP_PATH = "/tmp/a5_snap.jpg"

# Confidence floor for accepting a detection. The server already filters at
# 0.30; this guards against a marginal box sending us home with the wrong id.
MIN_CONF = 0.50

# How many faces to try before giving up. The obstacle has four.
MAX_PIVOTS = 4


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def fetch_sequence(name):
    """Ask the Algo server for a hardcoded A5 sequence.

    convert_input() matches on the text before the first '-', so the body is
    just the bare name.
    """
    r = requests.post(f"{BASE}/path", data=name,
                      headers={"Content-Type": "text/plain"}, timeout=30)
    r.raise_for_status()
    body = r.json()
    if body.get("error"):
        raise RuntimeError(f"Algo error for {name}: {body['error']}")
    return body["data"]["stm_commands"]


def capture(path=SNAP_PATH):
    """Grab a still. Returns True on success.

    libcamera-still does not exist on this OS -- it is rpicam-still. Nothing
    else may hold the camera: videostream/main.py must be stopped or this
    fails with "Device or resource busy".
    """
    if os.path.exists(path):
        os.remove(path)
    proc = subprocess.run(
        ["rpicam-still", "-e", "jpg", "-n", "-t", "800", "-o", path],
        capture_output=True, text=True, timeout=60)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        log(f"  capture failed (rc={proc.returncode}): {proc.stderr.strip()[-200:]}")
        return False
    return True


def infer(path=SNAP_PATH):
    """POST the image and return the best detection, or None.

    image-api wants the field called "image" and answers with a single best
    guess: {image_id, conf, xyxy}. It uses {'image_id': 'NA'} for "nothing
    recognised" and a bare HTTP 404 for "no objects detected" -- both mean the
    same thing to us, so neither is treated as an error.
    """
    with open(path, "rb") as f:
        r = requests.post(IMAGE_BASE + "/predict",
                          files={"image": (os.path.basename(path), f, "image/jpeg")},
                          timeout=90)
    if r.status_code == 404:
        log("  /predict -> no objects detected")
        return None
    r.raise_for_status()
    body = r.json()
    image_id = body.get("image_id")
    log(f"  /predict -> image_id={image_id} conf={body.get('conf')}")
    if not image_id or image_id == "NA":
        return None
    if float(body.get("conf", 0)) < MIN_CONF:
        log(f"  conf {body.get('conf')} below {MIN_CONF}, ignoring")
        return None
    # Normalise to the shape the rest of this script expects.
    return {"cls_id": image_id, "cls_name": image_id,
            "conf": body.get("conf"), "xyxy": body.get("xyxy")}


def run_sequence(commands, link, dry_run=False):
    """Execute one sequence. Returns a detection dict if a SNAP found one."""
    found = None
    for cmd in commands:
        if cmd.startswith("SNAP"):
            obstacle_id = cmd.replace("SNAP", "").split("_")[0] or "1"
            log(f"  SNAP (obstacle {obstacle_id})")
            if dry_run:
                continue
            if not capture():
                continue
            det = infer()
            if det:
                det["obstacle_id"] = obstacle_id
                found = det
                log(f"  >>> DETECTED {det['cls_id']} ({det['cls_name']}) "
                    f"conf={det['conf']}")
            else:
                log("  no valid image on this face")
            continue

        if cmd == "FIN":
            continue

        frame = link.translate(cmd) if dry_run else None
        if dry_run:
            log(f"  {cmd:<26} -> {frame}")
            continue

        log(f"  {cmd}")
        link.send(cmd)
        reply = link.recv()
        if reply is None:
            log("  !! no reply from STM (timed out) -- aborting sequence")
            return found
        if reply.startswith("ERROR"):
            log(f"  !! STM rejected it: {reply} -- aborting sequence")
            return found
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true",
                    help="print the plan without moving")
    ap.add_argument("--android", action="store_true",
                    help="report progress to the tablet (stop bt_echo first)")
    ap.add_argument("--pivots", type=int, default=MAX_PIVOTS)
    args = ap.parse_args()

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
            except OSError as e:
                log(f"  (android send failed: {e})")

    link = STMLink()
    if not args.dry_run:
        link.connect()
        log("STM connected")

    try:
        tell("status", "A5: approaching obstacle")
        log("=== A5_Initial ===")
        found = run_sequence(fetch_sequence("A5_Initial"), link, args.dry_run)

        pivots = 0
        while not found and pivots < args.pivots:
            pivots += 1
            tell("status", f"A5: looking for target, face {pivots + 1}")
            log(f"=== A5_Pivot ({pivots}/{args.pivots}) ===")
            found = run_sequence(fetch_sequence("A5_Pivot"), link, args.dry_run)

        if found:
            log(f"SUCCESS after {pivots} pivot(s): "
                f"image {found['cls_id']} ({found['cls_name']}) "
                f"on obstacle {found['obstacle_id']}")
            tell("image-rec", {"obstacle_id": found["obstacle_id"],
                              "image_id": found["cls_id"]})
            tell("status", f"Found target {found['cls_id']}")
            return 0

        log(f"no valid image found after {pivots} pivot(s)")
        tell("status", "A5: no target found")
        return 1
    finally:
        if not args.dry_run:
            link.disconnect()
        if android:
            android.disconnect()


if __name__ == "__main__":
    sys.exit(main())
