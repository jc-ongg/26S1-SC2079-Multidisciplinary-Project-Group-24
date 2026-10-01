#!/usr/bin/env python3
"""Check this laptop's servers behave exactly like the one that ran Task 1 3/3.

    python laptop_verify.py            (with laptop_start.py running)

Replays recorded inputs from that run (RPi-services/golden/) and compares:

  * path planning: two obstacle layouts the tablet sent on 2026-09-25, against
    the exact stm_commands the robot received
  * recognition: three captures from the 3/3 run (right arrow 38, S 28, 2 12)
    plus a bullseye face, against the ids and confidences returned that day
  * the tiled RAW display (/stitch) that Task 1 needs to qualify

Exit code 0 means everything matched.
"""
import argparse
import json
import sys
import urllib.request
import uuid
from pathlib import Path

GOLDEN = Path(__file__).resolve().parent / "golden"
CONF_TOL = 0.02


def post_text(url, text):
    req = urllib.request.Request(url, data=text.encode(),
                                 headers={"Content-Type": "text/plain"})
    return json.load(urllib.request.urlopen(req, timeout=120))


def post_image(url, path: Path):
    boundary = uuid.uuid4().hex
    body = (f"--{boundary}\r\nContent-Disposition: form-data; name=\"image\"; "
            f"filename=\"{path.name}\"\r\nContent-Type: image/jpeg\r\n\r\n").encode()
    body += path.read_bytes() + f"\r\n--{boundary}--\r\n".encode()
    req = urllib.request.Request(
        url, data=body, headers={"Content-Type": f"multipart/form-data; boundary={boundary}"})
    try:
        return json.load(urllib.request.urlopen(req, timeout=120))
    except urllib.error.HTTPError as e:
        if e.code == 404:                     # image-api's "no objects at all"
            return {"image_id": "NA"}
        raise


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo-port", type=int, default=8000)
    ap.add_argument("--image-port", type=int, default=8001)
    a = ap.parse_args()
    algo = f"http://127.0.0.1:{a.algo_port}"
    img = f"http://127.0.0.1:{a.image_port}"
    fails = 0

    def report(ok, label, detail=""):
        nonlocal fails
        fails += not ok
        print(f"  {'PASS' if ok else 'FAIL'}  {label}" + (f"\n        {detail}" if detail and not ok else ""))

    print("== path planning (Algo) ==")
    for case in json.loads((GOLDEN / "expected_path.json").read_text()):
        try:
            got = post_text(f"{algo}/path", case["payload"])["data"]["stm_commands"]
        except Exception as e:
            got = f"error: {e}"
        report(got == case["stm_commands"], case["payload"][29:70] + "...",
               f"expected {case['stm_commands']}\n        got      {got}")

    print("== recognition (image-api) ==")
    expected = json.loads((GOLDEN / "expected_predict.json").read_text())
    for name, exp in expected.items():
        try:
            r = post_image(f"{img}/predict", GOLDEN / name)
        except Exception as e:
            report(False, name, f"error: {e}")
            continue
        conf = r.get("conf")
        ok = (r.get("image_id") == exp["image_id"]
              and r.get("obstacle_id") == exp["obstacle_id"]
              and bool(r.get("bullseye")) == exp["bullseye"]
              and (exp["conf"] is None or (conf is not None and abs(conf - exp["conf"]) <= CONF_TOL)))
        label = f"{name:22} -> {exp['image_id']}" + (f" / id {exp['obstacle_id']}" if exp["obstacle_id"] else "")
        report(ok, label, f"expected {exp}\n        got      {r}")

    print("== tiled RAW display (image-api /stitch) ==")
    try:
        resp = urllib.request.urlopen(f"{img}/stitch", timeout=60)
        report(resp.headers.get_content_type() == "image/jpeg", "/stitch returns a JPEG")
    except Exception as e:
        report(False, "/stitch returns a JPEG", f"error: {e}")

    print("\n" + ("ALL MATCH -- recorded software responses match; physical calibration is not verified."
                  if not fails else f"{fails} MISMATCH(ES) -- responses differ from the original ONNX baseline."))
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
