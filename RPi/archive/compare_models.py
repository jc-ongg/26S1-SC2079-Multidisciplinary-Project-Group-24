#!/usr/bin/env python3
"""Capture one frame and run it through BOTH models for comparison.

    python3 compare_models.py            # capture fresh
    python3 compare_models.py some.jpg   # reuse an image

Point the camera at a symbol 20-50cm away (checklist A.2 distance) and run it.
Whichever model reads your arrows and digits more reliably is the one to use.
"""
import os, subprocess, sys, time
import requests
from settings import API_IP

ALGO = f"http://{API_IP}:8000/infer"          # best.pt
IMGAPI = f"http://{API_IP}:8001/predict"      # best_v11i_m.onnx
SNAP = "/tmp/compare.jpg"


def capture(path):
    if os.path.exists(path):
        os.remove(path)
    p = subprocess.run(["rpicam-still", "-e", "jpg", "-n", "-t", "900", "-o", path],
                       capture_output=True, text=True, timeout=60)
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        print("capture FAILED:", p.stderr.strip()[-200:])
        print("(is videostream/main.py still holding the camera?)")
        return False
    print(f"captured {os.path.getsize(path)} bytes")
    return True


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else SNAP
    if len(sys.argv) == 1 and not capture(path):
        return 1

    print("\n=== ALGO  best.pt  (conf>=0.30) ===")
    try:
        with open(path, "rb") as f:
            t0 = time.time()
            r = requests.post(ALGO, files={"file": ("f.jpg", f, "image/jpeg")}, timeout=90)
        d = r.json()
        print(f"  http {r.status_code}  {time.time()-t0:.2f}s  n={d.get('n')}")
        for det in d.get("detections", []):
            print(f"    id={det.get('cls_id')} {det.get('cls_name'):<8} "
                  f"conf={det.get('conf')} xyxy={det.get('xyxy')}")
        if not d.get("detections"):
            print("    (nothing detected)")
    except Exception as e:
        print("  FAILED:", e)

    print("\n=== IMAGE-API  best_v11i_m.onnx  (conf>0.5) ===")
    try:
        with open(path, "rb") as f:
            t0 = time.time()
            r = requests.post(IMGAPI, files={"image": ("f.jpg", f, "image/jpeg")}, timeout=90)
        d = r.json()
        print(f"  http {r.status_code}  {time.time()-t0:.2f}s")
        print("   ", str(d)[:400])
    except Exception as e:
        print("  FAILED:", e)

    print("\nUse whichever reads your symbols more reliably at 20-50cm.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
