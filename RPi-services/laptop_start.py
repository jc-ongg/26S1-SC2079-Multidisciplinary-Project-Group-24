#!/usr/bin/env python3
"""Start the two laptop servers Task 1 needs. Ctrl-C stops both.

    python laptop_start.py

    Algo server  :8000   path planning (POST /path)
    image-api    :8001   recognition (POST /predict) + tiled RAW display (GET /stitch)

Prints this laptop's IP so the Pi can be pointed at it:

    LAPTOP_IP=<that ip> ./run_week8.sh        (on the Pi)

Logs go to RPi-services/logs/.
"""
import argparse
import os
import socket
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

from laptop_setup import ALGO, IMAGE_API, MODEL, venv_python

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"


def up(url):
    try:
        return urllib.request.urlopen(url, timeout=3).status == 200
    except Exception:
        return False


def lan_ips():
    """Best guess at this laptop's LAN address(es)."""
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("192.168.50.5", 80))      # the Pi; no packet is actually sent
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo-port", type=int, default=8000)
    ap.add_argument("--image-port", type=int, default=8001)
    a = ap.parse_args()

    algo_py, img_py = venv_python(ALGO / ".venv"), venv_python(IMAGE_API / "venv")
    for p in (algo_py, img_py, MODEL):
        if not p.exists():
            sys.exit(f"Missing {p} -- run: python laptop_setup.py")

    LOGS.mkdir(exist_ok=True)
    procs = []
    for name, py, cwd, env, health in (
        ("Algo", algo_py, ALGO, {"ALGO_PORT": str(a.algo_port)},
         f"http://127.0.0.1:{a.algo_port}/status"),
        ("image-api", img_py, IMAGE_API, {"IMAGE_API_PORT": str(a.image_port)},
         f"http://127.0.0.1:{a.image_port}/health"),
    ):
        if up(health):
            sys.exit(f"{name}: something is already answering on {health} -- stop it first.")
        log = open(LOGS / f"{name}.log", "w")
        procs.append((name, health, log, subprocess.Popen(
            [str(py), "main.py"], cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
            env={**os.environ, **env, "PYTHONUNBUFFERED": "1"})))
        print(f"starting {name} (log: {LOGS / (name + '.log')})")

    try:
        deadline = time.time() + 180      # first model load can be slow
        pending = {n for n, *_ in procs}
        while pending and time.time() < deadline:
            for name, health, _, p in procs:
                if p.poll() is not None:
                    sys.exit(f"{name} exited early -- see {LOGS / (name + '.log')}")
                if name in pending and up(health):
                    print(f"  {name} up: {health}")
                    pending.discard(name)
            time.sleep(1)
        if pending:
            sys.exit(f"timed out waiting for: {', '.join(pending)}")

        ips = lan_ips()
        print("\nReady. This laptop's address(es):", ", ".join(ips) or "unknown")
        pi = [ip for ip in ips if ip.startswith("192.168.50.")]
        print("On the Pi:  LAPTOP_IP=%s ./run_week8.sh" % (pi[0] if pi else "<this laptop's 192.168.50.x ip>"))
        if not pi:
            print("  (not on the robot's hotspot yet -- join MDP-group-24 first)")
        print(f"After a run: http://127.0.0.1:{a.image_port}/stitch")
        print("Ctrl-C to stop.\n")
        while all(p.poll() is None for *_, p in procs):
            time.sleep(1)
        for name, _, _, p in procs:
            if p.poll() is not None:
                print(f"{name} stopped unexpectedly -- see {LOGS / (name + '.log')}")
    except KeyboardInterrupt:
        pass
    finally:
        for _, _, log, p in procs:
            if p.poll() is None:
                p.terminate()
        for _, _, log, p in procs:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                p.kill()
            log.close()
        print("stopped")


if __name__ == "__main__":
    main()
