#!/usr/bin/env python3
"""Start laptop Algo and an explicitly chosen CV backend. Never starts the Pi.

Use --check for read-only checks. Ctrl-C stops only children started here.
"""
import argparse
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.request

from laptop_setup import ALGO, IMAGE_API, MODEL, venv_python

HERE = Path(__file__).resolve().parent
LOGS = HERE / "logs"


def up(url):
    try:
        with urllib.request.urlopen(url, timeout=3) as response:
            return response.status == 200
    except Exception:
        return False


def port_busy(port):
    with socket.socket() as probe:
        probe.settimeout(1)
        return probe.connect_ex(("127.0.0.1", port)) == 0


def lan_ips():
    ips = set()
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect(("192.168.50.5", 80))  # Route lookup, no packet sent.
            ips.add(probe.getsockname()[0])
        ips.update(info[4][0] for info in socket.getaddrinfo(
            socket.gethostname(), None, socket.AF_INET))
    except OSError:
        pass
    return sorted(ip for ip in ips if not ip.startswith("127."))


def service_specs(args):
    shared = Path(args.python).resolve() if args.python else None
    algo_py = shared or venv_python(ALGO / ".venv")
    image_py = shared or venv_python(IMAGE_API / "venv")
    trial = args.cv_backend == "trial"
    model = ALGO / "best.pt" if trial else MODEL
    cv_command = ([str(image_py), "cv_trial_server.py", "--port", str(args.image_port)]
                  if trial else [str(image_py), "main.py"])
    specs = [
        ("Algo", [str(algo_py), "main.py"], ALGO,
         {"ALGO_PORT": str(args.algo_port), "MDP_MOVEMENT_PROFILE": args.movement_profile,
          "MDP_REVERSE_CALIBRATION": args.reverse_calibration,
          "MDP_FORWARD_CALIBRATION": args.forward_calibration}, args.algo_port, "/status"),
        ("cv-trial" if trial else "image-api", cv_command, ALGO if trial else IMAGE_API,
         {"IMAGE_API_PORT": str(args.image_port)}, args.image_port, "/health"),
    ]
    return specs, (algo_py, image_py, model)


def stop_children(procs):
    for _, _, _, proc in procs:
        if proc.poll() is None:
            proc.terminate()
    for _, _, log, proc in procs:
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=10)
        finally:
            log.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--algo-port", type=int, default=8000)
    ap.add_argument("--image-port", type=int, default=8001)
    ap.add_argument("--cv-backend", choices=("onnx", "trial"), default="onnx")
    ap.add_argument("--movement-profile", choices=("original", "measured25"), default="original")
    ap.add_argument("--reverse-calibration", choices=("original", "speed50-v2"), default="original")
    ap.add_argument("--forward-calibration", choices=("original", "speed50-v2"), default="original")
    ap.add_argument("--python", help="Existing Python environment for both services")
    ap.add_argument("--check", action="store_true", help="Check files/health only; start nothing")
    args = ap.parse_args()
    if args.movement_profile != "original" and (args.reverse_calibration != "original" or args.forward_calibration != "original"):
        ap.error("speed50-v2 calibration requires --movement-profile original")
    if args.algo_port == args.image_port or not all(
            1 <= port <= 65535 for port in (args.algo_port, args.image_port)):
        ap.error("Choose two different ports in 1..65535")
    specs, required = service_specs(args)
    missing = [path for path in set(required) if not path.is_file()]
    for path in missing:
        print(f"Missing: {path}")
    print(f"CV backend: {args.cv_backend}" + (" (temporary best.pt model)" if args.cv_backend == "trial" else ""))
    print(f"Movement profile: {args.movement_profile}; motor speed: {25 if args.movement_profile == 'measured25' else 50}")
    print(f"Reverse calibration: {args.reverse_calibration}")
    print(f"Forward calibration: {args.forward_calibration}")
    for name, _, _, _, port, health in specs:
        print(f"{name}: {'healthy' if up(f'http://127.0.0.1:{port}{health}') else 'not healthy'} on :{port}")
    ips = lan_ips()
    print("Laptop IPs:", ", ".join(ips) or "unknown")
    if args.check:
        return 1 if missing else 0
    if missing:
        return 1
    # Check BOTH ports before creating ANY child, including unhealthy listeners.
    if any(port_busy(spec[4]) for spec in specs):
        print("A requested port is occupied. Stop your existing servers yourself; nothing started.")
        return 1
    LOGS.mkdir(exist_ok=True)
    procs = []
    try:
        for name, command, cwd, env, port, health in specs:
            log = open(LOGS / f"{name}.log", "a", encoding="utf-8")
            try:
                proc = subprocess.Popen(
                    command, cwd=cwd, stdout=log, stderr=subprocess.STDOUT,
                    env={**os.environ, **env, "PYTHONUNBUFFERED": "1"},
                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            except BaseException:
                log.close()
                raise
            procs.append((name, f"http://127.0.0.1:{port}{health}", log, proc))
            print(f"Starting {name}; log: {LOGS / (name + '.log')}", flush=True)
        deadline = time.monotonic() + 180
        pending = {name for name, *_ in procs}
        while pending and time.monotonic() < deadline:
            for name, health, _, proc in procs:
                if proc.poll() is not None:
                    raise RuntimeError(f"{name} exited; check its log")
                if name in pending and up(health):
                    pending.remove(name)
            time.sleep(1)
        if pending:
            raise RuntimeError(f"Startup timed out: {pending}")
        pi_ips = [ip for ip in ips if ip.startswith("192.168.50.")]
        host = pi_ips[0] if pi_ips else "<laptop Wi-Fi IP>"
        option = " --calibrated" if args.movement_profile == "measured25" else ""
        print(f"Ready. On the Pi, YOU can run: LAPTOP_IP={host} python3 -u task1_main.py{option}", flush=True)
        print(f"CV images: http://127.0.0.1:{args.image_port}/stitch", flush=True)
        print("Ctrl-C stops these laptop servers only.", flush=True)
        while all(proc.poll() is None for *_, proc in procs):
            time.sleep(1)
        raise RuntimeError("A server stopped unexpectedly; check its log")
    except KeyboardInterrupt:
        return 0
    except (OSError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1
    finally:
        stop_children(procs)


if __name__ == "__main__":
    sys.exit(main())
