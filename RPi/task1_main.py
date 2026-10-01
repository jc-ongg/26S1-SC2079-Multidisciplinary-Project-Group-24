#!/usr/bin/env python3
"""Launch Task 1, or test Android controls using --manual without Algo/CV."""

import argparse
import json
import urllib.request


def check_profile(calibrated):
    """Verify planner/speed pairing before opening Bluetooth or serial."""
    from settings import API_IP, API_PORT
    with urllib.request.urlopen(f"http://{API_IP}:{API_PORT}/status", timeout=5) as response:
        status = json.load(response)
    expected = "measured25" if calibrated else "original"
    actual = status.get("movement_profile")
    if actual != expected and (calibrated or actual is not None):
        raise RuntimeError(f"Algo profile is {actual!r}; expected {expected!r}. Restart Algo with the matching profile.")
    if status.get("status") != "ok":
        raise RuntimeError("Algo status is not ok")
    return 25 if calibrated else 50


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", action="store_true", help="Run Android arrow controls only")
    parser.add_argument("--calibrated", action="store_true", help="Trial measured25 turns at motor speed 25; requires matching Algo")
    args = parser.parse_args()
    if args.manual and args.calibrated:
        parser.error("--calibrated is for Task 1, not --manual")

    if args.manual:
        from manual_control import main as run_manual
        run_manual()
    else:
        try:
            speed = check_profile(args.calibrated)
        except (OSError, ValueError, RuntimeError) as exc:
            parser.exit(1, f"Task 1 not started: {exc}\n")
        from final_week_8_backup import RaspberryPi
        robot = RaspberryPi()
        robot.stm_link.speed = speed
        print(f"Task 1 profile: {'measured25' if args.calibrated else 'original'}; motor speed: {speed}", flush=True)
        robot.start()


if __name__ == "__main__":
    main()
