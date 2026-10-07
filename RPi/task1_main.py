#!/usr/bin/env python3
"""Launch Task 1, or test Android controls using --manual without Algo/CV."""

import argparse
import json
import urllib.request


def check_profile():
    """Verify planner/speed pairing before opening Bluetooth or serial."""
    from settings import API_IP, API_PORT
    with urllib.request.urlopen(f"http://{API_IP}:{API_PORT}/status", timeout=5) as response:
        status = json.load(response)
    required = {'status': 'ok', 'movement_profile': 'turn-guide-speed50-grid5',
                'recommended_motor_speed': 50, 'grid_cell_cm': 5,
                'input_cell_cm': 10, 'arena_cells': 40}
    for key, expected in required.items():
        if status.get(key) != expected:
            raise RuntimeError(f"Algo {key} is {status.get(key)!r}; expected {expected!r}. Restart the standard laptop launcher.")
    return 50


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", action="store_true", help="Run Android arrow controls only")
    args = parser.parse_args()

    if args.manual:
        from manual_control import main as run_manual
        run_manual()
    else:
        try:
            speed = check_profile()
        except (OSError, ValueError, RuntimeError) as exc:
            parser.exit(1, f"Task 1 not started: {exc}\n")
        from task1_controller import RaspberryPi
        robot = RaspberryPi()
        robot.stm_link.speed = speed
        print("Task 1: motor speed 50; 5 cm planner; 10 cm tablet map", flush=True)
        robot.start()


if __name__ == "__main__":
    main()
