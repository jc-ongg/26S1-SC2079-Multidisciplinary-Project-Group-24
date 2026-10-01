#!/usr/bin/env python3
"""Launch Task 1, or test Android controls using --manual without Algo/CV."""

import argparse


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manual", action="store_true", help="Run Android arrow controls only")
    args = parser.parse_args()

    if args.manual:
        from manual_control import main as run_manual
        run_manual()
    else:
        from final_week_8_backup import RaspberryPi
        RaspberryPi().start()


if __name__ == "__main__":
    main()
