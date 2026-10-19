#!/usr/bin/env python3
"""Live check of the START O1 range rule, robot standing still.

Reads the STM32's !SENSOR frames (no motor commands), takes the ultrasonic
readings in blocks of 12 and prints, for every block:
  - the 12 readings,
  - the firmware's new pick (v19b-fix: farthest group of >= 3 within 4 cm,
    its median; no group = it would take 12 fresh readings),
  - what v19b's old rule would have picked (first 3 agreeing within 3 cm).
A running tally shows how often each pick came out. Stop with Ctrl+C.

The Pi sees !SENSOR at ~10 Hz, each carrying the latest echo, so a block is
close to, but not exactly, the 12 echoes the firmware itself collects.
Stop the bridge first: this needs the serial port to itself.

    python3 KC_o1_range_watch.py --port /dev/ttyACM0
"""

import argparse
from collections import Counter
from datetime import datetime
import statistics

import serial

N = 12          # KCF_RANGE_N
AGREE = 3       # KCF_RANGE_AGREE
TOL = 4.0       # KCF_RANGE_TOL


def new_pick(v):
    """Farthest reading with >= AGREE readings within TOL; median of them."""
    best = None
    for a in v:
        n = sum(abs(b - a) <= TOL for b in v)
        if n >= AGREE and (best is None or a > best):
            best = a
    if best is None:
        return None, 0
    group = [b for b in v if abs(b - best) <= TOL]
    return statistics.median(group), len(group)


def old_pick(v):
    """v19b kc_us_group: 3 in a row within 3 cm of the first; median of 3."""
    group = []
    for u in v:
        if group and abs(u - group[0]) > 3:
            group = []
        group.append(u)
        if len(group) == 3:
            return statistics.median(group)
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyACM0')
    args = ap.parse_args()
    block, buf = [], bytearray()
    tally_new, tally_old, blocks = Counter(), Counter(), 0
    print('Robot still, facing O1. Ctrl+C to stop. No motor commands are sent.', flush=True)
    try:
        with serial.Serial(args.port, 115200, timeout=0.2, exclusive=True) as link:
            while True:
                buf.extend(link.read(256))
                while b';' in buf:
                    raw, _, tail = buf.partition(b';')
                    buf = bytearray(tail)
                    i = raw.find(b'!SENSOR/')
                    if i < 0:
                        continue
                    p = raw[i:].decode('ascii', errors='replace').split('/')
                    try:
                        us, valid = float(p[3]), int(p[4]) == 1
                    except (IndexError, ValueError):
                        continue
                    if not valid or not 2 < us <= 180:
                        continue
                    block.append(us)
                    if len(block) < N:
                        continue
                    blocks += 1
                    pick, votes = new_pick(block)
                    old = old_pick(block)
                    tally_new['redo (no group)' if pick is None else f'{round(pick):d} cm'] += 1
                    tally_old['none' if old is None else f'{round(old):d} cm'] += 1
                    t = f'{datetime.now():%H:%M:%S}'
                    readings = ' '.join(f'{u:.0f}' for u in block)
                    new_text = 'REDO: no group of 3, would take 12 fresh' if pick is None else f'{pick:6.1f} cm ({votes}/12 agree)'
                    old_text = '--' if old is None else f'{old:6.1f} cm'
                    print(f'[{t}] #{blocks:3d}  readings: {readings}', flush=True)
                    print(f'            NEW pick: {new_text}   |   old v19b pick: {old_text}', flush=True)
                    print(f'            tally NEW: {dict(tally_new.most_common(4))}   OLD: {dict(tally_old.most_common(4))}', flush=True)
                    block = []
                if len(buf) > 4096:
                    buf.clear()
    except KeyboardInterrupt:
        print(f'\nStopped after {blocks} blocks. NEW: {dict(tally_new)}  OLD: {dict(tally_old)}')


if __name__ == '__main__':
    main()
