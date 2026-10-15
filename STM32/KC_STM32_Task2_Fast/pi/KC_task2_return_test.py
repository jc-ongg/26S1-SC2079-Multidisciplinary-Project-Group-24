#!/usr/bin/env python3
"""Test the two-step carpark return on its own, from a hand-placed start.

Place the robot beside O2's end, facing the carpark, as it would be after
O2's return turn. Reset the STM32 with the robot already there and still: the
gyro zero is then "facing the carpark". Key in the course (or pass the
flags), check the printed plan, press Enter, and the robot drives home with
KC_task2_pi_return (shift in front of O2, pass O1 with the side IR, enter,
park). Ctrl+C sends STOP.

Course frame (as the firmware): origin = robot centre at the carpark start,
+X = right looking out of the carpark toward O1, +Y = out of the carpark.

    python3 KC_task2_return_test.py
    python3 KC_task2_return_test.py --o1-dist 62 --gap 70 --o2-length 60 --side left --offset 52 --past 5
"""
import argparse
import json
import queue
import threading
import time
from pathlib import Path

import serial

from KC_task2_pi_return import (IR_AHEAD, PASS_SHIFT_MIN, Poses, PiReturn, ReturnError,
                                choose_entry, choose_shift, overlap)
from KC_task2_stm32_bridge import BridgeIO


def ask(value, prompt, kind=float, choices=None):
    while value is None:
        text = input(prompt).strip().lower()
        try:
            value = kind(text)
        except ValueError:
            continue
        if choices and value not in choices:
            value = None
    return value


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default='/dev/ttyACM0')
    ap.add_argument('--o1-dist', type=float, help='at the carpark start: front bumper to O1 front face (cm)')
    ap.add_argument('--gap', type=float, help="O1's back face to O2's front face (cm)")
    ap.add_argument('--o2-length', type=float, help="O2's length across the course (cm)")
    ap.add_argument('--side', choices=('left', 'right'),
                    help='side of the centre line the robot is on, looking OUT of the carpark toward O1')
    ap.add_argument('--offset', type=float, help="centre line to the robot's centre (cm)")
    ap.add_argument('--past', type=float,
                    help="O2's front face to the robot's front bumper, toward the carpark (cm; negative = still beside O2)")
    ap.add_argument('--yes', action='store_true', help='do not wait for Enter')
    args = ap.parse_args(argv)

    print('Course, in cm. Robot beside O2\'s end, facing the carpark.')
    o1_dist = ask(args.o1_dist, 'Carpark start: front bumper to O1 front face: ')
    gap = ask(args.gap, "O1 back face to O2 front face: ")
    length = ask(args.o2_length, "O2 length: ")
    side = ask(args.side, "Robot is on which side of the centre line, looking out of the carpark (left/right): ",
               str, ('left', 'right'))
    offset = ask(args.offset, "Centre line to robot centre: ")
    past = ask(args.past, "O2 front face to robot front bumper, toward the carpark: ")

    o1 = o1_dist + 11.5                 # firmware: o1_front = US range + 11.5 from the start centre
    o2 = o1 + 10 + gap
    x = offset if side == 'right' else -offset
    y = o2 - past + 11.5                # facing home: front bumper 11.5 nearer the carpark
    o2h = length / 2 + 2                # 2 cm for tape and placement
    facts = {'origin': 0.0, 'o1_front': o1, 'o2_front': o2, 'x': x, 'y': y, 'o2_half': o2h}
    print(f'\nO1 front y={o1:.1f}, O2 front y={o2:.1f}, O2 half length {o2h:.1f}; robot x={x:.1f} y={y:.1f}')

    # Placement checks with the same 5 cm sweep margin the route uses.
    import numpy as np
    here = (np.array([x]), np.array([y]), np.array([-1.0]), np.array([0.0]))
    if np.any(overlap(*here, (-o2h, o2, o2h, o2 + 10))):
        print('WARNING: the robot is within 5 cm of O2 as keyed in; the shift may not start.')
    if abs(x) < 18:
        raise SystemExit('The robot must be at least 18 cm from the centre line (O1).')
    if abs(x) >= PASS_SHIFT_MIN:
        plan = choose_shift(x, y, o1, o2, o2h)
        if plan:
            lane, d, sy = plan
            end = Poses(x, lane, d).end(0)
            print(f'Plan: drive to y={sy:.1f}, S-curve (diagonal {d:.0f}) into lane x={lane:.1f}, '
                  f'ending at y={sy + end[1]:.1f}')
            ex, ey = lane, sy + end[1]
        else:
            print('Plan: no shift fits before O1, keeping this lane')
            ex, ey = x, y
    else:
        print('Plan: lane kept (close enough for the side IR to see O1)')
        ex, ey = x, y
    entry = choose_entry(ex, min(ey, o2 - 18.5), y, o1, o2)
    if entry is None:
        print('WARNING: no carpark entry fits from that lane (the run will stop there)')
    else:
        print(f'Then pass O1 at speed 30 and turn in from about y={entry[1]:.1f} (diagonal {entry[0]:.0f}).')

    link = serial.Serial(args.port, 115200, timeout=0.1, write_timeout=1, exclusive=True)
    incoming, cancelled, lock = queue.Queue(), threading.Event(), threading.Lock()
    state = {'id': int(time.time()) % 100000 + 20000, 'last': 0.0}

    def reader():
        pending = bytearray()
        while not cancelled.is_set():
            try:
                pending.extend(link.read(256))
            except Exception as exc:
                incoming.put(exc)
                return
            while b';' in pending:
                end = pending.index(b';')
                raw = bytes(pending[:end + 1]).decode('ascii', errors='replace')
                del pending[:end + 1]
                if '!' in raw:
                    incoming.put(raw[raw.index('!'):])

    def send(component, command, first, second):
        with lock:
            delay = 0.03 - (time.monotonic() - state['last'])
            if delay > 0:
                time.sleep(delay)
            state['id'] += 1
            link.write(f":{state['id']}/{component}/{command}/{first}/{second};".encode('ascii'))
            link.flush()
            state['last'] = time.monotonic()
            return state['id']

    threading.Thread(target=reader, daemon=True).start()
    log_path = Path(f'KC_return_test_{time.time_ns()}.jsonl')
    try:
        with log_path.open('w') as log:
            io = BridgeIO(send, incoming, log)
            log.write(json.dumps({'time': time.time(), 'frame': f'facts {facts}'}) + '\n')
            frame = io.sensors(3)[-1]
            print(f'STM32 telemetry ok: heading {frame["heading"]:.1f}, IR L {frame["left"]:.1f} R {frame["right"]:.1f}, '
                  f'US {frame["us"]:.1f}')
            if abs(frame['heading']) > 3:
                print('WARNING: heading is not ~0. Reset the STM32 with the robot placed and still.')
            # The gyro zero faces the carpark: task heading 180 = sensor 0.
            facts['origin'] = -180.0
            if not args.yes:
                input('\nPress Enter to drive home (Ctrl+C to cancel) ')
            PiReturn(io, facts).run()
            print(f'Done. Log: {log_path}')
    except KeyboardInterrupt:
        print('STOP requested.')
    except ReturnError as exc:
        print(f'Return stopped: {exc}. Log: {log_path}')
    finally:
        cancelled.set()
        try:
            with lock:
                link.write(f":{state['id'] + 1}/MOTOR/STOP/0/0;".encode('ascii'))
        except Exception:
            pass
        link.close()


if __name__ == '__main__':
    main()
