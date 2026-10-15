#!/usr/bin/env python3
"""Camera distance calibration: arrow box size against ultrasonic distance.

Put the robot facing an arrow card, press Enter to record a sample, move the
robot, repeat. Type q and Enter to finish. Samples go to KC_range_calib.jsonl.
With --views, each Enter takes three photos: view shifted left, straight, right.
Stop the bridge first: this script needs the serial port. The STM only has to
be powered and calibrated (it sends !SENSOR telemetry every 100 ms).
"""
import argparse
import json
import time
from pathlib import Path

import requests
import serial

from KC_task2_stm32_bridge import Camera, pick_arrow


def read_us(link, seconds=0.8):
    """Median of the valid ultrasonic readings in the SENSOR telemetry."""
    link.reset_input_buffer()
    end, buf, values = time.time() + seconds, '', []
    while time.time() < end:
        buf += link.read(256).decode(errors='replace')
        while ';' in buf:
            line, buf = buf.split(';', 1)
            if '!SENSOR/' not in line:
                continue
            parts = line[line.index('!SENSOR/'):].split('/')
            try:
                if int(parts[4]) == 1:
                    values.append(float(parts[3]))
            except (IndexError, ValueError):
                pass
    values.sort()
    return values[len(values) // 2] if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', default='/dev/ttyACM0')
    parser.add_argument('--laptop-ip', default='192.168.50.218')
    parser.add_argument('--views', action='store_true', help='full sensor; photos with the view shifted left, centre, right')
    args = parser.parse_args()
    camera = Camera(full_sensor=args.views)
    views = [('left', -1.0), ('centre', 0.0), ('right', 1.0)] if camera.full_sensor else [('centre', 0.0)]
    link = serial.Serial(args.port, 115200, timeout=0.1)
    url = f'http://{args.laptop_ip}:8001/predict'
    folder = Path(f'KC_range_calib_{int(time.time())}')
    folder.mkdir()
    n = 0
    try:
        with open('KC_range_calib.jsonl', 'a') as log:
            while True:
                note = input('Enter = record (optionally type a note, e.g. "side 28"), q = quit: ').strip()
                if note.lower() == 'q':
                    break
                us = read_us(link)
                us_text = f'{us:.1f}' if us is not None else 'none'
                for view, shift in views:
                    camera.set_view(shift)
                    image = folder / f'calib_{n}_{view}.jpg'
                    camera.capture(image)
                    with image.open('rb') as f:
                        result = requests.post(url, files={'image': (image.name, f, 'image/jpeg')}, timeout=5).json()
                    best = pick_arrow(result.get('detections', []))
                    box = best['xyxy'] if best else []
                    width = box[2] - box[0] if box else 0.0
                    height = box[3] - box[1] if box else 0.0
                    centre = (box[0] + box[2]) / 2 if box else 0.0
                    conf = best['confidence'] if best else 0.0
                    side = best['class'] if best else None
                    row = {'us': us, 'view': view, 'w': width, 'h': height, 'cx': centre, 'conf': conf,
                           'side': side, 'note': note, 'image': str(image)}
                    log.write(json.dumps(row) + '\n')
                    log.flush()
                    print(f'US={us_text} cm  view={view:6s} box w={width:.1f} h={height:.1f} '
                          f'centre_x={centre:.0f}  {side} {conf:.2f}  {note}')
                n += 1
                camera.set_view(0.0)
    finally:
        link.close()
        camera.close()


if __name__ == '__main__':
    main()
