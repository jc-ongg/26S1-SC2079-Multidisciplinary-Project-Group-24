"""Offline check of KC_task2_pi_return against the 2026-10-15 robot layouts.

A fake robot executes the commands with the same turn model, but its true
pose starts 3 cm further out and 3 cm further up the lane than the native run
believed. The side IR sees O1 only when level with it; the ultrasonic sees the
carpark back wall 25 cm behind the start centre. Run: python3 test_pi_return.py
"""
import math
import sys

import KC_task2_pi_return as R

# Facts parsed from the robot logs (facts_from_frames), 14 runs.
LAYOUTS = [
    (161.2, 319.4, -34.6, 326.1, -19.6), (71.4, 182.3, -36.5, 188.2, -21.5),
    (70.6, 143.7, -52.6, 149.2, -37.6), (71.7, 143.7, -58.1, 149.1, -43.1),
    (71.7, 145.0, -59.1, 150.2, -44.1), (70.9, 145.8, -53.7, 151.3, -38.7),
    (70.7, 144.6, 67.6, 142.7, 47.1), (71.1, 146.5, 64.5, 145.2, 44.0),
    (70.7, 145.5, 85.1, 143.7, 64.6), (73.7, 140.3, -45.2, 146.0, -30.2),
    (71.5, 140.2, -54.6, 145.6, -39.6), (101.5, 218.5, -54.2, 225.9, -39.2),
    (140.7, 217.0, -42.0, 224.3, -27.0), (72.6, 218.3, -41.3, 224.4, -26.3),
    # Wide O2 (~140 cm, as tested for v9): lanes 85-95 out, 60 and 100 cm gaps.
    (72.0, 142.0, 90.0, 147.0, 70.0), (72.0, 142.0, -88.0, 147.0, -73.0),
    (72.0, 182.0, 95.0, 187.0, 75.0), (72.0, 182.0, -93.0, 187.0, -78.0),
]
ECHO = [45.0]
WALL_NOW = [-25.0]   # this run's back wall     # steepest angle at which the fake ultrasonic still echoes off the wall
WALL = -25.0      # carpark back wall (start centre 0, robot rear 13.5 from it)
SPEED = {100: 95.0, 80: 90.0, 70: 80.0, 45: 52.0, 30: 38.0, 22: 28.0, 16: 20.0, 12: 14.0, 50: 50.0}


class Clock:
    t = 0.0

    def __call__(self):
        return self.t


class FakeIO:
    def __init__(self, o1, x, y, err, o2=None, o2h=None):
        self.o1, self.x, self.y, self.h = o1, x + err[0], y + err[1], 180.0
        self.o2, self.o2h = o2, o2h
        self.lines = []
        self.waits = 0
        self.min_o1 = self.min_o2 = 1e9

    def frame(self):
        side_y = self.y - R.IR_AHEAD          # heading 180: the IR is 6.5 nearer home
        ir = abs(self.x) - R.IR_X_OFFSET if self.o1 <= side_y <= self.o1 + 10 and abs(self.h - 180) < 1 else 80.0
        return {'left': ir if self.x < 0 else 80.0, 'right': ir if self.x >= 0 else 80.0,
                'us': self.us(), 'us_valid': self.us() > 0, 'heading': self.h}

    def us(self):
        """Back wall at y = WALL; an HC-SR04 only echoes within ~20 deg of square."""
        off = abs((self.h - 180 + 180) % 360 - 180)
        if off > ECHO[0]:
            return -1.0
        ct = math.cos(math.radians(off))
        return (self.y - 11.5 * ct - WALL_NOW[0]) / ct

    def corners(self):
        r = math.radians(self.h)
        fx, fy = -math.sin(r), math.cos(r)            # forward
        rx, ry = math.cos(r), math.sin(r)             # right
        return [(self.x + a * fx + b * rx, self.y + a * fy + b * ry) for a in (11.5, -11.5) for b in (9.4, -9.4)]

    def _o1_gap(self):
        """Exact body-outline clearance (true pose) to O1 and to O2."""
        c = self.corners()
        self.min_o1 = min(self.min_o1, gap(c, (-5, self.o1, 5, self.o1 + 10)))
        if self.o2 is not None:
            self.min_o2 = min(self.min_o2, gap(c, (-self.o2h, self.o2, self.o2h, self.o2 + 10)))

    def move(self, command, speed, value, on_sensor=None):
        self.lines.append((command, speed, value))
        if command in ('FWD', 'REVS'):
            sign = 1 if command == 'FWD' else -1
            r = math.radians(self.h)
            CLOCK.t += R.STARTUP_S          # acceleration before cruise speed
            step = 0.5
            done = 0.0
            while done < value:
                d = min(step, value - done)
                self.x -= sign * d * math.sin(r)
                self.y += sign * d * math.cos(r)
                done += d
                CLOCK.t += d / SPEED[speed]
                self._o1_gap()
                if on_sensor and int(CLOCK.t * 10) != int((CLOCK.t - d / SPEED[speed]) * 10):
                    on_sensor(self.frame())
        else:
            side = 'left' if command == 'TURNL' else 'right'
            x0, y0, h0 = self.x, self.y, self.h
            r = math.radians(h0)
            for k in range(1, 11):                     # sweep through the turn
                f, s = R.turn_offsets(side, value * k / 10)
                self.x = x0 + s * math.cos(r) - f * math.sin(r)
                self.y = y0 + f * math.cos(r) + s * math.sin(r)
                self.h = h0 + (value if side == 'left' else -value) * k / 10
                self._o1_gap()
            CLOCK.t += 1.4
            if on_sensor:                      # last degrees + 150 ms settle: ~2 frames at 10 Hz
                on_sensor(self.frame())
                on_sensor(self.frame())

    def sensors(self, n):
        self.waits += n                        # frames the route stopped to wait for
        CLOCK.t += 0.1 * n
        return [self.frame() for _ in range(n)]

    def log(self, text):
        self.lines.append(text)


def gap(c, box):
    """Separating-axis distance between a body (4 corners) and an axis box."""
    b = [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]
    best = -1e9
    for P, Q in ((c, b), (b, c)):
        for i in range(4):
            ex, ey = P[(i + 1) % 4][0] - P[i][0], P[(i + 1) % 4][1] - P[i][1]
            n = math.hypot(ex, ey)
            nx, ny = ey / n, -ex / n
            pp = [a * nx + q * ny for a, q in P]
            qq = [a * nx + q * ny for a, q in Q]
            best = max(best, min(qq) - max(pp), min(pp) - max(qq))
    return best


CLOCK = Clock()
R.time.monotonic = CLOCK


def main():
    bad = 0
    # pose error toward O1 (-5) / away (+3); ultrasonic echoing to 45 deg / only 20 deg
    # Carpark back wall: the shallowest and the deepest the rules allow for this
    # O1 (opening <= O1 front - 57; robot rear 0-27 cm from the wall at the start).
    runs = [(L, e, ec, wk) for ec in (45.0, 20.0) for L in LAYOUTS for e in (-5, 3) for wk in (0, 1)]
    for (o1, o2, x, y, edge), e, ec, wk in runs:
        ECHO[0] = ec
        WALL_NOW[0] = -38.5 if wk else max(-38.5, min(-11.5, R.mouth(o1) - 50))
        # True O2 end: the back-face run overshoots ~8 past it (the Pi assumes >= 5).
        # e < 0: the true car is |e| nearer the centre line (toward O1) than odometry says.
        io = FakeIO(o1, x, y, ((-e if x < 0 else e), -3), o2, abs(edge) + R.IR_AHEAD - 8)
        facts = {'origin': 0.0, 'o1_front': o1, 'o2_front': o2, 'x': x, 'y': y, 'edge_x': edge}
        CLOCK.t = 0.0
        try:
            R.PiReturn(io, facts).run()
            err = None
        except R.ReturnError as exc:
            err = str(exc)
        c = io.corners()
        front_gap = min(cy for _, cy in c) - WALL_NOW[0]                 # nearest body point to the back wall
        inside = all(abs(cx) <= 30 and WALL_NOW[0] <= cy <= WALL_NOW[0] + 50 for cx, cy in c)
        # A lane too wide for any entry (85 cm, 65 cm gap: the firmware also
        # stopped there) must stop with an error before turning.
        known = x > 80 and err is not None and 'entry' in err and io.h == 180
        # Without an O1 sighting the end x carries the injected 3 cm error.
        ok = known or (err is None and inside and abs(io.x) <= 15 and io.min_o1 >= 2 and io.min_o2 >= 2
                       and 5 <= front_gap <= 20)
        bad += not ok
        fix = next((l for l in io.lines if isinstance(l, str) and 'O1 seen' in l), 'O1 not seen')
        shift = next((l.split('|')[0].strip() for l in io.lines if isinstance(l, str) and ('shift plan' in l or 'lane kept' in l or 'no shift' in l)), '')
        print(f"{('KNOWN' if known else 'ok   ') if ok else 'FAIL '} gap {o2 - o1 - 10:5.0f} lane {x:6.1f} err {e:+d} echo {ec:.0f} wall {WALL_NOW[0]:5.1f}: end x {io.x:5.1f} h {io.h - 180:+5.1f}, wall gap {front_gap:4.1f}, "
              f"O1 {io.min_o1:5.1f} O2 {io.min_o2:5.1f}, {CLOCK.t:4.1f} s, waits {io.waits} | {shift[9:]} | {fix.split('|')[0][9:].strip()} {err or ''}")
    print(f'{len(runs) - bad}/{len(runs)} runs parked (18 layouts x pose error toward/away from O1 x echo 45/20 deg x shallowest/deepest carpark)')
    return bad != 0


if __name__ == '__main__':
    sys.exit(main())
