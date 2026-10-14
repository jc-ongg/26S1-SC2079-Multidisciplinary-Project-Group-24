"""MDP Task 2: O1 swerve planner.

Fits a bicycle model to the turn measurements, simulates the O1 swerve that
the STM32 should run, checks body clearance against the 10x10 cm O1 under
radius / servo-lag / start-offset errors, and writes o1_bypass.svg.

Run:  python3 o1_planner.py
Needs numpy. Edit MEASUREMENTS and ROBOT after re-measuring.
"""
import numpy as np

# Centre-to-centre displacement of the robot, speed 50.
# kind: 'F' forward turn, 'B' reverse turn (rear swings to the named side).
# (kind, side, angle_deg, along_cm, sideways_cm, use_in_fit)
MEASUREMENTS = [
    ('F', 'L', 90, 15.0, 31.0, True),
    ('F', 'R', 90, 20.5, 38.0, True),
    ('B', 'L', 90, 29.5, 13.0, True),
    ('B', 'R', 90, 36.0, 21.0, True),
    ('F', 'L', 45, 13.25, 13.0, True),
    ('F', 'R', 45, 18.0, 15.0, True),
    ('B', 'L', 45, 12.5, 1.5, False),   # disagrees with the other 7 by ~5 cm, re-measure
    ('B', 'R', 45, 22.0, 2.5, True),
]

ROBOT = dict(length=23.0, width=18.8)
O1_SIZE = 10.0

# Swerve parameters (see README section in the chat):
Y_PEAK = 28.0     # lane offset of the rear axle beside O1, cm
D_TRIG = 30.0     # US (front bumper) to O1 face when steering starts, cm
X_RET = 3.0       # start turning back when rear axle is this far past O1's near face, cm
RAMP_HAT = 8.0    # cm travelled while the servo swings lock-to-lock (estimate; measure it)

DS = 0.5


def disp(kind, th, R, d):
    s, c = np.sin(th), np.cos(th)
    if kind == 'F':
        return R * s + d * (c - 1), R * (1 - c) + d * s
    return R * s + d * (1 - c), R * (1 - c) - d * s


def fit():
    best = None
    for RL in np.arange(15, 35, 0.1):
        for RR in np.arange(20, 40, 0.1):
            for d in np.arange(4, 14, 0.1):
                e = 0.0
                for k, sd, th, a, l, use in MEASUREMENTS:
                    if not use:
                        continue
                    pa, pl = disp(k, np.radians(th), RL if sd == 'L' else RR, d)
                    e += (pa - a) ** 2 + (pl - l) ** 2
                if best is None or e < best[0]:
                    best = (e, RL, RR, d)
    n = sum(1 for m in MEASUREMENTS if m[5]) * 2
    return best[1], best[2], best[3], np.sqrt(best[0] / n)


RL, RR, D_AX, RMS = fit()
REAR = ROBOT['length'] / 2 - D_AX     # rear bumper behind rear axle
FRONT = ROBOT['length'] / 2 + D_AX    # front bumper ahead of rear axle
HW = ROBOT['width'] / 2


def step(st, cmd, Rl, Rr, ramp):
    x, y, h, k = st
    tgt = {'L': 1 / Rl, 'R': -1 / Rr, 'S': 0.0}[cmd] if isinstance(cmd, str) else cmd
    rate = (1 / Rl + 1 / Rr) / ramp * DS if ramp > 0 else 9
    k += np.clip(tgt - k, -rate, rate)
    h += k * DS
    return (x + np.cos(h) * DS, y + np.sin(h) * DS, h, k)


def lead(R):
    # heading still gained while the servo returns from lock to centre
    return (1 / R) * (RAMP_HAT / 2) / 2


def predict(st, seq):
    for lock, stop in seq:
        for _ in range(400):
            if stop(st):
                break
            st = step(st, lock, RL, RR, RAMP_HAT)
    return st


def swerve(side, x1, Rl=None, Rr=None, ramp=RAMP_HAT, y0=0.0):
    """Simulate the STM32 state machine. x1 = O1 near face, rear axle starts at x=0.
    Returns path rows (x, y, heading, mode)."""
    Rl = Rl or RL
    Rr = Rr or RR
    sg = 1 if side == 'L' else -1
    o = 'R' if side == 'L' else 'L'
    Rs, Ro = (RL, RR) if side == 'L' else (RR, RL)
    st, mode, out = (0.0, y0, 0.0, 0.0), 0, []
    for _ in range(4000):
        x, y, h, _k = st
        if mode == 0 and x + FRONT >= x1 - D_TRIG:
            mode = 1
        if mode == 1:
            pk = predict(st, [(o, lambda s: sg * s[2] <= lead(Ro))])
            if sg * pk[1] >= Y_PEAK:
                mode = 2
        if mode == 2 and sg * h <= lead(Ro):
            mode = 3
        if mode == 3 and x >= x1 + X_RET:
            mode = 4
        if mode == 4:
            land = predict(st, [(side, lambda s: sg * s[2] >= -lead(Rs)),
                                ('S', lambda s: abs(s[3]) < 1e-4)])
            if sg * land[1] <= 0:
                mode = 5
        if mode == 5 and sg * h >= -lead(Rs):
            mode = 6
        if mode in (0, 6):
            cmd = float(np.clip(-0.10 * h - 0.006 * y, -1 / Rr, 1 / Rl))
        elif mode == 3:
            cmd = float(np.clip(-0.10 * h - 0.006 * (y - sg * Y_PEAK), -1 / Rr, 1 / Rl))
        else:
            cmd = {1: side, 2: o, 4: o, 5: side}[mode]
        st = step(st, cmd, Rl, Rr, ramp)
        out.append((st[0], st[1], st[2], mode))
        if mode == 6 and st[0] > x1 + O1_SIZE + 30 and abs(st[2]) < np.radians(1) and abs(st[1]) < 1:
            break
    return np.array(out)


def body(x, y, h):
    c, s = np.cos(h), np.sin(h)
    p = np.array([[-REAR, -HW], [FRONT, -HW], [FRONT, HW], [-REAR, HW]])
    return np.c_[x + p[:, 0] * c - p[:, 1] * s, y + p[:, 0] * s + p[:, 1] * c]


def gap(A, B):
    # separating-axis lower bound on the distance between convex polygons (<0 = overlap)
    best = -1e9
    for P, Q in ((A, B), (B, A)):
        for i in range(len(P)):
            e = P[(i + 1) % len(P)] - P[i]
            n = np.array([e[1], -e[0]]) / np.hypot(*e)
            pa, qb = P @ n, Q @ n
            best = max(best, qb.min() - pa.max(), pa.min() - qb.max())
    return best


def o1_poly(x1, yoff=0.0):
    h = O1_SIZE / 2
    return np.array([[x1, -h + yoff], [x1 + O1_SIZE, -h + yoff],
                     [x1 + O1_SIZE, h + yoff], [x1, h + yoff]])


def clearance(P, x1):
    sq = o1_poly(x1)
    return min(gap(body(x, y, h), sq) for x, y, h, _m in P[::2]
               if x + FRONT > x1 - 2 and x - 30 < x1 + O1_SIZE)


def write_svg(paths, x1, fname='o1_bypass.svg'):
    sc, pad = 4.0, 30
    xmin, xmax, ymin, ymax = -10, x1 + 110, -45, 45
    W = (xmax - xmin) * sc + 2 * pad
    H = (ymax - ymin) * sc + 2 * pad
    tx = lambda x: pad + (x - xmin) * sc
    ty = lambda y: pad + (ymax - y) * sc
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W:.0f} {2*H:.0f}" '
           f'font-family="Helvetica,Arial,sans-serif" font-size="13">',
           f'<rect width="{W:.0f}" height="{2*H:.0f}" fill="#fff"/>']
    for row, (side, P) in enumerate(paths):
        oy = row * H
        g = lambda x, y: f'{tx(x):.1f},{ty(y) + oy:.1f}'
        out.append(f'<text x="{pad}" y="{oy + 20}" font-weight="bold">Arrow {side}: '
                   f'rear-axle lane {Y_PEAK:.0f} cm, start steering at US = {D_TRIG:.0f} cm</text>')
        out.append(f'<line x1="{tx(xmin):.1f}" y1="{ty(0)+oy:.1f}" x2="{tx(xmax):.1f}" y2="{ty(0)+oy:.1f}" '
                   f'stroke="#bbb" stroke-dasharray="6 4"/>')
        out.append('<polygon points="' + ' '.join(g(*p) for p in o1_poly(x1)) + '" fill="#8b1a1a"/>')
        colors = {0: '#888', 1: '#1f6feb', 2: '#1f6feb', 3: '#2da44e', 4: '#d97706', 5: '#d97706', 6: '#888'}
        for i in range(0, len(P), 12):
            x, y, h, m = P[i]
            if x + FRONT < x1 - D_TRIG - 15 or x > x1 + 80:
                continue
            out.append('<polygon points="' + ' '.join(g(*p) for p in body(x, y, h)) +
                       f'" fill="none" stroke="{colors[int(m)]}" stroke-opacity="0.45"/>')
        out.append('<polyline points="' + ' '.join(g(x, y) for x, y, _h, _m in P[::2]) +
                   '" fill="none" stroke="#111" stroke-width="1.5"/>')
        i1 = int(np.argmax(P[:, 3] == 1))
        out.append(f'<circle cx="{tx(P[i1,0]):.1f}" cy="{ty(P[i1,1])+oy:.1f}" r="4" fill="#1f6feb"/>')
    lx = pad
    for c, t in (('#1f6feb', 'arc out (lock, then opposite lock)'), ('#2da44e', 'lane hold beside O1'),
                 ('#d97706', 'arc back to centre line'), ('#888', 'straight (gyro hold)')):
        out.append(f'<rect x="{lx}" y="{2*H-22}" width="14" height="14" fill="{c}"/>'
                   f'<text x="{lx+20}" y="{2*H-10}">{t}</text>')
        lx += 225
    out.append('</svg>')
    with open(fname, 'w') as f:
        f.write('\n'.join(out))


if __name__ == '__main__':
    print(f'fit: R_left={RL:.1f} cm  R_right={RR:.1f} cm  rear axle {D_AX:.1f} cm behind centre  (rms {RMS:.2f} cm)')
    for k, sd, th, a, l, use in MEASUREMENTS:
        pa, pl = disp(k, np.radians(th), RL if sd == 'L' else RR, D_AX)
        flag = '' if use else '   <- not used, check this one'
        print(f'  {k}{sd}{th:3d}: measured ({a:5.1f},{l:5.1f})  model ({pa:5.1f},{pl:5.1f}){flag}')
    x1 = 100.0
    paths = []
    for side in 'LR':
        P = swerve(side, x1)
        worst = 1e9
        for ramp in (4, 8, 14):
            for kR in (0.92, 1.0, 1.08):
                for y0 in (-2, 0, 2):
                    worst = min(worst, clearance(swerve(side, x1, RL * kR, RR * kR, ramp, y0), x1))
        M = P[:, 3]
        i1 = int(np.argmax(M == 1))
        done = np.where((M == 6) & (np.abs(P[:, 1]) < 1.5) & (np.abs(P[:, 2]) < np.radians(1.5)))[0]
        i6 = done[0] if len(done) else len(P) - 1
        heads = [np.degrees(P[int(np.argmax(M == m)) - 1, 2]) for m in (2, 3, 5, 6)]
        print(f'\narrow {side}: clearance nominal {clearance(P, x1):.1f} cm, worst case {worst:.1f} cm '
              f'(R +/-8%, servo lag 4-14 cm, start offset +/-2 cm)')
        print(f'  steering switches near headings {", ".join("%.0f" % h for h in heads)} deg')
        print(f'  swerve path {(i6 - i1) * DS:.0f} cm; back on centre line with front bumper '
              f'{P[i6, 0] + FRONT - (x1 + O1_SIZE):.0f} cm past O1 back face')
        paths.append((side, P))
    write_svg(paths, x1)
    print('\nwrote o1_bypass.svg')
