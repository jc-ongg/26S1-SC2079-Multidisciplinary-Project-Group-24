"""Two-step carpark return driven from the Pi (no reflash).

The STM32 runs O1 and O2 natively (stage o2-bypass) and stops after O2's
return turn, facing home. This module then drives home with plain encoder
commands (FWD / REVS / TURNL / TURNR), the way feat/task2-runner's go_home does:

1. In front of O2, shift into a lane 24 cm off the centre line (45 / diagonal
   / 45), sweep-checked against O1 and O2's measured extent. A short O1-O2 gap
   uses the closest lane up to 44 cm whose S still ends before O1.
2. Pass O1 (scan at O1_PASS_SPEED 45, other straights 100, turns 80) watching the centre-side IR. O1 fixes x (gap) and, at speed 30 only, y
   (first sighting of its back face); the carpark entry is then planned from
   the fixed pose, the car moves to its start and turns in, then parks on one
   ultrasonic reading.

The sweep checks are a direct port of Core/Inc/kc_entry_geometry.h and the
kc2_* functions in Core/Inc/kc_task2.h (same numbers, so a later firmware
build behaves the same). Frame: origin = start centre, +X right, +Y out from
the carpark, heading 180 = facing home. Distances in cm.
"""
import math
import statistics
import time
from collections import deque

import numpy as np

# ---------------------------------------------------------------- geometry
KCE_MARGIN = 5.0
KCE_MAX_DIAGONAL = 85.0
W = 9.4 + KCE_MARGIN     # inflated half width
L = 11.5 + KCE_MARGIN    # inflated half length
TURN45 = {'left': (13.25, -13.0), 'right': (18.0, 15.0)}   # (forward, right), speed 50
TURN90 = {'left': (15.0, -31.0), 'right': (20.5, 38.0)}    # legacy 90 deg fits (v9 crossings)
MAX_POSES = 320          # v9 KCE_MAX_POSES (90 deg templates: 4 x 77 = 308)
PASS_LANE = 24.0
PASS_LANES = (24.0, 27.0, 30.0, 33.0, 36.0, 40.0, 44.0)
PASS_SHIFT_MIN = 40.0
PASS_LANE_MIN = 18.0
IR_AHEAD = 6.5           # side IRs: 18 cm from the rear = 6.5 ahead of centre
IR_X_OFFSET = 10.4       # |x| = IR gap + this (firmware O1_EDGE calibration)
O1_SEEN_MAX = 50.0       # runner trusts the GP2Y0A21 to 60; only O1 is in view here
O1_LOOK_AHEAD = 15.0
O1_SLOW_ZONE = 20.0
O1_FIX_MAX = 15.0        # x (from the IR gap); v9 ignores O1 readings > 15 cm from odometry
O1_YFIX_MAX = 10.0       # y comes from drive time on the Pi: trust less
STRAIGHT_SPEED = 100    # every return straight (FWD/REVS) except the O1 scan; the STM32
                        # caps the PWM over each move's last 50/20/10 cm, so stops stay on target
RETURN_SPEED = 50       # the return's turns: their 45/90 deg models were measured at 50
                        # (robot 2026-10-15: two 90s at 100 left x 12.7 cm out at O1)
O1_PASS_SPEED = 100     # past O1: no slowdown (robot runs: the scan rarely caught O1). The
                        # side IR still watches; x counts as unconfirmed unless it does
# At speed 30 (runner CM_PER_S[30] = 38 cm/s, measured) drive time gives y at
# the O1 sighting. Faster (not measured) it does not: only x is fixed, and a
# generous speed (servo test: 58.5 cm/s at 50, +25%) only opens the IR watch
# window early.
PASS_CM_PER_S = 38.0 if O1_PASS_SPEED <= 30 else 1.25 * 58.5 / 50 * O1_PASS_SPEED
PASS_YFIX = O1_PASS_SPEED <= 30
# SENSOR frames come every 100 ms: above 40 O1 may be in view for one frame.
PASS_HITS = 2 if O1_PASS_SPEED <= 40 else 1
FAST_LEG_MIN = 40.0     # shorter fast legs join the O1 scan: one stop fewer
# Slanted carpark entry after the O1 scan: one turn of a small angle toward the
# centre line, then one straight into the carpark. Its length comes from the
# ultrasonic on the back wall read along the slanted heading at the end of the
# turn, so it does not depend on where in the carpark the run started. The
# STM32 slows every straight over its last 50/20/10 cm, so it runs at 100 and
# still stops on target. Any failed check turns back and uses the S entry.
SLANT_ENTRY = True
SLANT_GAP = 17.0        # front-centre to back wall, perpendicular (was 12; robot stopped 13 cm off)
SLANT_MAX_DEG = 40.0    # steepest turn-in where the ultrasonic still echoes off the back
                        # wall. NOT MEASURED on this robot: if the log shows "no steady
                        # ultrasonic echo at the slant", lower it to the angle that echoes
SLANT_OPENING_MAX_DEG = 45.0  # the angle is chosen so the car crosses the opening and ends
                        # near the centre line; every candidate is checked against the
                        # opening and the side walls for every carpark position
SLANT_CORNER_GAP = 12.0  # was 7: +5 cm, as the straight park
SLANT_NO_ECHO_MARGIN = 4.0
SLANT_ECHO_TRUSTED_DEG = 20.0  # robot 2026-10-15: clean back-wall echo at 16 deg, ~90 cm. Up to
                        # here the echo is relied on; steeper slants must also be safe without it  # more room when the run comes from odometry, not the echo  # nearest front corner to the back wall at the end of the slant
SLANT_MIN_DEG = 4.0
CIRCLE_R = {'left': 23.3, 'right': 30.1}  # rear-axle radii fitted to the 45 deg turn table
# Two-step return (one method for every layout): from O2's corner, turn 1 and
# leg 1 to a waypoint W just in front of and beside O1, then turn 2 and leg 2
# straight into the carpark (sized by the ultrasonic, as the slant entry).
DIRECT_RETURN = False   # see README: ~6-8 cm from O1 with no IR check; on only once odometry is known to be within ~3 cm
DIRECT_O1_MARGIN = 3.0  # on top of the 5 cm inflated body: >= 8 cm from O1 (no IR scan, odometry only)
DIRECT_LEG1_MAX_DEG = 60.0
STARTUP_S = 0.4
PARK_GAP = 17.0          # front to back wall (was 12, stopped 13 cm off; rear then ~10 inside)


def cround(v):
    """C roundf: halves away from zero."""
    return math.copysign(math.floor(abs(v) + 0.5), v)


class Poses:
    """kce_template_turn (v9): four sampled sweeps (2 turn models x leg 1 of
    0/+2 cm). turn 45: S with a diagonal; turn 90: crossing with a straight."""

    def __init__(self, x, target, diagonal, turn=45):
        self.target = target
        wide = turn > 60
        xs, ys, hs, ends = [], [], [], []
        for model in (0, 1):
            for coast in (0, 1):
                px, py, h = x, 0.0, 180.0
                xs.append(px); ys.append(py); hs.append(h)
                for leg in range(3):
                    left = (leg == 2) if x >= 0 else (leg == 0)
                    if wide:
                        f = diagonal + 2 * coast if leg == 1 else (15.0 if left else 20.5)
                        side = 0.0 if leg == 1 else (-31.0 if left else 38.0)
                    else:
                        f = diagonal + 2 * coast if leg == 1 else (13.25 if left else 18.0)
                        side = 0.0 if leg == 1 else (-13.0 if left else 15.0)
                    dh = 0.0 if leg == 1 else (1.0 if left else -1.0) * (90.0 if wide else 45.0)
                    r = math.radians(h)
                    dx = side * math.cos(r) - f * math.sin(r)
                    dy = f * math.cos(r) + side * math.sin(r)
                    steps = 16 if leg == 1 else (30 if wide else 20)
                    length = math.hypot(dx, dy) * (1.11072073 if wide else 1.02617215)  # arc/chord
                    for j in range(1, steps + 1):
                        u = j / steps
                        ox, oy = u * dx, u * dy
                        if model and leg != 1:
                            a, b, c = u**3 - 2 * u * u + u, -2 * u**3 + 3 * u * u, u**3 - u * u
                            ax, ay = -length * math.sin(r), length * math.cos(r)
                            bx, by = -length * math.sin(math.radians(h + dh)), length * math.cos(math.radians(h + dh))
                            ox, oy = a * ax + b * dx + c * bx, a * ay + b * dy + c * by
                        xs.append(px + ox); ys.append(py + oy); hs.append(h + u * dh)
                    px += dx; py += dy; h += dh
                ends.append(len(xs) - 1)
        self.x = np.array(xs); self.y = np.array(ys)
        hr = np.radians(np.array(hs)); self.c = np.cos(hr); self.s = np.sin(hr)
        self.ends = ends
        self.ok = len(xs) < MAX_POSES and 0 <= diagonal <= KCE_MAX_DIAGONAL

    def end(self, k=0):
        return float(self.x[self.ends[k]]), float(self.y[self.ends[k]])


def overlap(px, py, c, s, box):
    """kce_overlap for arrays of poses (py already includes the start y)."""
    xmin, ymin, xmax, ymax = box
    dx = (xmin + xmax) * .5 - px
    dy = (ymin + ymax) * .5 - py
    rx, ry = (xmax - xmin) * .5, (ymax - ymin) * .5
    ac, as_ = np.abs(c), np.abs(s)
    hit = ~((np.abs(dx) > rx + W * ac + L * as_) | (np.abs(dy) > ry + W * as_ + L * ac))
    hit &= ~(np.abs(dx * c + dy * s) > W + rx * ac + ry * as_)
    hit &= ~(np.abs(-dx * s + dy * c) > L + rx * as_ + ry * ac)
    return hit


def mouth(o1):
    return min(38.5, o1 - 60.0 + 3.0)


# Extra room (cm) kept from O1 by the entry checks while x has not been
# confirmed by the side IR (lane beyond its reach, or O1 not seen): odometry
# can be several cm out by then.
O1_EXTRA = [0.0]
O1_UNFIXED_MARGIN = 5.0


def carpark_clear(px, py, c, s, o1, o2):
    """kce_pose_clear for arrays; py includes the start y."""
    m = mouth(o1)
    if m < 11.5:
        return False
    ac, as_ = np.abs(c), np.abs(s)
    reach = W * as_ + L * ac
    if np.any(py - reach <= -11.5) or np.any(py + reach >= o2):
        return False
    sideways = W * ac + L * as_
    if np.any((py - reach <= m) & (np.abs(px) + sideways >= 30)):
        return False
    e = O1_EXTRA[0]
    for box in ((-5 - e, o1 - e, 5 + e, o1 + 10 + e), (30, -38.5, 40, m), (-40, -38.5, -30, m)):
        if np.any(overlap(px, py, c, s, box)):
            return False
    return True


def entry_clear(p, sy, o1, o2):
    """kce_clear: the centring S from start y `sy`."""
    for k in range(4):
        ex, ey = p.end(k)
        if abs(ex - p.target) > 2.5:
            return False
        if p.target == 0 and ey + sy + 16.5 >= o1:
            return False
    return carpark_clear(p.x, p.y + sy, p.c, p.s, o1, o2)


def straight_poses(x, frm, to):
    n = int(math.ceil(abs(to - frm) / 2)) + 1
    ys = np.array([frm + (to - frm) * k / n for k in range(n + 1)])
    return np.full_like(ys, x), ys, np.full_like(ys, -1.0), np.zeros_like(ys)


def turn_offsets(side, angle):
    """(forward, right) of the robot centre for a turn: the measured 45/90
    tables, else a rear-axle circle (8.2 cm behind the centre) fitted to the
    45 deg table (within ~1 cm of it)."""
    if angle == 45:
        return TURN45[side]
    if angle == 90:
        return TURN90[side]
    r, t = CIRCLE_R[side], math.radians(angle)
    f = r * math.sin(t) + 8.2 * (math.cos(t) - 1)
    across = r * (1 - math.cos(t)) + 8.2 * math.sin(t)
    return f, (-across if side == 'left' else across)


def body_gap(x, y, h, box):
    """Exact clearance (cm, separating axis) between the robot body at (x, y, h)
    and an axis-aligned box; negative = overlap."""
    r = math.radians(h)
    fx, fy, rx, ry = -math.sin(r), math.cos(r), math.cos(r), math.sin(r)
    c = [(x + a * fx + b * rx, y + a * fy + b * ry) for a in (11.5, -11.5) for b in (9.4, -9.4)]
    c = [c[0], c[1], c[3], c[2]]
    bx = [(box[0], box[1]), (box[2], box[1]), (box[2], box[3]), (box[0], box[3])]
    best = -1e9
    for P, Q in ((c, bx), (bx, c)):
        for i in range(4):
            ex, ey = P[(i + 1) % 4][0] - P[i][0], P[(i + 1) % 4][1] - P[i][1]
            n = math.hypot(ex, ey)
            nx, ny = ey / n, -ex / n
            pp = [u * nx + v * ny for u, v in P]
            qq = [u * nx + v * ny for u, v in Q]
            best = max(best, min(qq) - max(pp), min(pp) - max(qq))
    return best


def turns_across(x, turn):
    """Distance two turns already move toward the centre line."""
    if turn > 60:
        return 53.0 if x >= 0 else 51.5      # RIGHT 38 + LEFT 15 / LEFT 31 + RIGHT 20.5
    return 15 + (13.25 - 13) / 1.414213562 if x >= 0 else 13 + (18.0 - 15) / 1.414213562


def leg_for(x, across, turn):
    """Leg-1 length (diagonal or straight) to move `across` toward the centre."""
    if turn > 60:
        d = cround(across - turns_across(x, 90))
        if -2.5 < d < 0:
            d = 0.0
        return d
    return cround((across - turns_across(x, 45)) * 1.414213562)


def choose_entry(x, cy, known_y, o1, o2, turn=45):
    """kce_choose_turn (v9): (leg, start y) of the centring move, middle of the
    widest clear interval, or None. turn 90 = v9's crossing for wide lanes."""
    d = leg_for(x, abs(x), turn)
    if turn > 60:
        if d < 0:
            return None
    elif d == 1:
        d = 2
    p = Poses(x, 0.0, d, turn)
    if not p.ok:
        return None
    low, high = max(0.0, o1 - 100), min(o1 + 100, min(known_y - 2, cy + 60))
    best = run = 0
    begin = best_begin = 0.0
    y = float(math.ceil(low))
    while y <= math.floor(high):
        sx, syy, sc, ss = straight_poses(x, cy, y)
        if entry_clear(p, y, o1, o2) and carpark_clear(sx, syy, sc, ss, o1, o2):
            if not run:
                begin = y
            run += 1
            if run > best:
                best, best_begin = run, begin
        else:
            run = 0
        y += 1
    if best < 5:
        return None
    return d, best_begin + (best - 1) * .5


def obstacle_clear(px, py, c, s, o1, o2f, o2h):
    """kc2_pose_clear: O1 and O2's measured extent only."""
    return not (np.any(overlap(px, py, c, s, (-5, o1, 5, o1 + 10)))
                or np.any(overlap(px, py, c, s, (-o2h, o2f, o2h, o2f + 10))))


def choose_shift(x, y, o1, o2f, o2h, max_lane=None):
    """(lane x, leg, start y, turn) or None. In order of preference, each with
    45 deg turns then v9's 90 deg crossing (onto 30-40 cm, or no straight):
    'before' - ends with the IR short of O1 (the pass fixes x and y);
    'beside' - ends with >= 9 cm of O1 still beside the IR (x fix only);
    'any'    - may end past O1 (dead reckoning, as v9's crossing), only if
               a checked carpark entry exists from where it ends."""
    for tier in ('before', 'beside', 'any'):
        for turn in (45, 90):
            plan = _choose_shift(x, y, o1, o2f, o2h, tier, turn, max_lane)
            if plan:
                return plan + (turn,)
    return None


def _choose_shift(x, y, o1, o2f, o2h, tier, turn, max_lane=None):
    sg = 1.0 if x >= 0 else -1.0
    turns = turns_across(x, turn)
    lanes = PASS_LANES if turn <= 60 else (30.0, 35.0, 40.0, abs(x) - turns)
    if max_lane is not None:
        # Lane-first return: shift only as far as the slant needs, widest first.
        lanes = sorted({l for l in lanes if l <= max_lane} | ({max_lane} if max_lane >= PASS_LANE_MIN else set()),
                       reverse=True)
    for lane in lanes:
        if turn <= 60:
            if abs(x) - lane < 15.2:
                break
            d = max(leg_for(x, abs(x) - lane, 45), 2.0)   # never a 0 cm diagonal
            tg = sg * (abs(x) - turns - d / 1.414213562)
        else:
            if lane < 22.0:
                break
            d = leg_for(x, abs(x) - lane, 90)
            if d < 0:
                continue
            tg = sg * (abs(x) - turns - d)
        if d > KCE_MAX_DIAGONAL:
            continue
        if abs(tg) < PASS_LANE_MIN or abs(tg) > lane + 1.5:
            continue
        p = Poses(x, tg, d, turn)
        if not p.ok or any(abs(p.end(k)[0] - tg) > 2.5 for k in range(4)):
            continue
        end_y = p.end(0)[1]
        run = tries = 0
        sy = math.floor(y)
        while sy >= math.floor(y) - 100 and tries < 5:
            if tier == 'before' and sy + end_y - IR_AHEAD < o1 + 10 + 2:
                break
            if tier == 'beside' and sy + end_y - IR_AHEAD < o1 + 9:
                break                 # >= 9 cm of O1 left for the IR (2-3 frames at speed 30)
            sx, syy, sc, ss = straight_poses(x, y, sy)
            # The straight home only moves away from O2: check O1 alone.
            if (obstacle_clear(p.x, p.y + sy, p.c, p.s, o1, o2f, o2h)
                    and not np.any(overlap(sx, syy, sc, ss, (-5, o1, 5, o1 + 10)))):
                run += 1
                if run >= 5:
                    if tier == 'before':
                        return tg, d, sy + 2
                    # Only if the carpark entry exists from where it ends.
                    ex, ey = p.end(0)
                    entry = (choose_entry(ex, sy + 2 + ey, sy + 2 + ey, o1, o2f)
                             or choose_entry(ex, sy + 2 + ey, sy + 2 + ey, o1, o2f, 90))
                    if entry is not None and entry[1] <= sy + 2 + ey - 2:
                        return tg, d, sy + 2
                    if tier == 'beside':
                        break
                    tries += 1        # 'any': a few cm lower may leave room for the entry
                    run = 0
            else:
                run = 0
            sy -= 1
    return None


# -------------------------------------------------------------- native facts
def facts_from_frames(frames):
    """O1 front, O2 front, exit pose and O2's half length from the STM32's
    !KC_T2 event lines of a native o2-bypass run (firmware SmoothO1_v3 and
    later: STATE 4 = O2_RANGE, 15 = O2_BACK_EDGE, 16 = O2_RETURN_TURN).
    O2's front is set from the first range group taken in O2_RANGE, as the
    firmware does: US_VERIFIED, or (v11) US_REANCHORED when 8 steady echoes
    were accepted against the expected range."""
    f = {'origin': 0.0, 'o1_front': None, 'o2_front': None, 'x': None, 'y': None,
         'edge_x': None, 'side2': None}
    state = None
    o2_y = None
    last_pose = None
    for raw in frames:
        p = raw.strip(';').split('/')
        if len(p) < 4 or p[0] != '!KC_T2':
            continue
        name = p[1]
        try:
            a, b = float(p[2]), float(p[3])
        except ValueError:
            continue
        if name == 'HEADING_ORIGIN':
            f['origin'] = a
        elif name == 'F_O1_RANGE':
            f['o1_front'] = b
        elif name == 'O1_GAP' and f['o1_front'] is None:
            f['o1_front'] = a + 11.5 + (f['y'] or 0.0)
        elif name == 'STATE':
            s = int(a)
            # From the back face (15), its IR recovery (11), or the move to its
            # calculated far end when the face was never seen (22).
            if s == 16 and state in (11, 15, 22) and last_pose is not None:
                f['edge_x'] = last_pose[0]
            if s == 25:
                f['side2'] = 2
            if s == 4:
                o2_y = b                  # y when that O2_RANGE pass started
            if s != 22:
                state = s
        elif (name in ('US_VERIFIED', 'US_REANCHORED') and state == 4 and o2_y is not None
              and f['o2_front'] is None):
            f['o2_front'] = o2_y + 11.5 + a
        elif name == 'POSE':
            last_pose = (a, b)
            f['x'], f['y'] = a, b
    return f


# ---------------------------------------------------------------- the drive
# Sensor trouble only gets a small fix (heading turned back, park distance
# from the carpark limits). A route that fails its clearance check is never
# driven: that stops, as a crash would also end the run.
NO_STALL = True


class ReturnError(RuntimeError):
    pass


class PiReturn:
    """io must provide:
    move(command, speed, value, on_sensor=None) -> waits for DONE (raises on
        ERROR/ABORTED); on_sensor(frame) is called for each SENSOR frame
        while it runs.
    sensors(n) -> list of n fresh SENSOR frames (dicts: left, right, us,
        us_valid, heading).
    log(text)
    """

    def __init__(self, io, facts):
        self.io = io
        self.o1 = facts['o1_front']
        self.o2 = facts['o2_front']
        self.x, self.y, self.h = facts['x'], facts['y'], 180.0
        self.origin = facts['origin']
        # O2 is centred. The back-face run stopped with the IR (6.5 ahead) a few
        # cm past O2's far end: at speed 16 with 100 ms IR it overshoots ~8, so
        # +6.5 - 5 still leaves ~3 cm of the estimate beyond the real end.
        if facts.get('o2_half') is not None:      # keyed in (KC_task2_return_test.py)
            self.o2h = facts['o2_half']
        elif facts.get('edge_x') is not None:
            self.o2h = abs(facts['edge_x']) + IR_AHEAD - 5.0
        else:
            self.o2h = abs(self.x) - 20.0
        self.return_y = self.y
        self.turn_point = None                # where the lane straight ends for the one-turn entry
        self.recent = deque(maxlen=12)        # SENSOR frames received during moves
        for k in ('o1_front', 'o2_front', 'x', 'y'):
            if facts.get(k) is None:
                raise ReturnError(f'missing {k} from the native run')

    # pose bookkeeping (kc_pose_finish)
    def _straight(self, d):
        r = math.radians(self.h)
        self.x -= d * math.sin(r)
        self.y += d * math.cos(r)

    def _turn(self, side, angle):
        f, s = turn_offsets(side, angle)
        r = math.radians(self.h)
        self.x += s * math.cos(r) - f * math.sin(r)
        self.y += f * math.cos(r) + s * math.sin(r)
        self.h += angle if side == 'left' else -angle

    def _move(self, command, speed, value, on_sensor=None):
        """Every move records its SENSOR frames, so checks after it need not
        wait for new ones (no pause between commands)."""
        self.recent.clear()

        def record(frame):
            self.recent.append(frame)
            if on_sensor is not None:
                on_sensor(frame)
        self.io.move(command, speed, value, record)

    def _check_heading(self, what):
        # The last frame of the turn just done (its 150 ms settle), else wait.
        frame = self.recent[-1] if self.recent else self.io.sensors(1)[-1]
        actual = frame['heading'] - self.origin
        err = (actual - self.h + 180) % 360 - 180
        if abs(err) > 5:
            if not NO_STALL:
                raise ReturnError(f'{what}: heading {actual:.1f}, expected {self.h:.1f}')
            # Turn back onto the planned heading (pose moves with the turn).
            want, n = self.h, int(round(abs(err)))
            self.log(f'{what}: heading {actual:.1f}, expected {want:.1f}; turning {n} deg back')
            self.h = actual
            self.turn('right' if err > 0 else 'left', n)
            self.h = want
            return 0.0
        return err

    def fwd(self, d, speed, on_sensor=None):
        d = int(round(d))
        if d < 2:
            return
        self._move('FWD', speed, d, on_sensor)
        self._straight(d)

    def rev(self, d, speed=STRAIGHT_SPEED):
        d = int(round(d))
        if d < 2:
            return
        self._move('REVS', speed, d)
        self._straight(-d)

    def turn(self, side, angle):
        self._move('TURNL' if side == 'left' else 'TURNR', RETURN_SPEED, angle)
        self._turn(side, angle)

    def s_curve(self, leg, angle=45):
        """Turn toward the centre line, leg (diagonal at 45, straight across at
        90), turn back (as the firmware's RETURN_TRIM / CENTRE)."""
        first = 'right' if self.x >= 0 else 'left'
        back = 'left' if first == 'right' else 'right'
        self.turn(first, angle)
        self.fwd(leg, STRAIGHT_SPEED)          # skipped below 2 cm
        self.turn(back, angle)
        self._check_heading(f'after the {angle} deg move')

    def log(self, text):
        self.io.log(f'[RETURN] {text} | pose x={self.x:.1f} y={self.y:.1f} h={self.h:.0f}')

    # --------------------------------------------------------------- steps
    def run(self):
        t0 = time.monotonic()
        self.log(f'start: O1 front {self.o1:.1f}, O2 front {self.o2:.1f}, O2 half {self.o2h:.1f}')
        if abs(self.x) < PASS_LANE_MIN:
            raise ReturnError(f'lane {self.x:.1f} is too close to O1')
        if DIRECT_RETURN and self.direct_return():
            pass
        else:
            self.step1()
            if not self.step2():
                self.park()
        self.log(f'parked, return took {time.monotonic() - t0:.1f} s')

    def step1(self):
        if SLANT_ENTRY:
            # Straight down this lane past O1, then the entry. Shift before O1
            # only if no entry fits from in front of O1 in this lane.
            y_turn = self.o1 - 2
            # Plan as if O1 will not fix x (no slow scan): keep more room from O1.
            O1_EXTRA[0] = O1_UNFIXED_MARGIN
            tp = self._best_turn_point(self.x, y_turn)
            if tp is not None:
                self.turn_point = tp[0]
                self.log(f'lane kept: straight past O1 to y {tp[0]:.0f}, then one {tp[1]} deg turn into the carpark')
                return
            for turn in (45, 90):
                if choose_entry(self.x, y_turn, self.y, self.o1, self.o2, turn) is not None:
                    self.log(f'lane kept: straight past O1, then the {turn} deg S entry')
                    return
            plan = choose_shift(self.x, self.y, self.o1, self.o2, self.o2h)
            if plan is None:
                self.log('lane kept: no entry from in front of O1 and no shift fits')
                return
            lane, d, sy, turn = plan
            self.log(f'shift plan ({turn} deg): start y {sy:.1f}, lane {lane:.1f}, leg {d:.0f} '
                     f'(no entry fits from {abs(self.x):.0f} cm out in front of O1)')
            self.fwd(self.y - sy, STRAIGHT_SPEED)
            p = Poses(self.x, lane, d, turn)
            if not obstacle_clear(p.x, p.y + self.y, p.c, p.s, self.o1, self.o2, self.o2h):
                self._not_clear('shift no longer clear at the actual start')
            self.s_curve(d, turn)
            self.log('shifted')
            return
        if abs(self.x) < PASS_SHIFT_MIN:
            self.log('lane kept (already close enough to see O1)')
            return
        plan = choose_shift(self.x, self.y, self.o1, self.o2, self.o2h)
        if plan is None:
            self.log('no shift fits before O1: keeping the lane')
            return
        lane, d, sy, turn = plan
        self.log(f'shift plan ({turn} deg): start y {sy:.1f}, lane {lane:.1f}, leg {d:.0f}')
        self.fwd(self.y - sy, STRAIGHT_SPEED)
        p = Poses(self.x, lane, d, turn)
        if not obstacle_clear(p.x, p.y + self.y, p.c, p.s, self.o1, self.o2, self.o2h):
            self._not_clear('shift no longer clear at the actual start')
        self.s_curve(d, turn)
        self.log('shifted')

    def step2(self):
        """Clear O2, scan O1, enter. True when it has already parked (slant)."""
        # Clear O2's front first (as the firmware): the carpark-entry checks
        # want the whole body in front of O2's face before anything is planned.
        # A short fast leg joins the O1 scan instead (one stop fewer); the plan
        # is then made as from O2's front, which the scan drives away from.
        clear_o2 = self.o2 - 18.5
        slow = min(self.o1 + 10 + IR_AHEAD + O1_LOOK_AHEAD + O1_SLOW_ZONE, clear_o2)
        if (O1_PASS_SPEED < STRAIGHT_SPEED and self.y - slow >= FAST_LEG_MIN
                and abs(self.x) - IR_X_OFFSET <= O1_SEEN_MAX):
            self.fwd(self.y - slow, STRAIGHT_SPEED)   # only split off when the pass is slower
        plan, turn = self.entry_plan(min(self.y, clear_o2))
        if plan is None:
            plan, turn = self._fallback_plan(min(self.y, clear_o2), 'no carpark entry plan from this lane (45 or 90 deg)')
        d, ey = plan
        if ey > self.y - 2:
            # Already level with (or past) the entry start, e.g. after a shift
            # that ended past O1: reverse onto it (choose_entry checked that
            # straight) and go in on dead reckoning.
            self.log(f'entry start {ey:.1f} is behind the car: reversing onto it, no O1 pass')
            self.rev(ey - self.y)        # under 2 cm: no move
            p = Poses(self.x, 0.0, d, turn)
            if not entry_clear(p, self.y, self.o1, self.o2):
                self._not_clear('entry no longer clear at the actual start')
            self.log(f'entry ({turn} deg): start y {self.y:.1f}, leg {d:.0f}')
            self.s_curve(d, turn)
            return
        # One exact speed-30 move to the dead-reckoned entry start, watching the
        # centre-side IR. Encoder distance stays exact; only the y fix uses time.
        y0, x0 = self.y, self.x
        ir_side = 'left' if self.x < 0 else 'right'
        look_y = self.o1 + 10 + O1_LOOK_AHEAD     # IR y below which O1 can be in view
        yfix_ok = PASS_YFIX and y0 - IR_AHEAD > self.o1 + 10 + 2
        st = {'t0': time.monotonic(), 'hits': 0, 'first': None, 'min': None, 'seen': None}

        def watch(frame):
            travelled = max(0.0, time.monotonic() - st['t0'] - STARTUP_S) * PASS_CM_PER_S
            if y0 - travelled - IR_AHEAD > look_y or st['seen']:
                return
            v = frame[ir_side]
            if 5 <= v <= O1_SEEN_MAX:
                if not st['hits']:
                    st['first'], st['min'] = travelled, v
                st['min'] = min(st['min'], v)
                st['hits'] += 1
                if st['hits'] >= PASS_HITS:
                    st['seen'] = (st['min'], st['first'])
            else:
                st['hits'] = 0

        # With the slanted entry the scan only needs to take the IR past O1's
        # front face: end level with it, nearer the carpark start of the slant.
        # Always take the IR past O1's front face, even if an S entry would
        # start earlier (it can reverse onto its start): the x fix needs it.
        scan_end = (self.turn_point if self.turn_point is not None else self.o1 - 2) if SLANT_ENTRY else ey
        if scan_end > y0 - 2:
            scan_end = ey
        self.log(f'O1 pass: speed {O1_PASS_SPEED} to y {scan_end:.1f} (S entry would start at {ey:.1f}), {ir_side} IR watching')
        can_see = abs(self.x) - IR_X_OFFSET <= O1_SEEN_MAX
        if not can_see:
            self.log(f'lane {abs(self.x):.0f} cm out: the side IR cannot reach O1, full speed past it')
        self.fwd(y0 - scan_end, O1_PASS_SPEED if can_see else STRAIGHT_SPEED, watch if can_see else None)
        if st['seen']:
            gap, first = st['seen']
            dx = math.copysign(gap + IR_X_OFFSET, x0) - self.x
            dy = ((self.o1 + 10 + IR_AHEAD) - (y0 - first)) if yfix_ok else 0.0
            if abs(dx) <= O1_FIX_MAX and abs(dy) <= O1_YFIX_MAX:
                self.x += dx
                self.y += dy
                self.log(f'O1 seen at IR {gap:.1f}: fix dx {dx:+.1f} dy {dy:+.1f}')
            else:
                self.log(f'O1 fix dx {dx:+.1f} dy {dy:+.1f} rejected')
        else:
            self.log('O1 not seen: keeping dead reckoning')
        O1_EXTRA[0] = 0.0 if st['seen'] else O1_UNFIXED_MARGIN
        if SLANT_ENTRY and self.slant_entry():
            return True
        here = self.entry_here()
        if here is not None:
            # The entry is already clear from where the scan stopped (with
            # 2 cm either side, as the firmware's >= 5 cm interval): turn in
            # now instead of a correction move (start, 10 cm crawl, settle).
            d, turn = here
            self.log(f'entry ({turn} deg): clear from here, no correction move, leg {d:.0f}')
            self.s_curve(d, turn)
            return
        plan, turn = self.entry_plan()
        if plan is None:
            plan, turn = self._fallback_plan(self.y, 'no carpark entry plan from the fixed pose')
        d, ey = plan
        move = self.y - ey
        if move >= 2:
            self.fwd(move, STRAIGHT_SPEED)
        elif move <= -2:
            self.rev(-move)
        p = Poses(self.x, 0.0, d, turn)
        if not entry_clear(p, self.y, self.o1, self.o2):
            self._not_clear('entry no longer clear at the actual start')
        self.log(f'entry ({turn} deg): start y {self.y:.1f}, leg {d:.0f}')
        self.s_curve(d, turn)

    def slant_entry(self):
        """One turn toward the centre line, one straight into the carpark.
        True when parked; False (pose back to heading 180) to use the S entry."""
        side = 'left' if self.x < 0 else 'right'      # facing home, left is +x
        th = self._best_slant(self.x, self.y, self.h)
        if th is None and abs(self.h - 180) < 1:
            # Not from here (the turn would sweep O1, or end off-centre): the
            # same checked turn from further down the lane (robot 2026-10-16:
            # lane 18.8 just past O1, too narrow for any S).
            tp = self._best_turn_point(self.x, self.y - 2)
            if tp is not None:
                self.log(f'slant: not from y {self.y:.1f}; straight on to y {tp[0]:.1f} first')
                self.fwd(self.y - tp[0], STRAIGHT_SPEED)
                th = tp[1]
        if th is None:
            self.log('slant: no angle ends inside the carpark for every carpark position; S entry')
            return False
        self.log(f'slant entry: turn {side} {th:.0f} deg toward the centre line')
        self.turn(side, int(th))
        return self.finish_slant()

    def _turn_sweeps_o1(self, x, y, h, side, th):
        sweep = [self._after_turn(x, y, h, side, th * k / 5) for k in range(1, 6)]
        px = np.array([q[0] for q in sweep]); py = np.array([q[1] for q in sweep])
        hh = np.radians(np.array([q[2] for q in sweep]))
        e = O1_EXTRA[0]
        return bool(np.any(overlap(px, py, np.cos(hh), np.sin(hh), (-5 - e, self.o1 - e, 5 + e, self.o1 + 10 + e))))

    def _best_turn_point(self, x, y_hi):
        """(turn y, angle) for the one-turn entry: the lane straight runs on to
        the turn point, anywhere from y_hi (in front of O1) down to just
        outside the carpark. The plan ending nearest the centre wins; the
        earlier turn on a tie. None if no turn point works."""
        best = None
        y_lo = mouth(self.o1) + 25
        y = y_hi
        while y >= y_lo:
            th = self._best_slant(x, y, 180.0)
            if th is not None:
                side = 'left' if x < 0 else 'right'
                x1, y1, h1 = self._after_turn(x, y, 180.0, side, th)
                worst = self._slant_why(x1, y1, h1, score=True)[1]
                if best is None or worst < best[0] - 0.5:
                    best = (worst, y, th)
            y -= 4
        return (best[1], best[2]) if best else None

    def _best_slant(self, x, y, h):
        """Whole-degree turn-in for the one-turn entry from (x, y, h), or None:
        of the angles whose turn clears O1 and whose straight ends inside the
        carpark for every carpark position, the one ending nearest the centre."""
        side = 'left' if x < 0 else 'right'
        best = None
        for th in range(int(SLANT_MIN_DEG), int(min(SLANT_MAX_DEG, SLANT_OPENING_MAX_DEG)) + 1):
            x1, y1, h1 = self._after_turn(x, y, h, side, th)
            why, worst = self._slant_why(x1, y1, h1, score=True)
            if why or self._turn_sweeps_o1(x, y, h, side, th):
                continue
            if best is None or worst < best[0]:
                best = (worst, th)
        return best[1] if best else None

    def _slant_why(self, x, y, h, score=False):
        """None if a slanted straight from (x, y, h) ends inside the carpark for
        every back-wall position the rules allow; else why not."""
        th = abs((h - 180 + 180) % 360 - 180)
        st, ct = math.sin(math.radians(th)), math.cos(math.radians(th))
        r1 = math.radians(h)
        ux, uy = -math.sin(r1), math.cos(r1)
        reach, half_x = 11.5 * ct + 9.4 * st, 9.4 * ct + 11.5 * st
        gap = max(SLANT_GAP, SLANT_CORNER_GAP + 9.4 * st)
        m_hi = mouth(self.o1)
        worst = 0.0
        if uy >= 0 or ux * x >= 0:
            return ('it would not head home toward the centre line', 1e9) if score else 'it would not head home toward the centre line'
        walls = sorted({max(-38.5, min(-11.5, m_hi - 50)), -38.5})
        # With an echo the run is sized to the real wall (each wall below). With
        # none it is calculated from the shallowest wall (+margin) whatever the
        # real wall is, so that stop is checked against every wall too.
        no_echo_run = ((y + 11.5 * uy) - walls[-1]) / ct - (gap + SLANT_NO_ECHO_MARGIN) / ct
        for wall in walls:
            runs = [((y + 11.5 * uy) - wall) / ct - gap / ct]
            if th > SLANT_ECHO_TRUSTED_DEG:
                runs.append(no_echo_run)
            for run in runs:
                xf, yf = x + run * ux, y + run * uy
                m = wall + 50
                x_mouth = x + ((m - y) / uy) * ux if uy else xf
                why = None
                if abs(x_mouth) + half_x > 28 or abs(xf) + half_x > 28:
                    why = f'it would pass the carpark opening at x {x_mouth:+.1f}'
                elif yf + reach > m - 2:
                    why = f'its rear would stick out ({yf + reach:.1f} > opening {m:.1f})'
                if why:
                    return (why, 1e9) if score else why
                worst = max(worst, abs(xf))
        return (None, worst) if score else None

    def finish_slant(self):
        """Already heading into the carpark at a slant, one straight in, in two
        parts on the same heading (no extra turn):
        A - by calculation, to just outside the highest the opening can be
            (no carpark wall can be reached, however deep the carpark is);
        B - the rest, sized by the ultrasonic on the back wall, now close.
        No echo near the end: B by calculation, to the shallowest the back
        wall can be. False (after turning back to heading 180) only when the
        geometry check fails."""
        off = (self.h - 180 + 180) % 360 - 180            # + = turned left of 180
        th = abs(off)
        side = 'left' if off > 0 else 'right'
        st, ct = math.sin(math.radians(th)), math.cos(math.radians(th))
        r1 = math.radians(self.h)
        ux, uy = -math.sin(r1), math.cos(r1)
        reach = 11.5 * ct + 9.4 * st          # centre to the body point nearest the back wall
        half_x = 9.4 * ct + 11.5 * st         # body half-extent across
        m_hi = mouth(self.o1)                 # the opening is at most here (rules: O1 >= 60 cm away)
        gap = max(SLANT_GAP, SLANT_CORNER_GAP + 9.4 * st)

        def check(wall, x0, y0):
            """None if driving in from (x0, y0) to `gap` off a back wall at `wall`
            ends inside a carpark whose opening is at wall + 50; else why not."""
            run = ((y0 + 11.5 * uy) - wall) / ct - gap / ct
            xf, yf = x0 + run * ux, y0 + run * uy
            m = wall + 50
            x_mouth = x0 + ((m - y0) / uy) * ux if uy else xf
            if abs(x_mouth) + half_x > 28 or abs(xf) + half_x > 28:
                return f'it would pass the carpark opening at x {x_mouth:+.1f}'
            if yf + reach > m - 2:
                return f'its rear would stick out ({yf + reach:.1f} > opening {m:.1f})'
            return None

        # Before committing: the back wall is somewhere from -38.5 (rear gap 27
        # at the start) to m_hi - 50 (opening at most 57 cm from O1). The entry
        # must work for both, since the car will be at the opening before it
        # can measure.
        walls = sorted({max(-38.5, min(-11.5, m_hi - 50)), -38.5})
        why = next((w for w in (check(wl, self.x, self.y) for wl in walls) if w), None)
        if why:
            self.log(f'slant: {why} (some carpark positions); turning back for the S entry')
            if round(th) >= 1:
                self.turn('right' if side == 'left' else 'left', int(round(th)))
            self.h = 180.0
            return False
        # Part A: stop 5 cm above the highest possible opening.
        run_a = (self.y - (m_hi + 5 + reach)) / ct
        if run_a >= 2:
            self.log(f'slant: {run_a:.0f} cm to just outside the opening (y {m_hi + 5 + reach:.0f}), by calculation')
            self.fwd(run_a, STRAIGHT_SPEED)
        # Part B: the back wall is now within ~40-70 cm. The beam can also catch
        # the side wall's end at the opening (robot 2026-10-15), so only echoes
        # that put the wall where a back wall can be count.
        values = self._back_wall_echoes(list(self.recent)[-6:])
        if len(values) < 2:
            values = self._back_wall_echoes(list(self.recent)[-6:] + self.io.sensors(5))
        front = self.y + 11.5 * uy
        lo, hi = walls[0], walls[-1]          # where the rules allow the back wall
        if len(values) >= 2:
            us = statistics.median(values)
            # The echo distance is exact: it sizes the run. The wall position
            # from it carries the odometry y error, so clamp it to what the
            # rules allow before the geometry recheck.
            run = us - gap / ct
            wall = min(hi, max(lo, front - us * ct))
            source = f'ultrasonic {us:.1f}'
        else:
            wall = hi                         # the shallowest the back wall can be
            run = (front - wall) / ct - (gap + SLANT_NO_ECHO_MARGIN) / ct
            source = 'NO ECHO, calculated from the shallowest possible back wall'
        xf = self.x + run * ux
        why = check(wall, self.x, self.y)
        if why:
            # At the opening already: no turn-back fits here. Stop outside.
            raise ReturnError(f'slant: {why} with the back wall measured at y {wall:.1f}; stopped outside the carpark')
        self.log(f'slant entry: {source} (wall y {wall:.1f}), {run:.0f} cm in at {th:.0f} deg, ends x {xf:+.1f}')
        self.fwd(run, STRAIGHT_SPEED)
        return True

    def _back_wall_echoes(self, frames):
        """Ultrasonic readings (heading settled within 3 deg) that put the back
        wall at y -42..-8; the largest group within 4 cm of each other."""
        r = math.radians(self.h)
        uy = math.cos(r)
        ct = abs(uy)
        front_y = self.y + 11.5 * uy
        vals = [fr['us'] for fr in frames if fr['us_valid'] and 20 < fr['us'] < 300
                and abs((fr['heading'] - self.origin - self.h + 180) % 360 - 180) <= 3
                and -42 <= front_y - fr['us'] * ct <= -8]
        best = []
        for v in vals:
            group = [w for w in vals if abs(w - v) <= 4]
            if len(group) > len(best):
                best = group
        return best

    # ------------------------------------------------------ two-step return
    @staticmethod
    def _after_turn(x, y, h, side, angle):
        f, sd = turn_offsets(side, angle)
        r = math.radians(h)
        return (x + sd * math.cos(r) - f * math.sin(r), y + f * math.cos(r) + sd * math.sin(r),
                h + (angle if side == 'left' else -angle))

    def _arc(self, x, y, h, side, angle, n=6):
        return [self._after_turn(x, y, h, side, angle * k / n) for k in range(1, n + 1)]

    @staticmethod
    def _line(x, y, h, length, step=2.0):
        r = math.radians(h)
        n = max(1, int(length / step))
        return [(x - length * k / n * math.sin(r), y + length * k / n * math.cos(r), h) for k in range(n + 1)]

    def _clear(self, poses):
        px = np.array([q[0] for q in poses]); py = np.array([q[1] for q in poses])
        hh = np.radians(np.array([q[2] for q in poses]))
        m = DIRECT_O1_MARGIN
        o1box = (-5 - m, self.o1 - m, 5 + m, self.o1 + 10 + m)
        o2box = (-self.o2h, self.o2, self.o2h, self.o2 + 10)
        c, sn = np.cos(hh), np.sin(hh)
        return not (np.any(overlap(px, py, c, sn, o1box)) or np.any(overlap(px, py, c, sn, o2box)))

    def plan_direct(self):
        """(start y, turn 1, leg 1, turn 2, leg-2 angle) or None.
        Turns are (side, whole degrees). W runs over lanes 22-40 cm out and
        2-16 cm in front of O1; the quickest clear plan wins."""
        sg = 1.0 if self.x >= 0 else -1.0
        ys = min(self.y, self.o2 - 18.5)                 # body clear of O2's face before turning
        y_aim = mouth(self.o1) - 25
        best = None
        for dy in range(2, 17, 2):
            yw = self.o1 - dy
            for xa in range(22, 41):
                xw = sg * xa
                th2 = math.degrees(math.atan2(xa, yw - y_aim))
                if not SLANT_MIN_DEG <= th2 <= SLANT_MAX_DEG:
                    continue
                # Turn 1 from heading 180 toward W; leg 1 is then aimed from the turn's end.
                th1 = int(round(math.degrees(math.atan2(abs(self.x) - xa, ys - yw))))
                if abs(th1) > DIRECT_LEG1_MAX_DEG:
                    continue
                side1 = ('right' if sg > 0 else 'left') if th1 >= 0 else ('left' if sg > 0 else 'right')
                x1, y1, h1 = self._after_turn(self.x, ys, 180.0, side1, abs(th1)) if th1 else (self.x, ys, 180.0)
                r1 = math.radians(h1)
                u1 = (-math.sin(r1), math.cos(r1))
                leg1 = (xw - x1) * u1[0] + (yw - y1) * u1[1]
                if leg1 < 2:
                    continue
                xe, ye = x1 + leg1 * u1[0], y1 + leg1 * u1[1]
                # Turn 2 onto the leg-2 heading toward the centre line.
                h2 = 180.0 - sg * th2
                amt = int(round(h2 - h1))
                if abs(amt) > 90:
                    continue
                side2 = 'left' if amt > 0 else 'right'
                x2, y2, h2r = self._after_turn(xe, ye, h1, side2, abs(amt)) if amt else (xe, ye, h1)
                r2 = math.radians(h2r)
                u2 = (-math.sin(r2), math.cos(r2))
                if u2[1] >= 0 or u2[0] * x2 >= 0:
                    continue                                  # must head home and toward x = 0
                x_end = x2 + ((y_aim - y2) / u2[1]) * u2[0]
                if abs(x_end) > 10:
                    continue
                # Sweep: turn 1, leg 1, turn 2 and leg 2 down to O1's front - 20.
                poses = (self._arc(self.x, ys, 180.0, side1, abs(th1)) if th1 else [])
                poses += self._line(x1, y1, h1, leg1)
                poses += self._arc(xe, ye, h1, side2, abs(amt)) if amt else []
                run2 = max(0.0, (y2 - (self.o1 - 20)) / -u2[1])
                poses += self._line(x2, y2, h2r, run2)
                if not self._clear(poses):
                    continue
                # No IR scan on this route: x is odometry only. Take the plan
                # that keeps the most room to O1 (within the 18 deg leg-2 limit),
                # then the shortest.
                room = min(body_gap(q[0], q[1], q[2], (-5, self.o1, 5, self.o1 + 10)) for q in poses)
                cost = (self.y - ys) + leg1 + 0.4 * (abs(th1) + abs(amt))   # ~0.4 cm per degree of turn
                key = (round(room), -cost)
                if best is None or key > best[0]:
                    best = (key, ys, (side1, abs(th1)), leg1, (side2, abs(amt)), th2, xw, yw, room)
        if best:
            self.direct_room = best[-1]
        return best[1:-1] if best else None

    def direct_return(self):
        """True when parked; False when no two-step plan fits (caller falls back)."""
        plan = self.plan_direct()
        if plan is None:
            self.log('two-step: no plan clears O1 and O2 here; using the shift/scan route')
            return False
        ys, (side1, a1), leg1, (side2, a2), th2, xw, yw = plan
        self.log(f'two-step: waypoint ({xw:+.0f}, {yw:.0f}), {self.direct_room:.1f} cm from O1 as planned; '
                 f'turn {side1} {a1}, {leg1:.0f} cm, turn {side2} {a2}, then {th2:.0f} deg into the carpark')
        self.fwd(self.y - ys, STRAIGHT_SPEED)
        if a1:
            self.turn(side1, a1)
        self.fwd(leg1, STRAIGHT_SPEED)
        if a2:
            self.turn(side2, a2)
        if self.finish_slant():
            return True
        # No echo or a check failed: heading 180 again, the S entry from here.
        here = self.entry_here()
        if here is not None:
            d, turn = here
        else:
            plan, turn = self.entry_plan()
            if plan is None:
                plan, turn = self._fallback_plan(self.y, 'no carpark entry plan after the two-step return')
            d, ey = plan
            move = self.y - ey
            if move >= 2:
                self.fwd(move, STRAIGHT_SPEED)
            elif move <= -2:
                self.rev(-move)
        self.log(f'entry ({turn} deg): start y {self.y:.1f}, leg {d:.0f}')
        self.s_curve(d, turn)
        self.park()
        return True

    def entry_here(self):
        """(leg, turn) if the centring move is clear from the current pose,
        and from 2 cm either side of it; 45 deg first."""
        for turn in (45, 90):
            d = leg_for(self.x, abs(self.x), turn)
            if turn > 60 and d < 0:
                continue
            if turn <= 60 and d == 1:
                d = 2
            p = Poses(self.x, 0.0, d, turn)
            if p.ok and all(entry_clear(p, self.y + k, self.o1, self.o2) for k in (-2, 0, 2)):
                return d, turn
        return None

    def _not_clear(self, why):
        raise ReturnError(why)

    def _fallback_plan(self, from_y, why):
        """No entry with the 5 cm extra O1 margin (kept while the side IR has
        not confirmed x): the same fully checked planner with that extra
        margin halved, then off; the body clearance checks all stay. Robot
        2026-10-16 (O1 74.7, lane 23.8): the margin hid the only start window
        (y 59-73). Nothing fits even then: stop."""
        margin = O1_EXTRA[0]
        for extra in (margin / 2, 0.0):
            if extra >= margin:
                continue
            O1_EXTRA[0] = extra
            plan, turn = self.entry_plan(from_y)
            if plan is not None:
                self.log(f'{why} with {margin:.1f} cm extra O1 margin; checked {turn} deg entry '
                         f'from y {plan[1]:.1f} fits with {extra:.1f}')
                return plan, turn
        O1_EXTRA[0] = margin
        raise ReturnError(why)

    def entry_plan(self, from_y=None):
        """45 deg S first; v9's 90 deg crossing in front of the carpark if not."""
        cy = self.y if from_y is None else from_y
        for turn in (45, 90):
            plan = choose_entry(self.x, cy, self.return_y, self.o1, self.o2, turn)
            if plan is not None:
                return plan, turn
        return None, 45

    def park(self):
        # Frames from the end of the last turn, once its heading had settled,
        # so parking does not stop to measure; wait for fresh ones only if
        # fewer than 2 agree.
        def readings(frames):
            return [f['us'] for f in frames if f['us_valid'] and 2 < f['us'] < 200
                    and abs((f['heading'] - self.origin - self.h + 180) % 360 - 180) <= 3]
        values = readings(list(self.recent)[-3:])
        need = 2
        if len(values) < need or max(values) - min(values) > 4:
            values = readings(self.io.sensors(5))
            need = 3
        # The back wall can only be between the shallowest and the deepest
        # carpark (rules): a missing or wild reading is replaced from that.
        front = self.y - 11.5
        shallow = front - max(-38.5, min(-11.5, mouth(self.o1) - 50))
        deep = front + 38.5
        if len(values) < need:
            if not NO_STALL:
                raise ReturnError(f'no stable ultrasonic reading to park ({values})')
            us = min(deep, max(shallow, statistics.median(values))) if values else shallow
            self.log(f'WARNING park: no stable ultrasonic ({values}); using {us:.1f}')
        else:
            middle = statistics.median(values)
            agree = [v for v in values if abs(v - middle) <= 4]
            if len(agree) < need:
                if not NO_STALL:
                    raise ReturnError(f'ultrasonic readings disagree ({values})')
                us = min(deep, max(shallow, middle))
                self.log(f'WARNING park: ultrasonic readings disagree ({values}); using {us:.1f}')
            else:
                us = statistics.median(agree)
        expected = self.y - 11.5 + 25      # back wall ~25 behind the start centre (firmware PARK model)
        self.log(f'park: back wall expected ~{expected:.0f}, ultrasonic {us:.1f}')
        # The back wall is at most 38.5 behind the start centre (50 cm carpark,
        # robot placed anywhere inside): never drive past PARK_GAP from there,
        # whatever the ultrasonic says.
        limit = self.y - 11.5 + 38.5 - PARK_GAP
        drive = min(us - PARK_GAP, limit)
        if drive < us - PARK_GAP:
            self.log(f'park: ultrasonic {us:.1f} is beyond the deepest wall; driving only {drive:.0f}')
        if drive >= 2:
            self.fwd(drive, STRAIGHT_SPEED)
