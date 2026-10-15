/* Host simulation of the fast O1 phases in Core/Inc/kc_task2.h.
 * cc -std=c11 -O1 -Wall -Wextra -Werror -fsanitize=address,undefined -ICore/Inc test_fast_o1.c -lm -o /tmp/t && /tmp/t
 *
 * The simulated robot uses the fitted bicycle model (rear axle), a servo that
 * slews lock-to-lock over `ramp` cm, a motor lag, and an HC-SR04 that only
 * echoes from a face within 15 deg of square. Radii can be scaled to check
 * that odometry-based switching absorbs calibration error.
 */
#include <stdio.h>
#include <stdlib.h>
#include "kc_task2.h"

typedef struct { double x, y, h, k, v; } Bot; /* rear axle, x fwd, y left */
static Bot bot;
static int drive_mode = -1;
static double drive_off, drive_pwm, sim_rl, sim_rr, sim_ramp, sim_ramp_rl;
static double sim_rev_l, sim_rev_r, sim_brake = 0.03, sim_odo = 1.0, servo; /* steering as rolling curvature, 1/cm, left + */
static double o2_half = 50, clr2_min, sim_krev = 1.0;  /* reverse radius error on top of kR */
/* Robot 2026-10-15: entering the inward lock from the lane hold, the heading came
 * round ~7 cm later than this model. Modelled as a steering dead distance. */
static double sim_dead = 13.0, servo_hold_until = -1, servo_held;
static int prev_mode = -1;
/* Robot: after the diagonal brake the car skids ~5 cm the encoders never see.
 * The sim moves the true pose over the first 100 ms of the brake, not the odometry. */
static double sim_skid = 5.0, off_x, off_y;
static int prev_fmode;
static int braked;
static char last_error[64];
static int verbose;
static int clr_mode;

static void h_start(int m, unsigned s, unsigned a) { (void)m; (void)s; (void)a; }
static float h_stop(void) { drive_mode = -1; braked = 1; return 0; }
static void h_event(const char *n, float a, float b) {
    if (!strncmp(n, "ERROR_", 6)) snprintf(last_error, sizeof last_error, "%s", n);
    if (verbose && strcmp(n, "STATE")) printf("  %-16s %8.2f %8.2f\n", n, a, b);
}
static void h_drive(int m, float off, unsigned pwm) { drive_mode = m; drive_off = off; drive_pwm = pwm; braked = 0; }

/* body vs O1 square, separating-axis lower bound */
static double sep(double A[4][2], double B[4][2]) {
    double best = -1e9;
    for (int pass = 0; pass < 2; pass++) {
        double (*P)[2] = pass ? B : A, (*Q)[2] = pass ? A : B;
        for (int i = 0; i < 4; i++) {
            double ex = P[(i + 1) % 4][0] - P[i][0], ey = P[(i + 1) % 4][1] - P[i][1];
            double nx = ey, ny = -ex, n = sqrt(nx * nx + ny * ny); nx /= n; ny /= n;
            double pmin = 1e9, pmax = -1e9, qmin = 1e9, qmax = -1e9;
            for (int j = 0; j < 4; j++) {
                double p = P[j][0] * nx + P[j][1] * ny, q = Q[j][0] * nx + Q[j][1] * ny;
                if (p < pmin) pmin = p; if (p > pmax) pmax = p;
                if (q < qmin) qmin = q; if (q > qmax) qmax = q;
            }
            if (qmin - pmax > best) best = qmin - pmax;
            if (pmin - qmax > best) best = pmin - qmax;
        }
    }
    return best;
}
static double clearance(double x1) {
    double rear = 23.0 / 2 - 8.2, front = 23.0 / 2 + 8.2, hw = 9.4, c = cos(bot.h), s = sin(bot.h);
    double lx[4] = {-rear, front, front, -rear}, ly[4] = {-hw, -hw, hw, hw}, R[4][2];
    for (int j = 0; j < 4; j++) { R[j][0] = bot.x + lx[j] * c - ly[j] * s; R[j][1] = bot.y + lx[j] * s + ly[j] * c; }
    double O[4][2] = {{x1, -5}, {x1 + 10, -5}, {x1 + 10, 5}, {x1, 5}};
    return sep(R, O);
}
static void body(double R[4][2]) {
    double rear = 23.0 / 2 - 8.2, front = 23.0 / 2 + 8.2, hw = 9.4, c = cos(bot.h), s = sin(bot.h);
    double lx[4] = {-rear, front, front, -rear}, ly[4] = {-hw, -hw, hw, hw};
    for (int j = 0; j < 4; j++) { R[j][0] = bot.x + lx[j] * c - ly[j] * s; R[j][1] = bot.y + lx[j] * s + ly[j] * c; }
}
static double clearance2(double x2) {
    double R[4][2], O[4][2] = {{x2, -o2_half}, {x2 + 10, -o2_half}, {x2 + 10, o2_half}, {x2, o2_half}};
    body(R);
    return sep(R, O);
}
/* US from front bumper centre: nearest of O1 face / O2 face if squarely hit */
static double us_read(double x1, double x2, double o2w) {
    double fx = bot.x + 19.7 * cos(bot.h), fy = bot.y + 19.7 * sin(bot.h);
    if (fabs(bot.h) > 15 * M_PI / 180) return -1;
    double best = -1, faces[2][2] = {{x1, 6.5}, {x2, o2w / 2 + 1.5}};
    for (int f = 0; f < 2; f++) {
        double d = (faces[f][0] - fx) / cos(bot.h), yy = fy + d * sin(bot.h);
        if (d > 2 && d < 180 && fabs(yy) <= faces[f][1] && (best < 0 || d < best)) best = d;
    }
    return best;
}

typedef struct { int ok; double clr, clr2, end_y, end_h, end_x, t_ms, range2; } Result;

static Result run(int arrow, double d0, double gap, double kR, double ramp, double y0, int arrow_ms) {
    KcTask2 t; KcT2Hooks hooks = {h_start, h_stop, h_event, h_drive};
    memset(&bot, 0, sizeof bot); bot.x = -8.2; bot.y = y0;   /* robot centre at 0, as in firmware */
    /* `ramp` is now a scale on the measured steering lag TIME (1.0 = 0.445 s L->R) */
    sim_rl = KCF_R_LEFT * kR; sim_rr = KCF_R_RIGHT * kR; sim_ramp = ramp; sim_ramp_rl = ramp;
    /* robot: reverse radius did not follow the forward one (see kc_fast_o1.h) */
    sim_rev_l = KCF_RS_LEFT * sim_krev; sim_rev_r = KCF_RS_RIGHT * sim_krev;
    drive_mode = -1; braked = 1; last_error[0] = 0; servo = 0; clr2_min = 1e9; servo_hold_until = -1; prev_mode = -1;
    off_x = off_y = 0; prev_fmode = 0;
    uint32_t skid_from = 0;
    double x1 = 11.5 + d0;                     /* O1 near face; front bumper starts at 11.5 */
    double x2 = x1 + 10 + gap;
    double odist = 0, us = -1, heading0 = 0, v_dist = 0; uint32_t us_seq = 0, us_at = 0;
    KcT2Input in = {0};
    in.gyro_ready = 1; in.imu_at = 0; in.heading = 0; in.us_valid = 0;
    in.ox = 0; in.oy = -8.2;
    if (!kc_start(&t, hooks, &in, 77, 1)) { Result r = {0}; return r; }
    double clr = 1e9; uint32_t now = 0;
    for (; now < 30000 && t.active; now++) {
        double dt = 0.001;
        /* actuator model */
        /* Steering slews lock to lock in KCF_T_LR / KCF_T_RL seconds (x lag scale), moving
         * or not, linear in curvature: the servo test's yaw rate crossed the midpoint of the
         * two lock rates ~55% into a left-to-right flip, as this predicts (54%). servo = rolling
         * curvature; reversing uses the standstill-fit radius at the same servo position. */
        double st = 0, vt = 0, brake = braked;
        int m = drive_mode;
        if (m == KCF_DRIVE_LEFT || m == KCF_DRIVE_REV_LEFT || m == KCF_DRIVE_BRAKE_LEFT) st = 1 / sim_rl;
        else if (m == KCF_DRIVE_RIGHT || m == KCF_DRIVE_REV_RIGHT || m == KCF_DRIVE_BRAKE_RIGHT) st = -1 / sim_rr;
        else if (m == KCF_DRIVE_STRAIGHT) st = drive_off < 0 ? -drive_off / 48.4 / sim_rl : -drive_off / 59.6 / sim_rr;
        if (m >= KCF_DRIVE_BRAKE) brake = 1;
        if (prev_mode == KCF_DRIVE_STRAIGHT && (m == KCF_DRIVE_LEFT || m == KCF_DRIVE_RIGHT) && t.fmode == 4) {
            servo_hold_until = now + sim_dead / (bot.v > 20 ? bot.v : 20) * 1000; servo_held = servo;
        }
        prev_mode = m;
        if (now < servo_hold_until) st = servo_held;
        if (m >= 0 && !brake) {
            vt = drive_pwm * 58.5 / 3550.0;      /* servo test: PWM 3550 = 58.5 cm/s straight */
            if (m != KCF_DRIVE_STRAIGHT && m != KCF_DRIVE_REV_STRAIGHT) vt *= 0.75;  /* coasting inner wheel: 42-48 cm/s */
            if (m == KCF_DRIVE_REV_LEFT || m == KCF_DRIVE_REV_RIGHT || m == KCF_DRIVE_REV_STRAIGHT) vt = -vt;
        }
        double dsv = st - servo, srate = (1 / sim_rl + 1 / sim_rr) / ((dsv < 0 ? KCF_T_LR : KCF_T_RL) * sim_ramp) * dt;
        if (dsv > srate) dsv = srate; else if (dsv < -srate) dsv = -srate;
        servo += dsv;
        bot.v += (vt - bot.v) * (brake ? sim_brake : 0.012);  /* ~80 ms lag; brake rate varied */
        double ds = bot.v * dt;
        bot.k = bot.v >= 0 ? servo : servo * (servo > 0 ? sim_rl / sim_rev_l : sim_rr / sim_rev_r);
        bot.h += bot.k * ds; bot.x += cos(bot.h) * ds; bot.y += sin(bot.h) * ds;
        odist += ds * sim_odo;
        if (t.phase == KCT_F_SWERVE && prev_fmode == 9 && t.fmode == 10) skid_from = now;
        if (t.phase == KCT_F_SWERVE) prev_fmode = t.fmode;
        if (skid_from && now - skid_from < 100) {
            double e = sim_skid / 100;
            bot.x += cos(bot.h) * e; bot.y += sin(bot.h) * e; off_x += cos(bot.h) * e; off_y += sin(bot.h) * e;
        }
        if (now % 90 == 0) { us = us_read(x1, x2, 30); us_seq++; us_at = now; }
        double c = clearance(x1); if (c < clr) { clr = c; clr_mode = t.phase == KCT_F_SWERVE ? t.fmode : -1; }
        if (c < 0) { snprintf(last_error, sizeof last_error, "COLLISION_O1"); break; }
        double c2 = clearance2(x2); if (c2 < clr2_min) clr2_min = c2;
        if (c2 < 0) { snprintf(last_error, sizeof last_error, "COLLISION_O2"); break; }
        /* input */
        in.now = now; in.imu_at = now; in.heading = (float)(heading0 + bot.h * 180 / M_PI);
        in.gyro = (float)(bot.k * bot.v * 180 / M_PI);
        in.us_seq = us_seq; in.us_at = us_at; in.us = (float)us; in.us_valid = us > 0;
        in.ox = (float)(-(bot.y - off_y)); in.oy = (float)(bot.x - off_x); in.odist = (float)odist;
        if (now % 50 == 0) { in.speed = (float)((odist - v_dist) * 20.0); v_dist = odist; }  /* firmware: 50 ms window */
        t.heartbeat = now;
        if ((int)now == arrow_ms) kc_arrow(&t, 77, 1, (unsigned)arrow);
        kc_step(&t, &in);
    }
    Result r = {t.phase == KCT_DONE, clr, clr2_min, bot.y, bot.h * 180 / M_PI, bot.x + 19.7 - (x1 + 10), now,
                x2 - (bot.x + 19.7)};
    return r;
}

static double max_abs(double a, double b) { return fabs(a) > b ? fabs(a) : b; }

int main(int argc, char **argv) {
    if (getenv("DEAD")) sim_dead = atof(getenv("DEAD"));
    if (getenv("SKID")) sim_skid = atof(getenv("SKID"));
    /* ROBOTLIKE=1: spreads seen across the 2026-10-15 robot runs (F_RAMP 24.2-24.8, radius
     * 0.97-0.99, turn-in within +-0.9 cm), doubled. Default: the wider stress sweep. */
    int robotlike = getenv("ROBOTLIKE") != NULL;
    if (argc == 10) {  /* single verbose run: arrow(1|2) d0 gap kR lag brake y0 arrow_ms o2_half */
        verbose = 1; sim_brake = atof(argv[6]); o2_half = atof(argv[9]);
        Result r = run(atoi(argv[1]), atof(argv[2]), atof(argv[3]), atof(argv[4]), atof(argv[5]), atof(argv[7]), atoi(argv[8]));
        printf("ok=%d O1 clr=%.1f (mode %d) O2 clr=%.1f end y=%.1f h=%.1f front past O1=%.1f range to O2=%.1f t=%.0fms %s\n",
               r.ok, r.clr, clr_mode, r.clr2, r.end_y, r.end_h, r.end_x, r.range2, r.t_ms, last_error);
        return 0;
    }
    int fails = 0, n = 0;
    for (int arrow = 1; arrow <= 2; arrow++) {
        double w1 = 1e9, w2 = 1e9, wy = 0, wh = 0, rmin = 1e9, rmax = -1e9, tmax = 0;
        for (int di = 0; di < 2; di++)
        for (int gi = 0; gi < 4; gi++)
        for (int ki = 0; ki < 3; ki++)
        for (int ri = 0; ri < 3; ri++)
        for (int bi = 0; bi < 3; bi++)
        for (int yi = 0; yi < 2; yi++)
        for (int oi = 0; oi < 2; oi++)
        for (int ai = 0; ai < 2; ai++)
        for (int vi = 0; vi < 3; vi++)
        for (int ei = 0; ei < 3; ei++)
        for (int si = 0; si < 3; si++) {
            sim_skid = robotlike ? (double[]){4, 5, 6}[si] : (double[]){2.5, 5, 7.5}[si];
            sim_krev = robotlike ? (double[]){0.97, 1.0, 1.03}[vi] : (double[]){0.94, 1.0, 1.06}[vi];
            sim_dead = robotlike ? (double[]){12, 13, 14}[ei] : (double[]){11, 13, 15}[ei];
            double d0 = (double[]){60, 150}[di], gap = (double[]){60, 75, 100, 150}[gi];
            /* lag scale: the robot measures F_RAMP ~24.2-24.8 every run = 1.25 here; +-20% */
            double kR = (double[]){0.92, 1.0, 1.08}[ki], ramp = (double[]){1.0, 1.25, 1.5}[ri];
            if (robotlike) { kR = (double[]){0.97, 1.0, 1.03}[ki]; ramp = (double[]){1.15, 1.25, 1.35}[ri]; }
            /* per-ms speed decay while braking: robot slides 0.076-0.083 s = ~0.013 */
            sim_brake = robotlike ? (double[]){0.0115, 0.013, 0.0145}[bi] : (double[]){0.01, 0.03, 0.08}[bi];
            double y0 = (double[]){-2, 2}[yi];
            o2_half = (double[]){15, 50}[oi];
            int arrow_ms = ai ? 6000 : 400;          /* late arrow forces the stop-and-wait path */
            Result r = run(arrow, d0, gap, kR, ramp, y0, arrow_ms);
            n++;
            if (getenv("CSV")) printf("CSV,%d,%.0f,%.0f,%.2f,%.2f,%.3f,%.0f,%.0f,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.0f,%s\n", arrow, d0, gap, kR, ramp,
                                      sim_brake, y0, o2_half, arrow_ms, r.clr, r.clr2, r.end_y, r.end_h, r.range2, sim_krev, sim_dead, last_error);
            int bad = !r.ok || r.clr < 3.0 || r.clr2 < 3.0 || fabs(r.end_y) > 5 || fabs(r.end_h) > 4;
            if (r.clr < w1) w1 = r.clr;
            if (r.clr2 < w2) w2 = r.clr2;
            wy = max_abs(r.end_y, wy); wh = max_abs(r.end_h, wh);
            if (gap == 60) { if (r.range2 < rmin) rmin = r.range2; if (r.range2 > rmax) rmax = r.range2; }
            if (ai == 0 && r.t_ms > tmax) tmax = r.t_ms;
            if (bad) {
                printf("FAIL arrow=%c d0=%.0f gap=%.0f kR=%.2f krev=%.2f lag=%.2f brake=%.3f y0=%+.0f O2 half=%.0f arrow@%dms: ok=%d O1 clr=%.1f O2 clr=%.1f end y=%.1f h=%.1f t=%.0fms %s\n",
                       arrow == 1 ? 'L' : 'R', d0, gap, kR, sim_krev, ramp, sim_brake, y0, o2_half, arrow_ms, r.ok, r.clr, r.clr2, r.end_y, r.end_h, r.t_ms, last_error);
                fails++;
            }
        }
        printf("arrow %c: worst O1 clearance %.1f, worst O2 clearance %.1f, end |y| <= %.1f |h| <= %.1f, "
               "gap-60 range to O2 %.1f..%.1f, slowest (arrow early) %.2f s\n",
               arrow == 1 ? 'L' : 'R', w1, w2, wy, wh, rmin, rmax, tmax / 1000.0);
    }
    printf("%d/%d scenarios passed\n", n - fails, n);
    sim_brake = 0.013; o2_half = 50; sim_krev = 1.0; sim_dead = 13; sim_skid = 5;
    for (int arrow = 1; arrow <= 2; arrow++) {
        Result r = run(arrow, 100, 60, 1.0, 1.25, 0, 400);
        printf("nominal %c, O1 100 cm away, gap 60: %.2f s START to stopped, front %.1f cm past O1, %.1f cm from O2\n",
               arrow == 1 ? 'L' : 'R', r.t_ms / 1000.0, r.end_x, r.range2);
    }
    return fails != 0;
}
