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

typedef struct { int ok; double clr, end_y, end_h, end_x, t_ms; } Result;

static Result run(int arrow, double d0, double gap, double kR, double ramp, double y0, int arrow_ms) {
    KcTask2 t; KcT2Hooks hooks = {h_start, h_stop, h_event, h_drive};
    memset(&bot, 0, sizeof bot); bot.x = -8.2; bot.y = y0;   /* robot centre at 0, as in firmware */
    sim_rl = KCF_R_LEFT * kR; sim_rr = KCF_R_RIGHT * kR; sim_ramp = ramp; sim_ramp_rl = ramp * (KCF_RAMP_RL / KCF_RAMP_LR);
    drive_mode = -1; braked = 1; last_error[0] = 0;
    double x1 = 11.5 + d0;                     /* O1 near face; front bumper starts at 11.5 */
    double x2 = x1 + 10 + gap;
    double odist = 0, us = -1, heading0 = 0; uint32_t us_seq = 0, us_at = 0;
    KcT2Input in = {0};
    in.gyro_ready = 1; in.imu_at = 0; in.heading = 0; in.us_valid = 0;
    in.ox = 0; in.oy = -8.2;
    if (!kc_start(&t, hooks, &in, 77, 1)) { Result r = {0}; return r; }
    double clr = 1e9; uint32_t now = 0;
    for (; now < 30000 && t.active; now++) {
        double dt = 0.001;
        /* actuator model */
        double tk = 0, vt = 0;
        if (drive_mode >= 0 && !braked) {
            vt = drive_pwm * 58.5 / 3550.0;      /* servo test: PWM 3550 = 58.5 cm/s straight */
            if (drive_mode != KCF_DRIVE_STRAIGHT) vt *= 0.75;  /* braked inner wheel: 42-48 cm/s */
            if (drive_mode == KCF_DRIVE_LEFT) tk = 1 / sim_rl;
            else if (drive_mode == KCF_DRIVE_RIGHT) tk = -1 / sim_rr;
            else tk = drive_off < 0 ? -drive_off / 48.4 / sim_rl : -drive_off / 59.6 / sim_rr;
        }
        bot.v += (vt - bot.v) * (braked ? 0.03 : 0.012);  /* ~80 ms lag, faster braking */
        double ds = bot.v * dt;
        double dk = tk - bot.k, rate = (1 / sim_rl + 1 / sim_rr) / (dk < 0 ? ramp : sim_ramp_rl) * ds;
        if (dk > rate) dk = rate; else if (dk < -rate) dk = -rate;
        bot.k += dk; bot.h += bot.k * ds; bot.x += cos(bot.h) * ds; bot.y += sin(bot.h) * ds;
        odist += ds;
        if (now % 90 == 0) { us = us_read(x1, x2, 30); us_seq++; us_at = now; }
        double c = clearance(x1); if (c < clr) { clr = c; clr_mode = t.phase == KCT_F_SWERVE ? t.fmode : -1; }
        if (c < 0) { snprintf(last_error, sizeof last_error, "COLLISION_O1"); break; }
        if (bot.x + 19.7 * cos(bot.h) > x2 && fabs(bot.y) < 15 + 9.4) { snprintf(last_error, sizeof last_error, "COLLISION_O2"); break; }
        /* input */
        in.now = now; in.imu_at = now; in.heading = (float)(heading0 + bot.h * 180 / M_PI);
        in.gyro = (float)(bot.k * bot.v * 180 / M_PI);
        in.us_seq = us_seq; in.us_at = us_at; in.us = (float)us; in.us_valid = us > 0;
        in.ox = (float)(-bot.y); in.oy = (float)bot.x; in.odist = (float)odist; in.speed = (float)bot.v;
        t.heartbeat = now;
        if ((int)now == arrow_ms) kc_arrow(&t, 77, 1, (unsigned)arrow);
        kc_step(&t, &in);
    }
    Result r = {t.phase == KCT_DONE, clr, bot.y, bot.h * 180 / M_PI, bot.x + 19.7 - (x1 + 10), now};
    return r;
}

int main(int argc, char **argv) {
    if (argc == 7) {   /* single verbose run: arrow(1|2) d0 kR ramp y0 arrow_ms */
        verbose = 1;
        Result r = run(atoi(argv[1]), atof(argv[2]), 150, atof(argv[3]), atof(argv[4]), atof(argv[5]), atoi(argv[6]));
        printf("ok=%d clr=%.1f (mode %d) end y=%.1f h=%.1f front past O1=%.0f t=%.0fms %s\n", r.ok, r.clr, clr_mode, r.end_y, r.end_h, r.end_x, r.t_ms, last_error);
        return 0;
    }
    int fails = 0, n = 0;
    double worst = 1e9;
    for (int arrow = 1; arrow <= 2; arrow++)
    for (int di = 0; di < 3; di++)
    for (int ki = 0; ki < 3; ki++)
    for (int ri = 0; ri < 3; ri++)
    for (int yi = 0; yi < 3; yi++)
    for (int ai = 0; ai < 2; ai++) {
        double d0 = (double[]){60, 100, 150}[di], kR = (double[]){0.92, 1.0, 1.08}[ki];
        double ramp = (double[]){17, 22.7, 28}[ri], y0 = (double[]){-2, 0, 2}[yi];  /* L->R lag; R->L scaled */
        int arrow_ms = ai ? 6000 : 400;          /* late arrow forces the stop-and-wait path */
        Result r = run(arrow, d0, 150, kR, ramp, y0, arrow_ms);
        n++;
        int bad = !r.ok || r.clr < 3.0 || fabs(r.end_y) > 5 || fabs(r.end_h) > 4;  /* O2 code tolerates 5 deg */
        if (r.clr < worst) worst = r.clr;
        if (bad || verbose) {
            printf("%s arrow=%c d0=%.0f kR=%.2f ramp=%.0f y0=%+.0f arrow@%dms: ok=%d clr=%.1f end y=%.1f h=%.1f front past O1=%.0f t=%.0fms %s\n",
                   bad ? "FAIL" : "ok  ", arrow == 1 ? 'L' : 'R', d0, kR, ramp, y0, arrow_ms, r.ok, r.clr, r.end_y, r.end_h, r.end_x, r.t_ms, last_error);
            fails += bad;
        }
    }
    printf("%d/%d scenarios passed, worst O1 clearance %.1f cm\n", n - fails, n, worst);
    /* nominal timing, arrow known early */
    for (int arrow = 1; arrow <= 2; arrow++) {
        Result r = run(arrow, 100, 150, 1.0, 22.7, 0, 400);
        printf("nominal %c, O1 100 cm away: %.2f s to stopped on centre line, front %.0f cm past O1\n",
               arrow == 1 ? 'L' : 'R', r.t_ms / 1000.0, r.end_x);
    }
    /* short O1-O2 gaps: expect a safe stop or the documented error, never a collision */
    for (int arrow = 1; arrow <= 2; arrow++)
        for (int g = 60; g <= 90; g += 10) {
            Result r = run(arrow, 100, g, 1.0, 22.7, 0, 400);
            printf("gap %d %c: ok=%d front past O1=%.0f %s\n", g, arrow == 1 ? 'L' : 'R', r.ok, r.end_x, last_error);
        }
    return fails != 0;
}
