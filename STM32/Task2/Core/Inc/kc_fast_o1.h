/* Fast O1 (2026-10-14). Included by kc_task2.h.
 *
 * Rolling approach to O1 while the Pi reads the arrow, then a swerve whose
 * steering switches are decided from encoder+gyro odometry instead of fixed
 * turn angles, so radius and servo-lag errors are corrected on every run.
 *
 * Planner frame used here: px forward (= kc Y), py left (= -kc X), heading
 * left positive, origin = start heading. Odometry is the REAR AXLE midpoint.
 */
#ifndef KC_FAST_O1_H
#define KC_FAST_O1_H

#define KCF_R_LEFT      24.3f  /* rolling lock radius, servo test 2026-10-14 (standstill fit was 22.2) */
#define KCF_R_RIGHT     30.5f  /* rolling, servo test (standstill fit was 28.8) */
#define KCF_AXLE_CENTRE  8.2f  /* rear axle behind robot centre */
#define KCF_AXLE_FRONT  19.7f  /* rear axle to front bumper (US) */
#ifndef KCF_Y_PEAK
#define KCF_Y_PEAK      28.0f  /* rear-axle lane offset beside O1 */
#endif
/* Front-to-O1 distance when steering starts. Right lock is the wider turn, so
 * a RIGHT swerve moves sideways more slowly; start it earlier. */
#ifndef KCF_D_TRIG_L
#define KCF_D_TRIG_L    34.0f  /* O1 runs: LEFT passed ~26 cm lane; 34 adds ~3 cm at the near corner */
#endif
#ifndef KCF_D_TRIG_R
#define KCF_D_TRIG_R    35.0f  /* O1 runs: RIGHT came close to the near corner at 30 */
#endif
/* The reverse finish turns in beside O1, so the lane must be reached before
 * O1's near face even with a slow servo: steer out earlier. */
#ifndef KCF_RF_TRIG_EXTRA
#define KCF_RF_TRIG_EXTRA 12.0f
#endif
#ifndef KCF_X_RET
#define KCF_X_RET        3.0f  /* turn back once the rear axle is this far past O1's near face */
#endif
/* cm driven while the steering swings lock to lock, measured in the O1 runs
 * of 2026-10-14 at ~51 cm/s. Not symmetric: right->left is slower. The first
 * swing of every swerve is re-measured (F_RAMP) and both values are scaled. */
#define KCF_RAMP_LR     22.7f  /* left lock -> right lock */
#define KCF_RAMP_RL     30.4f  /* right lock -> left lock */
#define KCF_RAMP_MAX    45.0f
#define KCF_PWM_FAST  4500u
#ifndef KCF_PWM_TURN
#define KCF_PWM_TURN  3550u    /* swerve PWM; speed 50 = 3550 = ~58 cm/s straight */
#endif
/* Steering lag is time-based: ~0.445 s left->right, ~0.596 s right->left
 * lock to lock (O1 runs at ~51 cm/s gave 22.7 / 30.4 cm). Scaled by speed. */
#define KCF_T_LR 0.445f
#define KCF_T_RL 0.596f
#define KCF_SLOW_ZONE   30.0f  /* drop to turn PWM this far before the trigger */
#define KCF_DECIDE      15.0f  /* still no arrow this far before the trigger: stop and wait */
/* O1 range at START, standing: 12 echoes (~1 s), then the FARTHEST group
 * of at least 3 within 4 cm. Robot logs 2026-10-16: at 120-150 cm, 30-50%
 * of echoes read short (62-121: the floor, nearer things; the HC-SR04
 * reports the first echo, never one beyond O1) while the true range
 * repeats. One run read 122 68 62 68 105 62 118: the short 62-68 echoes
 * outnumber the real ones, so a majority vote fails there, and the first 3
 * agreeing echoes could also have been 62-68 (swerve ~55 cm early). */
#define KCF_RANGE_N     12
#define KCF_RANGE_AGREE  3
#define KCF_RANGE_TOL  4.0f
#define KCF_O2_GUARD    18.0f  /* brake if US sees something this close while turning back */

/* Reverse finish (fits the 60 cm O1-O2 gap). After the lane beside O1:
 * forward on the inward lock to KCF_RF_THETA, straight diagonal, brake,
 * set the same lock at standstill, reverse on it back to heading 0. The
 * turn-in point is chosen so the rear bumper ends KCF_RF_REAR past O1, and
 * the diagonal ends where the reverse arc lands on the centre line. */
#ifndef KCF_FINISH_REVERSE
#define KCF_FINISH_REVERSE 1
#endif
/* Reverse lock radius from the reverse arcs' chords, robot 2026-10-15:
 * right 30.2-30.8 (4 runs), left 23.4-23.6 (2 runs). Independent of the
 * forward-turn radius measured in the same runs (scale 0.93-1.07), so that
 * measurement is logged but not applied here. */
#define KCF_RS_LEFT     23.5f
#define KCF_RS_RIGHT    30.4f
#define KCF_AXLE_REAR    3.3f  /* rear bumper behind rear axle */
/* Per O1 arrow. A LEFT arrow turns in and reverses on the wider right lock,
 * which reaches further toward O2. */
#ifndef KCF_RF_THETA_L
#define KCF_RF_THETA_L  64.0f  /* deg, inward heading of the diagonal (test_fast_o1.c sweep) */
#endif
#ifndef KCF_RF_THETA_R
#define KCF_RF_THETA_R  63.0f
#endif
#ifndef KCF_RF_REAR_L
#define KCF_RF_REAR_L    7.0f  /* rear bumper aims to end this far past O1's back face */
#endif
#ifndef KCF_RF_REAR_R
#define KCF_RF_REAR_R   10.0f  /* RIGHT lands ~3 cm short of its aim in the sweep */
#endif
#define KCF_RF_THETA (sg>0?KCF_RF_THETA_L:KCF_RF_THETA_R)
#define KCF_RF_REAR  (sg>0?KCF_RF_REAR_L:KCF_RF_REAR_R)
/* Lateral trim for the end of O1, cm, + = end further LEFT than the odometry
 * centre line. Robot 2026-10-15: a RIGHT pass reported centred (-0.08) but was
 * ~5 cm left by tape; set from tape measurements, not from the log. */
#ifndef KCF_RF_Y_TRIM_L
#define KCF_RF_Y_TRIM_L  0.0f
#endif
#ifndef KCF_RF_Y_TRIM_R
#define KCF_RF_Y_TRIM_R  0.0f
#endif
#define KCF_RF_Y_TRIM (sg>0?KCF_RF_Y_TRIM_L:KCF_RF_Y_TRIM_R)
#ifndef KCF_RF_DTHETA
#define KCF_RF_DTHETA    6.0f  /* deg the inward angle may change to land on the target */
#endif
#ifndef KCF_RF_XS_MIN_L
#define KCF_RF_XS_MIN_L -3.0f  /* earliest turn-in: rear axle past O1's near face */
#endif
#ifndef KCF_RF_XS_MIN_R
#define KCF_RF_XS_MIN_R -3.0f  /* RIGHT arrow: inward turn on the tighter left lock */
#endif
#define KCF_RF_XS_MIN (sg>0?KCF_RF_XS_MIN_L:KCF_RF_XS_MIN_R)
/* Short-brake slides, robot 2026-10-15 (7 runs): the diagonal brake slid
 * 4.3-4.75 cm from 57-58 cm/s (0.076-0.083 s); after the reverse the car
 * turned on 1.6-2.1 deg more (~0.063 s at ~14 cm/s). */
#ifndef KCF_RF_SLIDE_S
#define KCF_RF_SLIDE_S  0.078f /* diagonal: slide = speed x this (s) */
#endif
#ifndef KCF_RF_REV_SLIDE_S
#define KCF_RF_REV_SLIDE_S 0.063f
#endif
/* The short brake locks the rear wheels: after the encoders stop the car
 * skids on, unseen. Robot 2026-10-15, tape vs log, 4 runs: it ended 3.7-5.5
 * cm further along the diagonal side than the odometry said, i.e. ~5 cm of
 * skid along the diagonal. Braked earlier by this much and added to the
 * pose once stopped. */
#ifndef KCF_RF_SKID
#define KCF_RF_SKID 5.0f
#endif
/* Robot runs 2026-10-15 (3 x LEFT): from straight to the inward lock the car
 * went ~7 cm further than the model before the heading came round. */
#ifndef KCF_RF_TURNIN_DEAD
#define KCF_RF_TURNIN_DEAD 12.0f  /* fitted in test_fast_o1.c to the robot's turn-in */
#endif
#ifndef KCF_RF_NUDGE_TOL
#define KCF_RF_NUDGE_TOL 1.0f  /* cm along the diagonal before a correction move */
#endif
#ifndef KCF_PWM_NUDGE
#define KCF_PWM_NUDGE  1600u
#endif
#ifndef KCF_PWM_REV
#define KCF_PWM_REV    2500u
#endif
#ifndef KCF_RF_SERVO_MS
#define KCF_RF_SERVO_MS 300u   /* centre to lock at standstill */
#endif
#ifndef KCF_RF_REV_LEAD
#define KCF_RF_REV_LEAD  0.0f  /* deg, plus the brake slide at the current speed */
#endif
#ifndef KCF_PWM_REV_SLOW
#define KCF_PWM_REV_SLOW 1600u /* last KCF_RF_REV_SLOW deg of the reverse */
#endif
#define KCF_RF_REV_SLOW 20.0f
#define KCF_RF_DIAG_MAX 50.0f  /* cm; the sweep's diagonals are ~30-35 */
#define KCF_RF_END_TOL   8.0f  /* cm off the centre line; the sweep ends within 2.4 */

/* Every mode from KCF_DRIVE_BRAKE on is a short brake. */
enum { KCF_DRIVE_STRAIGHT, KCF_DRIVE_LEFT, KCF_DRIVE_RIGHT, KCF_DRIVE_REV_LEFT, KCF_DRIVE_REV_RIGHT,
       KCF_DRIVE_REV_STRAIGHT, KCF_DRIVE_BRAKE, KCF_DRIVE_BRAKE_LEFT, KCF_DRIVE_BRAKE_RIGHT };

typedef struct { float x, y, h, k; } KcfSim;

static float kcf_lock_k(int m){
 return m==KCF_DRIVE_LEFT?1.0f/KCF_R_LEFT:m==KCF_DRIVE_RIGHT?-1.0f/KCF_R_RIGHT:0.0f;
}
/* Heading still gained while the servo returns from lock m to centre. */
static float kcf_lead(int m,float rlr,float rrl){
 return fabsf(kcf_lock_k(m))*(m==KCF_DRIVE_LEFT?rlr:rrl)*0.25f;
}
/* Curvature slews toward target; decreasing (toward right) uses the L->R lag. */
static void kcf_sim_step(KcfSim *s,float target,float ds,float rlr,float rrl){
 float dk=target-s->k,rate=(1.0f/KCF_R_LEFT+1.0f/KCF_R_RIGHT)/(dk<0?rlr:rrl)*ds;
 if(dk>rate)dk=rate;else if(dk<-rate)dk=-rate;
 s->k+=dk;s->h+=s->k*ds;s->x+=cosf(s->h)*ds;s->y+=sinf(s->h)*ds;
}
/* Lateral position where the heading is back to straight if we switch to `opp` now. */
static float kcf_peak(KcfSim s,int opp,float sg,float rlr,float rrl){
 for(int n=0;n<300 && sg*s.h>kcf_lead(opp,rlr,rrl);n++)kcf_sim_step(&s,kcf_lock_k(opp),1.0f,rlr,rrl);
 return s.y;
}
/* Lateral landing position if we switch to `side` now and centre the servo near straight. */
static float kcf_land(KcfSim s,int side,float sg,float rlr,float rrl){
 int n=0;
 for(;n<300 && sg*s.h<-kcf_lead(side,rlr,rrl);n++)kcf_sim_step(&s,kcf_lock_k(side),1.0f,rlr,rrl);
 for(;n<400 && fabsf(s.k)>1e-4f;n++)kcf_sim_step(&s,0.0f,1.0f,rlr,rrl);
 return s.y;
}
static float kcf_trig_extra(void){return KCF_FINISH_REVERSE?KCF_RF_TRIG_EXTRA:0.0f;}
static float kcf_rs(int m){return m==KCF_DRIVE_LEFT?KCF_RS_LEFT:KCF_RS_RIGHT;}
/* Reverse finish: run along heading h (diagonal plus brake slide) after which
 * reversing on lock `opp` back to heading 0 lands on the centre line. */
static float kcf_rf_run(float y,float h,float r,float sg){
 return (sg*y+r*(1.0f-cosf(h)))/kc_max(0.2f,fabsf(sinf(h)));
}
/* Final rear-axle x of the reverse finish if the steering centres now;
 * r = reverse lock radius. */
static float kcf_rf_finish(KcfSim s,float sg,float rlr,float rrl,float r,float *heading){
 for(int n=0;n<100 && fabsf(s.k)>1e-4f;n++)kcf_sim_step(&s,0.0f,1.0f,rlr,rrl);
 if(heading)*heading=s.h;
 return s.x+kc_max(0.0f,kcf_rf_run(s.y,s.h,r,sg))*cosf(s.h)-r*fabsf(sinf(s.h));
}
/* ... and if the inward turn starts now, nominal angle. */
static float kcf_rf_xend(KcfSim s,int opp,float sg,float rlr,float rrl){
 float hsw=kc_rad(KCF_RF_THETA)-kcf_lead(opp,rlr,rrl);
 for(int n=0;n<(int)(2.0f*KCF_RF_TURNIN_DEAD);n++)kcf_sim_step(&s,s.k,0.5f,rlr,rrl);
 for(int n=0;n<300 && sg*s.h>-hsw;n++)kcf_sim_step(&s,kcf_lock_k(opp),1.0f,rlr,rrl);
 return kcf_rf_finish(s,sg,rlr,rrl,kcf_rs(opp),0);
}
/* Lane hold gains (tuned in test_fast_o1.c for the measured 22 cm steering lag). */
#ifndef KCF_KY
#define KCF_KY 1.0f        /* deg of heading per cm of lateral error */
#endif
#ifndef KCF_KH
#define KCF_KH 2.5f        /* servo counts per deg of heading error */
#endif
#ifndef KCF_KD
#define KCF_KD 0.3f        /* servo counts per deg/s of yaw rate */
#endif
#ifndef KCF_HREF_MAX
#define KCF_HREF_MAX 12.0f
#endif
#ifndef KCF_OFF_MAX
#define KCF_OFF_MAX 30.0f
#endif
/* Servo offset (counts, + = right) holding lateral position y_target with heading 0. */
static float kcf_hold(float py,float h_deg,float gyro,float y_target){
 float href=kc_max(-KCF_HREF_MAX,kc_min(KCF_HREF_MAX,-KCF_KY*(py-y_target)));
 return kc_max(-KCF_OFF_MAX,kc_min(KCF_OFF_MAX,KCF_KH*(h_deg-href)+KCF_KD*gyro));
}
/* Model curvature follows the commanded lock with the servo ramp, per cm driven. */
static void kcf_track(KcTask2 *t,const KcT2Input *i,int lock){
 float ds=kc_max(0.0f,i->odist-t->flast);t->flast=i->odist;
 KcfSim s={0,0,0,t->fk};
 kcf_sim_step(&s,kcf_lock_k(lock),ds,t->framp_lr,t->framp_rl);
 t->fk=s.k;
}
static void kcf_begin(KcTask2 *t,const KcT2Input *i){
 t->fstart=i->now;t->flast=i->odist;t->fk=0;t->fmode=0;t->fshort=0;t->framp_lr=KCF_RAMP_LR;t->framp_rl=KCF_RAMP_RL;t->framp_done=0;
 t->fkscale=1.0f;t->ftarget=0;t->fhead=0;t->fat=i->now;t->fox=0;t->foy=0;
 if(t->cap==4){kc_event(t,"F_SERVO_TEST",KCF_RAMP_LR,KCF_RAMP_RL);kc_enter(t,KCT_F_SERVO_TEST,i->now);return;}
 /* Ask for the arrow immediately; the Pi keeps trying while we drive. */
 t->waiting_arrow=1;t->arrow=0;t->vision_failed=0;
 kc_event(t,"CAPTURE_EARLY",1,(float)t->token);
 kc_event(t,"FAST_O1",KCF_D_TRIG_L+kcf_trig_extra(),KCF_D_TRIG_R+kcf_trig_extra());
 if(KCF_FINISH_REVERSE)kc_event(t,"F_REVERSE_FINISH",2.0f,KCF_RF_TURNIN_DEAD);
 kc_enter(t,KCT_F_RANGE,i->now);
}
static void kcf_take_arrow(KcTask2 *t,float front){
 if(t->arrow && !t->side1){t->side1=t->arrow;t->waiting_arrow=0;kc_event(t,"F_ARROW1",(float)t->side1,front);}
}
static void kcf_step(KcTask2 *t,const KcT2Input *i,float actual){
 /* fox/foy: pose corrections the odometry cannot see (brake skid). */
 float px=i->oy+t->fox,py=-i->ox+t->foy,h=kc_rad(actual),front=px+KCF_AXLE_FRONT;
 switch(t->phase){
 case KCT_F_RANGE:
  kcf_take_arrow(t,front);
  if(i->us_seq!=t->us_seen){
   t->us_seen=i->us_seq;
   if(i->us_valid && isfinite(i->us) && i->us>2 && i->us<=180 && !kc_elapsed(i->now,i->us_at,250) && t->samples<KCF_RANGE_N)
    t->o1_votes[t->samples++]=i->us;
  }
  if(t->samples<KCF_RANGE_N){
   if(kc_elapsed(i->now,t->entered,5000))kc_fail(t,"ERROR_US_UNRELIABLE",i->now);
   break;
  }
  {
   /* The farthest echo with at least KCF_RANGE_AGREE within KCF_RANGE_TOL
    * (itself included): short echoes cannot outvote it. */
   int best=-1,bn=0;
   for(int a=0;a<KCF_RANGE_N;a++){
    int n=0;for(int b=0;b<KCF_RANGE_N;b++)n+=fabsf(t->o1_votes[b]-t->o1_votes[a])<=KCF_RANGE_TOL;
    if(n>=KCF_RANGE_AGREE && (best<0 || t->o1_votes[a]>t->o1_votes[best])){bn=n;best=a;}
   }
   kc_event(t,"F_O1_RANGE_VOTES",(float)bn,(float)KCF_RANGE_N);
   if(best<0){t->samples=0;break;}   /* no group of KCF_RANGE_AGREE: 12 fresh echoes */
   /* Median of the agreeing echoes (insertion sort of at most 12). */
   float g[KCF_RANGE_N];int n=0;
   for(int b=0;b<KCF_RANGE_N;b++)if(fabsf(t->o1_votes[b]-t->o1_votes[best])<=KCF_RANGE_TOL){
    float v=t->o1_votes[b];int k=n++;while(k>0 && g[k-1]>v){g[k]=g[k-1];k--;}g[k]=v;
   }
   t->range=n%2?g[n/2]:0.5f*(g[n/2-1]+g[n/2]);t->samples=0;
  }
  /* The reverse finish may start its swerve from a standstill (the model is per cm). */
  if(t->range<kc_max(KCF_D_TRIG_L,KCF_D_TRIG_R)+kcf_trig_extra()+(KCF_FINISH_REVERSE?2.0f:8.0f)){kc_fail(t,"ERROR_START_TOO_CLOSE",i->now);break;}
  t->o1_front=t->range+11.5f;t->us_anchor=t->range;t->o1_gap=t->range;
  kc_event(t,"F_O1_RANGE",t->range,t->o1_front);
  t->flast=i->odist;kc_enter(t,KCT_F_APPROACH,i->now);
  break;
 case KCT_F_APPROACH:{
  kcf_track(t,i,KCF_DRIVE_STRAIGHT);
  kcf_take_arrow(t,front);
  float trig=t->o1_front-(t->side1==2?KCF_D_TRIG_R:KCF_D_TRIG_L)-kcf_trig_extra();
  if(kc_elapsed(i->now,t->entered,15000)){kc_fail(t,"ERROR_F_APPROACH_TIMEOUT",i->now);break;}
  if(fabsf(actual)>10){kc_fail(t,"ERROR_STRAIGHT_HEADING",i->now);break;}
  /* Odometry says we are still >= trigger distance away; two close echoes mean it is wrong. */
  if(i->us_seq!=t->us_seen){
   t->us_seen=i->us_seq;
   if(i->us_valid && isfinite(i->us) && i->us>2 && i->us<kc_min(KCF_D_TRIG_L,KCF_D_TRIG_R)-10)t->count++;else t->count=0;
  }
  if(t->count>=2){kc_fail(t,"ERROR_F_O1_CLOSE",i->now);break;}
  if(!t->side1 && front>=trig-KCF_DECIDE){
   kc_brake(t);t->waiting_arrow=1;
   kc_event(t,"CAPTURE",1,(float)t->token);
   kc_enter(t,KCT_F_WAIT,i->now);break;
  }
  if(front>=trig){
   t->fmode=1;t->fpred=-1e9f;t->fok=0;
   kc_event(t,"F_SWERVE",(float)t->side1,i->speed);
   kc_enter(t,KCT_F_SWERVE,i->now);break;
  }
  t->io.drive(KCF_DRIVE_STRAIGHT,kcf_hold(py,actual,i->gyro,0.0f),
   front>=trig-KCF_SLOW_ZONE?KCF_PWM_TURN:KCF_PWM_FAST);
  break;}
 case KCT_F_WAIT:
  if(t->vision_failed || kc_elapsed(i->now,t->entered,25000)){kc_fail(t,"ERROR_VISION",i->now);break;}
  kcf_take_arrow(t,front);
  if(t->side1){t->flast=i->odist;kc_enter(t,KCT_F_APPROACH,i->now);}
  break;
 case KCT_F_SWERVE:{
  float sg=t->side1==1?1.0f:-1.0f,x1=t->o1_front;
  int side=t->side1==1?KCF_DRIVE_LEFT:KCF_DRIVE_RIGHT,opp=t->side1==1?KCF_DRIVE_RIGHT:KCF_DRIVE_LEFT;
  if(kc_elapsed(i->now,t->entered,12000)){kc_fail(t,"ERROR_F_SWERVE_TIMEOUT",i->now);break;}
  int lock=(t->fmode==1 || t->fmode==5)?side:(t->fmode==2 || t->fmode==4)?opp:KCF_DRIVE_STRAIGHT;
  if(t->fmode<=9)kcf_track(t,i,lock);
  KcfSim s={px,py,h,t->fk};
  int predict=i->odist-t->fpred>=0.5f;
  if(predict)t->fpred=i->odist;
  uint8_t before=t->fmode;
  /* Modes 10 and 13 wait for the brake: fok = when the wheels stopped. */
  int still=fabsf(i->speed)<1.5f;
  if(t->fmode==10 || t->fmode==13){if(!still)t->fok=0;else if(!t->fok)t->fok=i->now;}
  switch(t->fmode){
  case 1:if(predict && sg*kcf_peak(s,opp,sg,t->framp_lr,t->framp_rl)>=KCF_Y_PEAK){t->fmode=2;t->fsw=i->odist;}break;
  case 2:if(sg*h<=kcf_lead(opp,t->framp_lr,t->framp_rl))t->fmode=3;break;
  case 3:
   if(KCF_FINISH_REVERSE){
    /* The lane hold is steering: start the prediction from the measured
     * path curvature, not the model's straight-ahead. */
    KcfSim m=s;m.y-=KCF_RF_Y_TRIM;
    if(i->speed>15.0f)m.k=kc_rad(i->gyro)/i->speed;
    float target=x1+10.0f+KCF_AXLE_REAR+KCF_RF_REAR,xend=predict?kcf_rf_xend(m,opp,sg,t->framp_lr,t->framp_rl):0.0f;
    if(predict && px>=x1+KCF_RF_XS_MIN && xend>=target){t->fmode=4;t->ftarget=target;t->fkscale=1.0f;kc_event(t,"F_RF_XEND",xend,target);}
   }else if(px>=x1+KCF_X_RET)t->fmode=4;
   break;
  case 4:
   if(KCF_FINISH_REVERSE){
    /* Straighten when the predicted end reaches the target, within
     * KCF_RF_THETA +- KCF_RF_DTHETA: absorbs radius and servo errors. */
    float lead=kcf_lead(opp,t->framp_lr,t->framp_rl),a=-sg*h,xend=NAN,hd=h-sg*lead;
    /* Lock radius on this floor, this run: compare the gyro's path
     * curvature with the model once the steering has reached lock. */
    float kl=kcf_lock_k(opp);
    if(fabsf(t->fk-kl)<0.03f*fabsf(kl) && i->speed>15.0f && fabsf(i->gyro)>5.0f){
     float sc=kc_max(0.85f,kc_min(1.2f,fabsf(kl)*i->speed/fabsf(kc_rad(i->gyro))));
     t->fkscale+=0.05f*(sc-t->fkscale);
    }
    KcfSim st=s;st.y-=KCF_RF_Y_TRIM;
    if(predict && a>=kc_rad(KCF_RF_THETA-KCF_RF_DTHETA)-lead)
     xend=kcf_rf_finish(st,sg,t->framp_lr,t->framp_rl,kcf_rs(opp),&hd);
    if(a>=kc_rad(KCF_RF_THETA+KCF_RF_DTHETA)-lead || xend<=t->ftarget){
     /* The diagonal holds the heading the model says the turn settles at. */
     t->fmode=9;t->fhead=hd*57.29578f;t->fsw=i->odist;t->corrections=0;
     kc_event(t,"F_RF_TURN",xend,t->fhead);kc_event(t,"F_RF_RADIUS",t->fkscale,kcf_rs(opp));
    }
   }
   else if(predict && sg*h<0 && sg*kcf_land(s,side,sg,t->framp_lr,t->framp_rl)<=0)t->fmode=5;
   break;
  case 5:if(sg*h>=-kcf_lead(side,t->framp_lr,t->framp_rl))t->fmode=6;break;
  case 9:  /* diagonal: brake where slide + reverse arc end on the centre line */
   if(i->odist-t->fsw>KCF_RF_DIAG_MAX){kc_fail(t,"ERROR_F_DIAGONAL",i->now);break;}
   if(sg*h<0 && kcf_rf_run(py-KCF_RF_Y_TRIM,h,kcf_rs(opp),sg)<=KCF_RF_SLIDE_S*i->speed+KCF_RF_SKID){
    t->fmode=10;
    kc_event(t,"F_RF_BRAKE",px+(KCF_RF_SLIDE_S*i->speed+KCF_RF_SKID)*cosf(h)-kcf_rs(opp)*fabsf(sinf(h)),i->speed);
   }
   break;
  case 10: /* braking, wheels straight; then correct the stop point if needed */
   if((still && kc_elapsed(i->now,t->fok,100)) || kc_elapsed(i->now,t->fat,1500)){
    if(!t->corrections){  /* first stop after the diagonal: add the unseen skid */
     t->fox+=KCF_RF_SKID*cosf(h);t->foy+=KCF_RF_SKID*sinf(h);
     px+=KCF_RF_SKID*cosf(h);py+=KCF_RF_SKID*sinf(h);
     kc_event(t,"F_RF_SKID",KCF_RF_SKID,py);
    }
    float err=sg*h<0?kcf_rf_run(py-KCF_RF_Y_TRIM,h,kcf_rs(opp),sg):0.0f;
    if(fabsf(err)>KCF_RF_NUDGE_TOL && t->corrections<2){
     t->corrections++;t->fsw=err;t->fmode=14;kc_event(t,"F_RF_NUDGE",err,(float)t->corrections);
    }else t->fmode=11;
   }
   break;
  case 14: /* short straight move along the diagonal, sign of fsw = direction */
   if(t->fsw*kcf_rf_run(py-KCF_RF_Y_TRIM,h,kcf_rs(opp),sg)<=fabsf(i->speed)*KCF_RF_SLIDE_S*fabsf(t->fsw)
      || kc_elapsed(i->now,t->fat,2000))t->fmode=10;
   break;
  case 11: /* set the lock at standstill */
   if(kc_elapsed(i->now,t->fat,KCF_RF_SERVO_MS)){t->fmode=12;t->fsw=i->odist;}
   break;
  case 12: /* reverse on the lock back to heading 0 */
   if(sg*h>=-(kc_rad(KCF_RF_REV_LEAD)+fabsf(i->speed)*KCF_RF_REV_SLIDE_S/kcf_rs(opp))){
    t->fmode=13;
    /* Facing O2 now (within ~2 deg, nearly stopped): ask for its arrow here,
     * ~0.5 s before O2_RANGE would. Robot 2026-10-15: the reply took ~0.3 s
     * and the first O2 turn waited for it. O2_RANGE keeps this request. */
    if(t->cap!=1 && !t->waiting_arrow){
     t->waiting_arrow=2;t->arrow=0;t->vision_failed=0;
     kc_event(t,"CAPTURE_EARLY",2,(float)t->token);
    }
   }
   else if(t->fsw-i->odist>kcf_rs(opp)*kc_rad(KCF_RF_THETA+25.0f) || kc_elapsed(i->now,t->fat,4000)){
    kc_fail(t,"ERROR_F_REVERSE",i->now);break;
   }
   break;
  case 13: /* braking on the lock */
   if((still && kc_elapsed(i->now,t->fok,100)) || kc_elapsed(i->now,t->fat,1000)){
    t->fmode=8;kc_brake(t);kc_enter(t,KCT_F_SETTLE,i->now);
   }
   break;
  default:break;
  }
  if(t->fmode!=before){t->fat=i->now;t->fok=0;kc_event(t,"F_MODE",(float)t->fmode,actual);kc_event(t,"F_POSE",px,py);}
  if(!t->active || t->phase!=KCT_F_SWERVE)break;
  /* Measure the servo swing on this robot, this run: after the lock-to-lock
   * switch, the yaw rate passes the midpoint of the two lock rates half a
   * swing later. The turn back uses the measured value. */
  if((t->fmode==2 || t->fmode==3) && !t->framp_done && i->speed>15.0f){
   float mid=0.5f*(kcf_lock_k(side)+kcf_lock_k(opp))*i->speed*57.29578f;
   if(sg*i->gyro<=sg*mid){
    /* First swing is side->opp: L->R for a LEFT arrow. Scale the other direction. */
    float m=kc_max(3.0f,kc_min(KCF_RAMP_MAX,2.0f*(i->odist-t->fsw)));
    if(side==KCF_DRIVE_LEFT){t->framp_lr=m;t->framp_rl=m*(KCF_RAMP_RL/KCF_RAMP_LR);}
    else{t->framp_rl=m;t->framp_lr=m*(KCF_RAMP_LR/KCF_RAMP_RL);}
    t->framp_done=1;
    kc_event(t,"F_RAMP",m,(float)side);
   }
  }
  /* Turning back toward the centre line: a close echo ahead means O2 is near. */
  if((t->fmode>=4 && t->fmode<=6) || t->fmode==9){
   if(i->us_seq!=t->us_seen){
    t->us_seen=i->us_seq;
    if(i->us_valid && isfinite(i->us) && i->us>2 && i->us<=KCF_O2_GUARD && fabsf(actual)<=25)t->count++;
    else t->count=0;
   }
  }else t->count=0;
  if(t->count>=2){t->fshort=1;t->fmode=8;kc_brake(t);kc_event(t,"F_O2_CLOSE",i->us,actual);kc_enter(t,KCT_F_SETTLE,i->now);break;}
  if(t->fmode==6){
   int settled=fabsf(actual)<2.5f && fabsf(py)<3.0f && px>x1+15.0f;
   if(!settled)t->fok=0;else if(!t->fok)t->fok=i->now;
   if((settled && kc_elapsed(i->now,t->fok,60)) || px-(x1+10.0f)>75.0f){
    t->fmode=7;kc_enter(t,KCT_F_SETTLE,i->now);break;
   }
  }
  if(t->fmode==3)t->io.drive(KCF_DRIVE_STRAIGHT,kcf_hold(py,actual,i->gyro,sg*KCF_Y_PEAK),KCF_PWM_TURN);
  else if(t->fmode==6)t->io.drive(KCF_DRIVE_STRAIGHT,kcf_hold(py,actual,i->gyro,0.0f),KCF_PWM_TURN);
  else if(t->fmode==9)t->io.drive(KCF_DRIVE_STRAIGHT,
   kc_max(-KCF_OFF_MAX,kc_min(KCF_OFF_MAX,KCF_KH*(actual-t->fhead)+KCF_KD*i->gyro)),KCF_PWM_TURN);
  else if(t->fmode==10)t->io.drive(KCF_DRIVE_BRAKE,0.0f,0);
  else if(t->fmode==14)t->io.drive(t->fsw>0?KCF_DRIVE_STRAIGHT:KCF_DRIVE_REV_STRAIGHT,0.0f,KCF_PWM_NUDGE);
  else if(t->fmode==11 || t->fmode==13)t->io.drive(opp==KCF_DRIVE_LEFT?KCF_DRIVE_BRAKE_LEFT:KCF_DRIVE_BRAKE_RIGHT,0.0f,0);
  else if(t->fmode==12)t->io.drive(opp==KCF_DRIVE_LEFT?KCF_DRIVE_REV_LEFT:KCF_DRIVE_REV_RIGHT,0.0f,
   sg*actual<-KCF_RF_REV_SLOW?KCF_PWM_REV:KCF_PWM_REV_SLOW);
  else t->io.drive(lock,0.0f,KCF_PWM_TURN);
  break;}
 case KCT_F_SETTLE:{
  /* Coast with the steering still holding the line (a hard brake with the
   * front wheels steered twisted the robot ~4 deg), then brake and wait. */
  if(t->fmode==7){
   if(i->speed>8.0f && !kc_elapsed(i->now,t->entered,500)){
    t->io.drive(KCF_DRIVE_STRAIGHT,kcf_hold(py,actual,i->gyro,0.0f),0);break;
   }
   t->fmode=8;kc_brake(t);kc_enter(t,KCT_F_SETTLE,i->now);break;
  }
  if(!kc_elapsed(i->now,t->entered,150))break;
  float c=cosf(h),s=sinf(h);
  /* The trim is where the robot really is relative to the odometry: hand
   * the O2 code that corrected pose. */
  float sg=t->side1==1?1.0f:-1.0f,ytrim=KCF_FINISH_REVERSE?KCF_RF_Y_TRIM:0.0f;
  t->x=-(py-ytrim+KCF_AXLE_CENTRE*s);t->y=px+KCF_AXLE_CENTRE*c;t->h=0;
  kc_event(t,"F_O1_DONE",t->x,t->y);
  kc_event(t,"F_O1_TIME",(float)(i->now-t->fstart),actual);
  /* The O2 code assumes the centre line: off it, the ultrasonic looks past O2. */
  if(KCF_FINISH_REVERSE && fabsf(t->x)>KCF_RF_END_TOL){kc_fail(t,"ERROR_F_OFF_CENTRE",i->now);break;}
  /* Short O1-O2 gap needs camera ranging (not in this build). */
  if(t->fshort && fabsf(actual)>5){kc_fail(t,"ERROR_F_O2_CLOSE",i->now);break;}
  t->index=0;t->corrections=0;t->expected_range=NAN;t->o2_front=NAN;t->count=0;
  if(t->cap==1)kc_done(t,i->now);else kc_enter(t,KCT_O2_RANGE,i->now);
  break;}
 case KCT_F_SERVO_TEST:{
  /* 25 cm straight to reach speed, left lock to +35 deg, then right lock.
   * F_YAW logs (ms since flip, yaw rate) for 700 ms: the ramp between the
   * two steady yaw rates is the servo swing in real driving. */
  float d=i->odist-t->flast;
  if(kc_elapsed(i->now,t->entered,10000)){kc_fail(t,"ERROR_F_TEST_TIMEOUT",i->now);break;}
  if(t->fmode==0){
   if(d>=25.0f){t->fmode=1;kc_event(t,"F_SPEED",i->speed,(float)KCF_PWM_TURN);}
   else{t->io.drive(KCF_DRIVE_STRAIGHT,kcf_hold(py,actual,i->gyro,0.0f),KCF_PWM_TURN);break;}
  }
  if(t->fmode==1){
   if(actual>=35.0f){t->fmode=2;t->fok=i->now;t->fpred=i->odist;t->group_at=i->now;kc_event(t,"F_FLIP",i->speed,actual);}
   else{t->io.drive(KCF_DRIVE_LEFT,0.0f,KCF_PWM_TURN);break;}
  }
  if(!kc_elapsed(i->now,t->group_at,9)){t->io.drive(KCF_DRIVE_RIGHT,0.0f,KCF_PWM_TURN);break;}
  t->group_at=i->now;
  kc_event(t,"F_YAW",(float)(i->now-t->fok),i->gyro);
  if(kc_elapsed(i->now,t->fok,700) || actual<=-20.0f){
   kc_event(t,"F_FLIP_DIST",i->odist-t->fpred,i->speed);kc_done(t,i->now);break;
  }
  t->io.drive(KCF_DRIVE_RIGHT,0.0f,KCF_PWM_TURN);
  break;}
 default:kc_fail(t,"ERROR_STATE",i->now);break;
 }
}
#endif
