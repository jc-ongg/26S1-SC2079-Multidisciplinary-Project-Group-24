/* Fast O1 (Claude, 2026-10-14). Included by kc_task2.h.
 *
 * Rolling approach to O1 while the Pi reads the arrow, then a swerve whose
 * steering switches are decided from encoder+gyro odometry instead of fixed
 * turn angles, so radius and servo-lag errors are corrected on every run.
 * Geometry and robustness numbers: analysis/claude/o1_planner.py.
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
#define KCF_Y_PEAK      28.0f  /* rear-axle lane offset beside O1 */
/* Front-to-O1 distance when steering starts. Right lock is the wider turn, so
 * a RIGHT swerve moves sideways more slowly; start it earlier. */
#ifndef KCF_D_TRIG_L
#define KCF_D_TRIG_L    34.0f  /* O1 runs: LEFT passed ~26 cm lane; 34 adds ~3 cm at the near corner */
#endif
#ifndef KCF_D_TRIG_R
#define KCF_D_TRIG_R    35.0f  /* O1 runs: RIGHT came close to the near corner at 30 */
#endif
#define KCF_X_RET        3.0f  /* turn back once the rear axle is this far past O1's near face */
/* cm driven while the steering swings lock to lock, measured in the O1 runs
 * of 2026-10-14 at ~51 cm/s. Not symmetric: right->left is slower. The first
 * swing of every swerve is re-measured (F_RAMP) and both values are scaled. */
#define KCF_RAMP_LR     22.7f  /* left lock -> right lock */
#define KCF_RAMP_RL     30.4f  /* right lock -> left lock */
#define KCF_RAMP_MAX    45.0f
#define KCF_PWM_FAST  4500u
#define KCF_PWM_TURN  3550u    /* speed 50, the speed the radii were measured at */
#define KCF_SLOW_ZONE   30.0f  /* drop to turn PWM this far before the trigger */
#define KCF_DECIDE      15.0f  /* still no arrow this far before the trigger: stop and wait */
#define KCF_O2_GUARD    18.0f  /* brake if US sees something this close while turning back */

enum { KCF_DRIVE_STRAIGHT, KCF_DRIVE_LEFT, KCF_DRIVE_RIGHT };

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
 if(t->cap==4){kc_event(t,"F_SERVO_TEST",KCF_RAMP_LR,KCF_RAMP_RL);kc_enter(t,KCT_F_SERVO_TEST,i->now);return;}
 /* Ask for the arrow immediately; the Pi keeps trying while we drive. */
 t->waiting_arrow=1;t->arrow=0;t->vision_failed=0;
 kc_event(t,"CAPTURE_EARLY",1,(float)t->token);
 kc_event(t,"FAST_O1",KCF_D_TRIG_L,KCF_D_TRIG_R);
 kc_enter(t,KCT_F_RANGE,i->now);
}
static void kcf_take_arrow(KcTask2 *t,float front){
 if(t->arrow && !t->side1){t->side1=t->arrow;t->waiting_arrow=0;kc_event(t,"F_ARROW1",(float)t->side1,front);}
}
static void kcf_step(KcTask2 *t,const KcT2Input *i,float actual){
 float px=i->oy,py=-i->ox,h=kc_rad(actual),front=px+KCF_AXLE_FRONT;
 switch(t->phase){
 case KCT_F_RANGE:
  kcf_take_arrow(t,front);
  if(!kc_us_group(t,i,NAN,180.0f)){
   if(kc_elapsed(i->now,t->entered,3500))kc_fail(t,"ERROR_US_UNRELIABLE",i->now);
   break;
  }
  if(t->range<kc_max(KCF_D_TRIG_L,KCF_D_TRIG_R)+8.0f){kc_fail(t,"ERROR_START_TOO_CLOSE",i->now);break;}
  t->o1_front=t->range+11.5f;t->us_anchor=t->range;t->o1_gap=t->range;
  kc_event(t,"F_O1_RANGE",t->range,t->o1_front);
  t->flast=i->odist;kc_enter(t,KCT_F_APPROACH,i->now);
  break;
 case KCT_F_APPROACH:{
  kcf_track(t,i,KCF_DRIVE_STRAIGHT);
  kcf_take_arrow(t,front);
  float trig=t->o1_front-(t->side1==2?KCF_D_TRIG_R:KCF_D_TRIG_L);
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
  if(kc_elapsed(i->now,t->entered,8000)){kc_fail(t,"ERROR_F_SWERVE_TIMEOUT",i->now);break;}
  int lock=(t->fmode==1 || t->fmode==5)?side:(t->fmode==2 || t->fmode==4)?opp:KCF_DRIVE_STRAIGHT;
  kcf_track(t,i,lock);
  KcfSim s={px,py,h,t->fk};
  int predict=i->odist-t->fpred>=0.5f;
  if(predict)t->fpred=i->odist;
  uint8_t before=t->fmode;
  switch(t->fmode){
  case 1:if(predict && sg*kcf_peak(s,opp,sg,t->framp_lr,t->framp_rl)>=KCF_Y_PEAK){t->fmode=2;t->fsw=i->odist;}break;
  case 2:if(sg*h<=kcf_lead(opp,t->framp_lr,t->framp_rl))t->fmode=3;break;
  case 3:if(px>=x1+KCF_X_RET)t->fmode=4;break;
  case 4:if(predict && sg*h<0 && sg*kcf_land(s,side,sg,t->framp_lr,t->framp_rl)<=0)t->fmode=5;break;
  case 5:if(sg*h>=-kcf_lead(side,t->framp_lr,t->framp_rl))t->fmode=6;break;
  default:break;
  }
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
  if(t->fmode!=before){kc_event(t,"F_MODE",(float)t->fmode,actual);kc_event(t,"F_POSE",px,py);}
  /* Turning back toward the centre line: a close echo ahead means O2 is near. */
  if(t->fmode>=4 && i->us_seq!=t->us_seen){
   t->us_seen=i->us_seq;
   if(i->us_valid && isfinite(i->us) && i->us>2 && i->us<=KCF_O2_GUARD && fabsf(actual)<=25)t->count++;
   else t->count=0;
  }
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
  t->x=-(py+KCF_AXLE_CENTRE*s);t->y=px+KCF_AXLE_CENTRE*c;t->h=0;
  kc_event(t,"F_O1_DONE",t->x,t->y);
  kc_event(t,"F_O1_TIME",(float)(i->now-t->fstart),actual);
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
