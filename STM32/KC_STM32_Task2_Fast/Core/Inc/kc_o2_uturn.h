/* O2 corner: one continuous U-turn round O2's end (Claude; RIGHT route v15,
 * LEFT route v16). Included by kc_task2.h after kc_fast_o1.h.
 *
 * LEFT route (O2 on the right): RIGHT U-turn instead of the legacy RIGHT 180
 * (4.0 s, slow last 30 deg). The right lock is wider (2R ~61 cm against the
 * legacy 180's 57), so it never needs the straight; the back face is ~5-9 cm
 * further away (legacy back IR 14-17 cm).
 *
 * The old corner was LEFT 90, 10 cm forward, LEFT 90: three stops, ~5 s.
 * Two left-lock 90s move the robot only ~46 cm sideways, which left the back
 * face 7.7 cm from the IR (wheel nearly touching); the 10 cm straight makes
 * it 56. This drives the same shape without stopping: left lock, a short
 * straight at 90 deg, left lock, a straight at 180 deg, brake. The straight
 * ends when the model says the second half lands at KCU_SHIFT. The pose
 * comes from wheel+gyro odometry, as for the LEFT route's 180.
 *
 * Rolling (v17): the front face's end starts the U-turn at face speed
 * without braking (steering swings in from centre), and the straight at
 * 180 deg hands over to the back face as soon as the side IR has the face
 * twice (10-40 cm, within 3 cm) and the heading is within 3 deg. No face
 * (narrow O2), a reading under 10 cm or 30 cm without the face: brake and
 * use the standing check and back-face search as before.
 *
 * Local frame: w = sideways toward O2's back, a = heading change toward
 * the turn (rad); both mirrored by usg (+1 LEFT U-turn, -1 RIGHT). Odometry
 * is the rear axle; the centre's sideways shift equals the axle's at 0/180.
 */
#ifndef KC_O2_UTURN_H
#define KC_O2_UTURN_H

#ifndef KC_O2_UTURN
#define KC_O2_UTURN 3  /* bit 0: RIGHT route (else LEFT 90, 10 cm, LEFT 90); bit 1: LEFT route (else legacy RIGHT 180) */
#endif
#ifndef KC_O2_UTURN_ROLL
#define KC_O2_UTURN_ROLL 1  /* 1: no stop before or after the U-turn (v17); 0: brake, standing IR check */
#endif
#define KCU_ROLL_TOL      3.0f  /* deg off 180 allowed when handing over to the back face */
#define KCU_SHIFT        56.0f  /* cm, LEFT U-turn: LEFT 90 (31) + 10 + LEFT 90 (15); back IR ~17.7 */
#define KCU_SHIFT_R      57.0f  /* cm, RIGHT U-turn: the legacy RIGHT 180 (robot: 56.5-57.8) */
#define KCU_STRAIGHT_MAX 20.0f  /* cm at 90 deg; the model needs ~0-10 */
#define KCU_END_MIN       4.0f  /* cm at 180 deg before braking */
#define KCU_END_MAX      30.0f
#define KCU_END_TOL       2.0f  /* deg off 180 allowed at the brake */
#define KCU_TIMEOUT_MS   5000u
#define KCU_PI 3.14159265f

/* Curvature change per cm while the steering swings at speed v; swing_s is
 * the lock-to-lock time (KCF_T_RL swinging leftward, KCF_T_LR rightward). */
static float kcu_rate(float v,float swing_s){
 return (1.0f/KCF_R_LEFT+1.0f/KCF_R_RIGHT)/(kc_max(20.0f,v)*swing_s);
}
static void kcu_sim(float *w,float *a,float *k,float target,float rate,float ds){
 float dk=target-*k;
 if(dk>rate*ds)dk=rate*ds;else if(dk<-rate*ds)dk=-rate*ds;
 *k+=dk;*a+=*k*ds;*w+=sinf(*a)*ds;
}
/* Heading still gained while the steering centres from curvature k. */
static float kcu_lead(float k,float down){return k>0?k*k/(2.0f*down):0.0f;}
/* Sideways shift at the end if the second half starts now. */
static float kcu_predict(float w,float a,float k,float kl,float up,float down){
 int n=0;
 for(;n<400 && a+kcu_lead(k,down)<KCU_PI;n++)kcu_sim(&w,&a,&k,kl,up,0.5f);
 for(;n<600 && k>1e-4f;n++)kcu_sim(&w,&a,&k,0.0f,down,0.5f);
 return w;
}
static void kc_uturn_begin(KcTask2 *t,const KcT2Input *i,float actual,int rolling){
 t->usg=t->side2==2?1.0f:-1.0f; /* RIGHT route goes round O2 to the left */
 t->guard_us=0;t->acquired=0;t->index=0;
 t->turn_ox=i->ox;t->turn_oy=i->oy;t->turn_h0=actual;
 t->move_heading=t->h;t->start_x=t->x;t->start_y=t->y;
 t->umode=rolling?1:0;t->uat=i->now;t->ustart=i->now;t->ulast=i->now;t->uk=0;t->ukscale=1.0f;
 t->ud0=i->odist;t->uodo0=i->odist;t->upred=i->odist;t->ustraight=0;t->uv=0;
 kc_event(t,"UTURN_START",t->usg>0?KCU_SHIFT:-KCU_SHIFT_R,actual);
 if(rolling)kc_event(t,"UTURN_ROLL_IN",i->speed,t->x);
 kc_enter(t,KCT_O2_UTURN,i->now);
}
/* Pose from odometry at the end of the U-turn, plus any brake skid along the
 * heading; the legacy moves then steer to the new heading. */
static void kcu_finish_pose(KcTask2 *t,const KcT2Input *i,float actual,float skid){
 float h1=kc_rad(actual),h0=kc_rad(t->turn_h0);
 t->x=t->start_x+(i->ox-t->turn_ox)-KCF_AXLE_CENTRE*(sinf(h1)-sinf(h0))-skid*sinf(h1);
 t->y=t->start_y+(i->oy-t->turn_oy)+KCF_AXLE_CENTRE*(cosf(h1)-cosf(h0))+skid*cosf(h1);
 t->h=t->move_heading+t->usg*180.0f;
 if(t->io.goal)t->io.goal(t->origin_heading+t->h);
}
static float kcu_hold(float actual,float goal,float gyro){
 return kc_max(-KCF_OFF_MAX,kc_min(KCF_OFF_MAX,KCF_KH*(actual-goal)+KCF_KD*gyro));
}
static void kc_uturn_step(KcTask2 *t,const KcT2Input *i,float actual){
 float sg=t->usg,hn=kc_rad(t->move_heading),dx=i->ox-t->turn_ox,dy=i->oy-t->turn_oy;
 float w=sg*(-dx*cosf(hn)-dy*sinf(hn)),a=sg*kc_rad(actual-t->move_heading),v=i->speed;
 float rlock=sg>0?KCF_R_LEFT:KCF_R_RIGHT,shift=sg>0?KCU_SHIFT:KCU_SHIFT_R;
 float t_in=sg>0?KCF_T_RL:KCF_T_LR,t_out=sg>0?KCF_T_LR:KCF_T_RL;
 float kl=t->ukscale/rlock,up=kcu_rate(v,t_in),down=kcu_rate(v,t_out);
 int lock=sg>0?KCF_DRIVE_LEFT:KCF_DRIVE_RIGHT;
 /* Steering model (time based, toward the turn +) for when the speed is too low to measure. */
 float dt=(float)(uint32_t)(i->now-t->ulast)*0.001f;t->ulast=i->now;
 {
  float target=(t->umode<=1 || t->umode==3)?kl:0.0f,full=1.0f/KCF_R_LEFT+1.0f/KCF_R_RIGHT;
  float step=full/(target>t->uk?t_in:t_out)*dt,dk=target-t->uk;
  t->uk+=dk>step?step:dk<-step?-step:dk;
 }
 float k=v>15.0f?sg*kc_rad(i->gyro)/v:t->uk;
 if(t->umode<=4){
  if(kc_elapsed(i->now,t->ustart,KCU_TIMEOUT_MS)){kc_fail(t,"ERROR_UTURN_TIMEOUT",i->now);return;}
  if(a<-0.35f || a>KCU_PI+0.5f){kc_fail(t,"ERROR_UTURN_HEADING",i->now);return;}
  if(i->odist-t->uodo0>170.0f){kc_fail(t,"ERROR_UTURN_DISTANCE",i->now);return;}
 }
 switch(t->umode){
 case 0:
  /* Steer to lock at standstill first, as the legacy turn does. */
  if(!kc_elapsed(i->now,t->uat,KCF_RF_SERVO_MS)){t->io.drive(sg>0?KCF_DRIVE_BRAKE_LEFT:KCF_DRIVE_BRAKE_RIGHT,0.0f,0);break;}
  t->umode=1;t->uat=i->now;
  /* fall through */
 case 1:
  /* This floor's lock radius: gyro curvature once the steering is at lock. */
  if(kc_elapsed(i->now,t->uat,450) && v>15.0f && sg*i->gyro>5.0f){
   float sc=kc_max(0.85f,kc_min(1.2f,k*rlock));
   t->ukscale+=0.05f*(sc-t->ukscale);
  }
  if(a+kcu_lead(k,down)>=0.5f*KCU_PI){t->umode=2;t->ud0=i->odist;t->upred=-1e9f;kc_event(t,"UTURN_90",w,actual-t->move_heading);}
  else{t->io.drive(lock,0.0f,KCF_PWM_TURN);break;}
  /* fall through */
 case 2:{
  float run=i->odist-t->ud0;
  if(i->odist-t->upred>=0.5f){
   t->upred=i->odist;
   float pred=kcu_predict(w,a,k,kl,up,down);
   if(pred>=shift || run>=KCU_STRAIGHT_MAX){
    t->umode=3;t->ustraight=run;kc_event(t,"UTURN_STRAIGHT",run,pred);
    t->io.drive(lock,0.0f,KCF_PWM_TURN);break;
   }
  }
  t->io.drive(KCF_DRIVE_STRAIGHT,kcu_hold(actual,t->move_heading+sg*90.0f,i->gyro),KCF_PWM_TURN);
  break;}
 case 3:
  if(a+kcu_lead(k,down)>=KCU_PI){t->umode=4;t->ud0=i->odist;t->count=0;t->samples=0;t->ir_seen=i->ir_seq;}
  else{t->io.drive(lock,0.0f,KCF_PWM_TURN);break;}
  /* fall through */
 case 4:{
  float run=i->odist-t->ud0,err=actual-(t->move_heading+sg*180.0f);
  int settled=run>=KCU_END_MIN && fabsf(err)<=KCU_END_TOL && fabsf(i->gyro)<=15.0f,brake;
  if(KC_O2_UTURN_ROLL){
   int close=0;
   if(i->ir_seq!=t->ir_seen){
    t->ir_seen=i->ir_seq;float r=kc_inner(t,i);
    if(isfinite(r) && r<10.0f)close=1;
    else if(isfinite(r) && r<=40.0f){t->metric[0]=t->metric[1];t->metric[1]=r;if(t->count<2)t->count++;t->samples=0;}
    else{t->count=0;if(t->samples<255)t->samples++;}
   }
   if(!close && t->count>=2 && fabsf(t->metric[0]-t->metric[1])<=3.0f && run>=KCU_END_MIN &&
      fabsf(err)<=KCU_ROLL_TOL && fabsf(i->gyro)<=20.0f){
    kcu_finish_pose(t,i,actual,0.0f);t->umode=6;
    t->back_gap=0.5f*(t->metric[0]+t->metric[1]);
    kc_event(t,"UTURN",w,t->ustraight);
    kc_event(t,"UTURN_TIME",(float)(uint32_t)(i->now-t->ustart),actual-t->h);
    kc_event(t,"UTURN_ROLL_OUT",t->back_gap,v);
    kc_event(t,"POSE",t->x,t->y);
    kc_back_face_start(t,i);break;
   }
   /* Nothing beside us once straight (narrow O2): stop here, not 30 cm on. */
   brake=close || (settled && t->samples>=2) || run>=KCU_END_MAX;
  }else brake=settled || run>=KCU_END_MAX;
  if(brake){t->uv=v;t->io.stop();t->umode=5;t->uat=i->now;break;}
  t->io.drive(KCF_DRIVE_STRAIGHT,kcu_hold(actual,t->move_heading+sg*180.0f,i->gyro),KCF_PWM_TURN);
  break;}
 default:{
  /* Braked and settled (the adapter waits for quiet encoders); odometry
   * has the brake travel now. Add the unseen skid along the heading. */
  kcu_finish_pose(t,i,actual,KCF_RF_SKID*(t->uv/57.0f)*(t->uv/57.0f));
  kc_event(t,"UTURN",w,t->ustraight);
  kc_event(t,"UTURN_TIME",(float)(uint32_t)(i->now-t->ustart),actual-t->h);
  kc_event(t,"POSE",t->x,t->y);
  if(fabsf(actual-t->h)>5.0f){kc_event(t,"FINAL_HEADING",actual,t->h);kc_fail(t,"ERROR_FINAL_HEADING",i->now);return;}
  t->acquired=0;t->index=0;t->count=0;
  kc_enter(t,KCT_O2_BACK_ACQUIRE,i->now);
  break;}
 }
}
#endif
