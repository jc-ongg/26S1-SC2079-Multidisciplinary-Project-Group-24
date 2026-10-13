/* KC Task 2 trial. Portable state machine; no RTOS or HAL dependency.
 * Coordinates: start centre (0,0), right +X, outward +Y, left yaw positive.
 * Straight distances use encoders INCLUDING interrupted-search braking.
 * Turn translations remain provisional speed-50 centre-to-centre calibration.
 */
#ifndef KC_TASK2_H
#define KC_TASK2_H
#include <stdint.h>
#include <string.h>
#include <math.h>

enum KcMove { KCT_FWD, KCT_REV, KCT_LEFT, KCT_RIGHT, KCT_REV_LEFT };
enum KcPhase { KCT_IDLE, KCT_O1_RANGE, KCT_O1_VISION, KCT_O1_PLAN,
 KCT_O2_RANGE, KCT_O2_VISION, KCT_O2_TARGET, KCT_O2_FIRST,
 KCT_IR_ACQUIRE, KCT_IR_MEASURE, KCT_IR_EDGE, KCT_IR_RECOVER, KCT_O2_CORNER,
 KCT_O2_BACK_TURN, KCT_O2_BACK_ACQUIRE, KCT_O2_BACK_EDGE, KCT_O2_RETURN_TURN,
 KCT_RETURN, KCT_O1_EDGE, KCT_REAR_CLEAR, KCT_CENTRE,
 KCT_PARK, KCT_MOVE, KCT_DONE, KCT_ERROR, KCT_O2_BACK_CLEAR };
typedef struct {
 uint32_t now, us_seq, ir_seq, us_at, ir_at, imu_at;
 float us, left_ir, right_ir, heading, gyro;
 uint8_t us_valid, gyro_ready, motion_status; /* 0 busy, 1 done, 2 failed */
 float progress; /* magnitude of mean rear-wheel encoder travel for current command */
} KcT2Input;
typedef struct {
 void (*start)(int move, unsigned speed, unsigned amount);
 float (*stop)(void); /* brake, settle, finalize gyro; return final encoder travel */
 void (*event)(const char *name, float a, float b);
} KcT2Hooks;
typedef struct {
 KcT2Hooks io;
 enum KcPhase phase, after, edge_after;
 uint32_t token, started, entered, heartbeat, us_seen, ir_seen, group_at, edge_started, park_ir_seen;
 uint8_t active, cap, side1, side2, waiting_arrow, arrow, vision_failed, park_reversed;
 uint8_t move_live, guard_us, acquired, count, samples, corrections, index, ir_retry, centre_right;
 int move;
 float x,y,h, origin_heading, move_heading, start_x,start_y, amount;
 float o1_gap,o1_front,o2_front, us_anchor, group[3], range, last_progress;
 float expected_range, target, back_gap, edge_progress, metric[2], gap;
 float expected_heading, centre_diagonal;
 unsigned speed;
 uint8_t park_pairs, park_recheck_used, park_rechecking, park_segments;
 float park_l,park_r,park_forward_total;
} KcTask2;

static float kc_min(float a,float b){return a<b?a:b;}
static float kc_max(float a,float b){return a>b?a:b;}
static int kc_elapsed(uint32_t now,uint32_t then,uint32_t limit){return (uint32_t)(now-then)>limit;}
static float kc_rad(float degrees){return degrees*0.017453292519943295f;}
static void kc_event(KcTask2 *t,const char *s,float a,float b){t->io.event(s,a,b);}
static void kc_enter(KcTask2 *t,enum KcPhase p,uint32_t now){
 t->phase=p;t->entered=now;t->count=0;t->samples=0;t->group_at=now;
 kc_event(t,"STATE",(float)p,t->y);
}
static void kc_pose_finish(KcTask2 *t,float distance){
 if(t->move==KCT_FWD || t->move==KCT_REV){
  float d=distance*(t->move==KCT_REV?-1.0f:1.0f);
  t->x=t->start_x-d*sinf(kc_rad(t->move_heading));
  t->y=t->start_y+d*cosf(kc_rad(t->move_heading));
 }else{
  /* Physical left/right are NOT symmetric. Only completed turns use this model. */
  float f,s;
  if(t->move==KCT_LEFT){f=t->amount==45?13.25f:15.0f;s=t->amount==45?-13.0f:-31.0f;}
  else if(t->move==KCT_RIGHT){f=t->amount==45?18.0f:20.5f;s=t->amount==45?15.0f:38.0f;}
  else{f=-12.5f;s=-1.5f;} /* measured reverse-LEFT45, clockwise yaw */
  float r=kc_rad(t->move_heading);
  t->x=t->start_x+s*cosf(r)-f*sinf(r);
  t->y=t->start_y+f*cosf(r)+s*sinf(r);
  t->h=t->move_heading+(t->move==KCT_LEFT?t->amount:-t->amount);
 }
 t->move_live=0;
 kc_event(t,"POSE",t->x,t->y);
}
static float kc_brake(KcTask2 *t){
 float d=t->io.stop();
 if(t->move_live && (t->move==KCT_FWD || t->move==KCT_REV))kc_pose_finish(t,d);
 t->move_live=0;
 return d;
}
static void kc_fail(KcTask2 *t,const char *why,uint32_t now){
 kc_brake(t);t->active=0;t->waiting_arrow=0;
 kc_enter(t,KCT_ERROR,now);kc_event(t,why,t->x,t->y);
}
static void kc_done(KcTask2 *t,uint32_t now){
 kc_brake(t);t->active=0;t->waiting_arrow=0;
 kc_enter(t,KCT_DONE,now);kc_event(t,"COMPLETE",t->x,t->y);
}
static void kc_begin_move(KcTask2 *t,int move,unsigned speed,unsigned n,enum KcPhase phase,uint32_t now){
 t->move=move;t->speed=speed;t->amount=(float)n;t->move_heading=t->h;
 t->start_x=t->x;t->start_y=t->y;t->last_progress=0;t->move_live=1;
 t->io.start(move,speed,n);kc_enter(t,phase,now);
 kc_event(t,"MOTION",(float)move,(float)n);
}
static void kc_move_to(KcTask2 *t,int move,unsigned speed,unsigned n,enum KcPhase after,uint32_t now){
 t->after=after;kc_begin_move(t,move,speed,n,KCT_MOVE,now);
}
static float kc_inner(const KcTask2 *t,const KcT2Input *i){return t->side2==1?i->right_ir:i->left_ir;}
static int kc_us_group(KcTask2 *t,const KcT2Input *i,float anchor,float allowance){
 if(i->us_seq==t->us_seen)return 0;
 t->us_seen=i->us_seq;
 if(kc_elapsed(i->now,t->group_at,1000))t->samples=0;
 if(!i->us_valid || !isfinite(i->us) || i->us<=2 || i->us>180 || kc_elapsed(i->now,i->us_at,250))return 0;
 if(isfinite(anchor) && fabsf(i->us-anchor)>allowance){kc_event(t,"US_REJECT",i->us,anchor);return 0;}
 if(t->samples && fabsf(i->us-t->group[0])>3)t->samples=0;
 if(!t->samples)t->group_at=i->now;
 t->group[t->samples++]=i->us;
 if(t->samples<3)return 0;
 float a=t->group[0],b=t->group[1],c=t->group[2];
 t->range=kc_max(kc_min(a,b),kc_min(kc_max(a,b),c));t->samples=0;
 kc_event(t,"US_VERIFIED",t->range,anchor);return 1;
}
static void kc_capture(KcTask2 *t,unsigned obstacle,uint32_t now){
 if(t->waiting_arrow!=obstacle){t->waiting_arrow=(uint8_t)obstacle;t->arrow=0;}
 t->vision_failed=0;
 kc_enter(t,obstacle==1?KCT_O1_VISION:KCT_O2_VISION,now);
 kc_event(t,"CAPTURE",(float)obstacle,(float)t->token);
}
static int kc_start(KcTask2 *t,KcT2Hooks hooks,const KcT2Input *i,uint32_t token,unsigned cap){
 if(cap>3 || !i->gyro_ready || !isfinite(i->heading) || !isfinite(i->gyro) || fabsf(i->gyro)>3 || kc_elapsed(i->now,i->imu_at,250))return 0;
 memset(t,0,sizeof(*t));t->io=hooks;t->active=1;t->cap=(uint8_t)cap;
 t->token=token;t->started=i->now;t->heartbeat=i->now;t->origin_heading=i->heading;
 t->us_seen=i->us_seq;t->ir_seen=i->ir_seq;t->us_anchor=NAN;t->expected_range=NAN;t->gap=NAN;t->o1_gap=NAN;
 t->io.stop();kc_enter(t,KCT_O1_RANGE,i->now);kc_event(t,"BUILD_TASK2KC",2,0);kc_event(t,"PROFILE",26,850);kc_event(t,"FINE_POSITION",2,26);kc_event(t,"O1_ACCEPTANCE",30,34);return 1;
}
/* Only reply to the current run and requested obstacle; late replies cannot steer a new run. */
static int kc_arrow(KcTask2 *t,uint32_t token,unsigned obstacle,unsigned side){
 if(!t->active || token!=t->token || obstacle!=t->waiting_arrow || (side!=1 && side!=2))return 0;
 t->arrow=(uint8_t)side;return 1;
}
static void kc_range_control(KcTask2 *t,const KcT2Input *i,enum KcPhase p,float target){
 if(!kc_us_group(t,i,t->expected_range,isfinite(t->expected_range)?4.0f:180.0f)){
  if(kc_elapsed(i->now,t->entered,3500))kc_fail(t,"ERROR_US_UNRELIABLE",i->now);
  return;
 }
 if(p==KCT_O1_RANGE && !isfinite(t->us_anchor)){
  t->o1_front=t->range+11.5f;t->us_anchor=t->range;
 }
 if(p==KCT_O2_RANGE && !isfinite(t->o2_front))t->o2_front=t->y+11.5f+t->range;
 /* Request only the obstacle currently faced. Retain a valid result through
  * all positioning corrections; using it cannot bypass range positioning. */
 if((p==KCT_O1_RANGE || p==KCT_O2_RANGE) && !t->waiting_arrow){
  t->waiting_arrow=p==KCT_O1_RANGE?1:2;t->arrow=0;t->vision_failed=0;
  kc_event(t,"CAPTURE_EARLY",t->waiting_arrow,(float)t->token);
 }
 /* Retain the calibrated RIGHT26..30cm band introduced for the old3cm
  * correction policy. The new2cm fine correction reduces quantisation. */
 float tolerance=(p==KCT_O1_RANGE || (p==KCT_O2_TARGET && t->side2==2))?2.0f:1.5f;
 if(p==KCT_O2_RANGE && t->arrow){
  /* Once vision has resolved direction, position directly for that turn.
   * Do not keep chasing28cm only to reposition for LEFT26cm next. */
  target=t->arrow==1?26:28;tolerance=t->arrow==1?1.5f:2.0f;
 }
 if(fabsf(t->range-target)<=tolerance){
  t->corrections=0;
  if(p==KCT_O1_RANGE){
   t->o1_gap=t->range;kc_event(t,"O1_GAP",t->o1_gap,t->o1_gap-32.0f);
   kc_capture(t,1,i->now);
  }
  else if(p==KCT_O2_RANGE)kc_capture(t,2,i->now);
  else if(t->cap==2)kc_done(t,i->now);
  else kc_enter(t,KCT_O2_FIRST,i->now);
  return;
 }
 if(t->corrections++>=4){kc_fail(t,"ERROR_POSITION_LIMIT",i->now);return;}
 float error=t->range-target;
 int move=error>0?KCT_FWD:KCT_REV;
 /* A1cm command completes immediately at the legacy target-minus1cm
  * threshold. A2cm command has a nonzero travel threshold, and unlike the
  * old3cm minimum can correct a1.5–2cm error without crossing the whole band.
  * Encoder-verified braking is required before any change of direction. */
 unsigned travel=(unsigned)lroundf(kc_max(2.0f,kc_min(fabsf(error),100.0f)));
 if(move==KCT_REV){
  /* At the initial pose rear clearance is unknown. After our own straight
   * approach, net forward encoder travel bounds the distance we can retrace.
   * Leave 1 cm of that known corridor unused for braking/measurement error. */
  float rear_budget;
  if(p==KCT_O1_RANGE){
   rear_budget=kc_max(0,t->y-1.0f);
   if(rear_budget<3){kc_fail(t,"ERROR_START_TOO_CLOSE",i->now);return;}
  }else rear_budget=kc_max(0,60-23-t->range-8);
  if(travel>rear_budget)travel=(unsigned)floorf(rear_budget);
  if(travel<2){kc_fail(t,"ERROR_REAR_BUDGET",i->now);return;}
 }
 t->guard_us=1;kc_move_to(t,move,travel<=4?12:(move==KCT_FWD?(travel>=40?100:22):16),travel,p,i->now);
}
static void kc_o1_plan(KcTask2 *t,const KcT2Input *i){
 static const unsigned left_dist[7]={45,10,45,9,45,10,45};
 /* RIGHT: +6cm before inward turn; +14cm on inward diagonal compensates
  * the lateral change from replacing final forward-RIGHT45 by reverse-LEFT45.
  * Physical RIGHT finish was3cm right: diagonal22->26 adds2.83cm left
  * and2.83cm toward O2. Net Y versus old forward-finish route is-14.51cm.
  * Finish anchor includes that Y change; it remains a calibration estimate. */
 static const unsigned right_dist[7]={45,7,45,20,45,26,45};
 const unsigned *v=t->side1==1?left_dist:right_dist;
 if(t->index>=7){
  /* Empirical post-O1 anchor; keeps the proven centred endpoint convention. */
  t->y=t->o1_front+10+11.5f+(t->side1==1?34.0f:18.243f);
  t->x=0;t->index=0;t->corrections=0;t->expected_range=NAN;t->o2_front=NAN;
  if(t->cap==1)kc_done(t,i->now);else kc_enter(t,KCT_O2_RANGE,i->now);
  return;
 }
 unsigned k=t->index++;
 int outward=t->side1==1?KCT_LEFT:KCT_RIGHT;
 int inward=t->side1==1?KCT_RIGHT:KCT_LEFT;
 int move=(k%2)?KCT_FWD:((k==0 || k==6)?outward:inward);
 if(t->side1==2 && k==6)move=KCT_REV_LEFT;
 unsigned amount=v[k];
 if(k==3){
  /* Correct longitudinal start-gap variation on the heading-zero side lane,
   * before the inward turn. Never alter diagonal centring distances.
   * Rounded integer travel leaves at most0.5cm longitudinal residual.
   * Nominal32cm and the added RIGHT6cm clearance remain the reference. */
  if(!isfinite(t->o1_gap) || t->o1_gap<30 || t->o1_gap>34){
   kc_fail(t,"ERROR_O1_GAP",i->now);return;
  }
  amount=(unsigned)lroundf((float)v[k]+t->o1_gap-32.0f);
  kc_event(t,"O1_LANE_ADJUST",t->o1_gap,(float)amount);
 }
 t->guard_us=0;kc_move_to(t,move,50,amount,KCT_O1_PLAN,i->now);
}
static void kc_park_recheck(KcTask2 *t,const KcT2Input *i){
 /* One stopped revalidation, with new source generations after braking.
  * Recheck may confirm an inside gap; it cannot authorize a new far drive. */
 kc_brake(t);t->park_recheck_used=1;t->park_rechecking=1;
 kc_enter(t,KCT_PARK,i->now);t->us_seen=i->us_seq;t->ir_seen=i->ir_seq;
 t->park_ir_seen=i->ir_seq;t->park_pairs=0;
 kc_event(t,"PARK_US_RECHECK",t->expected_range,0);
}
static void kc_step(KcTask2 *t,const KcT2Input *i){
 if(!t->active)return;
 /* Steering settle / encoder quantisation can report a few reverse counts
  * before forward traction. Preserve signed accumulation in the adapter;
  * tolerate only a bounded 2.5 mm net rollback for pose/validation here. */
 KcT2Input settled=*i;
 if((t->move==KCT_FWD || t->move==KCT_REV) &&
    isfinite(settled.progress) && settled.progress<0 && settled.progress>=-0.25f)
  settled.progress=0;
 i=&settled;
 if(kc_elapsed(i->now,t->heartbeat,2000)){kc_fail(t,"ERROR_PI_HEARTBEAT",i->now);return;}
 if(kc_elapsed(i->now,t->started,180000)){kc_fail(t,"ERROR_RUN_TIMEOUT",i->now);return;}
 if(!isfinite(i->heading) || !i->gyro_ready || kc_elapsed(i->now,i->imu_at,300)){
  kc_fail(t,"ERROR_IMU",i->now);return;
 }
 float actual=i->heading-t->origin_heading;
 if(t->phase==KCT_MOVE){
  /* The signed A-minus-B translation estimate is meaningful for straight
   * commands only. Inner-wheel reversal/coasting during a turn can make it
   * negative; turns are validated using motion status and gyro heading. */
  if(!isfinite(i->progress) ||
     ((t->move==KCT_FWD || t->move==KCT_REV) && i->progress<0)){
   kc_fail(t,"ERROR_ENCODER",i->now);return;
  }
  if(i->motion_status==2){kc_fail(t,"ERROR_MOTION",i->now);return;}
  if(kc_elapsed(i->now,t->entered,20000)){kc_fail(t,"ERROR_MOTION_TIMEOUT",i->now);return;}
  if(t->move==KCT_FWD || t->move==KCT_REV){
   if(fabsf(actual-t->h)>5){kc_fail(t,"ERROR_STRAIGHT_HEADING",i->now);return;}
   /* Encoder-bounded approach: transient US jumps cannot extend this command.
    * Two fresh close echoes can brake a forward approach before encoder DONE. */
   if(t->guard_us && t->move==KCT_FWD && i->us_seq!=t->us_seen){
    t->us_seen=i->us_seq;
    if(i->us_valid && !kc_elapsed(i->now,i->us_at,250) && isfinite(i->us) && i->us<=15)t->count++;
    else t->count=0;
    if(t->count>=2){float d=kc_brake(t);t->expected_range=t->range-d;
     if(t->after==KCT_PARK)t->park_forward_total+=d;
     t->guard_us=0;kc_enter(t,t->after,i->now);return;}
   }
  }
  if(i->motion_status==1){
   float d=i->progress;
   if(t->after==KCT_PARK && t->move==KCT_FWD)t->park_forward_total+=d;
   kc_pose_finish(t,d);
   if(fabsf(actual-t->h)>5){kc_fail(t,"ERROR_FINAL_HEADING",i->now);return;}
   if(t->guard_us){t->expected_range=t->range+(t->move==KCT_REV?d:-d);t->guard_us=0;}
   kc_enter(t,t->after,i->now);
  }
  return;
 }
 switch(t->phase){
 case KCT_O1_RANGE:kc_range_control(t,i,KCT_O1_RANGE,32);break;
 case KCT_O2_RANGE:kc_range_control(t,i,KCT_O2_RANGE,28);break; /* common view pose fits minimum-gap rear budget */
 case KCT_O2_TARGET:kc_range_control(t,i,KCT_O2_TARGET,t->side2==1?26:28);break;
 case KCT_O1_VISION:case KCT_O2_VISION:
  if(t->vision_failed || kc_elapsed(i->now,t->entered,25000)){kc_fail(t,"ERROR_VISION",i->now);break;}
  if(t->arrow){
   unsigned obstacle=t->waiting_arrow;t->waiting_arrow=0;
   if(obstacle==1){t->side1=t->arrow;t->index=0;kc_enter(t,KCT_O1_PLAN,i->now);}
   else{t->side2=t->arrow;t->expected_range=t->range;t->corrections=0;kc_enter(t,KCT_O2_TARGET,i->now);}
  }break;
 case KCT_O1_PLAN:kc_o1_plan(t,i);break;
 case KCT_O2_FIRST:
  t->guard_us=0;t->acquired=0;t->ir_retry=0;
  kc_move_to(t,t->side2==1?KCT_LEFT:KCT_RIGHT,50,90,KCT_IR_ACQUIRE,i->now);break;
 case KCT_IR_ACQUIRE:{
  if(kc_elapsed(i->now,i->ir_at,300)){kc_fail(t,"ERROR_IR_STALE",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  if(isfinite(v) && v>=10 && v<=30)t->count++;else t->count=0;
  if(t->count>=2){t->acquired=1;t->edge_after=KCT_O2_CORNER;t->edge_started=i->now;kc_begin_move(t,KCT_FWD,16,100,KCT_IR_EDGE,i->now);}
  else if(kc_elapsed(i->now,t->entered,400)){
   t->edge_after=KCT_IR_MEASURE;kc_begin_move(t,KCT_REV,16,35,KCT_IR_MEASURE,i->now);
  }break;}
 case KCT_IR_MEASURE:{
  if(kc_elapsed(i->now,i->ir_at,300)){kc_fail(t,"ERROR_IR_STALE",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  if(t->move_live){
   if(fabsf(actual-t->h)>5 || i->motion_status==2 || kc_elapsed(i->now,t->entered,20000)){kc_fail(t,"ERROR_ACQUIRE_MOTION",i->now);break;}
   if(!isfinite(v) || v<10 || (v<=30 && ++t->count>=2)){
    kc_brake(t);kc_enter(t,KCT_IR_MEASURE,i->now);
   }else{if(v>30)t->count=0;if(i->motion_status==1){kc_brake(t);kc_enter(t,KCT_IR_MEASURE,i->now);}}
  }else{
   if(isfinite(v) && v>=10 && v<=30)t->count++;else t->count=0;
   if(t->count>=2){t->acquired=1;t->edge_after=KCT_O2_CORNER;t->edge_started=i->now;kc_begin_move(t,KCT_FWD,16,100,KCT_IR_EDGE,i->now);}
   else if(kc_elapsed(i->now,t->entered,2000))kc_fail(t,"ERROR_IR_ACQUIRE",i->now);
  }break;}
 case KCT_IR_EDGE:case KCT_O2_BACK_EDGE:{
  if(!t->move_live){t->acquired=0;t->ir_retry=0;t->edge_started=i->now;kc_begin_move(t,KCT_FWD,16,100,KCT_O2_BACK_EDGE,i->now);break;}
  if(kc_elapsed(i->now,i->ir_at,300) || fabsf(actual-t->h)>5 || i->motion_status==2){kc_fail(t,"ERROR_EDGE_GUARD",i->now);break;}
  if(kc_elapsed(i->now,t->edge_started,30000)){kc_fail(t,"ERROR_EDGE_TIMEOUT",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  if(!isfinite(v) || v<10){
   if(t->ir_retry++){kc_fail(t,"ERROR_IR_CLOSE",i->now);break;}
   t->edge_after=t->phase==KCT_IR_EDGE?KCT_O2_CORNER:KCT_O2_RETURN_TURN;
   float cap=t->amount,d=kc_brake(t);t->target=kc_max(0,cap-d);
   kc_enter(t,KCT_IR_RECOVER,i->now);break;
  }
  if(v<=40){t->acquired=1;t->count=0;if(t->phase==KCT_O2_BACK_EDGE)t->back_gap=v;}
  else if(t->acquired && v>=60)t->count++;else t->count=0;
  if(t->count>=3){enum KcPhase next=t->phase==KCT_IR_EDGE?KCT_O2_CORNER:KCT_O2_RETURN_TURN;kc_brake(t);kc_enter(t,next,i->now);}
  else if(i->motion_status==1)kc_fail(t,"ERROR_NO_EDGE",i->now);
  break;}
 case KCT_IR_RECOVER:{
  if(kc_elapsed(i->now,i->ir_at,300) || kc_elapsed(i->now,t->entered,2000)
    || kc_elapsed(i->now,t->edge_started,30000)){kc_fail(t,"ERROR_IR_RECOVERY",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;float v=kc_inner(t,i);
  if(t->acquired && isfinite(v) && v>=60){
   t->samples=0;if(++t->count>=3)kc_enter(t,t->edge_after,i->now);
  }else if(isfinite(v) && v>=10 && v<=30){
   t->count=0;
   if(++t->samples>=2){
    if(t->target<2){kc_fail(t,"ERROR_EDGE_LIMIT",i->now);break;}
    enum KcPhase leg=t->edge_after==KCT_O2_CORNER?KCT_IR_EDGE:KCT_O2_BACK_EDGE;
    t->acquired=1;kc_begin_move(t,KCT_FWD,16,(unsigned)floorf(t->target),leg,i->now);
   }
  }else{t->count=0;t->samples=0;}
  break;}
 case KCT_O2_CORNER:
  t->guard_us=0;kc_move_to(t,t->side2==1?KCT_RIGHT:KCT_LEFT,50,90,KCT_O2_BACK_TURN,i->now);break;
 case KCT_O2_BACK_TURN:
  /* RIGHT route: stationary back-face IR was 7.7 cm and the wheel nearly
   * touched. Advance along the SIDE after the first LEFT 90, while heading
   * outward (+Y), before the next LEFT 90 across the back. This translates
   * that turn 10 cm behind O2 without the forbidden extra lateral advance.
   * Nominal new IR gap 7.7+10=17.7 cm; wheel gap about 13.7 cm.
   * Translation is provisional; actual back IR is still required afterward. */
  if(t->side2==2){
   t->guard_us=0;kc_event(t,"BACK_CLEARANCE",10,17.7f);
   kc_move_to(t,KCT_FWD,16,10,KCT_O2_BACK_CLEAR,i->now);break;
  }
  t->acquired=0;kc_move_to(t,KCT_RIGHT,50,90,KCT_O2_BACK_ACQUIRE,i->now);
  t->index=0;break;
 case KCT_O2_BACK_CLEAR:
  t->acquired=0;kc_move_to(t,KCT_LEFT,50,90,KCT_O2_BACK_ACQUIRE,i->now);
  t->index=0;break;
 case KCT_O2_BACK_ACQUIRE:{
  if(kc_elapsed(i->now,i->ir_at,300) || kc_elapsed(i->now,t->entered,2000)){
   kc_fail(t,"ERROR_BACK_NOT_ACQUIRED",i->now);break;
  }
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;float v=kc_inner(t,i);
  if(isfinite(v) && v>=10 && v<=40){
   t->metric[t->count%2]=v;t->count++;
   if(t->count>=2 && fabsf(t->metric[0]-t->metric[1])<=3){
    t->back_gap=(t->metric[0]+t->metric[1])/2;t->acquired=1;t->ir_retry=0;t->edge_started=i->now;
    kc_begin_move(t,KCT_FWD,16,100,KCT_O2_BACK_EDGE,i->now);
   }
  }else t->count=0;
  break;}
 case KCT_O2_RETURN_TURN:
  kc_move_to(t,t->side2==1?KCT_RIGHT:KCT_LEFT,50,90,KCT_RETURN,i->now);break;
 case KCT_RETURN:{
  if(t->cap==3){kc_done(t,i->now);break;}
  float distance=t->y-(t->o1_front+20);
  if(distance<0 || distance>150){kc_fail(t,"ERROR_RETURN_GEOMETRY",i->now);break;}
  t->acquired=0;t->gap=NAN;t->count=0;t->index=0;
  kc_begin_move(t,KCT_FWD,100,(unsigned)ceilf(kc_max(2,distance)),KCT_O1_EDGE,i->now);break;}
 case KCT_O1_EDGE:{
  if(kc_elapsed(i->now,i->ir_at,300) || fabsf(actual-t->h)>5 || i->motion_status==2){kc_fail(t,"ERROR_O1_EDGE_GUARD",i->now);break;}
  if(kc_elapsed(i->now,t->entered,30000)){kc_fail(t,"ERROR_O1_EDGE_TIMEOUT",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  if(isfinite(v) && v>=10 && v<=45 && t->y<=t->o1_front+50){
   t->metric[t->samples%2]=v;t->samples++;
   if(t->samples>=2 && fabsf(t->metric[0]-t->metric[1])<=3){
    t->acquired=1;if(t->metric[0]<=40 && t->metric[1]<=40)t->gap=(t->metric[0]+t->metric[1])/2;
   }t->count=0;
  }else if(t->acquired && isfinite(v) && v>=50)t->count++;else{t->samples=0;t->count=0;}
  /* If O2 is wide, IR cannot reach O1: use measured straight travel + calibrated
   * turn pose to stop at O1 front; report that this is an estimated landmark. */
  float current_y=t->start_y-i->progress;
  if(t->count>=3 || (!t->acquired && current_y<=t->o1_front+3)){
   kc_brake(t);
   if(isfinite(t->gap))t->x=(t->side2==1?1:-1)*(t->gap+10.4f);
   else kc_event(t,"O1_POSE_ESTIMATE",t->x,t->y);
   kc_enter(t,KCT_REAR_CLEAR,i->now);
  }else if(i->motion_status==1){
   if(t->index++){kc_fail(t,"ERROR_O1_SEARCH_LIMIT",i->now);break;}
   kc_pose_finish(t,i->progress);
   kc_begin_move(t,KCT_FWD,12,30,KCT_O1_EDGE,i->now);
  }break;}
 case KCT_REAR_CLEAR:{
  float d=t->y-(t->o1_front-14.5f); /* rear past O1 front by3 cm */
  if(d>40){kc_fail(t,"ERROR_O1_CLEAR_GEOMETRY",i->now);break;}
  if(d>=2)kc_move_to(t,KCT_FWD,16,(unsigned)ceilf(d),KCT_CENTRE,i->now);
  else {kc_enter(t,KCT_CENTRE,i->now);}
  t->index=0;break;}
 case KCT_CENTRE:{
  if(t->index==0){
   float shift=t->x>=0?15+(13.25f-13)/1.414213562f:13+(18.0f-15)/1.414213562f;
   float d=(fabsf(t->x)-shift)*1.414213562f;
   if(d<0 || d>70){kc_fail(t,"ERROR_CENTRE_GEOMETRY",i->now);break;}
   t->centre_diagonal=roundf(d);if(t->centre_diagonal==1)t->centre_diagonal=2;
   t->centre_right=t->x>=0;
   t->index=1;kc_move_to(t,t->x>=0?KCT_RIGHT:KCT_LEFT,50,45,KCT_CENTRE,i->now);
  }else if(t->index==1){t->index=2;
   if(t->centre_diagonal>=2)kc_move_to(t,KCT_FWD,100,(unsigned)t->centre_diagonal,KCT_CENTRE,i->now);
  }else if(t->index==2){t->index=3;kc_move_to(t,t->centre_right?KCT_LEFT:KCT_RIGHT,50,45,KCT_CENTRE,i->now);
  }else{t->corrections=0;t->expected_range=NAN;kc_enter(t,KCT_PARK,i->now);}
  break;}
 case KCT_PARK:{
  if(kc_elapsed(i->now,i->ir_at,300)){kc_fail(t,"ERROR_PARK_IR",i->now);break;}
  if(fabsf(actual-t->h)>5){kc_fail(t,"ERROR_PARK_HEADING",i->now);break;}
  float l=i->left_ir,r=i->right_ir;
  /* Existing 60 cm park signature, with 2 cm more upper-sum allowance
   * for measured IR variation. Both individual side walls remain required. */
  int walls=isfinite(l)&&isfinite(r)&&l>=10&&l<=40&&r>=10&&r<=40&&l+r>=41&&l+r<=59;
  if(i->ir_seq!=t->ir_seen){
   t->ir_seen=i->ir_seq;
   if((int32_t)(i->ir_at-t->entered)<150 || !walls)t->count=0;
   else{
    if(t->count && (fabsf(l-t->metric[0])>3 || fabsf(r-t->metric[1])>3))t->count=0;
    t->metric[0]=l;t->metric[1]=r;if(t->count<2)t->count++;
   }
  }
  float park_anchor=t->park_rechecking?NAN:t->expected_range;
  if(!kc_us_group(t,i,park_anchor,isfinite(park_anchor)?4:180)){
   uint32_t wait=isfinite(park_anchor)?800:3500;
   if(kc_elapsed(i->now,t->entered,wait)){
    if(!t->park_recheck_used)kc_park_recheck(t,i);
    else kc_fail(t,"ERROR_PARK_DEPTH_UNCONFIRMED",i->now);
   }
   break;
  }
  if(t->park_rechecking && !(t->range>=10 && t->range<=20)){
   /* Wrong/long echoes after a failed check cannot cause blind advancement. */
   if(kc_elapsed(i->now,t->entered,3500))kc_fail(t,"ERROR_PARK_DEPTH_UNCONFIRMED",i->now);
   break;
  }
  if(t->range>=10&&t->range<=20){
   if(walls && t->count>=2){
    if(t->park_rechecking)kc_event(t,"PARK_US_REANCHORED",t->range,t->expected_range);
    kc_event(t,"PARK_INSIDE",l,r);kc_done(t,i->now);
   }
   /* Stay stationary for fresh confirmation; do not drive farther when
    * the front distance already places the full robot inside the park. */
   else if(kc_elapsed(i->now,t->entered,2000))kc_fail(t,"ERROR_PARK_WALLS",i->now);
   break;
  }
  if(t->range<10){
   /* One stationary, geometry-bounded retreat inside a50cm-deep park.
    * Require BOTH nearby side walls and two agreeing fresh pairs. Allow the
    * observed sum59.9 only for retreat eligibility, not final confirmation. */
   int corridor=isfinite(l)&&isfinite(r)&&l>=10&&l<=40&&r>=10&&r<=40&&l+r>=41&&l+r<=61;
   if(i->ir_seq!=t->park_ir_seen){
    t->park_ir_seen=i->ir_seq;
    if(!corridor || (int32_t)(i->ir_at-t->entered)<150)t->park_pairs=0;
    else{
     if(t->park_pairs && (fabsf(l-t->park_l)>3 || fabsf(r-t->park_r)>3))t->park_pairs=0;
     t->park_l=l;t->park_r=r;if(t->park_pairs<2)t->park_pairs++;
    }
   }
   if(t->park_reversed || t->range<4){kc_fail(t,"ERROR_PARK_CONFIRM",i->now);break;}
   if(!corridor || t->park_pairs<2){
    if(kc_elapsed(i->now,t->entered,2000))kc_fail(t,"ERROR_PARK_REAR_UNKNOWN",i->now);
    break;
   }
   float budget=50-23-t->range-7; /* retain >=7cm rear clearance */
   unsigned d=(unsigned)ceilf(12-t->range);
   if(d>10 || d+1>budget){kc_fail(t,"ERROR_PARK_REAR_BUDGET",i->now);break;}
   t->park_reversed=1;t->guard_us=1;
   kc_event(t,"PARK_REVERSE",(float)d,budget);
   kc_move_to(t,KCT_REV,12,d,KCT_PARK,i->now);break;
  }
  /* Long final approach: bounded segments, always remeasure before more.
   * Existing range filter caps180cm. Total forward encoder travel is bounded,
   * independently of segmentation, plus existing heartbeat/run/motion limits. */
  float budget=150-t->park_forward_total;
  unsigned d=(unsigned)lroundf(kc_max(3,kc_min(t->range-12,40)));
  if(budget<3 || t->park_segments>=8){kc_fail(t,"ERROR_PARK_APPROACH_LIMIT",i->now);break;}
  if(d>budget)d=(unsigned)floorf(budget);
  t->park_segments++;t->guard_us=1;
  kc_event(t,"PARK_APPROACH",(float)d,t->range);
  kc_move_to(t,KCT_FWD,t->range>50?22:12,d,KCT_PARK,i->now);break;}
 default:kc_fail(t,"ERROR_STATE",i->now);break;
 }
}
#endif
