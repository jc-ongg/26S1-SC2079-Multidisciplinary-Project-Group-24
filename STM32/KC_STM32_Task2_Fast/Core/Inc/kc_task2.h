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
#include "kc_entry_geometry.h"
#ifndef KC_FAST_O1
#define KC_FAST_O1 1
#endif
#ifndef KC_O2_FACE_SPEED
#define KC_O2_FACE_SPEED 50  /* following O2's faces with the side IR (was 16, then 30; braked stop + skid in pose) */
#endif
#ifndef KC_ENTRY_HEADING_TOL
#define KC_ENTRY_HEADING_TOL 3.0f  /* deg, car-park entry and return-lane moves (was 2) */
#endif
#ifndef KC_O2_BACK_FAST
#define KC_O2_BACK_FAST 90   /* back face, until KC_O2_BACK_SLOWDOWN cm before the far end mirrored from the near end (O2 centred on the line) */
#endif
#define KC_O2_BACK_SLOWDOWN 45.0f /* ~10 cm stop overshoot at the near end + 35 cm to slow to face speed */
#ifndef KC_O2_CLEAR_SPEED
#define KC_O2_CLEAR_SPEED 30 /* RIGHT route's 10 cm back clearance: short, must stop accurately */
#endif
/* From fast-carpark (Tan Jou Yuan, 2026-10-15): O2's end is passed when the
 * side IR reads this much more than the face it was following (robot: face
 * 19-20, then 43-62 from something ~50 cm beyond the end; the fixed >= 60
 * never held 3 times and the car drove 95 cm into it). Capped at the old 60. */
#define KC_EDGE_CLEAR_MARGIN 15.0f
#define KC_EDGE_WALL_US 20.0f /* two forward echoes this close while following a face: stop */
/* Narrow O2 (robot 2026-10-15, ~20-25 cm): the first 90 leaves the robot past
 * O2's end. 3 open IR readings start the reverse search at once (was a 0.4 s
 * wait), at this speed (was 16: 2.0-2.4 s for 24-28 cm). */
#define KC_O2_SEARCH_SPEED 40
#define KC_O2_SEARCH_OPEN 3
#ifndef KC_O2_CORNER_180
#define KC_O2_CORNER_180 1  /* LEFT route: one RIGHT 180 at O2's corner instead of RIGHT 90 + RIGHT 90 */
#endif
#ifndef KC_PARK_REQUIRE_WALLS
#define KC_PARK_REQUIRE_WALLS 0  /* 1: side IR must see both carpark walls before PARK_INSIDE */
#endif
#ifndef KC_IR_PERIOD_MS
#define KC_IR_PERIOD_MS 50   /* side IR update period (main.c irSensor task); 50 for face speed 30 */
#endif

enum KcMove { KCT_FWD, KCT_REV, KCT_LEFT, KCT_RIGHT, KCT_REV_LEFT };
enum KcPhase { KCT_IDLE, KCT_O1_RANGE, KCT_O1_VISION, KCT_O1_PLAN,
 KCT_O2_RANGE, KCT_O2_VISION, KCT_O2_TARGET, KCT_O2_FIRST,
 KCT_IR_ACQUIRE, KCT_IR_MEASURE, KCT_IR_EDGE, KCT_IR_RECOVER, KCT_O2_CORNER,
 KCT_O2_BACK_TURN, KCT_O2_BACK_ACQUIRE, KCT_O2_BACK_EDGE, KCT_O2_RETURN_TURN,
 KCT_RETURN, KCT_O1_EDGE, KCT_REAR_CLEAR, KCT_CENTRE,
 KCT_PARK, KCT_MOVE, KCT_DONE, KCT_ERROR, KCT_O2_BACK_CLEAR, KCT_ENTRY_ALIGN,
 KCT_RETURN_CLEAR, KCT_RETURN_TRIM_ALIGN, KCT_RETURN_TRIM,
 KCT_F_RANGE, KCT_F_APPROACH, KCT_F_WAIT, KCT_F_SWERVE, KCT_F_SETTLE, KCT_F_SERVO_TEST,
 KCT_O2_UTURN, KCT_O2_BACK_SEARCH };
typedef struct {
 uint32_t now, us_seq, ir_seq, us_at, ir_at, imu_at;
 float us, left_ir, right_ir, heading, gyro;
 uint8_t us_valid, gyro_ready, motion_status; /* 0 busy, 1 done, 2 failed */
 float progress; /* magnitude of mean rear-wheel encoder travel for current command */
 float ox, oy, odist, speed; /* rear-axle odometry (X right, Y forward, cm), total travel, cm/s */
} KcT2Input;
typedef struct {
 void (*start)(int move, unsigned speed, unsigned amount);
 float (*stop)(void); /* brake, settle, finalize gyro; return final encoder travel */
 void (*event)(const char *name, float a, float b);
 void (*drive)(int mode, float servo_offset, unsigned pwm); /* continuous drive, fast O1 */
 void (*goal)(float heading); /* heading legacy moves steer to, after a continuous-drive turn */
} KcT2Hooks;
typedef struct {
 KcT2Hooks io;
 enum KcPhase phase, after, edge_after;
 uint32_t token, started, entered, heartbeat, us_seen, ir_seen, group_at, edge_started, park_ir_seen;
 uint8_t active, cap, side1, side2, waiting_arrow, arrow, vision_failed, park_reversed;
 uint8_t move_live, guard_us, acquired, count, samples, corrections, index, ir_retry, centre_right, edge_segments, index_ir_reject, us_rej_count, back_fast, o2_ranged;
 int move;
 float x,y,h, origin_heading, move_heading, start_x,start_y, amount;
 float o1_gap,o1_front,o2_front, us_anchor, group[3], range, last_progress;
 float expected_range, target, back_gap, edge_progress, metric[2], gap;
 float expected_heading, centre_diagonal, entry_y, return_y, centre_turn, trim_turn, turn_ox, turn_oy, turn_h0, us_rej_ref, o2_end_x, back_fast_until;
 unsigned speed;
 uint8_t return_prepared;
 float return_target;
 uint8_t park_pairs, park_recheck_used, park_rechecking, park_segments;
 float park_l,park_r,park_forward_total;
 uint8_t fmode,fshort,framp_done;
 uint32_t fstart,fok,fat;
 float fk,flast,fpred,framp_lr,framp_rl,fsw,ftarget,fhead,fkscale,fox,foy;
 uint8_t umode;               /* O2 U-turn (kc_o2_uturn.h) */
 uint8_t back_searched;       /* back face looked for by reversing (narrow O2) */
 uint8_t wall_hits;           /* forward echoes within KC_EDGE_WALL_US while following a face */
 float face_ref;              /* side-IR distance of the O2 face being followed (0 = unknown) */
 uint32_t uat,ustart,ulast;
 float uk,ukscale,ud0,uodo0,upred,ustraight,uv,usg;
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
 if(isfinite(anchor) && fabsf(i->us-anchor)>allowance){
  kc_event(t,"US_REJECT",i->us,anchor);
  /* Range positioning only: 8 echoes agreeing within 2 cm and within 10 cm
   * of the encoder estimate are the real distance (robot 2026-10-15: a
   * steady 33-34 cm against 28.5 expected failed the run). A steadier
   * wrong echo further off is still rejected. */
  int ranging=t->phase==KCT_O1_RANGE || t->phase==KCT_O2_RANGE || t->phase==KCT_O2_TARGET;
  if(ranging && fabsf(i->us-anchor)<=10.0f){
   if(t->us_rej_count && fabsf(i->us-t->us_rej_ref)<=2.0f){
    if(++t->us_rej_count>=8){
     t->range=t->us_rej_ref;t->us_rej_count=0;t->samples=0;
     kc_event(t,"US_REANCHORED",t->range,anchor);return 1;
    }
   }else{t->us_rej_ref=i->us;t->us_rej_count=1;}
  }else t->us_rej_count=0;
  return 0;
 }
 t->us_rej_count=0;
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
/* Always included: the platform adapter uses its constants. KC_FAST_O1 only
 * decides whether the rolling swerve runs (1) or the stop-and-turn O1 (0). */
#include "kc_fast_o1.h"
static float kc_edge_clear_th(const KcTask2 *t){
 return t->face_ref>0?kc_min(60.0f,kc_max(30.0f,t->face_ref+KC_EDGE_CLEAR_MARGIN)):60.0f;
}
static int kc_on_face(const KcTask2 *t,float v){return v<=40 && v<kc_edge_clear_th(t)-5;}
/* Back face with the IR already on it: O2 is centred on the O1-O2 line, so
 * the far end mirrors the near one. Run fast until KC_O2_BACK_SLOWDOWN
 * before it, then face speed. */
static void kc_back_face_start(KcTask2 *t,const KcT2Input *i){
 t->acquired=1;t->ir_retry=0;t->edge_started=i->now;t->edge_segments=0;t->face_ref=t->back_gap;
 float s=t->o2_end_x>0?1.0f:-1.0f,fast=s*t->x+fabsf(t->o2_end_x)-KC_O2_BACK_SLOWDOWN;
 if(isfinite(fast) && fast>=25.0f){
  t->back_fast=1;t->back_fast_until=fast;kc_event(t,"BACK_FAST",fast,t->o2_end_x);
  kc_begin_move(t,KCT_FWD,KC_O2_BACK_FAST,(unsigned)lroundf(fast+40.0f),KCT_O2_BACK_EDGE,i->now);
 }else{t->back_fast=0;kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_O2_BACK_EDGE,i->now);}
}
#include "kc_o2_uturn.h"
static int kc_start(KcTask2 *t,KcT2Hooks hooks,const KcT2Input *i,uint32_t token,unsigned cap){
 if(cap>(KC_FAST_O1?4u:3u) || !i->gyro_ready || !isfinite(i->heading) || !isfinite(i->gyro) || fabsf(i->gyro)>3 || kc_elapsed(i->now,i->imu_at,250))return 0;
 memset(t,0,sizeof(*t));t->io=hooks;t->active=1;t->cap=(uint8_t)cap;
 t->token=token;t->started=i->now;t->heartbeat=i->now;t->origin_heading=i->heading;
 t->us_seen=i->us_seq;t->ir_seen=i->ir_seq;t->us_anchor=NAN;t->expected_range=NAN;t->gap=NAN;t->o1_gap=NAN;
 t->io.stop();
#if KC_FAST_O1
 /* START ack first: the bridge only handles CAPTURE events after it. */
 t->phase=KCT_IDLE;kc_event(t,"BUILD_TASK2KC",2,0);kc_event(t,"PROFILE",26,850);kc_event(t,"FINE_POSITION",2,26);kc_event(t,"PARK_ENTRY",5,22);kc_event(t,"RETURN_LANE_TRIAL",2,22);
 kcf_begin(t,i);return 1;
#else
 kc_enter(t,KCT_O1_RANGE,i->now);kc_event(t,"BUILD_TASK2KC",2,0);kc_event(t,"PROFILE",26,850);kc_event(t,"FINE_POSITION",2,26);kc_event(t,"O1_ACCEPTANCE",30,34);kc_event(t,"PARK_ENTRY",5,22);kc_event(t,"RETURN_LANE_TRIAL",2,22);return 1;
#endif
}
/* Only reply to the current run and requested obstacle; late replies cannot steer a new run. */
static int kc_arrow(KcTask2 *t,uint32_t token,unsigned obstacle,unsigned side){
 if(!t->active || token!=t->token || obstacle!=t->waiting_arrow || (side!=1 && side!=2))return 0;
 t->arrow=(uint8_t)side;return 1;
}
/* O2 ranges the first 90 accepts (side 1 LEFT, 2 RIGHT). LEFT 90 (15 cm
 * forward): side IR ~12-21 from 24.5 to 34 (needs 10-30). RIGHT 90 (20.5
 * forward): the lower 26 keeps the corner off the face; up to 34 the IR
 * reads ~16-20. Corrections aim at 30, mid-band: robot 2026-10-15 (v19, O2
 * RIGHT) read 33.3, 21.3, 33.1, 24.6, 31.5 at an O2 face steady within
 * +-2 cm, chased the noise five times and stopped (ERROR_POSITION_LIMIT). Before the arrow is
 * known a range suiting either is accepted; if the arrow needs the other,
 * O2_TARGET corrects then (no dearer than now). Robot 2026-10-15: the old
 * 26.5-29.5 / 24.5-28.5 bands cost 2-3 cm creeps of 0.6-0.8 s, twice in one
 * run. */
static int kc_o2_band(float r,int side){return side==1?(r>=24.5f && r<=34.0f):(r>=26.0f && r<=34.0f);}
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
 int inband;
 if(p==KCT_O2_RANGE || p==KCT_O2_TARGET){
  int side=p==KCT_O2_TARGET?t->side2:t->arrow; /* 0: arrow not known yet */
  target=30.0f;
  inband=side?kc_o2_band(t->range,side):(kc_o2_band(t->range,1) || kc_o2_band(t->range,2));
 }else inband=fabsf(t->range-target)<=tolerance;
 if(inband){
  t->corrections=0;
  /* Arrows this range already suits (bit 1 LEFT, 2 RIGHT): O2_VISION then
   * skips the repeat check. */
  if(p==KCT_O2_RANGE)t->o2_ranged=(uint8_t)((kc_o2_band(t->range,1)?1:0)|(kc_o2_band(t->range,2)?2:0));
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
/* Long straight moves on the way home at speed 100: motorPidForward caps PWM
 * by distance left (5000 <50 cm, 3000 <20, 850 <10), so they still stop to
 * ~1 cm. Below ~35 cm the 850 tail is slower than speed 22 itself. */
static unsigned kc_long_speed(float cm,unsigned slow){return cm>=35.0f?100u:slow;}
/* Entry turn angle: 45 normally, 90 for the wide-lane crossing (0 = unset). */
static float kc_centre_turn(const KcTask2 *t){return t->centre_turn>60?90.0f:45.0f;}
static float kc_trim_turn(const KcTask2 *t){return t->trim_turn>60?90.0f:45.0f;}
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
  if((t->move==KCT_FWD || t->move==KCT_REV) && (t->after==KCT_CENTRE || t->after==KCT_ENTRY_ALIGN || t->after==KCT_RETURN_TRIM || t->after==KCT_RETURN_TRIM_ALIGN) && fabsf(actual-t->h)>KC_ENTRY_HEADING_TOL){kc_event(t,"ENTRY_HEADING",actual,t->h);kc_fail(t,"ERROR_ENTRY_HEADING",i->now);return;}
  if((t->after==KCT_CENTRE || t->after==KCT_RETURN_TRIM) && t->move==KCT_FWD && i->progress>t->amount+2){kc_fail(t,"ERROR_ENTRY_TRAVEL",i->now);return;}
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
   if((t->move==KCT_LEFT || t->move==KCT_RIGHT) && t->amount>=180){
    /* No measured fit for a continuous 180: wheel + gyro odometry of the
     * rear axle, moved to the robot centre (8.2 cm ahead of the axle). */
    float h0=kc_rad(t->turn_h0),h1=kc_rad(actual);
    t->x=t->start_x+(i->ox-t->turn_ox)-KCF_AXLE_CENTRE*(sinf(h1)-sinf(h0));
    t->y=t->start_y+(i->oy-t->turn_oy)+KCF_AXLE_CENTRE*(cosf(h1)-cosf(h0));
    t->h=t->move_heading+(t->move==KCT_LEFT?t->amount:-t->amount);
    t->move_live=0;kc_event(t,"TURN180",t->x-t->start_x,t->y-t->start_y);kc_event(t,"POSE",t->x,t->y);
   }else kc_pose_finish(t,d);
   if(fabsf(actual-t->h)>((t->after==KCT_CENTRE || t->after==KCT_ENTRY_ALIGN || t->after==KCT_RETURN_TRIM || t->after==KCT_RETURN_TRIM_ALIGN)?KC_ENTRY_HEADING_TOL:5)){kc_event(t,"FINAL_HEADING",actual,t->h);kc_fail(t,"ERROR_FINAL_HEADING",i->now);return;}
   if(t->guard_us){t->expected_range=t->range+(t->move==KCT_REV?d:-d);t->guard_us=0;}
   kc_enter(t,t->after,i->now);
  }
  return;
 }
 if(t->phase==KCT_O2_UTURN){kc_uturn_step(t,i,actual);return;}
#if KC_FAST_O1
 if(t->phase>=KCT_F_RANGE && t->phase<=KCT_F_SERVO_TEST){kcf_step(t,i,actual);return;}
#endif
 switch(t->phase){
 case KCT_O1_RANGE:kc_range_control(t,i,KCT_O1_RANGE,32);break;
 case KCT_O2_RANGE:kc_range_control(t,i,KCT_O2_RANGE,28);break; /* common view pose fits minimum-gap rear budget */
 case KCT_O2_TARGET:kc_range_control(t,i,KCT_O2_TARGET,t->side2==1?26:28);break;
 case KCT_O1_VISION:case KCT_O2_VISION:
  if(t->vision_failed || kc_elapsed(i->now,t->entered,25000)){kc_fail(t,"ERROR_VISION",i->now);break;}
  if(t->arrow){
   unsigned obstacle=t->waiting_arrow;t->waiting_arrow=0;
   if(obstacle==1){t->side1=t->arrow;t->index=0;kc_enter(t,KCT_O1_PLAN,i->now);}
   else{
    t->side2=t->arrow;t->expected_range=t->range;t->corrections=0;
    /* Already positioned for this arrow at O2_RANGE and not moved since:
     * skip the repeat range check (~0.3 s). */
    if(t->o2_ranged&t->side2){kc_event(t,"O2_RANGE_KEPT",t->range,(float)t->side2);kc_enter(t,t->cap==2?KCT_O2_TARGET:KCT_O2_FIRST,i->now);}
    else kc_enter(t,KCT_O2_TARGET,i->now);
   }
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
  if(!isfinite(v) || v>40)t->samples++;else t->samples=0;
  if(t->count>=2){t->acquired=1;t->face_ref=v;t->edge_after=KCT_O2_CORNER;t->edge_started=i->now;t->edge_segments=0;kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_IR_EDGE,i->now);}
  else if(t->samples>=KC_O2_SEARCH_OPEN || kc_elapsed(i->now,t->entered,400)){
   /* Nothing beside us: past O2's end (narrow O2). Back along the lane to it. */
   kc_event(t,"FACE_SEARCH",v,t->x);
   t->edge_after=KCT_IR_MEASURE;kc_begin_move(t,KCT_REV,KC_O2_SEARCH_SPEED,35,KCT_IR_MEASURE,i->now);
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
   if(t->count>=2){t->acquired=1;t->face_ref=v;t->edge_after=KCT_O2_CORNER;t->edge_started=i->now;t->edge_segments=0;kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_IR_EDGE,i->now);}
   else if(kc_elapsed(i->now,t->entered,2000))kc_fail(t,"ERROR_IR_ACQUIRE",i->now);
  }break;}
 case KCT_IR_EDGE:case KCT_O2_BACK_EDGE:{
  if(!t->move_live){t->acquired=0;t->ir_retry=0;t->face_ref=0;t->edge_started=i->now;t->edge_segments=0;kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_O2_BACK_EDGE,i->now);break;}
  if(kc_elapsed(i->now,i->ir_at,300) || fabsf(actual-t->h)>5 || i->motion_status==2){kc_fail(t,"ERROR_EDGE_GUARD",i->now);break;}
  if(kc_elapsed(i->now,t->edge_started,30000)){kc_fail(t,"ERROR_EDGE_TIMEOUT",i->now);break;}
  /* Something close ahead (arena wall, another object): stop, never drive into it. */
  if(i->us_seq!=t->us_seen){
   t->us_seen=i->us_seq;
   if(i->us_valid && !kc_elapsed(i->now,i->us_at,250) && isfinite(i->us) && i->us>2 && i->us<=KC_EDGE_WALL_US)t->wall_hits++;
   else t->wall_hits=0;
   if(t->wall_hits>=2){kc_event(t,"EDGE_WALL",i->us,t->x);kc_fail(t,"ERROR_EDGE_WALL",i->now);break;}
  }
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  if(t->back_fast){
   /* Fast part of the back face: the first reading off the face brakes at
    * once (the end came early); the normal check then runs at face speed. */
   if(isfinite(v) && v>40){
    kc_brake(t);t->back_fast=0;kc_event(t,"BACK_FAST_BRAKE",v,t->x);
    kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_O2_BACK_EDGE,i->now);break;
   }
   if(i->progress>=t->back_fast_until){
    /* Switch to face speed without stopping, ~35 cm before the far end. */
    kc_pose_finish(t,i->progress);t->back_fast=0;kc_event(t,"BACK_FAST_END",t->x,v);
    kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,100,KCT_O2_BACK_EDGE,i->now);break;
   }
  }
  if(!isfinite(v) || v<10){
   t->back_fast=0;
   if(t->ir_retry++){kc_fail(t,"ERROR_IR_CLOSE",i->now);break;}
   t->edge_after=t->phase==KCT_IR_EDGE?KCT_O2_CORNER:KCT_O2_RETURN_TURN;
   float cap=t->amount,d=kc_brake(t);t->target=kc_max(0,cap-d);
   kc_enter(t,KCT_IR_RECOVER,i->now);break;
  }
  if(kc_on_face(t,v)){
   t->acquired=1;t->count=0;if(t->phase==KCT_O2_BACK_EDGE)t->back_gap=v;
   t->face_ref=t->face_ref>0?0.7f*t->face_ref+0.3f*v:v;
  }
  else if(t->acquired && v>=kc_edge_clear_th(t))t->count++;
  else if(v<=40)t->count=0; /* nearer than the clear level: still beside O2 */
  /* Between the face and the clear level (an object further out): keep the count. */
  if(t->count>=3){
   if(t->phase==KCT_IR_EDGE && KC_O2_UTURN_ROLL && t->io.drive && t->move_live && (KC_O2_UTURN&(t->side2==2?1:2))){
    /* Roll straight into the U-turn: pose now, no brake. The brake used to
     * add ~10 cm before o2_end_x was taken; keep the far-end estimate. */
    kc_pose_finish(t,i->progress);
    t->o2_end_x=t->x+(t->x>=0?10.0f:-10.0f);
    t->back_fast=0;kc_uturn_begin(t,i,actual,1);break;
   }
   enum KcPhase next=t->phase==KCT_IR_EDGE?KCT_O2_CORNER:KCT_O2_RETURN_TURN;kc_brake(t);
   if(t->phase==KCT_IR_EDGE)t->o2_end_x=t->x;  /* near end (plus stop overshoot) */
   t->back_fast=0;kc_enter(t,next,i->now);
  }
  else if(i->motion_status==1){
   /* A face longer than one 100 cm move (robot 2026-10-15: O2 ~114 cm wide,
    * back leg starting ~10 cm inside its end): keep following it. */
   /* Only while the face is beside us now (60 cm), or to confirm an end
    * already being seen (10 cm): the arena wall is 50 cm past O2's end. */
   if(t->acquired && t->edge_segments<2 && (kc_on_face(t,v) || t->count>0)){
    t->edge_segments++;kc_brake(t);kc_event(t,"EDGE_CONTINUE",(float)t->edge_segments,v);
    kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,kc_on_face(t,v)?60:10,t->phase,i->now);
   }else kc_fail(t,"ERROR_NO_EDGE",i->now);
  }
  break;}
 case KCT_IR_RECOVER:{
  if(kc_elapsed(i->now,i->ir_at,300) || kc_elapsed(i->now,t->entered,2000)
    || kc_elapsed(i->now,t->edge_started,30000)){kc_fail(t,"ERROR_IR_RECOVERY",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;float v=kc_inner(t,i);
  if(t->acquired && isfinite(v) && v>=kc_edge_clear_th(t)){
   t->samples=0;if(++t->count>=3)kc_enter(t,t->edge_after,i->now);
  }else if(isfinite(v) && v>=10 && v<=30){
   t->count=0;
   if(++t->samples>=2){
    if(t->target<2){kc_fail(t,"ERROR_EDGE_LIMIT",i->now);break;}
    enum KcPhase leg=t->edge_after==KCT_O2_CORNER?KCT_IR_EDGE:KCT_O2_BACK_EDGE;
    t->acquired=1;kc_begin_move(t,KCT_FWD,KC_O2_FACE_SPEED,(unsigned)floorf(t->target),leg,i->now);
   }
  }else{t->count=0;t->samples=0;}
  break;}
 case KCT_O2_CORNER:
  if(t->io.drive && (KC_O2_UTURN&(t->side2==2?1:2))){kc_uturn_begin(t,i,actual,0);break;}
  if(KC_O2_CORNER_180 && t->side2==1){
   /* LEFT route: the two RIGHT 90s at the corner had nothing between them.
    * One RIGHT 180 skips a stop and a restart; the wide right lock leaves
    * ~2.5 cm more back-face gap than the pair. Pose from odometry. */
   t->guard_us=0;t->acquired=0;t->index=0;
   t->turn_ox=i->ox;t->turn_oy=i->oy;t->turn_h0=actual;
   kc_move_to(t,KCT_RIGHT,50,180,KCT_O2_BACK_ACQUIRE,i->now);break;
  }
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
   kc_move_to(t,KCT_FWD,KC_O2_CLEAR_SPEED,10,KCT_O2_BACK_CLEAR,i->now);break;
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
  /* Narrow O2 (robot 2026-10-15: ~25 cm): the U-turn ends ~12 cm further
   * along than the legacy 180 and can stop past the back face's far end.
   * Nothing beside us for 0.5 s: reverse slowly along the lane to find it. */
  if(!t->back_searched && kc_elapsed(i->now,t->entered,500) && t->count==0 && (!isfinite(v) || v>40)){
   t->back_searched=1;kc_event(t,"BACK_SEARCH",v,t->x);
   kc_begin_move(t,KCT_REV,16,45,KCT_O2_BACK_SEARCH,i->now);break;
  }
  if(isfinite(v) && v>=10 && v<=40){
   t->metric[t->count%2]=v;t->count++;
   if(t->count>=2 && fabsf(t->metric[0]-t->metric[1])<=3){
    t->back_gap=(t->metric[0]+t->metric[1])/2;kc_back_face_start(t,i);
   }
  }else t->count=0;
  break;}
 case KCT_O2_BACK_SEARCH:{
  /* Reversing along the back lane: two face readings brake, then the
   * standing check runs again. The whole 45 cm without the face fails. */
  if(kc_elapsed(i->now,i->ir_at,300) || fabsf(actual-t->h)>5 || i->motion_status==2 || kc_elapsed(i->now,t->entered,8000)){
   kc_fail(t,"ERROR_BACK_SEARCH",i->now);break;
  }
  if(i->ir_seq!=t->ir_seen){
   t->ir_seen=i->ir_seq;float v=kc_inner(t,i);
   if(isfinite(v) && v>=10 && v<=40){
    if(++t->count>=2){kc_brake(t);kc_event(t,"BACK_FOUND",v,t->x);kc_enter(t,KCT_O2_BACK_ACQUIRE,i->now);break;}
   }else t->count=0;
  }
  if(i->motion_status==1){kc_brake(t);kc_fail(t,"ERROR_BACK_NOT_ACQUIRED",i->now);}
  break;}
 case KCT_O2_RETURN_TURN:
  kc_move_to(t,t->side2==1?KCT_RIGHT:KCT_LEFT,50,90,KCT_RETURN,i->now);break;
 case KCT_RETURN:{
  if(t->cap==3){kc_done(t,i->now);break;}
  if(!t->return_prepared){
   /* Clear O2 longitudinally before any inward lane correction. */
   float d=t->y-(t->o2_front-18.5f);
   if(!isfinite(d) || d>80){kc_fail(t,"ERROR_RETURN_CLEAR",i->now);break;}
   if(d>=2)kc_move_to(t,KCT_FWD,kc_long_speed(d,22),(unsigned)ceilf(d),KCT_RETURN_CLEAR,i->now);
   else kc_enter(t,KCT_RETURN_CLEAR,i->now);
   break;
  }
  t->return_y=t->y;
  float distance=t->y-(t->o1_front+20);
  if(!isfinite(distance) || distance>150){kc_fail(t,"ERROR_RETURN_GEOMETRY",i->now);break;}
  t->acquired=0;t->gap=NAN;t->count=0;t->index=0;
  if(fabsf(t->x)-10.4f>60.0f){
   /* Lane beyond side-IR reach of O1 (wide O2): the slow O1 search can see
    * nothing. Drive straight to the planned entry start; REAR_CLEAR replans
    * from the odometry pose as before. */
   float dd,ey=t->o1_front+3.0f;
   if(!kce_choose(t->x,t->y,t->y,t->o1_front,t->o2_front,&dd,&ey))
    if(!kce_choose_turn(t->x,t->y,t->y,t->o1_front,t->o2_front,&dd,&ey,90))ey=t->o1_front+3.0f;
   float go=t->y-ey;
   kc_event(t,"O1_POSE_ESTIMATE",t->x,ey);
   if(go>=2)kc_move_to(t,KCT_FWD,kc_long_speed(go,22),(unsigned)lroundf(go),KCT_REAR_CLEAR,i->now);
   else kc_enter(t,KCT_REAR_CLEAR,i->now);
   break;
  }
  kc_begin_move(t,KCT_FWD,distance<=10?12:100,(unsigned)ceilf(kc_max(2,distance)),KCT_O1_EDGE,i->now);break;}
 case KCT_RETURN_CLEAR:{
  if(fabsf(actual-t->h)>KC_ENTRY_HEADING_TOL){kc_fail(t,"ERROR_ENTRY_HEADING",i->now);break;}
  if(fabsf(t->x)>45){
   float trim_target=kce_trim_target(t->x);
   int trimmed=kce_trim_choose(t->x,t->y,t->o1_front,t->o2_front,&t->centre_diagonal,&t->entry_y);
   if(!trimmed){
    /* Full trim does not fit between the obstacles (short O1-O2 gap). A
     * smaller one, to ~40 cm, keeps O1 within side-IR range on the way home. */
    /* Wide O2 (robot 2026-10-15: ~140 cm) leaves the lane ~85 cm out: step
     * the partial target outward until one fits; <=70 still parks directly. */
    for(int k=0;k<4 && !trimmed;k++){
     trim_target=kce_partial_target(t->x)*(1.0f+0.25f*(float)k);
     if(fabsf(trim_target)<fabsf(t->x)-2.0f)
      trimmed=kce_trim_choose_to(t->x,trim_target,t->y,t->o1_front,t->o2_front,&t->centre_diagonal,&t->entry_y);
    }
    if(trimmed)kc_event(t,"RETURN_LANE_PARTIAL",trim_target,t->centre_diagonal);
   }
   if(!trimmed){
    /* No inward trim fits between the obstacles (short O1-O2 gap with a wide
     * O2: run 443289900, lane -52.6). If the normal parking entry already
     * passes the same footprint checks from this lane, keep the lane;
     * REAR_CLEAR rechecks it beside O1 with the IR-corrected pose. */
    float direct_d,direct_y;
    if(kce_choose(t->x,t->y,t->y,t->o1_front,t->o2_front,&direct_d,&direct_y)){
     kc_event(t,"RETURN_LANE_DIRECT",direct_y,direct_d);
     t->return_prepared=1;kc_enter(t,KCT_RETURN,i->now);break;
    }
    /* Very wide O2 (robot 2026-10-15: 140 cm, lane 85-95 out): no 45 deg
     * move fits. Cross with two 90 deg turns: in front of the carpark if
     * that fits (REAR_CLEAR plans it), else here onto a 30-40 cm lane. */
    if(kce_choose_turn(t->x,t->y,t->y,t->o1_front,t->o2_front,&direct_d,&direct_y,90)){
     kc_event(t,"RETURN_LANE_CROSS",direct_y,direct_d);
     t->return_prepared=1;kc_enter(t,KCT_RETURN,i->now);break;
    }
    /* k=3: no straight between the turns, landing 22-30 cm out (lanes ~76-81). */
    for(int k=0;k<4 && !trimmed;k++){
     float lane=k<3?30.0f+5.0f*(float)k:fabsf(t->x)-(t->x>=0?53.0f:51.5f);
     if(lane<22.0f)break;
     trim_target=(t->x>=0?1.0f:-1.0f)*lane;
     trimmed=kce_trim_choose_turn(t->x,trim_target,t->y,t->o1_front,t->o2_front,&t->centre_diagonal,&t->entry_y,90);
    }
    if(!trimmed){kc_fail(t,"ERROR_RETURN_LANE_PATH",i->now);break;}
    t->trim_turn=90;kc_event(t,"RETURN_LANE_CROSS_TRIM",trim_target,t->centre_diagonal);
   }else t->trim_turn=45;
   t->return_target=trim_target;
   kc_event(t,"RETURN_LANE_PLAN",t->entry_y,t->centre_diagonal);
   float d=t->y-t->entry_y;
   if(fabsf(d)>=2)kc_move_to(t,d>0?KCT_FWD:KCT_REV,12,(unsigned)lroundf(fabsf(d)),KCT_RETURN_TRIM_ALIGN,i->now);
   else kc_enter(t,KCT_RETURN_TRIM_ALIGN,i->now);
  }else{t->return_prepared=1;kc_enter(t,KCT_RETURN,i->now);}
  break;}
 case KCT_RETURN_TRIM_ALIGN:
  if(fabsf(actual-t->h)>KC_ENTRY_HEADING_TOL || !kce_template_turn(t->x,t->return_target,t->centre_diagonal,kc_trim_turn(t)) ||
      !kce_clear(t->y,t->o1_front,t->o2_front)){
   kc_fail(t,"ERROR_RETURN_LANE_RECHECK",i->now);break;
  }
  t->centre_right=t->x>=0;t->index=0;kc_enter(t,KCT_RETURN_TRIM,i->now);break;
 case KCT_RETURN_TRIM:{
  unsigned turn=(unsigned)kc_trim_turn(t);
  if(t->index==0){t->index=1;kc_move_to(t,t->centre_right?KCT_RIGHT:KCT_LEFT,50,turn,KCT_RETURN_TRIM,i->now);}
  else if(t->index==1){t->index=2;
   if(t->centre_diagonal>=2)kc_move_to(t,KCT_FWD,kc_long_speed(t->centre_diagonal,22),(unsigned)t->centre_diagonal,KCT_RETURN_TRIM,i->now);}
  else if(t->index==2){t->index=3;kc_move_to(t,t->centre_right?KCT_LEFT:KCT_RIGHT,50,turn,KCT_RETURN_TRIM,i->now);}
  else{
   if(fabsf(t->x-t->return_target)>2.5f){kc_fail(t,"ERROR_RETURN_LANE_ENDPOINT",i->now);break;}
   kc_event(t,"RETURN_LANE_END",t->x,t->y);t->return_prepared=1;kc_enter(t,KCT_RETURN,i->now);
  }
  break;}
 case KCT_O1_EDGE:{
  if(kc_elapsed(i->now,i->ir_at,300) || fabsf(actual-t->h)>5 || i->motion_status==2){kc_fail(t,"ERROR_O1_EDGE_GUARD",i->now);break;}
  if(kc_elapsed(i->now,t->entered,30000)){kc_fail(t,"ERROR_O1_EDGE_TIMEOUT",i->now);break;}
  if(i->ir_seq==t->ir_seen)break;
  t->ir_seen=i->ir_seq;
  float v=kc_inner(t,i);
  /* Only a reading where O1 should be is O1 (robot 2026-10-15: lane 100 cm
   * out, the IR saw something 36 cm away, the pose was reset to 46 cm out
   * and the 45 deg entry drove into the carpark wall). From a lane beyond
   * IR range this rejects everything and the odometry lane is kept. */
  float expect=fabsf(t->x)-10.4f;
  if(isfinite(v) && v>=10 && v<=45 && fabsf(v-expect)>15.0f){
   if(!t->index_ir_reject){t->index_ir_reject=1;kc_event(t,"O1_IR_IGNORED",v,expect);}
   v=NAN;
  }
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
  /* Do NOT blindly advance until fully past O1. Find where the whole entry
   * sweep fits first. A wide return lane needs an earlier start, not a longer
   * diagonal from the old fixed rear-clear pose. Reverse only on the lane
   * already traversed; no unmeasured lateral reverse or new turn type. */
  if(fabsf(actual-t->h)>KC_ENTRY_HEADING_TOL){kc_fail(t,"ERROR_ENTRY_NO_PATH",i->now);break;}
  t->centre_turn=45;
  if(!kce_choose(t->x,t->y,t->return_y,t->o1_front,t->o2_front,&t->centre_diagonal,&t->entry_y)){
   t->centre_turn=90;
   if(!kce_choose_turn(t->x,t->y,t->return_y,t->o1_front,t->o2_front,&t->centre_diagonal,&t->entry_y,90)){
    kc_fail(t,"ERROR_ENTRY_NO_PATH",i->now);break;
   }
  }
  kc_event(t,"ENTRY_PLAN",t->entry_y,t->centre_diagonal);
  if(t->centre_turn>60)kc_event(t,"ENTRY_TURN",t->centre_turn,t->x);
  float d=t->y-t->entry_y;t->index=0;t->guard_us=0;
  if(fabsf(d)>=2){
   unsigned amount=(unsigned)lroundf(fabsf(d));
   kc_move_to(t,d>0?KCT_FWD:KCT_REV,d>0?kc_long_speed((float)amount,12):12,amount,KCT_ENTRY_ALIGN,i->now);
  }else kc_enter(t,KCT_ENTRY_ALIGN,i->now);
  break;}
 case KCT_ENTRY_ALIGN:
  /* Recheck at ACTUAL encoder endpoint, including completed braking. */
  if(fabsf(actual-t->h)>KC_ENTRY_HEADING_TOL || !kce_template_turn(t->x,0,t->centre_diagonal,kc_centre_turn(t)) ||
      !kce_clear(t->y,t->o1_front,t->o2_front)){
   kc_fail(t,"ERROR_ENTRY_RECHECK",i->now);break;
  }
  t->centre_right=t->x>=0;t->index=0;kc_enter(t,KCT_CENTRE,i->now);break;
 case KCT_CENTRE:{
  unsigned turn=(unsigned)kc_centre_turn(t);
  if(t->index==0){
   if(!kce_template_turn(t->x,0,t->centre_diagonal,(float)turn) || !kce_clear(t->y,t->o1_front,t->o2_front)){
    kc_fail(t,"ERROR_ENTRY_RECHECK",i->now);break;
   }
   t->index=1;kc_move_to(t,t->centre_right?KCT_RIGHT:KCT_LEFT,50,turn,KCT_CENTRE,i->now);
  }else if(t->index==1){t->index=2;
   if(t->centre_diagonal>=2)kc_move_to(t,KCT_FWD,kc_long_speed(t->centre_diagonal,22),(unsigned)t->centre_diagonal,KCT_CENTRE,i->now);
  }else if(t->index==2){t->index=3;kc_move_to(t,t->centre_right?KCT_LEFT:KCT_RIGHT,50,turn,KCT_CENTRE,i->now);
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
   if((walls && t->count>=2) || !KC_PARK_REQUIRE_WALLS){
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
   /* Without the wall check, the short retreat runs back along our own entry path. */
   if(!KC_PARK_REQUIRE_WALLS){corridor=1;t->park_pairs=2;}
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
  kc_move_to(t,KCT_FWD,kc_long_speed((float)d,t->range>50?22:12),d,KCT_PARK,i->now);break;}
 default:kc_fail(t,"ERROR_STATE",i->now);break;
 }
}
#endif
