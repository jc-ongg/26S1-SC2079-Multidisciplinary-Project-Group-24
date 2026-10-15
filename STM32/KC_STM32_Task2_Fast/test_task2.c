#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "Core/Inc/kc_task2.h"
#include "Core/Inc/kc_forward_profile.h"
#include "Core/Inc/kc_stop_settle.h"
static unsigned starts,stops;static float brake_distance;
static float simulated_o2_front=151.5f;
static int commands[128];static unsigned amounts[128];
static void start(int m,unsigned speed,unsigned n){assert(speed>0&&speed<=100);if(m!=KCT_FWD)assert(speed<=50);assert(n>0);commands[starts]=m;amounts[starts++]=n;brake_distance=0;}
/* Robot model for continuous drive (U-turn): rear-axle kinematics, servo
 * swing in time, first-order speed. Scales: true lock radius, servo lag,
 * top speed at the swerve PWM, unseen brake skid. */
static float um_rscale=1,um_lag=1,um_vmax=51,um_skid=1;
static int um_mode=-1;static float um_off;static unsigned um_pwm;
static float um_x,um_y,um_h,um_k,um_v,um_dist,um_goal=NAN,um_tx,um_ty;
static unsigned um_drives;
static float um_x0,um_y0,um_h0,um_reach,um_outer=1; /* outer side: +1 right (LEFT U-turn) */
static float um_ir_back=20;  /* side IR once straight after the U-turn (80: narrow O2, nothing there) */
static void drive(int m,float off,unsigned pwm){um_mode=m;um_off=off;um_pwm=pwm;um_drives++;}
static void goal(float h){um_goal=h;}
static float stop(void){
 stops++;
 if(um_v>0){float sk=um_skid*KCF_RF_SKID*(um_v/57)*(um_v/57);um_tx-=sk*sinf(um_h);um_ty+=sk*cosf(um_h);um_v=0;}
 um_mode=-1;return brake_distance;
}
static void um_step(float dt){
 float rl=KCF_R_LEFT*um_rscale,rr=KCF_R_RIGHT*um_rscale,k=0;
 if(um_mode==KCF_DRIVE_LEFT || um_mode==KCF_DRIVE_BRAKE_LEFT)k=1/rl;
 else if(um_mode==KCF_DRIVE_RIGHT || um_mode==KCF_DRIVE_BRAKE_RIGHT)k=-1/rr;
 else if(um_mode==KCF_DRIVE_STRAIGHT)k=um_off<0?(-um_off/48.4f)/rl:-(um_off/59.6f)/rr;
 float full=1/KCF_R_LEFT+1/KCF_R_RIGHT,step=full/((k>um_k?KCF_T_RL:KCF_T_LR)*um_lag)*dt,dk=k-um_k;
 um_k+=dk>step?step:dk<-step?-step:dk;
 float vt=(um_mode>=0 && um_mode<KCF_DRIVE_BRAKE)?um_vmax*(float)um_pwm/(float)KCF_PWM_TURN:0;
 um_v+=(vt-um_v)*(dt/0.12f>1?1:dt/0.12f);
 float ds=um_v*dt,hm=um_h+0.5f*um_k*ds;um_h+=um_k*ds;
 um_x-=ds*sinf(hm);um_y+=ds*cosf(hm);um_tx-=ds*sinf(hm);um_ty+=ds*cosf(hm);um_dist+=ds;
 /* Front corner on the outside of the turn (19.7 ahead of the axle, 9.5
  * to the side): how far past the corner start it reaches, along the face. */
 float cx=um_tx-19.7f*sinf(um_h)+9.5f*um_outer*cosf(um_h),cy=um_ty+19.7f*cosf(um_h)+9.5f*um_outer*sinf(um_h);
 float reach=(cx-um_x0)*(-sinf(um_h0))+(cy-um_y0)*cosf(um_h0);
 if(reach>um_reach)um_reach=reach;
}
/* Run the U-turn phase in 5 ms steps; returns ms taken. */
static unsigned um_run(KcTask2 *t,KcT2Input *i){
 um_x=i->ox;um_y=i->oy;um_tx=um_x;um_ty=um_y;um_h=kc_rad(i->heading-t->origin_heading);um_k=0;um_dist=i->odist;
 /* Rolling in (umode 1 at once): still at face speed, steering centred. */
 um_v=t->umode==1?um_vmax*(float)(KC_O2_FACE_SPEED*71)/(float)KCF_PWM_TURN:0;um_mode=KCF_DRIVE_STRAIGHT;um_off=0;
 um_x0=um_x;um_y0=um_y;um_h0=um_h;um_reach=-1e9f;
 unsigned ms=0;
 for(;ms<8000 && t->active && t->phase==KCT_O2_UTURN;ms+=5){
  um_step(0.005f);
  i->now+=5;i->us_at=i->ir_at=i->imu_at=i->now;t->heartbeat=i->now;
  if(ms%50==0){i->ir_seq++;float ir=t->umode>=4?um_ir_back:80;i->left_ir=i->right_ir=ir;}
  i->ox=um_x;i->oy=um_y;i->odist=um_dist;i->speed=um_v;i->gyro=um_k*um_v*57.29578f;
  i->heading=t->origin_heading+um_h*57.29578f;
  kc_step(t,i);
 }
 return ms;
}
static unsigned wide_events,pose_estimates;
static int trace_events=-1;
static void event(const char *s,float a,float b){
 if(trace_events<0)trace_events=getenv("KC_TRACE")!=NULL;
 if(trace_events && strcmp(s,"STATE"))fprintf(stderr,"   %s %.2f %.2f\n",s,a,b);
 /* Wide-lane skip of the O1 search only (lane more than 60 cm out); the
  * same event also marks a lane already beside O1 with no IR fix. */
 if(!strcmp(s,"O1_POSE_ESTIMATE") && fabsf(a)-10.4f>60)pose_estimates++;
 if(!strncmp(s,"ERROR_",6))fprintf(stderr,"%s %.2f %.2f\n",s,a,b);
 if(!strncmp(s,"RETURN_LANE_CROSS",17) || !strcmp(s,"ENTRY_TURN"))wide_events++;
}
static KcT2Hooks hooks={start,stop,event,drive,goal};
static KcT2Input input(void){KcT2Input i={0};i.now=1000;i.imu_at=1000;i.us_at=1000;i.ir_at=1000;i.gyro_ready=1;i.us_valid=1;i.us=60;i.left_ir=80;i.right_ir=80;return i;}
static void advance(KcTask2 *t,KcT2Input *i){i->now+=100;i->us_at=i->ir_at=i->imu_at=i->now;i->us_seq++;i->ir_seq++;t->heartbeat=i->now;kc_step(t,i);}
static void reset(void){starts=stops=0;brake_distance=0;}
static float sim_front_edge=6,edge_run,sim_near;
static unsigned turns180,uturns;
static void simulate(unsigned a1,unsigned a2,unsigned cap){
 reset();KcTask2 t;KcT2Input i=input();assert(kc_start(&t,hooks,&i,77,cap));sim_near=NAN;
 unsigned loops=0,previous_starts=0;enum KcPhase last=KCT_IDLE;
 for(;t.active&&loops<4000;loops++){
  if(t.phase==KCT_O2_UTURN){
   uturns++;um_run(&t,&i);
   /* Hand back an exact heading: the coarse loop below sets i.heading=t.h. */
   continue;
  }
  if(t.waiting_arrow){assert(!kc_arrow(&t,76,t.waiting_arrow,1));assert(kc_arrow(&t,77,t.waiting_arrow,t.waiting_arrow==1?a1:a2));}
  if(starts!=previous_starts){i.progress=0;previous_starts=starts;}
  i.motion_status=0;
  if(t.phase!=last){i.progress=0;last=t.phase;}
  if(t.phase==KCT_MOVE){
   i.progress=t.amount;brake_distance=i.progress;i.motion_status=1;
   i.heading=t.move_heading+(t.move==KCT_LEFT?t.amount:(t.move==KCT_RIGHT||t.move==KCT_REV_LEFT)?-t.amount:0);
   if((t.move==KCT_LEFT||t.move==KCT_RIGHT)&&t.amount>=180){
    /* Rear-axle odometry for a continuous 180, giving the same end pose as
     * two calibrated 90s (RIGHT: 17.5 back, 58.5 right; LEFT: 16 back, 46 left). */
    float h0=kc_rad(t.move_heading),h1=kc_rad(i.heading);
    float f=t.move==KCT_RIGHT?-17.5f:-16.0f,sd=t.move==KCT_RIGHT?58.5f:-46.0f;
    float dxc=-f*sinf(h0)+sd*cosf(h0),dyc=f*cosf(h0)+sd*sinf(h0);
    i.ox=t.turn_ox+dxc+8.2f*(sinf(h1)-sinf(h0));i.oy=t.turn_oy+dyc-8.2f*(cosf(h1)-cosf(h0));
    turns180++;
   }
  }else i.heading=t.h;
  if(t.phase==KCT_O1_RANGE)i.us=71.5f-t.y-11.5f;
  else if(t.phase==KCT_O2_RANGE||t.phase==KCT_O2_TARGET)i.us=simulated_o2_front-t.y-11.5f;
  else if(t.phase==KCT_PARK)i.us=t.y+13.5f; /* rear wall Y=-25; forward lens 11.5 */
  else i.us=40;
  float ir=20;
  if(t.phase==KCT_IR_EDGE || t.phase==KCT_O2_BACK_EDGE){
   /* Front face: its length left from where following starts (wide O2:
    * whole 100 cm moves run out and EDGE_CONTINUE takes over). O2 is
    * centred on the O1-O2 line, so the back face ends at the near end's
    * mirror image, wherever the back-face move started. */
   if(t.move_live){i.progress+=1;brake_distance=i.progress;if(i.progress>=t.amount)i.motion_status=1;}
   float live_x=t.start_x-(t.move_live?i.progress:0)*sinf(kc_rad(t.move_heading));
   if(t.phase==KCT_IR_EDGE){
    edge_run+=t.move_live?1:0;
    ir=edge_run>=sim_front_edge?80:20;
    if(ir>40 && isnan(sim_near))sim_near=live_x;
   }else ir=(sim_near>0?live_x<=-sim_near:live_x>=-sim_near)?80:20;
  }else if(t.phase==KCT_O1_EDGE){
   edge_run=0;
   i.progress+=1;brake_distance=i.progress;
   i.motion_status=i.progress>=t.amount?1:0;
   /* Side IR reaches O1 only from a normal lane. From a wide-O2 lane it can
    * still see something else ~36 cm away (robot 2026-10-15), never O1. */
   ir=i.progress>=4 && i.progress<14?(fabsf(t.x)<50?20:36):80;
  }else edge_run=0;
  i.left_ir=i.right_ir=24.6f;
  if(t.phase!=KCT_PARK){if(a2==1)i.right_ir=ir;else i.left_ir=ir;}
  advance(&t,&i);
 }
 if(t.phase!=KCT_DONE)fprintf(stderr,"simulation %u/%u cap%u phase%d x%.2f y%.2f loops%u\n",a1,a2,cap,t.phase,t.x,t.y,loops);
 assert(t.phase==KCT_DONE);assert(starts<128);
 if(cap==1){assert(starts==8);assert(fabsf(t.x)<0.01f);assert(fabsf(t.h)<0.01f);}
}
int main(void){
 simulate(1,1,1);simulate(2,1,1);
 /* O2 LEFT: one continuous RIGHT U-turn (else the legacy RIGHT 180). */
 turns180=0;uturns=0;simulate(1,1,0);
 assert(uturns==((KC_O2_UTURN&2)?1u:0u) && turns180==((KC_O2_UTURN&2)?0u:KC_O2_CORNER_180?1u:0u));
 /* O2 RIGHT: one continuous LEFT U-turn (else 90, 10 cm, 90). */
 turns180=0;uturns=0;simulate(1,2,0);assert(turns180==0);
 assert(uturns==((KC_O2_UTURN&1)?1u:0u));
 simulate(2,1,0);simulate(2,2,0);
 simulate(1,1,2);simulate(1,2,3);
 /* All arrows at the rules minimum60cm and maximum150cm obstacle gap. */
 for(unsigned spacing=0;spacing<2;spacing++){
  simulated_o2_front=71.5f+10+(spacing?150:60);
  pose_estimates=0;
  for(unsigned a=1;a<=2;a++)for(unsigned b=1;b<=2;b++)simulate(a,b,0);
  assert(pose_estimates==0); /* normal lanes never skip the O1 search */
 }
 /* Very wide O2 (robot 2026-10-15: ~140 cm): front edge ~37 cm along the
  * face, back face ~135 cm; the return lane ends 80-95 cm out and needs the
  * 90 deg crossing. All arrows, short and long O1-O2 gaps. */
 for(unsigned spacing=0;spacing<3;spacing++){
  simulated_o2_front=71.5f+10+(spacing==0?60:spacing==1?81:150);
  sim_front_edge=37;
  unsigned crossings=0;pose_estimates=0;
  for(unsigned a=1;a<=2;a++)for(unsigned b=1;b<=2;b++){wide_events=0;simulate(a,b,0);crossings+=wide_events>0;}
  fprintf(stderr,"wide O2, gap %.0f: 4/4 parked, %u used the 90 deg crossing, %u skipped the O1 search\n",simulated_o2_front-81.5f,crossings,pose_estimates);
  assert(spacing==2 || crossings==4); /* a 150 cm gap has room for the 45 deg trim */
  /* Lanes still 70+ cm out at the return skip the side-IR O1 search; a
   * crossing or trim that brings the lane in first keeps it. Which courses
   * do which depends on where the back face ends (O2 centred). */
  assert(pose_estimates<=4u);
 }
 sim_front_edge=6;
 simulated_o2_front=151.5f;
#if KC_O2_UTURN
 /* O2 U-turns: lock radius +-8%, servo lag +-25%, swerve speed 45-58 cm/s,
  * brake skid 0.5-1.5x, heading at the corner +-2 deg. RIGHT route (LEFT
  * U-turn) must land 56 cm over (back IR ~17.7, as 90+10+90); LEFT route
  * (RIGHT U-turn) at least the legacy 180's 57 and at most 72 (back IR
  * <= ~32 of the 40 the face check allows). On 180, pose known.
  * Starts: 0 standing (after a recovery), 1 rolling in from the face at
  * face speed, 2 rolling in with nothing beside the robot afterwards
  * (narrow O2): that one must brake for the standing check and search. */
 for(int route=1;route<=2;route++)for(int kind=0;kind<3;kind++){
  if(!(KC_O2_UTURN&(route==2?1:2)))continue;
  if(kind && !KC_O2_UTURN_ROLL)continue;
  float rs[]={0.92f,1,1.08f},lg[]={0.75f,1,1.25f},vm[]={45,51,58},sk[]={0.5f,1.5f},he[]={-2,0,2};
  float wmin=1e9f,wmax=-1e9f,hmax=0,emax=0,rmin=1e9f,rmax=-1e9f,smax=0,stmax=0;unsigned tmax=0,tsum=0,n=0,braked=0;
  float h0=route==2?-90.0f:90.0f,h1=-h0,x0=route==2?60.0f:-60.0f;
  um_outer=route==2?1.0f:-1.0f;um_ir_back=kind==2?80.0f:20.0f;
  for(int a=0;a<3;a++)for(int b=0;b<3;b++)for(int c=0;c<3;c++)for(int d=0;d<2;d++)for(int e=0;e<3;e++){
   um_rscale=rs[a];um_lag=lg[b];um_vmax=vm[c];um_skid=sk[d];
   reset();KcTask2 t;KcT2Input i=input();assert(kc_start(&t,hooks,&i,1,0));
   t.side2=(uint8_t)route;t.h=h0;t.x=x0;t.y=140;i.heading=h0+he[e];i.ox=10;i.oy=20;um_goal=NAN;
   if(kind){advance(&t,&i);kc_uturn_begin(&t,&i,h0+he[e],1);}
   else{t.phase=KCT_O2_CORNER;advance(&t,&i);}
   assert(t.phase==KCT_O2_UTURN);
   unsigned ms=um_run(&t,&i);
   enum KcPhase want=(KC_O2_UTURN_ROLL && kind!=2)?KCT_O2_BACK_EDGE:KCT_O2_BACK_ACQUIRE;
   /* A slow servo at top speed may not hold 180 within 3 deg inside 30 cm:
    * it then brakes and uses the standing check (counted, not an error). */
   if(want==KCT_O2_BACK_EDGE && t.phase==KCT_O2_BACK_ACQUIRE && um_dist-t.ud0>=KCU_END_MAX){braked++;want=KCT_O2_BACK_ACQUIRE;}
   if(t.phase!=want)fprintf(stderr,"uturn route %d kind %d r%.2f lag%.2f v%.0f skid%.1f h%+.0f: phase %d\n",route,kind,rs[a],lg[b],vm[c],sk[d],he[e],t.phase);
   assert(t.phase==want);
   if(want==KCT_O2_BACK_EDGE)assert(t.move_live && commands[starts-1]==KCT_FWD && t.acquired && fabsf(t.back_gap-20)<0.01f);
   assert(isfinite(um_goal) && fabsf(um_goal-h1)<0.01f && fabsf(t.h-h1)<0.01f);
   /* Both routes move toward +Y (O2's back) in this frame. */
   float w=um_ty-20,hd=um_h*57.29578f-h1;
   float hs=kc_rad(h0+he[e]);
   float cx=x0+(um_tx-10)-8.2f*(sinf(um_h)-sinf(hs)),cy=140+(um_ty-20)+8.2f*(cosf(um_h)-cosf(hs));
   float pe=hypotf(t.x-cx,t.y-cy);
   /* Along the back face, from the corner start (+ = toward the far end). */
   float along=(route==2?-1.0f:1.0f)*(cx-x0);
   if(w<wmin)wmin=w;if(w>wmax)wmax=w;if(fabsf(hd)>hmax)hmax=fabsf(hd);if(pe>emax)emax=pe;
   if(um_reach<rmin)rmin=um_reach;if(um_reach>rmax)rmax=um_reach;if(t.ustraight>smax)smax=t.ustraight;
   if(along>stmax)stmax=along;
   if(ms>tmax)tmax=ms;tsum+=ms;n++;
   if(getenv("UTURN_VERBOSE"))
    fprintf(stderr,"  route %d kind %d r%.2f lag%.2f v%.0f skid%.1f h%+.0f: shift %.1f head %+.1f pose err %.1f straight %.1f along %.1f reach %.1f ms %u\n",
     route,kind,rs[a],lg[b],vm[c],sk[d],he[e],w,hd,pe,t.ustraight,along,um_reach,ms);
  }
  fprintf(stderr,"O2 %s route U-turn (%s lock, %s), %u cases: shift %.1f..%.1f cm (target %.0f), heading within %.1f deg, pose error <= %.1f cm, "
   "straight <= %.1f cm, outer corner reach %.1f..%.1f cm, ends <= %.1f cm along the back face, %.2f s mean, %.2f s worst, %u braked instead of rolling out\n",
   route==2?"RIGHT":"LEFT",route==2?"left":"right",kind==0?"standing start":kind==1?"rolling":"rolling, narrow O2",
   n,wmin,wmax,route==2?KCU_SHIFT:KCU_SHIFT_R,hmax,emax,smax,rmin,rmax,stmax,tsum/1000.0f/n,tmax/1000.0f,kind==2?0u:braked);
  assert(braked*10<=n); /* rolling out is the normal case */
  /* Pose error is the +-50% brake skid, along the back face; a slow servo
   * can end the 180 hold at its 30 cm cap ~3 deg off (the back face corrects). */
  if(route==2)assert(wmin>=53 && wmax<=62);else assert(wmin>=55 && wmax<=72);
  assert(hmax<=3.5f && emax<=3.0f && tmax<=3500);
  um_rscale=um_lag=um_skid=1;um_vmax=51;um_outer=1;um_ir_back=20;
 }
#endif
 reset();KcTask2 t;KcT2Input i=input();i.gyro=24;assert(!kc_start(&t,hooks,&i,1,1));assert(starts==0);
 i=input();assert(kc_start(&t,hooks,&i,1,1));i.now+=2100;i.imu_at=i.now;kc_step(&t,&i);assert(t.phase==KCT_ERROR&&starts==0);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 t.expected_range=30;t.phase=KCT_O2_TARGET;t.side2=2;t.range=30;
 for(int k=0;k<40&&t.active;k++){i.us=63;advance(&t,&i);}
 assert(t.phase==KCT_ERROR&&starts==0); /* stable wrong echo cannot authorize travel */
 /* Robot 2026-10-15: after the approach move the US read a steady 33-34 cm
  * against 28.46 expected; every echo was rejected and the run failed. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_O2_RANGE;t.expected_range=28.46f;t.y=158;t.entered=i.now;
 for(int k=0;k<40&&t.active&&starts==0;k++){i.us=33.4f+(k%3)*0.3f;advance(&t,&i);}
 assert(t.active&&starts==1&&commands[0]==KCT_FWD&&amounts[0]==5);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.phase=KCT_O2_TARGET;t.side2=2;t.expected_range=9.3f;
 for(int k=0;k<3;k++){i.us=9.3f;advance(&t,&i);}
 assert(commands[0]==KCT_REV&&amounts[0]<=19); /* minimum gap rear budget */
 i.motion_status=2;advance(&t,&i);assert(t.phase==KCT_ERROR);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.side2=1;t.phase=KCT_O2_BACK_EDGE;t.h=0;
 advance(&t,&i); /* start search */
 for(int k=0;k<5;k++){i.right_ir=k<2?20:45;i.progress=k+1;brake_distance=i.progress;advance(&t,&i);}
 assert(t.phase==KCT_O2_BACK_EDGE); /* 45 is not clear */
 for(int k=0;k<3;k++){i.right_ir=80;i.progress++;brake_distance=i.progress;advance(&t,&i);}
 assert(t.phase==KCT_O2_RETURN_TURN);assert(fabsf(t.y-i.progress)<0.01f); /* actual interrupted encoder distance retained */
 kc_fail(&t,"ERROR_ABORTED",i.now);assert(!t.active);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 kc_move_to(&t,KCT_FWD,50,7,KCT_O1_PLAN,i.now);
 i.progress=-0.02f;advance(&t,&i);assert(t.active&&t.phase==KCT_MOVE);
 i.progress=7;i.motion_status=1;advance(&t,&i);assert(t.active&&fabsf(t.y-7)<0.001f);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 kc_move_to(&t,KCT_FWD,50,7,KCT_O1_PLAN,i.now);
 i.progress=-0.3f;advance(&t,&i);assert(!t.active&&t.phase==KCT_ERROR);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 t.y=26.36f;t.us_anchor=58.31f;t.expected_range=31.95f;
 for(int k=0;k<3;k++){i.us=29.8f;advance(&t,&i);}
 assert(t.active&&commands[0]==KCT_REV&&amounts[0]==2);
 i.progress=2.6f;i.motion_status=1;advance(&t,&i);
 i.motion_status=0;
 for(int k=0;k<3;k++){i.us=32.4f;advance(&t,&i);}
 assert(t.active&&t.phase==KCT_O1_VISION);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 for(int k=0;k<3;k++){i.us=29.8f;advance(&t,&i);}
 assert(!t.active&&starts==0); /* unknown rear space at initial pose */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 kc_move_to(&t,KCT_RIGHT,50,90,KCT_O2_BACK_ACQUIRE,i.now);
 i.progress=-4;advance(&t,&i);assert(t.active&&t.phase==KCT_MOVE);
 i.heading=-90;i.motion_status=1;advance(&t,&i);
 assert(t.active&&t.phase==KCT_O2_BACK_ACQUIRE&&fabsf(t.h+90)<0.001f);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 kc_move_to(&t,KCT_LEFT,50,90,KCT_O2_BACK_ACQUIRE,i.now);
 i.progress=-4;i.heading=60;i.motion_status=1;advance(&t,&i);
 assert(!t.active&&t.phase==KCT_ERROR); /* direction-check relaxation cannot bypass gyro validation */
 /* Early arrow is cached, but cannot skip an unfinished range command. */
 reset();i=input();assert(kc_start(&t,hooks,&i,777,1));
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_MOVE&&t.waiting_arrow==1);
 assert(!kc_arrow(&t,777,2,2));assert(kc_arrow(&t,777,1,2));
 advance(&t,&i);assert(t.phase==KCT_MOVE&&t.arrow==2);
 i.progress=t.amount;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 for(int k=0;k<3;k++){i.us=32;advance(&t,&i);}
 assert(t.phase==KCT_O1_VISION&&t.arrow==2);
 advance(&t,&i);assert(t.phase==KCT_O1_PLAN&&t.side1==2);
 /* O2 uses its own cache; it cannot reuse the earlier O1 arrow. */
 t.phase=KCT_O2_RANGE;t.waiting_arrow=0;t.arrow=2;t.expected_range=NAN;t.o2_front=NAN;
 for(int k=0;k<3;k++){i.us=40;advance(&t,&i);}
 assert(t.waiting_arrow==2&&t.arrow==0&&t.phase==KCT_MOVE);
 assert(!kc_arrow(&t,777,1,1));assert(kc_arrow(&t,777,2,1));
 i.progress=t.amount;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 for(int k=0;k<3;k++){i.us=26;advance(&t,&i);}
 assert(t.phase==KCT_O2_VISION&&t.arrow==1);
 /* Ranged for LEFT (26 cm) with the arrow already known and no move since:
  * the repeat range check is skipped. */
 advance(&t,&i);assert(t.side2==1&&t.phase==(t.cap==2?KCT_O2_TARGET:KCT_O2_FIRST));
 /* Narrow O2 (robot 2026-10-15, ~25 cm wide, LEFT route): the U-turn
  * stopped at x 12.5, past the back face's far end, and the side IR saw
  * nothing. Reverse along the back lane until the face shows, then go on. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_O2_BACK_ACQUIRE;t.entered=i.now;t.side2=1;t.h=-90;t.x=12.54f;t.y=178.08f;t.o2_end_x=-20.37f;i.heading=-90;
 i.left_ir=24.6f;i.right_ir=80;
 for(int k=0;k<10&&t.phase==KCT_O2_BACK_ACQUIRE;k++)advance(&t,&i);
 assert(t.phase==KCT_O2_BACK_SEARCH&&t.back_searched&&starts==1&&commands[0]==KCT_REV&&amounts[0]==45);
 i.progress=4;advance(&t,&i);assert(t.phase==KCT_O2_BACK_SEARCH);
 i.right_ir=21;i.progress=8;brake_distance=8;advance(&t,&i);assert(t.phase==KCT_O2_BACK_SEARCH);
 i.progress=10;brake_distance=10;advance(&t,&i);
 assert(t.phase==KCT_O2_BACK_ACQUIRE && fabsf(t.x-2.54f)<0.01f);
 i.progress=0;advance(&t,&i);advance(&t,&i);
 assert(t.phase==KCT_O2_BACK_EDGE && starts==2 && commands[1]==KCT_FWD && t.acquired);
 /* The whole 45 cm without the face: error, no second search. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_O2_BACK_ACQUIRE;t.entered=i.now;t.side2=1;t.h=-90;t.x=12.54f;t.y=178.08f;t.o2_end_x=-20.37f;i.heading=-90;
 i.left_ir=24.6f;i.right_ir=80;
 for(int k=0;k<10&&t.phase==KCT_O2_BACK_ACQUIRE;k++)advance(&t,&i);
 assert(t.phase==KCT_O2_BACK_SEARCH);
 i.progress=45;i.motion_status=1;advance(&t,&i);
 assert(t.phase==KCT_ERROR && starts==1);
 i.motion_status=0;
 /* LEFT O2 band is 24.5-28.5: a run stopping at 27.5 needs no correction,
  * 24.0 is still corrected (IR would be too close after the turn). */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_O2_TARGET;t.side2=1;t.expected_range=28.3f;
 for(int k=0;k<3;k++){i.us=28.3f;advance(&t,&i);}
 assert(starts==0&&t.phase==KCT_O2_FIRST);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_O2_TARGET;t.side2=1;t.expected_range=24.0f;
 for(int k=0;k<3;k++){i.us=24.0f;advance(&t,&i);}
 assert(starts==1&&commands[0]==KCT_REV);
 /* RIGHT back-clearance advance shifts the next turn outward, not sideways. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,3));
 t.phase=KCT_O2_BACK_TURN;t.side2=2;t.x=49.12f;t.y=157.26f;t.h=0;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.amount==10&&t.speed==KC_O2_CLEAR_SPEED&&!t.guard_us);
 i.progress=10.3f;i.motion_status=1;advance(&t,&i);
 assert(fabsf(t.x-49.12f)<0.01f&&fabsf(t.y-167.56f)<0.01f);
 i.motion_status=0;advance(&t,&i);assert(t.move==KCT_LEFT&&t.amount==90);
 i.heading=90;i.motion_status=1;i.progress=0;advance(&t,&i);
 assert(t.phase==KCT_O2_BACK_ACQUIRE);
 i.motion_status=0;
 for(int k=0;k<2;k++){i.left_ir=17.7f;advance(&t,&i);}
 assert(t.phase==KCT_O2_BACK_EDGE&&t.acquired);
 /* The measured close gap remains rejected, even with the route change. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,3));t.phase=KCT_O2_BACK_ACQUIRE;t.side2=2;
 for(int k=0;k<22&&t.active;k++){i.left_ir=7.7f;advance(&t,&i);}
 assert(!t.active&&starts==0);
 /* LEFT route still goes directly into its previously tested right turn. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,3));t.phase=KCT_O2_BACK_TURN;t.side2=1;
 advance(&t,&i);assert(t.move==KCT_RIGHT&&t.amount==90);
 /* Wide O2 (robot 2026-10-15, ~114 cm): the back leg used up its 100 cm move
  * with the face still beside it. Keep following, at most twice more. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=1;t.acquired=1;t.edge_segments=0;
 kc_begin_move(&t,KCT_FWD,16,100,KCT_O2_BACK_EDGE,i.now);t.edge_started=i.now;assert(starts==1);
 i.right_ir=20;i.motion_status=1;advance(&t,&i);
 assert(t.active&&t.phase==KCT_O2_BACK_EDGE&&t.move_live&&starts==2&&t.amount==60);
 advance(&t,&i);assert(t.active&&starts==3);
 advance(&t,&i);assert(!t.active&&t.phase==KCT_ERROR&&starts==3);
 i.motion_status=0;
 /* Fast back face (O2 centred): near end found at x=-86.76; back leg from
  * x=-65.25 runs at KC_O2_BACK_FAST for 107 cm, then switches to face speed
  * without braking; an early end during the fast part brakes at once. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=1;t.h=-90;i.heading=-90;
 t.phase=KCT_O2_BACK_ACQUIRE;t.entered=i.now;t.x=-65.25f;t.y=222;t.o2_end_x=-86.76f;
 i.right_ir=17;for(int k=0;k<3&&starts==0;k++)advance(&t,&i);
 assert(starts==1&&commands[0]==KCT_FWD&&t.speed==KC_O2_BACK_FAST&&t.back_fast&&fabsf(t.back_fast_until-107.0f)<0.5f);
 i.progress=60;advance(&t,&i);assert(starts==1&&t.back_fast);
 {unsigned st=stops;i.progress=107.5f;advance(&t,&i);
  assert(starts==2&&stops==st&&!t.back_fast&&t.speed==KC_O2_FACE_SPEED&&t.phase==KCT_O2_BACK_EDGE&&fabsf(t.x-(-65.25f+107.5f))<0.5f);}
 i.progress=0;
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=1;t.h=-90;i.heading=-90;
 t.phase=KCT_O2_BACK_ACQUIRE;t.entered=i.now;t.x=-65.25f;t.y=222;t.o2_end_x=-86.76f;
 i.right_ir=17;for(int k=0;k<3&&starts==0;k++)advance(&t,&i);
 i.progress=30;i.right_ir=80;advance(&t,&i);  /* end much earlier than predicted */
 assert(stops>=1&&starts==2&&!t.back_fast&&t.speed==KC_O2_FACE_SPEED);
 i.progress=0;
 /* A narrow O2 (near end 30 cm out) gets no fast part. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=2;t.h=90;i.heading=90;
 t.phase=KCT_O2_BACK_ACQUIRE;t.entered=i.now;t.x=14.16f;t.y=223;t.o2_end_x=30.16f;
 i.left_ir=17;for(int k=0;k<3&&starts==0;k++)advance(&t,&i);
 assert(starts==1&&t.speed==KC_O2_FACE_SPEED&&!t.back_fast);
 /* Move ends with nothing beside the robot (end missed): never drive on. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=1;t.acquired=1;t.edge_segments=0;
 kc_begin_move(&t,KCT_FWD,16,100,KCT_O2_BACK_EDGE,i.now);t.edge_started=i.now;
 i.right_ir=50;i.motion_status=1;advance(&t,&i);
 assert(!t.active&&t.phase==KCT_ERROR&&starts==1);
 i.motion_status=0;
 /* Reported physical parking: valid front gap, asymmetric side readings,
  * sum 57.7 cm. Fresh stable side samples must confirm after settling. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us=12.61f;i.left_ir=39.4f;i.right_ir=18.3f;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(!t.active&&t.phase==KCT_DONE&&starts==0);
 /* Missing side wall cannot be accepted or cause more forward travel. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us=12.61f;i.left_ir=80;i.right_ir=18.3f;
 for(int k=0;k<3;k++)advance(&t,&i);
#if KC_PARK_REQUIRE_WALLS
 assert(t.active&&t.phase==KCT_PARK&&starts==0);
 for(int k=0;k<22&&t.active;k++)advance(&t,&i);
 assert(!t.active&&t.phase==KCT_ERROR&&starts==0);
#else
 /* Wall check off: the front gap alone parks, still without more travel. */
 assert(!t.active&&t.phase==KCT_DONE&&starts==0);
#endif
 assert(kc_forward_pwm_limit(9,0)==850&&kc_forward_pwm_limit(9,1)==850);
 assert(kc_forward_pwm_limit(15,0)==3000&&kc_forward_pwm_limit(15,1)==3000);
 assert(kc_forward_pwm_limit(30,1)==5000&&kc_forward_pwm_limit(60,1)==0);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side1=2;t.phase=KCT_O1_PLAN;t.index=1;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.speed==50&&t.amount==7);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_RETURN;t.return_prepared=1;t.y=150;t.o1_front=70;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.speed==100);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));i.us=120;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.move==KCT_FWD&&t.speed==100);
 /* Reported RIGHT oscillation begins at29.17cm: accept it without motion. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,2));t.phase=KCT_O2_TARGET;t.side2=2;t.expected_range=29.15f;
 for(int k=0;k<3;k++){i.us=29.17f;advance(&t,&i);}
 assert(t.phase==KCT_DONE&&starts==0);
 /* If genuinely below the band, one 3cm reverse can land in the band. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,2));t.phase=KCT_O2_TARGET;t.side2=2;t.expected_range=25.18f;
 for(int k=0;k<3;k++){i.us=25.18f;advance(&t,&i);}
 assert(t.move==KCT_REV&&t.amount==3&&starts==1);
 i.progress=3.4f;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 for(int k=0;k<3;k++){i.us=29.43f;advance(&t,&i);}
 assert(t.phase==KCT_DONE&&starts==1);
 /* RIGHT route clears before inward turn, finishes with measured REVL45. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.phase=KCT_O1_PLAN;t.side1=2;t.index=3;t.o1_gap=32;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.amount==20&&t.speed==50);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.phase=KCT_O1_PLAN;t.side1=2;t.index=5;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.amount==26);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.phase=KCT_O1_PLAN;t.side1=2;t.index=6;t.h=45;
 i.heading=45;advance(&t,&i);assert(t.move==KCT_REV_LEFT&&t.amount==45&&t.speed==50);
 i.heading=0;i.motion_status=1;i.progress=-12;advance(&t,&i);
 assert(t.active&&fabsf(t.h)<.001f&&fabsf(t.x-7.77817f)<.01f&&fabsf(t.y+9.89949f)<.01f);
 /* Actual reported too-deep park: only one bounded reverse after fresh walls. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.h=180;i.heading=180;
 i.us=5.54f;i.left_ir=20.4f;i.right_ir=39.5f;
 for(int k=0;k<6;k++)advance(&t,&i);
 assert(t.phase==KCT_MOVE&&t.move==KCT_REV&&t.amount==7&&t.speed==12&&t.park_reversed&&starts==1);
 i.progress=7.1f;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 i.us=12.64f;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_DONE&&starts==1);
 /* Unknown rear corridor: no reverse allowed just because front is close. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us=5.54f;i.left_ir=80;i.right_ir=20;
 for(int k=0;k<24&&t.active;k++)advance(&t,&i);
#if KC_PARK_REQUIRE_WALLS
 assert(t.phase==KCT_ERROR&&starts==0);
#else
 /* Wall check off: one bounded 7 cm retreat back along the entry path. */
 assert(t.phase==KCT_MOVE&&t.move==KCT_REV&&t.amount==7&&starts==1);
#endif
 /* A second too-close episode cannot trigger repeated reversing. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.park_reversed=1;
 i.us=5.54f;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_ERROR&&starts==0);
 /* Stable inside reading can be revalidated once despite old encoder anchor. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.expected_range=18.55f;
 i.us=11.94f;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<25&&t.active;k++)advance(&t,&i);
 assert(t.phase==KCT_DONE&&t.park_recheck_used&&starts==0);
 /* Erratic/invalid US never gets inferred from side walls alone. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us_valid=0;i.us=300;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<80&&t.active;k++)advance(&t,&i);
 assert(t.phase==KCT_ERROR&&t.park_recheck_used&&starts==0);
 /* A far wrong cluster after recovery cannot authorize an extra drive. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.expected_range=18.55f;
 i.us=120;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<55&&t.active;k++)advance(&t,&i);
 assert(t.phase==KCT_ERROR&&starts==0);
 /* An inside cluster cannot confirm when one wall is missing. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.expected_range=18.55f;
 i.us=11.94f;i.left_ir=80;i.right_ir=24;
 for(int k=0;k<40&&t.active;k++)advance(&t,&i);
#if KC_PARK_REQUIRE_WALLS
 assert(t.phase==KCT_ERROR&&starts==0);
#else
 /* Wall check off: after the one stopped recheck, the inside gap parks. */
 assert(t.phase==KCT_DONE&&t.park_recheck_used&&starts==0);
#endif
 /* Reported90.17cm approach:40cm then38cm, rechecked against ACTUAL travel. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us=90.17f;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 /* Approach legs of 35 cm or more run at transit speed: the distance-based
  * PWM cap (5000/3000/850) still brings the robot in slowly at the end. */
 assert(t.phase==KCT_MOVE&&t.move==KCT_FWD&&t.amount==40&&t.speed==100);
 i.progress=40.4f;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 assert(fabsf(t.park_forward_total-40.4f)<.01f);
 i.us=49.77f;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_MOVE&&t.amount==38&&t.speed==100);
 i.progress=38.2f;i.motion_status=1;advance(&t,&i);i.motion_status=0;i.us=11.57f;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_DONE&&starts==2&&fabsf(t.park_forward_total-78.6f)<.01f);
 /* Distance and segment bounds prevent an endlessly receding wall. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.park_forward_total=149;
 i.us=90;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_ERROR&&starts==0);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;t.park_segments=8;
 i.us=90;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_ERROR&&starts==0);
 /* Continued motion prevents completion/next reverse until100ms quiet. */
 KcStopSettle stop_state;
 kc_settle_begin(&stop_state,1000,100,200);
 assert(kc_settle_poll(&stop_state,1050,101,199)==0);
 assert(kc_settle_poll(&stop_state,1149,101,199)==0);
 assert(kc_settle_poll(&stop_state,1150,101,199)==1);
 kc_settle_begin(&stop_state,1000,100,200);
 for(unsigned now=1100;now<=1800;now+=100)assert(kc_settle_poll(&stop_state,now,(uint16_t)now,200)==0);
 assert(kc_settle_poll(&stop_state,1801,1801,200)==-1);
 kc_settle_begin(&stop_state,0xfffffff0u,65535,0);
 assert(kc_settle_poll(&stop_state,20,0,65535)==0);
 assert(kc_settle_poll(&stop_state,120,0,65535)==1);
 /* Reported oscillation readings are all accepted without another correction.
  * Both routes adjust only the heading-zero side-lane straight. */
 const float accepted[]={30.0f,30.05f,32.0f,33.55f,33.8f,34.0f};
 for(unsigned a=0;a<sizeof(accepted)/sizeof(accepted[0]);a++){
  for(unsigned side=1;side<=2;side++){
   reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
   t.y=68.92f;t.us_anchor=100.6f;t.expected_range=32;
   for(int k=0;k<3;k++){i.us=accepted[a];advance(&t,&i);}
   assert(t.phase==KCT_O1_VISION&&starts==0&&t.o1_gap==accepted[a]);
   assert(kc_arrow(&t,1,1,side));advance(&t,&i);
   assert(t.phase==KCT_O1_PLAN);
   t.index=3;t.h=0;kc_o1_plan(&t,&i);
   float base=side==1?9.0f:20.0f;
   assert(t.move==KCT_FWD&&t.speed==50);
   assert(fabsf(t.amount-base-(accepted[a]-32))<=0.5001f);
   /* Relative inward-turn Y differs by only integer rounding, X unchanged. */
   float nominal_y=t.start_y+base;
   assert(fabsf((t.start_y+t.amount-accepted[a])-(nominal_y-32))<=0.5001f);
  }
 }
 /* Outside-band readings still correct; no unverified gap can drive lane adjustment. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.y=65;t.expected_range=34;
 for(int k=0;k<3;k++){i.us=34.1f;advance(&t,&i);}
 assert(t.phase==KCT_MOVE&&t.move==KCT_FWD&&t.amount==2&&t.speed==12);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));t.side1=2;t.index=3;
 kc_o1_plan(&t,&i);assert(t.phase==KCT_ERROR&&starts==0);
 /* Recognised O2 LEFT at25.43cm must not chase the28cm view position. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,2));t.phase=KCT_O2_RANGE;t.waiting_arrow=2;t.arrow=1;t.expected_range=25.84f;
 for(int k=0;k<3;k++){i.us=25.43f;advance(&t,&i);}
 assert(t.phase==KCT_O2_VISION&&t.arrow==1&&starts==0);
 advance(&t,&i);assert(t.phase==KCT_O2_TARGET&&t.side2==1);
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_DONE&&starts==0);
 /* Reported LEFT/LEFT return: old Y57.90 path collides with park envelope.
  * New plan stages earlier on the already-traversed return lane. */
 float diagonal,entry;
 assert(kce_template(56.11f,58));assert(!kce_clear(57.90f,73.27f,143.17f));
 assert(kce_choose(56.11f,73.61f,140.41f,73.27f,143.17f,&diagonal,&entry));
 assert(diagonal==58&&entry>85&&entry<110);
 assert(kce_clear(entry,73.27f,143.17f));
 assert(kce_reposition_clear(56.11f,73.61f,entry,73.27f,143.17f));
 assert(!kce_choose(56.11f,73.61f,80,73.27f,143.17f,&diagonal,&entry));
 assert(!kce_choose(100,73.61f,140,73.27f,143.17f,&diagonal,&entry));
 assert(!kce_choose(NAN,73.61f,140,73.27f,143.17f,&diagonal,&entry));
 assert(!kce_reposition_clear(0,55,100,73.27f,143.17f)); /* Cannot back through O1. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_REAR_CLEAR;t.x=56.11f;t.y=73.61f;t.return_y=140.41f;t.o1_front=73.27f;t.o2_front=143.17f;t.h=180;i.heading=180;
 advance(&t,&i);assert(t.phase==KCT_MOVE&&t.move==KCT_REV&&t.speed==12);
 float reposition=t.amount;
 i.motion_status=1;i.progress=reposition+.8f;advance(&t,&i);i.motion_status=0;
 assert(t.phase==KCT_ENTRY_ALIGN);advance(&t,&i);assert(t.phase==KCT_CENTRE);
 advance(&t,&i);assert(t.move==KCT_RIGHT&&t.speed==50);
 i.heading=135;i.motion_status=1;i.progress=22;advance(&t,&i);i.motion_status=0;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.amount==58&&t.speed==100);
 i.progress=60.1f;i.motion_status=0;i.heading=135;advance(&t,&i);
 assert(t.phase==KCT_ERROR); /* Diagonal outside checked travel budget brakes. */
 /* Reposition overrun beyond the safe interval cannot start a turn. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_ENTRY_ALIGN;t.x=56.11f;t.y=120;t.o1_front=73.27f;t.o2_front=143.17f;t.centre_diagonal=58;t.h=180;i.heading=180;
 advance(&t,&i);assert(t.phase==KCT_ERROR&&starts==0);
 /* Wide LEFT return is shortened before O1: 56cm lane -> about30cm,
  * instead of a58cm diagonal at the park. Every planned template stays clear. */
 assert(kce_trim_choose(56.11f,124.67f,73.27f,143.17f,&diagonal,&entry));
 assert(diagonal==15 && entry>=118 && entry<=124);
 assert(kce_clear(entry,73.27f,143.17f));
 float end_x=kce_poses[kce_ends[0]].x,end_y=entry+kce_poses[kce_ends[0]].y;
 assert(fabsf(end_x-30)<1);
 assert(kce_choose(end_x,end_y,end_y,73.27f,143.17f,&diagonal,&entry));
 assert(diagonal<25);
 /* A path with less than5cm feasible staging interval is rejected. */
 assert(!kce_trim_choose(-42,124.67f,73.27f,143.17f,&diagonal,&entry));
 /* Run 443289900 (O1 60 cm, O1-O2 60 cm, wide O2): no trim fits, but the
  * direct entry from the -52.6 lane passes; RETURN_CLEAR now goes direct. */
 assert(!kce_trim_choose(-58.15f,124.47f,72.28f,143.71f,&diagonal,&entry));
 assert(kce_trim_choose_to(-58.15f,kce_partial_target(-58.15f),124.47f,72.28f,143.71f,&diagonal,&entry));
 /* Run 700394561 (O2 LEFT): home lane +67.6 needs a 74 cm entry diagonal. */
 assert(kce_choose(67.62f,124.55f,124.55f,70.36f,144.60f,&diagonal,&entry));
 assert(!kce_trim_choose(-52.58f,123.60f,71.59f,143.69f,&diagonal,&entry));
 assert(kce_choose(-52.58f,123.60f,123.60f,71.59f,143.69f,&diagonal,&entry));
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_RETURN;t.x=56.11f;t.y=140.41f;t.o1_front=73.27f;t.o2_front=143.17f;t.h=180;i.heading=180;
 advance(&t,&i);assert(t.phase==KCT_MOVE&&t.after==KCT_RETURN_CLEAR&&t.speed==22&&t.amount==16);
 for(unsigned k=0;k<30&&t.phase!=KCT_O1_EDGE&&t.active;k++){
  i.motion_status=0;i.progress=0;i.heading=t.h;
  if(t.phase==KCT_MOVE){i.progress=t.amount;i.motion_status=1;
   i.heading=t.move_heading+(t.move==KCT_LEFT?t.amount:t.move==KCT_RIGHT?-t.amount:0);
  }
  advance(&t,&i);
 }
 assert(t.phase==KCT_O1_EDGE&&t.return_prepared&&fabsf(t.x-30)<1);
 /* Real run428994270: O1front72.95, O2front231.50. Previously
  * the final-entry lookup began currentY-40 and falsely rejected the lane. */
 assert(kce_trim_choose(51.40f,212.35f,72.95f,231.50f,&diagonal,&entry));
 assert(diagonal==9 && entry>182 && entry<212);
 assert(kce_clear(entry,72.95f,231.50f));
 end_x=kce_poses[kce_ends[0]].x;end_y=entry+kce_poses[kce_ends[0]].y;
 float staged_y=end_y;
 assert(kce_choose(end_x,end_y,end_y,72.95f,231.50f,&diagonal,&entry));
 assert(staged_y-entry>40); /* Long straight connection must be checked. */
 assert(kce_reposition_clear(end_x,staged_y,entry,72.95f,231.50f));
 assert(!kce_reposition_clear(0,staged_y,entry,72.95f,231.50f)); /* O1 blocks centreline. */
 /* Robot 2026-10-15: from a lane 100 cm out the side IR saw an object 36 cm
  * away during the O1 search; taking it for O1 moved the pose 54 cm and the
  * 45 deg entry hit the carpark wall. Inconsistent readings are ignored. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side2=1;t.h=180;i.heading=180;
 t.x=100;t.y=90;t.o1_front=71.6f;t.o2_front=172;t.return_y=150;
 kc_begin_move(&t,KCT_FWD,12,30,KCT_O1_EDGE,i.now);i.right_ir=36;
 for(int k=0;k<30&&t.phase==KCT_O1_EDGE;k++){i.progress=(float)k;advance(&t,&i);}
 assert(t.active&&t.phase!=KCT_O1_EDGE&&fabsf(t.x-100)<0.5f);
 i.progress=0;
 /* A very wide lane never falls back to the old 45 deg diagonal: it uses the
  * modelled 90 deg crossing (same footprint checks) ... */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));wide_events=0;
 t.phase=KCT_RETURN_CLEAR;t.x=100;t.y=120;t.o1_front=73;t.o2_front=143;t.h=180;i.heading=180;
 advance(&t,&i);assert(t.active&&wide_events>0);
 /* ... and a lane beyond even that (straight across > 85 cm) still stops. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));
 t.phase=KCT_RETURN_CLEAR;t.x=200;t.y=120;t.o1_front=73;t.o2_front=143;t.h=180;i.heading=180;
 advance(&t,&i);assert(t.phase==KCT_ERROR&&starts==0);
 puts("Task2 native state-machine scenarios passed (four arrow combinations + guards).");return 0;
}
