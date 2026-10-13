#include <assert.h>
#include <stdio.h>
#include "Core/Inc/kc_task2.h"
#include "Core/Inc/kc_forward_profile.h"
#include "Core/Inc/kc_stop_settle.h"
static unsigned starts,stops;static float brake_distance;
static int commands[128];static unsigned amounts[128];
static void start(int m,unsigned speed,unsigned n){assert(speed>0&&speed<=100);if(m!=KCT_FWD)assert(speed<=50);assert(n>0);commands[starts]=m;amounts[starts++]=n;brake_distance=0;}
static float stop(void){stops++;return brake_distance;}
static void event(const char *s,float a,float b){(void)s;(void)a;(void)b;}
static KcT2Hooks hooks={start,stop,event};
static KcT2Input input(void){KcT2Input i={0};i.now=1000;i.imu_at=1000;i.us_at=1000;i.ir_at=1000;i.gyro_ready=1;i.us_valid=1;i.us=60;i.left_ir=80;i.right_ir=80;return i;}
static void advance(KcTask2 *t,KcT2Input *i){i->now+=100;i->us_at=i->ir_at=i->imu_at=i->now;i->us_seq++;i->ir_seq++;t->heartbeat=i->now;kc_step(t,i);}
static void reset(void){starts=stops=0;brake_distance=0;}
static void simulate(unsigned a1,unsigned a2,unsigned cap){
 reset();KcTask2 t;KcT2Input i=input();assert(kc_start(&t,hooks,&i,77,cap));
 unsigned loops=0,previous_starts=0;enum KcPhase last=KCT_IDLE;
 for(;t.active&&loops<1600;loops++){
  if(t.waiting_arrow){assert(!kc_arrow(&t,76,t.waiting_arrow,1));assert(kc_arrow(&t,77,t.waiting_arrow,t.waiting_arrow==1?a1:a2));}
  if(starts!=previous_starts){i.progress=0;previous_starts=starts;}
  i.motion_status=0;
  if(t.phase!=last){i.progress=0;last=t.phase;}
  if(t.phase==KCT_MOVE){
   i.progress=t.amount;brake_distance=i.progress;i.motion_status=1;
   i.heading=t.move_heading+(t.move==KCT_LEFT?t.amount:(t.move==KCT_RIGHT||t.move==KCT_REV_LEFT)?-t.amount:0);
  }else i.heading=t.h;
  if(t.phase==KCT_O1_RANGE)i.us=71.5f-t.y-11.5f;
  else if(t.phase==KCT_O2_RANGE||t.phase==KCT_O2_TARGET)i.us=151.5f-t.y-11.5f;
  else if(t.phase==KCT_PARK)i.us=t.y+13.5f; /* rear wall Y=-25; forward lens 11.5 */
  else i.us=40;
  float ir=20;
  if(t.phase==KCT_IR_EDGE || t.phase==KCT_O2_BACK_EDGE){
   if(t.move_live){i.progress+=1;brake_distance=i.progress;}
   ir=i.progress>=(t.phase==KCT_IR_EDGE?6:30)?80:20;
  }else if(t.phase==KCT_O1_EDGE){
   i.progress+=1;brake_distance=i.progress;
   i.motion_status=i.progress>=t.amount?1:0;
   ir=i.progress>=4 && i.progress<14?20:80;
  }
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
 simulate(1,1,0);simulate(1,2,0);simulate(2,1,0);simulate(2,2,0);
 simulate(1,1,2);simulate(1,2,3);
 reset();KcTask2 t;KcT2Input i=input();i.gyro=24;assert(!kc_start(&t,hooks,&i,1,1));assert(starts==0);
 i=input();assert(kc_start(&t,hooks,&i,1,1));i.now+=2100;i.imu_at=i.now;kc_step(&t,&i);assert(t.phase==KCT_ERROR&&starts==0);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,1));
 t.expected_range=30;t.phase=KCT_O2_TARGET;t.side2=2;t.range=30;
 for(int k=0;k<40&&t.active;k++){i.us=63;advance(&t,&i);}
 assert(t.phase==KCT_ERROR&&starts==0); /* stable wrong echo cannot authorize travel */
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
 advance(&t,&i);assert(t.phase==KCT_O2_TARGET&&t.side2==1);
 /* RIGHT back-clearance advance shifts the next turn outward, not sideways. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,3));
 t.phase=KCT_O2_BACK_TURN;t.side2=2;t.x=49.12f;t.y=157.26f;t.h=0;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.amount==10&&t.speed==16&&!t.guard_us);
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
 assert(t.active&&t.phase==KCT_PARK&&starts==0);
 for(int k=0;k<22&&t.active;k++)advance(&t,&i);
 assert(!t.active&&t.phase==KCT_ERROR&&starts==0);
 assert(kc_forward_pwm_limit(9,0)==850&&kc_forward_pwm_limit(9,1)==850);
 assert(kc_forward_pwm_limit(15,0)==3000&&kc_forward_pwm_limit(15,1)==3000);
 assert(kc_forward_pwm_limit(30,1)==5000&&kc_forward_pwm_limit(60,1)==0);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.side1=2;t.phase=KCT_O1_PLAN;t.index=1;
 advance(&t,&i);assert(t.move==KCT_FWD&&t.speed==50&&t.amount==7);
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_RETURN;t.y=150;t.o1_front=70;
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
 assert(t.phase==KCT_ERROR&&starts==0);
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
 assert(t.phase==KCT_ERROR&&starts==0);
 /* Reported90.17cm approach:40cm then38cm, rechecked against ACTUAL travel. */
 reset();i=input();assert(kc_start(&t,hooks,&i,1,0));t.phase=KCT_PARK;t.entered=i.now;
 i.us=90.17f;i.left_ir=25;i.right_ir=24;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_MOVE&&t.move==KCT_FWD&&t.amount==40&&t.speed==22);
 i.progress=40.4f;i.motion_status=1;advance(&t,&i);i.motion_status=0;
 assert(fabsf(t.park_forward_total-40.4f)<.01f);
 i.us=49.77f;
 for(int k=0;k<3;k++)advance(&t,&i);
 assert(t.phase==KCT_MOVE&&t.amount==38&&t.speed==12);
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
 puts("Task2 native state-machine scenarios passed (four arrow combinations + guards).");return 0;
}
