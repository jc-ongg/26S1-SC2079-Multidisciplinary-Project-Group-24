/* gcc -std=gnu11 -O1 -Wall -ICore/Inc test_o1_range.c -lm -o o1r && ./o1r
 * Fast O1 START range vote (KCF_RANGE_N echoes, largest group within 3 cm,
 * farther on a tie), replaying echoes from robot logs 2026-10-16. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "kc_task2.h"

static float stop_(void){return 0;}
static void start_(int m,unsigned s,unsigned n){(void)m;(void)s;(void)n;}
static void event_(const char *s,float a,float b){if(!strcmp(s,"F_O1_RANGE_VOTES"))printf("    votes %.0f/%.0f\n",a,b);}
static void drive_(int m,float off,unsigned pwm){(void)m;(void)off;(void)pwm;}
static void goal_(float h){(void)h;}
static KcT2Hooks hooks={start_,stop_,event_,drive_,goal_};

/* Feed echoes (repeating the list) until F_RANGE decides; return the range. */
static float run(const float *e,int n,int *fed){
 KcTask2 t;KcT2Input i;memset(&i,0,sizeof i);
 i.now=1000;i.imu_at=i.us_at=i.ir_at=1000;i.gyro_ready=1;i.left_ir=i.right_ir=80;
 assert(kc_start(&t,hooks,&i,1,1));
 assert(t.phase==KCT_F_RANGE);
 int k=0;
 for(;k<200 && t.active && t.phase==KCT_F_RANGE;k++){
  i.now+=80;i.us_at=i.imu_at=i.ir_at=i.now;t.heartbeat=i.now;i.us_seq++;
  i.us=e[k%n];i.us_valid=1;kc_step(&t,&i);
 }
 *fed=k;
 assert(t.active);
 return t.range;
}

int main(void){
 struct {const char *name;float want;int n;float e[16];} cases[]={
  {"148.9 run",148.9f,6,{148,77,145,69,71,150}},
  {"128.8 run",128.8f,11,{89,129,129,103,130,94,130,96,129,128,129}},
  {"149.4 run",149.4f,9,{149,151,111,64,150,107,82,149,150}},
  {"148.5 run",148.5f,5,{148,147,85,126,148}},
  {"risky 118 run (62/68 cluster)",120.0f,7,{122,68,62,68,105,62,118}},
  {"60 cm, steady",60.0f,4,{60,60.5f,59.8f,60.2f}},
 };
 for(unsigned c=0;c<sizeof cases/sizeof cases[0];c++){
  int fed;float r=run(cases[c].e,cases[c].n,&fed);
  printf("%-32s -> O1 range %.1f after %d echoes (true ~%.0f)\n",cases[c].name,r,fed,cases[c].want);
  assert(fabsf(r-cases[c].want)<=3.5f);
 }
 /* The wrong cluster must never win the risky run, whatever the order. */
 const float base[]={122,68,62,68,105,62,118,120,121,119};
 for(int rot=0;rot<10;rot++){
  float e[10];for(int k=0;k<10;k++)e[k]=base[(k+rot)%10];
  int fed;float r=run(e,10,&fed);
  assert(r>110.0f);
 }
 puts("O1 range vote passed");
 return 0;
}
