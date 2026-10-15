/* Host test exercises the actual native adapter with mocked hardware. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
enum { FWD,REVS,TURNL,TURNR,REVL };
typedef struct {int command;uint32_t param1Speed,param2DistAngle,cmdId;} MotorCommand_t;
typedef struct {uint16_t count;} Timer;
static Timer htim2,htim3,htim12;
#define __HAL_TIM_GET_COUNTER(t) ((t)->count)
#define __HAL_TIM_GET_COMPARE(t,ch) ((t)->count)
#define __HAL_TIM_SET_COMPARE(t,ch,v) ((t)->count=(uint16_t)(v))
#define WHEEL_CIRCUMFERENCE_CM (6.5f*3.14f)
#define ENCODER_COUNTS_PER_REVOLUTION 1540.0f
#define SERVO_CENTER 146
#define SERVO_LEFT_MAX 98
#define SERVO_RIGHT_MAX 206
#define TIM_CHANNEL_1 1
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
static uint32_t now=1000,kc_us_seq,kc_ir_seq,kc_us_at=1000,kc_ir_at=1000,kc_imu_at=1000,lastAngleUpdateTime,kc_imu_recoveries;
static float ultrasonicDistanceCm=32,irLeftDistanceCm=25,irRightDistanceCm=24,globalHeading,currentAngle,gyro_z_dps,desiredHeading;
static uint8_t ultrasonicValid=1,gyro_calibrated=1,isTurning,isFrontCalib,kc_native_fast_forward;
static unsigned pid_calls,brakes;
static uint32_t HAL_GetTick(void){return now;}
static void osDelay(unsigned ms){now+=ms;}
static void motorStop(void){brakes++;}
/* Last PWM per motor; negative = reverse, -1000 = coast (motorStopA/B). */
static int pwm_a,pwm_b;
static void motorForwardA(int v){pwm_a=v;}
static void motorForwardB(int v){pwm_b=v;}
static void motorReverseA(int v){pwm_a=-v;}
static void motorReverseB(int v){pwm_b=-v;}
static void motorStopA(void){pwm_a=-1000;}
static void motorStopB(void){pwm_b=-1000;}
static void setServoAngle(int pwm){(void)pwm;}
static void uartSend(const char *s){(void)s;}
static uint8_t motorPidForward(MotorCommand_t c,uint8_t init){
 (void)c;assert(init);pid_calls++;
 htim2.count+=700;htim3.count-=700;
 return 1;
}
static uint8_t motorPidReverse(MotorCommand_t c,uint8_t init){(void)c;(void)init;assert(0);return 0;}
static uint8_t motorTurn(MotorCommand_t c,uint8_t init){(void)c;(void)init;assert(0);return 0;}
#include "Core/Src/kc_task2_platform.inc"
static void tick(uint32_t stamp){
 now=stamp;kc_ping_at=now;kc_imu_at=now;kc_ir_at=now;kc_us_at=now;
 MotorCommand_t cmd={0};assert(kc_platform_tick(cmd,0)==0);
}
int main(void){
 /* Completed reverse: its delta has ALREADY been committed. Do not add twice. */
 globalHeading=-180.6f;currentAngle=-1.6f;isTurning=0;
 KcT2Input sample=kc_platform_input();assert(fabsf(sample.heading+180.6f)<.001f);
 isTurning=1;sample=kc_platform_input();assert(fabsf(sample.heading+182.2f)<.001f);
 globalHeading=225;currentAngle=1.9f;sample=kc_platform_input();assert(fabsf(sample.heading-226.9f)<.001f);
 globalHeading=currentAngle=0;isTurning=0;
 KcT2Hooks hooks={kc_platform_start,kc_platform_stop,kc_platform_event,kc_platform_drive,kc_platform_goal};
 KcT2Input i=kc_platform_input();assert(kc_start(&kc_engine,hooks,&i,77,1));kc_active=1;
 kc_move_to(&kc_engine,KCT_FWD,100,10,KCT_O1_RANGE,now);
 tick(1000);assert(kc_sub_settling&&kc_sub_live&&kc_sub_status==0&&pid_calls==1);
 float first=kc_progress;
 htim2.count+=100;htim3.count-=100;
 tick(1050);assert(kc_progress>first&&kc_engine.phase==KCT_MOVE&&pid_calls==1);
 tick(1149);assert(kc_engine.phase==KCT_MOVE&&pid_calls==1);
 tick(1150);assert(kc_engine.phase==KCT_O1_RANGE&&!kc_sub_live&&pid_calls==1);
 assert(fabsf(kc_engine.y-800*WHEEL_CIRCUMFERENCE_CM/ENCODER_COUNTS_PER_REVOLUTION)<.001f);
 assert(brakes>=2);
 kc_platform_abort();assert(!kc_active&&!kc_engine.active);
 /* Fast-O1 drive: brake modes hold both inputs high (forward PWM 0, the
  * short brake on this board), never motorStop's coast; reverse lock turns
  * drive the outer wheel backward and leave the inner one coasting. */
 /* A stop while moving short-brakes (both inputs high) and waits for quiet
  * encoders before releasing, instead of motorStop's coast. */
 kc_odo_speed=30;pwm_a=pwm_b=500;{uint32_t t0=now;unsigned b0=brakes;kc_platform_stop();
 assert(pwm_a==0&&pwm_b==0&&now-t0>=60&&brakes==b0+1);}kc_odo_speed=0;
 /* A new straight move started while one runs (back-face speed change)
  * commits the gyro angle first: motorPidForward's init zeroes it. */
 kc_platform_start(KCT_FWD,90,147);kc_sub_settling=0;globalHeading=10;currentAngle=2.5f;isTurning=1;
 kc_platform_start(KCT_FWD,50,100);assert(fabsf(globalHeading-12.5f)<1e-4f&&currentAngle==0);
 kc_sub_live=0;globalHeading=currentAngle=0;isTurning=0;
 /* A braked straight move adds the unseen skid: 5 cm x (57/57)^2 at 57 cm/s. */
 kc_platform_start(KCT_FWD,50,100);htim2.count+=500;htim3.count-=500;kc_odo_speed=57;
 {float d=kc_platform_stop();float seen=500*WHEEL_CIRCUMFERENCE_CM/ENCODER_COUNTS_PER_REVOLUTION;
  assert(fabsf(d-(seen+5.0f))<0.05f);}kc_odo_speed=0;
 kc_platform_drive(KCF_DRIVE_STRAIGHT,0,3550);assert(isTurning&&pwm_a==40&&pwm_b==40);
 kc_platform_drive(KCF_DRIVE_BRAKE,0,0);assert(pwm_a==0&&pwm_b==0&&htim12.count==SERVO_CENTER+3);
 kc_platform_drive(KCF_DRIVE_BRAKE_RIGHT,0,0);assert(pwm_a==0&&pwm_b==0&&htim12.count==SERVO_RIGHT_MAX);
 kc_platform_drive(KCF_DRIVE_REV_RIGHT,0,2500);assert(pwm_a==-40&&pwm_b==-1000);
 kc_platform_drive(KCF_DRIVE_REV_LEFT,0,2500);assert(pwm_a==-1000&&pwm_b==-80&&htim12.count==SERVO_LEFT_MAX);
 kc_platform_drive(KCF_DRIVE_REV_STRAIGHT,0,1600);assert(pwm_a==-120&&pwm_b==-120);
 /* O2 U-turn hand-back: the stop commits the gyro angle, then the legacy
  * heading goal (held by straights, added to by turns) moves to 180 later. */
 kc_platform_stop();globalHeading=-90;currentAngle=0;desiredHeading=-90;
 kc_platform_drive(KCF_DRIVE_LEFT,0,3550);assert(isTurning&&htim12.count==SERVO_LEFT_MAX);
 currentAngle=178.5f;kc_platform_stop();
 assert(!isTurning&&kc_drive_mode==-1&&fabsf(globalHeading-88.5f)<0.01f&&currentAngle==0);
 kc_platform_goal(90);assert(desiredHeading==90);
 /* v17 rolling hand-overs. Face end into the U-turn: the legacy straight's
  * heading change (currentAngle) is kept, its PID is no longer called and
  * the wheels keep their PWM. */
 kc_platform_stop();globalHeading=-90;
 kc_platform_start(KCT_FWD,50,100);kc_sub_init=0;isTurning=1;currentAngle=1.25f;
 kc_platform_drive(KCF_DRIVE_LEFT,0,3550);
 assert(!kc_sub_live&&kc_drive_mode==KCF_DRIVE_LEFT&&kc_drive_pwm==3550&&isTurning&&currentAngle==0&&fabsf(globalHeading+88.75f)<0.01f);
 /* U-turn into the back face: the drive's angle is committed before the
  * PID (whose init zeroes currentAngle) takes the motors. */
 currentAngle=178.5f;kc_platform_start(KCT_FWD,90,85);
 assert(kc_sub_live&&kc_sub_init&&kc_drive_mode==-1&&currentAngle==0&&fabsf(globalHeading-89.75f)<0.01f);
 puts("Native adapter holds brake, includes coasting travel, and does not restart completed PID; fast-O1 brake/reverse modes drive the right pins; U-turn hands the heading goal back.");
}
