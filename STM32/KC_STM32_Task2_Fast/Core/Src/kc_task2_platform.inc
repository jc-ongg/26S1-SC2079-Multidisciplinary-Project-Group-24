/* Included once by main.c after existing motor primitives. */
#include "kc_task2.h"
#include "kc_stop_settle.h"
static KcTask2 kc_engine;
static MotorCommand_t kc_subcmd;
static uint8_t kc_sub_init, kc_sub_live, kc_sub_status, kc_sub_settling;
static KcStopSettle kc_settle;
static uint16_t kc_enc_a,kc_enc_b;
static float kc_progress;
static volatile uint8_t kc_active;
static volatile uint32_t kc_run_token,kc_ping_at,kc_reply_token;
static volatile uint8_t kc_reply_obstacle,kc_reply_side,kc_reply_failed;
static volatile uint8_t kc_waiting_obstacle;
static char kc_failure[48];

/* Rear-axle odometry for the fast O1 swerve (X right, Y forward, cm). In a
 * lock turn the inner wheel is braked and skids, so use the driven wheel and
 * the gyro: axle travel = outer wheel travel - half track * heading change. */
#define KC_HALF_TRACK_CM 8.0f /* rear wheel centre-to-centre / 2. MEASURE. */
static float kc_odo_x,kc_odo_y,kc_odo_dist,kc_odo_speed,kc_odo_h,kc_odo_origin,kc_odo_v_dist;
static uint16_t kc_odo_a,kc_odo_b;
static uint32_t kc_odo_v_at;
static uint8_t kc_odo_ready;
static volatile int8_t kc_drive_mode=-1; /* -1 off, else KCF_DRIVE_* */
static int kc_drive_pwm;
static float kc_heading_now(void){
 taskENTER_CRITICAL();float h=globalHeading+(isTurning?currentAngle:0.0f);taskEXIT_CRITICAL();
 return h;
}
static void kc_odometry_reset(float origin){
 kc_odo_origin=origin;kc_odo_x=0;kc_odo_y=-KCF_AXLE_CENTRE;kc_odo_dist=0;kc_odo_speed=0;
 kc_odo_v_dist=0;kc_odo_v_at=HAL_GetTick();kc_odo_ready=0;
}
static void kc_odometry(void){
 uint16_t a=(uint16_t)__HAL_TIM_GET_COUNTER(&htim2),b=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
 float h=(kc_heading_now()-kc_odo_origin)*0.017453292519943295f;
 if(!kc_odo_ready){kc_odo_a=a;kc_odo_b=b;kc_odo_h=h;kc_odo_ready=1;return;}
 int16_t da=(int16_t)(a-kc_odo_a),db=(int16_t)(b-kc_odo_b);
 kc_odo_a=a;kc_odo_b=b;
 const float k=WHEEL_CIRCUMFERENCE_CM/ENCODER_COUNTS_PER_REVOLUTION;
 float dl=(float)da*k,dr=-(float)db*k,dh=h-kc_odo_h,d;
 kc_odo_h=h;
 if(kc_drive_mode==KCF_DRIVE_LEFT || kc_drive_mode==KCF_DRIVE_REV_LEFT)d=dr-KC_HALF_TRACK_CM*dh;
 else if(kc_drive_mode==KCF_DRIVE_RIGHT || kc_drive_mode==KCF_DRIVE_REV_RIGHT)d=dl+KC_HALF_TRACK_CM*dh;
 else d=0.5f*(dl+dr);
 float hm=h-0.5f*dh;
 kc_odo_x-=d*sinf(hm);kc_odo_y+=d*cosf(hm);kc_odo_dist+=d;
 uint32_t now=HAL_GetTick();
 if(now-kc_odo_v_at>=50){
  kc_odo_speed=(kc_odo_dist-kc_odo_v_dist)*1000.0f/(float)(now-kc_odo_v_at);
  kc_odo_v_dist=kc_odo_dist;kc_odo_v_at=now;
 }
}
/* Continuous drive for the fast O1 swerve. Writes the servo register
 * directly: setServoAngle() blocks for 100 ms. Lock turns copy motorTurn:
 * outer wheel driven, inner wheel braked, as in the radius calibration. */
static void kc_platform_drive(int mode,float offset,unsigned pwm){
 if(kc_drive_mode<0){
  taskENTER_CRITICAL();currentAngle=0;isTurning=1;isFrontCalib=0;lastAngleUpdateTime=HAL_GetTick();taskEXIT_CRITICAL();
  kc_drive_pwm=0;
 }
 kc_drive_mode=(int8_t)mode;
 int target=pwm>7000u?7000:(int)pwm;
 if(target>kc_drive_pwm+40)kc_drive_pwm+=40;
 else if(target<kc_drive_pwm-40)kc_drive_pwm-=40;
 else kc_drive_pwm=target;
 float ccr;
 if(mode>=KCF_DRIVE_BRAKE){
  /* Short brake, gyro still integrating, steer as asked. TIM4/TIM9 outputs
   * are active-low: motorStop() (compare 7199) drives both inputs low, which
   * lets the wheels coast (robot 2026-10-15: ~46 cm/s^2, 26 cm from 58 cm/s).
   * Compare 0 holds both inputs high: short brake. */
  kc_drive_pwm=0;motorForwardA(0);motorForwardB(0);
  ccr=mode==KCF_DRIVE_BRAKE_LEFT?SERVO_LEFT_MAX:mode==KCF_DRIVE_BRAKE_RIGHT?SERVO_RIGHT_MAX:SERVO_CENTER+3.0f;
 }
 else if(mode==KCF_DRIVE_LEFT){ccr=SERVO_LEFT_MAX;motorStopA();motorForwardB(kc_drive_pwm);}
 else if(mode==KCF_DRIVE_RIGHT){ccr=SERVO_RIGHT_MAX;motorStopB();motorForwardA(kc_drive_pwm);}
 /* Reverse lock turns copy motorTurn REVL/REVR. */
 else if(mode==KCF_DRIVE_REV_LEFT){ccr=SERVO_LEFT_MAX;motorStopA();motorReverseB(kc_drive_pwm);}
 else if(mode==KCF_DRIVE_REV_RIGHT){ccr=SERVO_RIGHT_MAX;motorStopB();motorReverseA(kc_drive_pwm);}
 else if(mode==KCF_DRIVE_REV_STRAIGHT){ccr=SERVO_CENTER+3.0f;motorReverseA(kc_drive_pwm);motorReverseB(kc_drive_pwm);}
 else{
  if(offset>30.0f)offset=30.0f;else if(offset<-30.0f)offset=-30.0f;
  ccr=SERVO_CENTER+3.0f+offset; /* +3: same forward trim as motorPidForward */
  /* Slow the inner wheel to match the steering. Equal PWM on both rear
   * wheels fights small corrections (the O1 runs never settled on the line). */
  float k=offset<0?(-offset/48.4f)/KCF_R_LEFT:-(offset/59.6f)/KCF_R_RIGHT; /* 1/cm, left + */
  float f=fminf(0.5f,fabsf(k)*KC_HALF_TRACK_CM);
  int inner=(int)((float)kc_drive_pwm*(1.0f-f)/(1.0f+f));
  if(k>0){motorForwardA(inner);motorForwardB(kc_drive_pwm);}
  else{motorForwardA(kc_drive_pwm);motorForwardB(inner);}
 }
 __HAL_TIM_SET_COMPARE(&htim12,TIM_CHANNEL_1,(uint32_t)lroundf(ccr));
}

static void kc_encoder_sample(void){
 if(!kc_sub_live)return;
 uint16_t a=(uint16_t)__HAL_TIM_GET_COUNTER(&htim2),b=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
 int16_t da=(int16_t)(a-kc_enc_a),db=(int16_t)(b-kc_enc_b);
 kc_enc_a=a;kc_enc_b=b;
 float d=((float)da-(float)db)*0.5f*WHEEL_CIRCUMFERENCE_CM/ENCODER_COUNTS_PER_REVOLUTION;
 kc_progress+=kc_subcmd.command==REVS?-d:d;
}
static float kc_platform_stop(void){
 motorStop();kc_native_fast_forward=0;kc_drive_mode=-1;kc_drive_pwm=0;
 /* Keep the gyro running during braking and servo centring. */
 if(isTurning){
  setServoAngle(SERVO_CENTER);osDelay(100);
  taskENTER_CRITICAL();globalHeading+=currentAngle;currentAngle=0;isTurning=0;isFrontCalib=0;taskEXIT_CRITICAL();
 }
 kc_encoder_sample();kc_sub_live=0;kc_sub_status=0;kc_sub_settling=0;
 return kc_progress<0 && kc_progress>=-0.25f?0:kc_progress;
}
static void kc_platform_start(int move,unsigned speed,unsigned amount){
 kc_native_fast_forward=(move==KCT_FWD && speed==100);
 kc_subcmd.command=move==KCT_FWD?FWD:move==KCT_REV?REVS:move==KCT_LEFT?TURNL:move==KCT_RIGHT?TURNR:REVL;
 kc_subcmd.param1Speed=speed*71;kc_subcmd.param2DistAngle=amount;kc_subcmd.cmdId=kc_run_token;
 kc_progress=0;kc_enc_a=(uint16_t)__HAL_TIM_GET_COUNTER(&htim2);kc_enc_b=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
 kc_sub_live=1;kc_sub_init=1;kc_sub_status=0;kc_sub_settling=0;
}
static void kc_platform_event(const char *event,float a,float b){
 char msg[128];
 if(strcmp(event,"CAPTURE")==0 || strcmp(event,"CAPTURE_EARLY")==0){
  kc_waiting_obstacle=(uint8_t)a;
  snprintf(msg,sizeof(msg),"!KC_T2/%s/%u/%lu;",event,(unsigned)a,(unsigned long)kc_run_token);
 }
 else snprintf(msg,sizeof(msg),"!KC_T2/%s/%.2f/%.2f;",event,(double)a,(double)b);
 if(strncmp(event,"ERROR_",6)==0)snprintf(kc_failure,sizeof(kc_failure),"%s",event);
 uartSend(msg);
}
static KcT2Input kc_platform_input(void){
 KcT2Input i={0};
 taskENTER_CRITICAL();
 i.now=HAL_GetTick();i.us_seq=kc_us_seq;i.ir_seq=kc_ir_seq;i.us_at=kc_us_at;i.ir_at=kc_ir_at;i.imu_at=kc_imu_at;
 i.us=ultrasonicDistanceCm;i.us_valid=ultrasonicValid;i.left_ir=irLeftDistanceCm;i.right_ir=irRightDistanceCm;
 /* Completed straight primitives commit currentAngle into globalHeading but
  * may retain currentAngle. Match SENSOR/OLED: add it only while tracking. */
 i.heading=globalHeading+(isTurning?currentAngle:0.0f);i.gyro=gyro_z_dps;i.gyro_ready=gyro_calibrated;
 taskEXIT_CRITICAL();
 i.motion_status=kc_sub_status;i.progress=kc_progress;
 i.ox=kc_odo_x;i.oy=kc_odo_y;i.odist=kc_odo_dist;i.speed=kc_odo_speed;return i;
}
static void kc_platform_abort(void){
 if(kc_active){kc_fail(&kc_engine,"ERROR_ABORTED",HAL_GetTick());kc_active=0;}
}
static uint8_t kc_platform_tick(MotorCommand_t command,uint8_t entering){
 if(entering){
  kc_platform_stop();kc_run_token=command.cmdId;kc_ping_at=HAL_GetTick();kc_reply_side=0;kc_reply_failed=0;kc_failure[0]=0;
  KcT2Hooks hooks={kc_platform_start,kc_platform_stop,kc_platform_event,kc_platform_drive};KcT2Input i=kc_platform_input();
  kc_odometry_reset(i.heading);i=kc_platform_input();
  if(!kc_start(&kc_engine,hooks,&i,command.cmdId,command.param2DistAngle)){
   /* Say which start check failed: yaw rate (must be <=3 deg/s), IMU sample age (<=250 ms), gyro calibrated. */
   char why[96];
   snprintf(why,sizeof(why),"!KC_T2/ERROR_NOT_STATIONARY_READY/%.2f/%lu/%u;",(double)i.gyro,(unsigned long)(i.now-i.imu_at),(unsigned)i.gyro_ready);
   uartSend(why);
   snprintf(kc_failure,sizeof(kc_failure),"ERROR_NOT_STATIONARY_READY");return 2;
  }
  /* Turns aim at desiredHeading, which otherwise keeps the previous run's
   * last target: a few degrees of residual made entry moves fail their
   * heading check. Start every run with it on the actual heading. */
  taskENTER_CRITICAL();desiredHeading=i.heading;taskEXIT_CRITICAL();
  kc_platform_event("HEADING_ORIGIN",i.heading,0);kc_platform_event("HEADING_FEEDBACK",1,2);kc_active=1;
  if(kc_imu_recoveries)kc_platform_event("IMU_RECOVERIES",(float)kc_imu_recoveries,0);
 }
 if(kc_sub_live){
  uint8_t status=0;
  if(kc_sub_settling){
   /* Never call a finished PID primitive again: keep brake applied while
    * recording coast in the SAME command, before any next command starts. */
   kc_encoder_sample();
   int result=kc_settle_poll(&kc_settle,HAL_GetTick(),kc_enc_a,kc_enc_b);
   status=result>0?1:result<0?2:0;
   if(result<0)kc_platform_event("ERROR_BRAKE_NOT_SETTLED",kc_progress,0);
  }else{
   if(kc_subcmd.command==FWD)status=motorPidForward(kc_subcmd,kc_sub_init);
   else if(kc_subcmd.command==REVS)status=motorPidReverse(kc_subcmd,kc_sub_init);
   else status=motorTurn(kc_subcmd,kc_sub_init);
   kc_sub_init=0;kc_encoder_sample();
   if(status==1 && (kc_subcmd.command==FWD || kc_subcmd.command==REVS)){
    motorStop();kc_sub_settling=1;
    kc_settle_begin(&kc_settle,HAL_GetTick(),kc_enc_a,kc_enc_b);status=0;
   }
  }
  kc_sub_status=status==1?1:status?2:0;
  if(status){kc_sub_live=0;kc_sub_settling=0;kc_platform_event("ENCODER_FINISH",kc_progress,status);}
 }
 kc_engine.heartbeat=kc_ping_at;
 kc_odometry();
 KcT2Input i=kc_platform_input();
 if(kc_reply_token==kc_run_token){
  if(kc_reply_side){kc_arrow(&kc_engine,kc_reply_token,kc_reply_obstacle,kc_reply_side);kc_reply_side=0;}
  if(kc_reply_failed && kc_reply_obstacle==kc_engine.waiting_arrow){kc_engine.vision_failed=1;kc_reply_failed=0;}
 }
 kc_step(&kc_engine,&i);
 kc_waiting_obstacle=kc_engine.waiting_arrow;
 if(!kc_engine.active){kc_active=0;return kc_engine.phase==KCT_DONE?1:2;}
 return 0;
}
