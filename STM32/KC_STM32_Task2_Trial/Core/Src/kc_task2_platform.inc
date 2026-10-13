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

static void kc_encoder_sample(void){
 if(!kc_sub_live)return;
 uint16_t a=(uint16_t)__HAL_TIM_GET_COUNTER(&htim2),b=(uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
 int16_t da=(int16_t)(a-kc_enc_a),db=(int16_t)(b-kc_enc_b);
 kc_enc_a=a;kc_enc_b=b;
 float d=((float)da-(float)db)*0.5f*WHEEL_CIRCUMFERENCE_CM/ENCODER_COUNTS_PER_REVOLUTION;
 kc_progress+=kc_subcmd.command==REVS?-d:d;
}
static float kc_platform_stop(void){
 motorStop();kc_native_fast_forward=0;
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
 i.heading=globalHeading+currentAngle;i.gyro=gyro_z_dps;i.gyro_ready=gyro_calibrated;
 taskEXIT_CRITICAL();
 i.motion_status=kc_sub_status;i.progress=kc_progress;return i;
}
static void kc_platform_abort(void){
 if(kc_active){kc_fail(&kc_engine,"ERROR_ABORTED",HAL_GetTick());kc_active=0;}
}
static uint8_t kc_platform_tick(MotorCommand_t command,uint8_t entering){
 if(entering){
  kc_platform_stop();kc_run_token=command.cmdId;kc_ping_at=HAL_GetTick();kc_reply_side=0;kc_reply_failed=0;kc_failure[0]=0;
  KcT2Hooks hooks={kc_platform_start,kc_platform_stop,kc_platform_event};KcT2Input i=kc_platform_input();
  if(!kc_start(&kc_engine,hooks,&i,command.cmdId,command.param2DistAngle)){
   snprintf(kc_failure,sizeof(kc_failure),"ERROR_NOT_STATIONARY_READY");return 2;
  }kc_active=1;
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
