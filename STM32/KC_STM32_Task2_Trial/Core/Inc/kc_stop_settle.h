/* Encoder-verified braking interval. No HAL/RTOS dependency. */
#ifndef KC_STOP_SETTLE_H
#define KC_STOP_SETTLE_H
#include <stdint.h>
typedef struct { uint32_t started, quiet_since; uint16_t a,b; } KcStopSettle;
static inline void kc_settle_begin(KcStopSettle *s,uint32_t now,uint16_t a,uint16_t b){
 s->started=s->quiet_since=now;s->a=a;s->b=b;
}
/* 0 settling, 1 quiet100ms, -1 not stationary within800ms. */
static inline int kc_settle_poll(KcStopSettle *s,uint32_t now,uint16_t a,uint16_t b){
 if(a!=s->a || b!=s->b){s->quiet_since=now;s->a=a;s->b=b;}
 if((uint32_t)(now-s->started)>800)return -1;
 return (uint32_t)(now-s->quiet_since)>=100?1:0;
}
#endif
