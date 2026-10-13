/* Native speed-100 trial; original manual/legacy slowdown is retained. */
#ifndef KC_FORWARD_PROFILE_H
#define KC_FORWARD_PROFILE_H
#include <stdint.h>
static inline int kc_forward_pwm_limit(float remaining_cm,uint8_t native_fast){
 (void)native_fast; /* high transit speed must still brake at the calibrated cap */
 if(remaining_cm<10)return 850;
 if(remaining_cm<20)return 3000;
 if(remaining_cm<50)return 5000;
 return 0; /* no additional slowdown cap */
}
#endif
