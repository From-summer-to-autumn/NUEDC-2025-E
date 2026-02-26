#include "servo.h"
#include "math.h"
#include "clock.h"

void set_servo_2(uint32_t value)
{
    DL_TimerG_setCaptureCompareValue(servo_2_INST,value,GPIO_servo_2_C0_IDX);
}
