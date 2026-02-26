#include "servo.h"

void set_servo_1(uint32_t value)
{
    DL_TimerG_setCaptureCompareValue(servo_1_INST,value,GPIO_servo_1_C0_IDX);
    if(value < 500) value = 500;
    if(value > 2500)value = 2500;
}

void set_servo_2(uint32_t value)
{
    DL_TimerG_setCaptureCompareValue(servo_2_INST,value,GPIO_servo_2_C0_IDX);
    if(value < 500) value = 500;
    if(value > 2500)value = 2500;
}

