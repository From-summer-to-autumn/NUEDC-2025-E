#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "ti_msp_dl_config.h"

// 电机最大输入的PWM值
#define MOTOR_PWM_MAX 8000

void set_motor(int fi_value, int bi_value);
void stop_motor(void);

#endif