#ifndef _MOTOR_H
#define _MOTOR_H
#include "ti_msp_dl_config.h"
#include "board.h"
int Calculate_target(int Target);
void Step_Set_PWM(int L_Target);
int Calculate_target(int Target) ;
int Calculate_Angle_PWM(float target_angle);
int get_fixed_speed_pwm_period(void);
// int Calculate_MA_Angle_PWM(float target_angle);
// int Calculate_MB_Angle_PWM(float target_angle);
// int Calculate_A_Angle_to_turn(float target_angle);
void Calculate_A_Angle_to_turn(float angle_to_add);
int Process_MotorA_Movement_PWM();
void Calculate_B_Angle_to_turn(float angle_to_add);
int Process_MotorB_Movement_PWM();
void laser_on();

#endif