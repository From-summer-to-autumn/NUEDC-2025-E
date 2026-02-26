#include "motor.h"

// 设置fi引脚的PWM比较值
static void set_bi(uint16_t dat)
{
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,dat,GPIO_PWM_Motor_C1_IDX);
}
// 设置bi引脚的PWM比较值
static void set_fi(uint16_t dat)
{
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST,dat,GPIO_PWM_Motor_C0_IDX);
}

// 限制输入的PWM最大值
// 仅内部使用
static void restrict_pwm_max_value(int* value)
{
    if( *value > MOTOR_PWM_MAX )
    {
        *value = MOTOR_PWM_MAX;
    }
	else if( *value < -MOTOR_PWM_MAX )
    {
        *value = -MOTOR_PWM_MAX;
    }
}

// 设置电机速度
void set_motor(int fi_value, int bi_value)
{
    restrict_pwm_max_value(&fi_value);
    restrict_pwm_max_value(&bi_value);

	if(fi_value>=0)
	{
		DL_GPIO_setPins(GPIO_Motor_A24_PORT,GPIO_Motor_A24_PIN);
		DL_GPIO_clearPins(GPIO_Motor_A30_PORT,GPIO_Motor_A30_PIN);
		set_fi(fi_value);
	}
	else
	{
		DL_GPIO_clearPins(GPIO_Motor_A24_PORT,GPIO_Motor_A24_PIN);
		DL_GPIO_setPins(GPIO_Motor_A30_PORT,GPIO_Motor_A30_PIN);
		set_fi(-fi_value);
	}
	if(bi_value>=0)
	{
        DL_GPIO_setPins(GPIO_Motor_B24_PORT,GPIO_Motor_B24_PIN);
		DL_GPIO_clearPins(GPIO_Motor_A22_PORT,GPIO_Motor_A22_PIN);
		set_bi(bi_value);
	}
	else
	{
		DL_GPIO_setPins(GPIO_Motor_A22_PORT,GPIO_Motor_A22_PIN);
		DL_GPIO_clearPins(GPIO_Motor_B24_PORT,GPIO_Motor_B24_PIN);
		set_bi(-bi_value);
	}

}

// 电机停止
void stop_motor(void)
{
    set_motor(0,0);
}