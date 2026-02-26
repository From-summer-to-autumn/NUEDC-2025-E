#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "stdbool.h"
#include "trace.h"
#include "board.h"
#include "imu660rb.h"
#include "oled_hardware_i2c.h"

pid_type_def speed_pid_left, speed_pid_right;
int speed_set, speed_left, speed_right;
int error; 
volatile bool D1, D2, D3, D4, D5, D6, D7;
fp32 PID_speed_left[3]={80.0f,2.0000f,5.0f};
fp32 PID_speed_right[3]={80.0f,2.0000f,5.0f};
float yaw_ref, first_yaw;
extern uint8_t flag, flag_to_turn;
uint8_t flag_have_turn = 0;
uint8_t flag_yaw, flag_turn = 1, period = 0, N = 0, n = 0, period_turn = 0;
uint16_t sum_encoder = 0;
#define yaw euler.angle.yaw 
#define LimitMax(input, max)   \
{                              \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
}


void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    pid->mode = mode;
    pid->Kp = PID[0];
    pid->Ki = PID[1];
    pid->Kd = PID[2];
    pid->max_out = max_out;
    pid->max_iout = max_iout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}


fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = set - ref;
    if (pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
        pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        LimitMax(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        LimitMax(pid->out, pid->max_out);
    }
    return pid->out;
}


void PID_clear(pid_type_def *pid)
{
    if (pid == NULL)
    {
        return;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    pid->fdb = pid->set = 0.0f;
}

int error_calc(void)
{	
	
	  if((D1 == 1)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))        error = -30;      //000 0001
	  else if((D1 == 1)&&(D2 == 1)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = -20;     //000 0011
	  else if((D1 == 0)&&(D2 == 1)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = -15;     //000 0010
	  else if((D1 == 0)&&(D2 == 1)&&(D3 == 1)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = -10;     //000 0110
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 1)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = -5;     //000 0100
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 1)&&(D4 == 1)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = -3;     //000 1100
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 1)&&(D5 == 0)&&(D6 == 0)&&(D7 == 0))   error = 0;     //000 1000 //中间
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 1)&&(D5 == 1)&&(D6 == 0)&&(D7 == 0))   error = 3;     //001 1000
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 1)&&(D6 == 0)&&(D7 == 0))   error = 5;     //001 0000
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 1)&&(D6 == 1)&&(D7 == 0))   error = 10;     //011 0000
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 1)&&(D7 == 0))   error = 15;     //010 0000
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 1)&&(D7 == 1))   error = 20;     //110 0000
	  else if((D1 == 0)&&(D2 == 0)&&(D3 == 0)&&(D4 == 0)&&(D5 == 0)&&(D6 == 0)&&(D7 == 1))   error = 30;      //100 0000
	  else error = 0;
	  return error;
}

void follow_line(void)
{
	error_calc();
	PID_calc(&speed_pid_left, enc_pulse[0], speed_set - error);
	PID_calc(&speed_pid_right, enc_pulse[1], speed_set + error);
	speed_right = speed_pid_right.out;
	speed_left = speed_pid_left.out;
	set_motor(speed_left,speed_right);	
}

void Control_init(void)
{
	PID_init(&speed_pid_left, PID_DELTA, PID_speed_left, 8000, 20);
	PID_init(&speed_pid_right, PID_DELTA, PID_speed_right, 8000, 20);
}

void Control(void)	//每隔10ms调用一次
{
    D1 = trace_out1();
    D2 = trace_out2();
    D3 = trace_out3();
    D4 = trace_out4();
    D5 = trace_out5();
    D6 = trace_out6();
    D7 = trace_out7();
}


uint8_t cross_detect(void)
{
		if((D1==0&&D2==0&&D6==1&&D7==1)) 
		{
			if((trace_out1()==0&&trace_out2()==0&&trace_out6()==1&&trace_out7()==1)) 
				return 1;
			else
				return 0;
		}
		else return 0;
}

void turn(int angle)
{
	int delta_angle = (int)yaw - angle;
	if(delta_angle > 180) delta_angle = delta_angle - 360;
	else if(delta_angle < -180) delta_angle = 360 + delta_angle;
	if(delta_angle < 2 && delta_angle> -2) 
	{
		flag_turn = 1;
	}
	else
	{
		set_motor(-1000, 1000);
	}
}

void turn_left()
{
	static bool was_on_cross = false;
	static bool ready_to_turn = false;
	static bool ok_to_turn = false;
	bool is_on_cross = cross_detect();
	if(sum_encoder >= (16200 - l_sum_encoder) &&n == N)
	{
		flag = 0;
	}
	if(ok_to_turn)
	{
		// turn() 函数会在角度到达目标时将 flag_turn 置 1
		if(flag_turn == 1)
		{
			// 转弯完成，重置所有静态变量，为下一次调用做准备
			was_on_cross = false;
			ready_to_turn = false;
			ok_to_turn = false;
			flag_to_turn = 0;
			flag_have_turn = 1;
			period++;
			period_turn++;
			uart0_send_char('a');
			uart0_send_char('0');
			if(period == 4)
			{
				n++;
				period = 0;
				if(n == N)
				{
					l_sum_encoder = 0;
					r_sum_encoder = 0;
				}
			}
		}
		else
		{
			turn(yaw_ref + 85); // 执行90度左转
		}
	}
	// 状态2：已越过路口，直行一小段距离后准备转弯
	else if(ready_to_turn)
	{
		// delay() 用于在转弯前让小车稍微前移，以使转弯中心更准确
		if(delay_turn() == 1)
		{
			ok_to_turn = true; // 延时结束，进入转弯状态
			ready_to_turn = false;
			if(sum_encoder == 0)
			{
				sum_encoder = l_sum_encoder;
			}
		}
		else
		{
			// 延时期间，保持直行巡线
			follow_line();
		}
	}
	// 状态1：正常巡线，检测是否越过路口
	else
	{
		// 这个条件只在小车传感器刚刚完全离开路口线的瞬间触发一次
		if (was_on_cross && !is_on_cross) {
			yaw_ref = yaw;
//			distance_car = 0;
			flag_turn = 0; // 清除旧的转弯完成标志
			ready_to_turn = true; // 进入下一个状态
			uart0_send_char('a');
			uart0_send_char('1');
		}
		else
		{
			follow_line();
		}
	}

	// 持续记录上一时刻是否在路口上
	was_on_cross = is_on_cross;
}





uint8_t delay_turn(void){
	static uint32_t i;
	i++;
	if(i == 25){
		i = 0;
		return 1;
	}
	else return 0;
}

uint8_t delay_long(void){
	static uint32_t i;
	i++;
	if(i == 60){
		i = 0;
		return 1;
	}
	else return 0;
}


