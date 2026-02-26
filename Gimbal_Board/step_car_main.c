/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "board.h"
#include "interrupts.h"
#include "clock.h"
#include "step.h"
#include "oled_hardware_i2c.h"
#include "PID.h"
#include "stdbool.h"
#include "servo.h"

#define RX_BUFFER_SIZE 9
#define RX_BUFFER_SIZE_CAR 2
uint8_t flag_servo = 0;
volatile uint8_t res_data[RX_BUFFER_SIZE];
volatile uint8_t gEchoData;
volatile uint8_t res_car[RX_BUFFER_SIZE_CAR];
volatile uint8_t gEchoData_car;
volatile uint8_t gByteCount;
pid_type_def x_pid, y_pid;
int servo_1 = 1500, servo_2 = 1500;
int error_x = 11, error_y = 0, x_set, y_set, x_ref, y_ref; 
uint8_t flag_not_found;
fp32 PID_error_x[3]={0.08f,0.0f,0.04f};
fp32 PID_error_y[3]={0.8f,0.05f,0.0f};
uint8_t flag_find_first, is_set = 0, is_turning;
uint8_t is_turning;
int32_t L_PWM,R_PWM;
int Flag = 0;
// float ATarget_angle=0,BTarget_angle=0;
uint8_t have_found = false;
uint8_t have_found_first = false;
uint8_t period = 0;
uint16_t flag_hit;



int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    NVIC_EnableIRQ(1);
    DL_Timer_startCounter(PWM_0_INST);
	DL_Timer_startCounter(servo_2_INST);
	NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
    OLED_Init();
    OLED_Clear();
    PID_init(&x_pid, PID_DELTA, PID_error_x, 1000, 1.0);
    PID_init(&y_pid, PID_DELTA, PID_error_y, 300, 20);
    mspm0_delay_ms(200);
    if(have_found_first)
    {
        PID_error_x[0]=0.12f;
        PID_error_x[1]=0.0f;
        PID_error_x[2]=0.06f;
    }
    set_servo_2(1105);
    mspm0_delay_ms(300);
    Flag = 1;
    // L_PWM=Calculate_target(1);//设置电机每分钟60转
    // R_PWM=Calculate_target(1);//设置电机每分钟60转
    // Set_PWM(L_PWM,R_PWM);//PWM波驱动电机
    // Set_PWM(0,0);//PWM波驱动电机
    while (1) {
        OLED_ShowNum(0,0, Flag, 1, 8);
        OLED_ShowNum(0,1, have_found, 1, 8);
        OLED_ShowNum(0,3, x_ref, 3, 8);
        OLED_ShowNum(40,3, y_ref, 3, 8);
        OLED_ShowNum(0,5, 490, 3, 8);
        OLED_ShowNum(40,5, 70, 3, 8);
        OLED_ShowNum(0,6, period, 2, 8);
    }
}


void GROUP1_IRQHandler(void)
{
	if((DL_GPIO_getEnabledInterruptStatus(Key_PORT,Key_B16_PIN) & Key_B16_PIN) != 0)
	{
        Flag = 1;
        DL_GPIO_clearPins(LED_PORT,LED_B22_PIN);
        DL_GPIO_clearInterruptStatus(Key_PORT, Key_B16_PIN);
	}

	else if((DL_GPIO_getEnabledInterruptStatus(Key_PORT,Key_B20_PIN) & Key_B20_PIN) != 0)
	{
        Flag = 2;
        PID_error_x[0] = 0.15;
        PID_error_y[1] = 0;
        PID_error_y[2] = 0.2;
        DL_GPIO_setPins(LED_PORT,LED_B22_PIN);
        DL_GPIO_clearInterruptStatus(Key_PORT, Key_B20_PIN);
	}

	else if((DL_GPIO_getEnabledInterruptStatus(Key_PORT,Key_B4_PIN) & Key_B4_PIN) != 0)
	{
        Flag = 3;
        DL_GPIO_clearPins(LED_PORT,LED_B22_PIN);
        DL_GPIO_clearInterruptStatus(Key_PORT, Key_B4_PIN);
	}

    else if((DL_GPIO_getEnabledInterruptStatus(Key_PORT,Key_B15_PIN) & Key_B15_PIN) != 0)
	{
        Flag = 4;
        DL_GPIO_setPins(LED_PORT,LED_B22_PIN);
        DL_GPIO_clearInterruptStatus(Key_PORT, Key_B15_PIN);			
	}
}

void UART_1_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(UART_1_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            // Read the received byte and store it in our buffer
            res_car[gByteCount] = DL_UART_Main_receiveData(UART_1_INST);
            gByteCount++;

            if (gByteCount == RX_BUFFER_SIZE_CAR) {
                gByteCount = 0;
                if(res_car[0] == 'a'){
                    is_turning = res_car[1] - '0';
                    period++;
                    if(period == 4)
                        period = 1;
                }
            }
            break;
        default:
            break;
    }
}

void TIMER_0_INST_IRQHandler(void)
{
    if(DL_TimerG_getPendingInterrupt(TIMER_0_INST))
    {
        if(DL_TIMER_IIDX_ZERO)
        {       
                if(Flag == 1)
                {
                    if((error_x <= 5) && (error_x >= -5))
                    {
                        flag_hit++;
                        if(flag_hit>6)
                        {
                        laser_on();
                        Flag = 0;
                        }
                    }

                    if(have_found_first == false)
                    {
                        L_PWM=Calculate_target(17);
                        Step_Set_PWM(L_PWM);
                    }
                    else if(have_found == true)
                    { 
                        PID_calc(&x_pid, x_ref - 490, 0);
                        L_PWM=Calculate_target(-x_pid.out);
                        Step_Set_PWM(L_PWM);
                    }
                    else {
                        L_PWM=Calculate_target(0);
                        Step_Set_PWM(L_PWM);
                    }
                }
                else if(Flag == 2)//单击BLS开启或关闭电机
                {
                    PID_calc(&x_pid, x_ref - 490, 0);
                    if(is_turning)
                    {
                    L_PWM=Calculate_target(-x_pid.out + 10);//设置电机每分钟60转
                    Step_Set_PWM(L_PWM);//PWM波驱动电机
                    }
                    else if(period == 1){
                        L_PWM=Calculate_target(-x_pid.out - 8);//设置电机每分钟60转
                        Step_Set_PWM(L_PWM);//PWM波驱动电机
                    }
                    else if(period == 3){
                        L_PWM=Calculate_target(-x_pid.out + 8);//设置电机每分钟60转
                        Step_Set_PWM(L_PWM);//PWM波驱动电机
                    }
                    else{
                        L_PWM=Calculate_target(-x_pid.out);//设置电机每分钟60转
                        Step_Set_PWM(L_PWM);//PWM波驱动电机
                    }
                    // if(period == 1)
                    // {
                    // L_PWM=Calculate_target(-x_pid.out - 4);//设置电机每分钟60转
                    // Step_Set_PWM(L_PWM);//PWM波驱动电机
                    // }
                    // else if(period == 2)
                    // {
                    // L_PWM=Calculate_target(-x_pid.out - 1);//设置电机每分钟60转
                    // Step_Set_PWM(L_PWM);//PWM波驱动电机
                    // }
                    // else if(period == 3)
                    // {
                    // L_PWM=Calculate_target(-x_pid.out + 4);//设置电机每分钟60转
                    // Step_Set_PWM(L_PWM);//PWM波驱动电机
                    // }
                    // else if(period == 4)
                    // {
                    // L_PWM=Calculate_target(-x_pid.out + 1);//设置电机每分钟60转
                    // Step_Set_PWM(L_PWM);//PWM波驱动电机
                    // }
                }
                else if(Flag == 3)
                {
                     
                    L_PWM= 0;
			        R_PWM = Process_MotorB_Movement_PWM();
			        Step_Set_PWM(L_PWM);
                }
                else if(Flag == 4)
                { 
                    L_PWM= 0;
			        R_PWM = Process_MotorB_Movement_PWM();
			        Step_Set_PWM(L_PWM);
                }
                else {
                Step_Set_PWM(0);
                }
        }
    }
}


void UART_0_INST_IRQHandler(void)
{
    static uint8_t i = 0;
    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            gEchoData = DL_UART_Main_receiveData(UART_0_INST);
            
            res_data[i] = DL_UART_Main_receiveData(UART_0_INST);
            DL_UART_Main_transmitData(UART_0_INST, res_data[i]);
            i++;
            if(res_data[0] != 0x46) i=0;
            if((i ==2) && (res_data[1] != 0x45)) i = 0;
            if(i == 9)
            {
                if(res_data[RX_BUFFER_SIZE-1] == 0x41)
                {
                    if(res_data[2] - '0' == 9 && res_data[3] - '0' == 9 && res_data[4] - '0' == 9 && res_data[5] - '0' == 9 && res_data[6] - '0' == 9 &&res_data[7] - '0' == 9)
                    {
                        // flag_not_found++;
                        // if(flag_not_found == 20)
                        // {
                        //     flag_not_found = 0;
                            have_found = false;
                        // }
                        // else
                        // {
                        //     Step_Set_PWM(0);
                        // }
                    }
                    else {
                        x_ref = 100*(res_data[2] - '0') + 10*(res_data[3] - '0') + res_data[4] - '0';
                        y_ref = 100*(res_data[5] - '0') + 10*(res_data[6] - '0') + res_data[7] - '0';
                        have_found = true;
                        have_found_first = true;
                        error_x = x_ref - 490;
                        // error_y = y_ref - 70;
                        // if(have_found == true)
                        // {PID_calc(&y_pid, y_ref - 70, 0);
                        // set_servo_2(1050 + y_pid.out);}
                    }

                }
                i = 0;
            }

            break;
        default:
            break;
    }
}

