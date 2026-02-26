/*
 * Copyright (c) 2021, Texas Instruments Incorporated
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
#include "main.h"
#include "stdio.h"
#include "hc05.h"
#include "trace.h"
#include "motor.h"
#include "encoder.h"
#include "pid.h"
#include "board.h"
#include "imu660rb.h"
#include "interrupt.h"

uint8_t oled_buffer[32];
uint32_t sys_tick;
extern int speed_set, speed_left, speed_right;
uint16_t error_abs, error_out;
extern int error; 
extern volatile bool D1, D2, D3, D4, D5, D6, D7;
extern float first_yaw;
uint8_t flag=0;
extern uint8_t flag_yaw;
extern uint8_t flag_beep;
extern uint8_t flag_stop;
extern uint8_t period, n, N;
//extern bool ok_to_turn;
int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    OLED_Init();
    Control_init();
    //Bluetooth_Init();
    IMU660RB_Init();
    /* Don't remove this! */
    Encoder_init();
    Interrupt_Init();  
    // //定时器初始化
    //Ultrasonic_Init();
    speed_set = 60;
    set_motor(0,0);	

    // OLED_ShowString(0,7,(uint8_t *)"IMU660RB Demo",8);

    // OLED_ShowString(0,0,(uint8_t *)"Pitch",8);
    // OLED_ShowString(0,2,(uint8_t *)" Roll",8);
    // OLED_ShowString(0,4,(uint8_t *)"  Yaw",8);

    // OLED_ShowString(16*6,3,(uint8_t *)"Accel",8);
    // OLED_ShowString(17*6,4,(uint8_t *)"Gyro",8);

    while (1) 
     {  
/*         DL_GPIO_setPins(GPIO_Motor_A22_PORT,GPIO_Motor_A22_PIN);
		DL_GPIO_clearPins(GPIO_Motor_B24_PORT,GPIO_Motor_B24_PIN); */
        //sprintf((char *)oled_buffer, "%-6.1f", euler.angle.pitch);
        // OLED_ShowString(5*8,0,oled_buffer,16);
        // sprintf((char *)oled_buffer, "%-6.1f", euler.angle.roll);
        // OLED_ShowString(5*8,2,oled_buffer,16);
        // if(flag_beep) beep();

/*         DL_GPIO_setPins(GPIO_Motor_B24_PORT,GPIO_Motor_B24_PIN);
		DL_GPIO_clearPins(GPIO_Motor_A22_PORT,GPIO_Motor_A22_PIN); */
/*         		DL_GPIO_clearPins(GPIO_Motor_B24_PORT,GPIO_Motor_B24_PIN);
		DL_GPIO_setPins(GPIO_Motor_A22_PORT,GPIO_Motor_A22_PIN); */
        if(ahrs.initialising)
        {
            sprintf((char *)oled_buffer, "Init");
            OLED_ShowString(5*8,3,oled_buffer,8);
        }
        else{

            sprintf((char *)oled_buffer, "%-6.1f", euler.angle.yaw+180);
            OLED_ShowString(5*8,3,oled_buffer,8);
            OLED_ShowNum(0, 2, enc_pulse[0], 3, 8);
            OLED_ShowNum(20, 5, D1, 1, 8);
            OLED_ShowNum(30, 5, D2, 1, 8);
            OLED_ShowNum(40, 5, D3, 1, 8);
            OLED_ShowNum(50, 5, D4, 1, 8);
            OLED_ShowNum(60, 5, D5, 1, 8);
            OLED_ShowNum(70, 5, D6, 1, 8);
            OLED_ShowNum(80, 5, D7, 1, 8);
            //OLED_ShowChar(0,3,'N', 8);
            OLED_ShowNum(16, 3, N, 2, 8);
            OLED_ShowNum(0,1,l_sum_encoder,5,8);
            OLED_ShowNum(60,1,r_sum_encoder,5,8);
           // OLED_ShowNum(0,5,ok_to_turn,1,8);   
        }
         
    }
}


void GROUP1_IRQHandler(void)
{
	if((DL_GPIO_getEnabledInterruptStatus(KEY_PORT,KEY_B16_PIN) & KEY_B16_PIN) != 0)
	{
		flag = 1;
		first_yaw = euler.angle.yaw;
        period = 0;
        DL_GPIO_clearInterruptStatus(GPIOB, KEY_B16_PIN);
	}

	else if((DL_GPIO_getEnabledInterruptStatus(KEY_PORT,KEY_B20_PIN) & KEY_B20_PIN) != 0)
	{
		N--;
        //set_motor(2000, 2000);
        DL_GPIO_clearInterruptStatus(GPIOB, KEY_B20_PIN);
	}

	else if((DL_GPIO_getEnabledInterruptStatus(KEY_PORT,KEY_B4_PIN) & KEY_B4_PIN) != 0)
	{
        N++;
        DL_GPIO_clearInterruptStatus(GPIOB, KEY_B4_PIN);
	}

    else if((DL_GPIO_getEnabledInterruptStatus(KEY_PORT,KEY_B15_PIN) & KEY_B15_PIN) != 0)
	{
        flag = 0;
        DL_GPIO_clearInterruptStatus(GPIOB, KEY_B15_PIN);			
	}

    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        #if defined GPIO_IMU660RB_INT_IIDX
            case GPIO_IMU660RB_INT_IIDX:
                Read_IMU660RB();
                // Control();
                // if(flag == 0)
                //     set_motor(0,0);
                // else if(flag == 1)
                //     turn_left();
                break;
        #endif
    }
}
