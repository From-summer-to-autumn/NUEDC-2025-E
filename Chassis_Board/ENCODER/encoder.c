#include "encoder.h"
#include "pid.h"
#include "motor.h"
extern uint8_t period_turn;
/* 假设 A 相接在 PB8/PB10，B 相分别接在 PB9/PB11 */
#define ENC1_B_PORT GPIOB
#define ENC1_B_PIN  DL_GPIO_PIN_9
#define ENC2_B_PORT GPIOB
#define ENC2_B_PIN  DL_GPIO_PIN_11

static volatile uint32_t pulse_hw[2] = {0};
// 100ms 周期脉冲数
volatile uint32_t enc_pulse[2] = {0};
extern uint8_t flag;
uint32_t l_sum_encoder,r_sum_encoder = 0;
uint8_t flag_to_turn;
extern uint8_t flag_have_turn;
/* 捕获中断服务函数 */
/* CAPTURE_0_INST: 编码器1 (PB8) 的捕获定时器实例 */
/* CAPTURE_1_INST: 编码器2 (PB10) 的捕获定时器实例 */
void CAPTURE_encoderl_INST_IRQHandler(void)
{
   if (DL_TimerA_getPendingInterrupt(TIMA0) == DL_TIMER_IIDX_CC0_DN)
    {
        pulse_hw[ENCODER_1]++;
    }
}

void CAPTURE_encoderr_INST_IRQHandler(void)
{
    if (DL_TimerG_getPendingInterrupt(TIMG8) == DL_TIMER_IIDX_CC0_DN)
    {
        pulse_hw[ENCODER_2]++;
    }
}

/* 10ms 定时中断服务函数：计算瞬时速度 */
void TIMER_0_INST_IRQHandler(void)
{
    Encoder_timerISR();
     Control();
    if(flag == 0)
        set_motor(0,0);
    else if(flag == 1)
    {
        if(flag_have_turn)
        {
            if(delay_long() == 1)
            {
                flag_to_turn = 1;
                flag_have_turn = 0;
            }
        }
        if(flag_to_turn == 1 || period_turn == 0)
            {turn_left();}
        else
            follow_line();
    }
}

/* 初始化编码器模块 */
void Encoder_init(void)
{
    /* 使能捕获和定时器中断 */
    NVIC_EnableIRQ(CAPTURE_encoderl_INST_INT_IRQN);
    NVIC_EnableIRQ(CAPTURE_encoderr_INST_INT_IRQN);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
    /* 启动计时器（启动捕获通道和10ms定时器） */
    DL_TimerG_startCounter(CAPTURE_encoderl_INST);
    DL_TimerG_startCounter(CAPTURE_encoderr_INST);
    DL_Timer_startCounter(TIMER_0_INST);
}

void Encoder_timerISR(void)
{
    // 原子操作：将硬件捕获积累值搬移到输出变量
    __disable_irq();
    enc_pulse[0] = pulse_hw[0]*10;
    enc_pulse[1] = pulse_hw[1]*10;
    l_sum_encoder += enc_pulse[0];
    r_sum_encoder += enc_pulse[1];
    pulse_hw[0] = pulse_hw[1] = 0;
    __enable_irq();
}