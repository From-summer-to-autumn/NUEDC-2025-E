#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "pid.h"

uint8_t enable_group1_irq = 0;

void SysTick_Handler(void)
{
    tick_ms++;
}

void Interrupt_Init(void)
{
    if(enable_group1_irq)
    {
        NVIC_EnableIRQ(1);
    }
}

