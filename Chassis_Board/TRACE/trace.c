#include "trace.h"

int d1;

bool trace_out1(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT1_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}

bool trace_out2(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT2_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}


bool trace_out3(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT3_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}


bool trace_out4(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT4_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}


bool trace_out5(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT5_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}


bool trace_out6(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT6_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}


bool trace_out7(void)
{
    d1 = 0;
    d1 = DL_GPIO_readPins(GPIO_trace_PORT,GPIO_trace_OUT7_PIN);
    if(d1 > 0) return 1;
    else       return 0;
}