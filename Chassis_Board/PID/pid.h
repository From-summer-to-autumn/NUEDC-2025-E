#ifndef __PID_H
#define __PID_H

#include "ti_msp_dl_config.h"

typedef float fp32;

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

typedef struct
{
    uint8_t mode;
    fp32 Kp;
    fp32 Ki;
    fp32 Kd;

    fp32 max_out; 
    fp32 max_iout; 

    fp32 set;
    fp32 fdb;

    fp32 out;
    fp32 Pout;
    fp32 Iout;
    fp32 Dout;
    fp32 Dbuf[3];  
    fp32 error[3]; 
} pid_type_def;
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID struct data point
  * @param[in]      mode: PID_POSITION: normal pid
  *                 PID_DELTA: delta pid
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid max out
  * @param[in]      max_iout: pid max iout
  * @retval         none
  */
/**
  * @brief          pid struct data init
  * @param[out]     pid: PID??????
  * @param[in]      mode: PID_POSITION:??PID
  *                 PID_DELTA: ??PID
  * @param[in]      PID: 0: kp, 1: ki, 2:kd
  * @param[in]      max_out: pid????
  * @param[in]      max_iout: pid??????
  * @retval         none
  */
extern void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout);

/**
  * @brief          pid calculate 
  * @param[out]     pid: PID struct data point
  * @param[in]      ref: feedback data 
  * @param[in]      set: set point
  * @retval         pid out
  */
/**
  * @brief          pid??
  * @param[out]     pid: PID??????
  * @param[in]      ref: ????
  * @param[in]      set: ???
  * @retval         pid??
  */
extern fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set);

/**
  * @brief          pid out clear
  * @param[out]     pid: PID struct data point
  * @retval         none
  */
/**
  * @brief          pid ????
  * @param[out]     pid: PID??????
  * @retval         none
  */
extern void PID_clear(pid_type_def *pid);

int error_calc(void);

void Control(void);

void Control_init(void);

uint8_t cross_detect(void);

void follow_line(void);

void turn_left();

void turn(int angle);

uint8_t delay_turn(void);

uint8_t delay_long(void);

#endif
