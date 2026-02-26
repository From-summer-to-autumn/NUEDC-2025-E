#ifndef __ENCODER_H
#define __ENCODER_H

#include <stdint.h>
#include <stdbool.h>
#include "ti_msp_dl_config.h"

/* 编码器方向 */
typedef enum {
    ENC_DIR_STOP = 0,
    ENC_DIR_CW   = 1,  /* 顺时方向 */
    ENC_DIR_CCW  = -1  /* 逆时方向 */
} EncoderDirection;

typedef enum {
    ENCODER_1 = 0,
    ENCODER_2 = 1
} EncoderID;

/* 编码器数据结构 */
void Encoder_init(void);

// 100 ms 定时器中断中调用，处理值迁移
void Encoder_timerISR(void);

// 全局可访问变量储存 100ms 脉冲量
extern volatile uint32_t enc_pulse[2];
extern uint32_t l_sum_encoder,r_sum_encoder;
#ifdef __cplusplus
}
#endif

#endif /* ENCODER_H */