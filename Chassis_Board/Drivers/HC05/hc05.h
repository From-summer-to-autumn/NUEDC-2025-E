#ifndef __HC05_H__
#define __HC05_H__

#include "string.h"
#include "ti_msp_dl_config.h"
#include "board.h"

// 是否开启串口0调试     1开始  0关闭
#define  DEBUG   0

#define  BLERX_LEN_MAX  200

#define  BLUETOOTH_LINK      ( ( DL_GPIO_readPins( HC05_PORT, HC05_STATE_PIN ) & HC05_STATE_PIN ) ? 1 : 0 )

#define  CONNECT             1       //蓝牙连接成功
#define  DISCONNECT          0       //蓝牙连接断开

extern unsigned char BLERX_BUFF[BLERX_LEN_MAX];
extern unsigned char BLERX_FLAG;
extern unsigned char BLERX_LEN;

void Bluetooth_Init(void);
unsigned char Get_Bluetooth_ConnectFlag(void);
void Bluetooth_Mode(void);
void Receive_Bluetooth_Data(void);
void BLE_send_String(unsigned char *str);
#endif