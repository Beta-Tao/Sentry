#ifndef _REMOTE_CTRL_H_
#define _REMOTE_CTRL_H_

#include "stm32f4xx_hal.h"

#define BSP_USART1_DMA_RX_BUF_LEN 30u                            //遥控器数据接收DMA存储长度

extern uint8_t autoMode;
extern uint8_t shootMode;

extern uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];
void RemoteCtl_Data_Receive_Start(void);
void RemoteCtl_Data_Receive(void);

void Remote_Process(void);
void Remote_InitFlag(void);
#endif
