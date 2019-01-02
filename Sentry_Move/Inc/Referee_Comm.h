#ifndef _REFEREE_COMM_H_
#define _REFEREE_COMM_H_

#include "stm32f4xx_hal.h"

#define BSP_USART6_DMA_RX_BUF_LEN 128U
#define BSP_USART6_DMA_TX_BUF_LEN 30u

extern uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
extern uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void);

void Referee_Data_Receive(void);

void Referee_Decode(uint8_t *pData);

#endif
