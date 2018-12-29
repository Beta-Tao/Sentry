#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"
#include "macro.h"

#define BSP_UART8_DMA_RX_BUF_LEN 	20u		//注意此处只能使用20u，只能用相同的传输缓存大小保证数据帧格式正确
#define COMM_FRAME_LEN				20u		//֡ͷ (float)relaYaw (float)relaPitch (float)refYawVel (float)refPitchVel
											//(uint8_t)aimMode (uint8_t)loadMode (uint8_t)shootMode

extern uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
extern uint32_t rx_data_len;

void Comm_RevStart(void);

void Comm_RevData(void);

void Comm_Decode(uint8_t *pData);

#endif
