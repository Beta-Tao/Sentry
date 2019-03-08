#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"

#define BSP_UART8_DMA_RX_BUF_LEN 	18u
#define COMM_FRAME_LEN				BSP_UART8_DMA_RX_BUF_LEN		
											//帧头 (float)relaYaw (float)relaPitch (float)refYawVel (float)refPitchVel
											// (uint8_t)GimbalMode

extern uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];

extern unsigned char commOutputBuffer[COMM_FRAME_LEN];

void Comm_Float2Byte(float *target, unsigned char *buf, uint8_t loc);

void Comm_GenerateData(void);

void Comm_GetData(void);

void Comm_SendData(void);

#endif
