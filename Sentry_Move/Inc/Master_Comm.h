#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"
#include "macro.h"

#define BSP_UART8_DMA_RX_BUF_LEN 	30u
#define COMM_FRAME_LEN				20u		// 帧头 (float)relaYaw (float)relaPitch (float)refYawVel (float)refPitchVel
											//(uint8_t)aimMode (uint8_t)loadMode (uint8_t)shootMode
/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-7500		//即转速
#define GM_YAW_VEL_MAX			7500

#define GM_PITCH_VEL_MIN		-13000		//即转速
#define GM_PITCH_VEL_MAX		13000

extern uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];

extern unsigned char commOutputBuffer[25];

void Comm_Float2Byte(float *target, unsigned char *buf, uint8_t loc);

void Comm_GenerateData(void);

void Comm_GetData(void);

void Comm_SendData(void);

#endif
