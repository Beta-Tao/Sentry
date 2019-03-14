#ifndef _PC_COMM_H_
#define _PC_COMM_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

#define BSP_USART3_DMA_RX_BUF_LEN	12u		//0xA5 0x5A angle1 angle2 0xAA 0xAA

#define PC_FRAME_LEN			BSP_USART3_DMA_RX_BUF_LEN
#define START_CHECK_FIRST		0xA5
#define START_CHECK_SECOND		0x5A
#define END_CHECK_FIRST			0xAA
#define END_CHECK_SECOND		0xAA

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	PosCtrlType_e posCtrlType;
}PCFrame_t;

extern PCFrame_t PCAngle_t;

extern uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void);

void PC_Data_Receive(void);

void PC_Decode(uint8_t *pData);

#endif
