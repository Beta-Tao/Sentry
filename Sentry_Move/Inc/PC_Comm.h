#ifndef _PC_COMM_H_
#define _PC_COMM_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

#define BSP_USART3_DMA_RX_BUF_LEN	sizeof(PCFrame_t) - 1		
	//由于存储变量的布局，PCFrame_t的结构体为20个字节

#define PC_FRAME_LEN			BSP_USART3_DMA_RX_BUF_LEN
#define START_CHECK_FIRST		0xA5
#define START_CHECK_SECOND		0x5A
#define END_CHECK_FIRST			0xAA
#define END_CHECK_SECOND		0xAA

#define PC_IT_CYCLE		20

typedef enum
{
	PC_COMM_DROP = 0,
	PC_COMM_NORMAL,
}PCCommState_e;

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	uint8_t posCtrlType;
	
	float distance;
	
	uint8_t isTraced;
	
	uint8_t isInSight;
}PCFrame_t;

typedef struct
{
	PCFrame_t PCData;
	
	volatile PCCommState_e PCCommState;
}PCComm_t;

extern PCComm_t PCComm;

extern uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void);

void PC_CommInit(void);

void PC_Data_Receive(void);

void PC_Decode(uint8_t *pData);

void PC_IsCommDrop(void);

#endif
