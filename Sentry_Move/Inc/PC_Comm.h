#ifndef _PC_COMM_H_
#define _PC_COMM_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

#define BSP_USART3_DMA_RX_BUF_LEN	sizeof(PCRxFrame_t) - 1 + 4
//考虑存储变量的布局
#define PC_COMM_TX_FRAME_LEN		sizeof(PCTxComm_t) + 3

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
}PCRxCommState_e;

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	float distance;
	
	uint8_t isBig;			//是否是大装甲板
	
	uint8_t isInSight;		//是否在视野中
	
	uint8_t isFind;			//是否识别到
}PCRxFrame_t;

typedef struct
{
	PCRxFrame_t PCData;
	
	volatile PCRxCommState_e PCRxCommState;
}PCRxComm_t;

typedef struct
{
	float yaw;
	
	float pitch;
}PCTxComm_t;

extern PCRxComm_t PCRxComm;
extern PCTxComm_t PCTxComm;

void PC_Data_Receive_Start(void);

void PC_CommInit(void);

void PC_Data_Receive(void);

void PC_Decode(uint8_t *pData);

void PC_IsCommDrop(void);

void PC_SendData(void);

#endif
