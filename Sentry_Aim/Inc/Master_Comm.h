#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"

#define MASTER_FRAME_HEAD			0x50

#define BSP_UART8_DMA_TX_BUF_LEN 	sizeof(MasterTxData_t) + 1
#define COMM_TX_FRAME_LEN			BSP_UART8_DMA_TX_BUF_LEN

#define BSP_UART8_DMA_RX_BUF_LEN	sizeof(MasterRxData_t) + 1
#define COMM_RX_FRAME_LEN			BSP_UART8_DMA_RX_BUF_LEN

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	float distance;
	
	uint8_t gimbalMode;
	 
	uint8_t loaderMode;
	
	uint8_t shooterMode;
	
	uint8_t isFind;
}MasterRxData_t;

typedef struct
{
	float yaw;
	
	float pitch;
	
	float yawErr;
	
	float pitchErr;
}MasterTxData_t;		//ÔÆÌ¨¾ø¶Ô½Ç¶È

extern MasterRxData_t masterRxData;
extern MasterTxData_t masterTxData;

void Master_Data_Receive_Start(void);

void Master_CommInit(void);

void Master_RevData(void);

void Master_Decode(uint8_t *pData);

void Master_GenerateData(void);

void Master_SendData(void);

#endif
