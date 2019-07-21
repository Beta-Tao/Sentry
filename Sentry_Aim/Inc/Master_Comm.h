#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"

#define MASTER_FRAME_HEAD			0x50

#define BSP_UART8_DMA_TX_BUF_LEN 	3
#define COMM_TX_FRAME_LEN			BSP_UART8_DMA_TX_BUF_LEN

#define BSP_UART8_DMA_RX_BUF_LEN	11
#define COMM_RX_FRAME_LEN			BSP_UART8_DMA_RX_BUF_LEN

typedef struct
{
	float remoteYawAngle;
	
	float remotePitchAngle;
	
	uint8_t gimbalMode;
	
	uint8_t shooterMode;
}MasterRxData_t;

typedef struct
{
	uint8_t loaderMode;
	
	uint8_t isInsight;
}MasterTxData_t;

extern MasterRxData_t masterRxData;
extern MasterTxData_t masterTxData;

void Master_Data_Receive_Start(void);

void Master_CommInit(void);

void Master_RevData(void);

void Master_Decode(uint8_t *pData);

void Master_GenerateData(void);

void Master_SendData(void);

#endif
