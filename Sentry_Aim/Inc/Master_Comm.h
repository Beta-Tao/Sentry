#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"

#define MASTER_FRAME_HEAD			0x50
#define BSP_UART8_DMA_RX_BUF_LEN 	sizeof(MasterData_t) + 1
#define COMM_FRAME_LEN				BSP_UART8_DMA_RX_BUF_LEN

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	uint32_t posCtrlType;
	
	float loadVel;
	
	uint32_t shootVel;
	
	uint8_t gimbalMode;
	
	uint8_t loaderMode;
	
	uint8_t shooterMode;
	
}MasterData_t;

extern MasterData_t masterData;

void Master_Data_Receive_Start(void);

void Master_RevData(void);

void Master_Decode(uint8_t *pData);

#endif
