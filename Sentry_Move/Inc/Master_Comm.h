#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"
#include "Gimbal_Ctrl.h"
#include "Loader_Ctrl.h"
#include "Shooter_Ctrl.h"

#define BSP_UART8_DMA_RX_BUF_LEN 	sizeof(MasterData_t) + 1
#define COMM_FRAME_LEN				BSP_UART8_DMA_RX_BUF_LEN
#define MASTER_FRAME_HEAD			0x50

typedef struct
{
	float yawAngle;
	
	float pitchAngle;
	
	uint8_t posCtrlType;
	
	uint8_t gimbalMode;
	
	uint8_t loaderMode;
	
	uint8_t shooterMode;
	
}MasterData_t;

extern uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];

extern unsigned char commOutputBuffer[COMM_FRAME_LEN];

extern MasterData_t masterData;

void Master_CommInit(void);

void Master_GenerateData(void);

void Master_GetData(void);

void Master_SendData(void);

#endif
