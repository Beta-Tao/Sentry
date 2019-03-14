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
	
	PosCtrlType_e posCtrlType;
	
	float loadVel;
	
	uint32_t shootVel;
	
	GimbalMode_e gimbalMode;
	
	LoaderMode_e loaderMode;
	
	ShooterMode_e shooterMode;
	
}MasterData_t;

extern uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];

extern unsigned char commOutputBuffer[COMM_FRAME_LEN];

extern MasterData_t masterData;

void Comm_GenerateData(void);

void Comm_GetData(void);

void Comm_SendData(void);

#endif
