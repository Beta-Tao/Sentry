#ifndef _IMU_COMM_H_
#define _IMU_COMM_H_

#include "stm32f4xx_hal.h"

#define BSP_USART6_DMA_RX_BUF_LEN	20u

#define PRE				0x5A
#define	TYPE			0xA5

typedef struct
{
	uint8_t pre;		//前导码
}IMURawData_t;

typedef struct
{
	float yawVel;
	
	float rawYaw;
	
	float lastRawYaw;
	
	float detaRawYaw;
	
	float yaw;			//积累的绝对Yaw轴角度
}IMUData_t;

extern IMUData_t HIData;

void IMU_Data_Receive_Start(void);

void IMU_CommInit(void);

void IMU_Data_Receive(void);

void IMU_Decode(uint8_t *pData);

#endif
