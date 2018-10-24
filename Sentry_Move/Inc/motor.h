#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define BUF_SIZE	10

typedef struct
{
	int16_t rawPos;						//位置
	int16_t posBuf[BUF_SIZE];			//位置的寄存器
	
	int16_t rawRotateSpeed;				//转速
	int16_t rotateSpeedBuf[BUF_SIZE];	//转速的寄存器
	
}Motor_t;

extern volatile Motor_t chassisL;	//左轮电机
extern volatile Motor_t chassisR;	//右轮电机

#endif
