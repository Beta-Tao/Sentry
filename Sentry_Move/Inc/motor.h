#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "PID.h"

#define FILTER_BUF_SIZE		10

typedef struct
{
	int16_t refPos;						//位置期望
	int16_t rawPos;						//当前位置
	int16_t posBuf[FILTER_BUF_SIZE];			//位置的寄存器
	
	int16_t refRotateSpeed;				//速度期望
	int16_t rawRotateSpeed;				//当前转速
	int16_t rotateSpeedBuf[FILTER_BUF_SIZE];	//转速的寄存器
}Motor_t;

extern volatile Motor_t chassisL;	//左轮电机
extern volatile Motor_t chassisR;	//右轮电机

extern int16_t chassisSpeedRef;

void Motor_SetRotateSpeed(Motor_t *motor, int16_t speed);
void Motor_SetPos(Motor_t *motor, int16_t pos);
void Motor_UpdateCMRef(void);

#endif
