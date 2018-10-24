#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define BUF_SIZE	10

typedef struct
{
	int16_t rawPos;						//λ��
	int16_t posBuf[BUF_SIZE];			//λ�õļĴ���
	
	int16_t rawRotateSpeed;				//ת��
	int16_t rotateSpeedBuf[BUF_SIZE];	//ת�ٵļĴ���
	
}Motor_t;

extern volatile Motor_t chassisL;	//���ֵ��
extern volatile Motor_t chassisR;	//���ֵ��

#endif
