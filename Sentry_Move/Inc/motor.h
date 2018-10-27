#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"
#include "PID.h"

#define FILTER_BUF_SIZE		10

typedef struct
{
	int16_t refPos;						//λ������
	int16_t rawPos;						//��ǰλ��
	int16_t posBuf[FILTER_BUF_SIZE];			//λ�õļĴ���
	
	int16_t refRotateSpeed;				//�ٶ�����
	int16_t rawRotateSpeed;				//��ǰת��
	int16_t rotateSpeedBuf[FILTER_BUF_SIZE];	//ת�ٵļĴ���
}Motor_t;

extern volatile Motor_t chassisL;	//���ֵ��
extern volatile Motor_t chassisR;	//���ֵ��

extern int16_t chassisSpeedRef;

void Motor_SetRotateSpeed(Motor_t *motor, int16_t speed);
void Motor_SetPos(Motor_t *motor, int16_t pos);
void Motor_UpdateCMRef(void);

#endif
