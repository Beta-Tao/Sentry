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
	
	int16_t refSpeed;				//�ٶ�����
	int16_t rawSpeed;				//��ǰת��
	int16_t speedBuf[FILTER_BUF_SIZE];	//ת�ٵļĴ���
}Motor_t;

extern volatile Motor_t chassisL;	//���ֵ��
extern volatile Motor_t chassisR;	//���ֵ��

extern int16_t chassisSpeedRef;
extern int16_t chassisDir;

void Motor_SetSpeed(volatile Motor_t *motor, int16_t speed);
void Motor_SetPos(Motor_t *motor, int16_t pos);
void Motor_UpdateCMRef(void);
void Motor_CtrChassis(void);

#endif
