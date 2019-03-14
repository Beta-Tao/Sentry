#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

/* Ħ���ֵ�����Ƴ��� */
#define FM_MAX		1300 - 1
#define FM_STOP		1000 - 1

typedef enum
{
	SHOOTER_CEASE	= 0,
	SHOOTER_OPEN	= 1,
}ShooterMode_e;

typedef struct
{
	ShooterMode_e mode;
	
	uint32_t shootVel;
}Shooter_t;

/* Ħ���ֵ�� */
extern Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter);

void Shooter_UpdateState(void);

void Shooter_MotorCtrl(Shooter_t *shooter);

void Shooter_SetVel(uint32_t vel);

#endif
