#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

/* Ħ���ֵ�����Ƴ��� */
#define FM_VEL_MIN		-10000
#define FM_VEL_MAX		10000

#define SHOOTER_ACC	10000
#define SHOOTER_DEC	10000

/* ����ID */
#define FM_L_ID			0x203
#define FM_R_ID			0x204

typedef enum
{
	SHOOTER_CEASE	= 0,
	SHOOTER_OPEN	= 1,
}ShooterMode_e;

typedef struct
{
	
	ShooterMode_e mode;
	
	Motor_t FM_Left;
	
	Motor_t FM_Right;
	
	float shootVel;
	
}Shooter_t;


/* Ħ���ֵ�� */
extern Shooter_t sentryShooter;

extern float g_ShootVel;	//�����ٶ�

void Shooter_CtrlInit(Shooter_t *shooter);

void Shooter_MotorCtrl(Shooter_t *shooter);

#endif
