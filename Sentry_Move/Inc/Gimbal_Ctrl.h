#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* ��̨������Ƴ��� */
#define GM_YAW_VEL_MIN			-30		//��ת�� rpm
#define GM_YAW_VEL_MAX			30

#define GM_YAW_ACC				100
#define	GM_YAW_DEC				100

#define GM_PITCH_VEL_MIN		-30		//��ת�� ��/s
#define GM_PITCH_VEL_MAX		30

#define GM_PITCH_ACC			100
#define	GM_PITCH_DEC			100
 
/* ����ID */
#define GM_YAW_ID					0x201
#define GM_PITCH_ID					0x202

typedef enum
{
	GIMBAL_REMOTE 	= 0,
	GIMBAL_STOP		= 1,
	GIMBAL_INIT		= 2,
	GIMBAL_TRACE	= 3,
}GimbalMode_e;

typedef struct
{
	GimbalMode_e mode;
	Motor_t GM_Pitch;
	Motor_t GM_Yaw;
}Gimbal_t;

/* ��̨��� */
extern Gimbal_t sentryGimbal;

void Gimbal_CtrlInit(Gimbal_t *gimbal);

void Gimbal_UpdateState(Gimbal_t *gimbal);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
