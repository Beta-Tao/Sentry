#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* ��̨������Ƴ��� */
#define GM_YAW_VEL_MIN			-7500		//��ת��
#define GM_YAW_VEL_MAX			7500

#define GM_YAW_MOTOR_ACC		1000
#define	GM_YAW_MOTOR_DEC		1000

#define GM_PITCH_VEL_MIN		-13000		//��ת��
#define GM_PITCH_VEL_MAX		13000

#define GM_PITCH_MOTOR_ACC		1000
#define	GM_PITCH_MOTOR_DEC		1000
 
/* ����ID */
#define GM_YAW_ID					0x201
#define GM_PITCH_ID					0x202

/* ��̨��� */
extern Motor_t GM_Pitch;
extern Motor_t GM_Yaw;

extern float refVel;

void Gimbal_CtrlInit(void);

void Gimbal_UpdateState(void);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
