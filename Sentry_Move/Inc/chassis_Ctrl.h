#ifndef _CHASSIS_CTRL_H_
#define _CHASSIS_CTRL_H_

#include "Motor_Ctrl.h"

/* ���̵�����Ƴ��� */
#define CM_VEL_MIN				-6000		//��ת��
#define CM_VEL_MAX				6000

#define CHASSIS_MOTOR_ACC		1000
#define	CHASSIS_MOTOR_DEC		1000
 
/* ����ID */
#define CM_L_ID					0x201
#define CM_R_ID					0x202

extern int16_t chassisVel;
extern int16_t chassisDir;

/* ���̵�� */
extern Motor_t CM_Left;
extern Motor_t CM_Right;

void Chassis_CtrlInit(void);

void Chassis_UpdateRef(void);

void Chassis_MotorCtrl(Motor_t *motor);

#endif
