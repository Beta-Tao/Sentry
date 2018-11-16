#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* ��̨������Ƴ��� */
/* ���̵�����Ƴ��� */
#define GM_VEL_MIN					-6000		//��ת��
#define GM_VEL_MAX					6000
#define GM_PITCH_MIN				-90
#define GM_PITCH_MAX				90
#define GM_YAW_MIN					180
#define GM_YAW_MAX					-180

#define GIMBAL_MOTOR_ACC			1000		//����������1ms�����Ե�λ������ÿms���ӵ�ת��
#define	GIMBAL_MOTOR_DEC			1000		//ÿms���ٵ�ת��
#define GM_REMOTE_SENSITY			50

/* ����ID */
#define GM_P_ID						0x203
#define GM_Y_ID						0x204

/* ��̨��� */
extern Motor_t GM_Pitch;
extern Motor_t GM_Yaw;

void Gimbal_CtrlInit(void);

void Gimbal_UpdateState(void);

void Gimbal_UpdateGMRef(void);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
