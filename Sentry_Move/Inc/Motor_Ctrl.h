#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "main.h"
#include "stm32f4xx.h"

/* ������Ʋ��� */
#define C620_CUR_MIN			-15000		//ʵ���ò���16384
#define C620_CUR_MAX			15000
#define C610_CUR_MIN			-15000
#define C610_CUR_MAX			15000
#define C620_POS_MIN			0			//���ص�λ��
#define C620_POS_MAX			8191u

/* ģʽ���� */
#define SENTRY_DETECT_VEL			4000

/* �ƶ�ģʽflag */
#define SENTRY_REMOTE				0u
#define SENTRY_DETECT				1u
#define SENTRY_DODGE				2u

/* ����ģʽflag */
#define SENTRY_CEASE_FIRE			0u
#define SENTRY_AIM					1u
#define SENTRY_OPEN_FIRE			2u

/* CANͨѶID */
#define FIRST_FOUR_ID				0x200
#define SECOND_FOUR_ID				0x1FF

/* ����ͺ� */
typedef enum
{
	M_3508,
	M_2006
}MotorType_m;

/* ������� */
typedef enum
{
	C620,
	C610
}ESCType_m;

/* ����ٶȿ��ƽṹ�� */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;
	
	float acc;			//�ٶȿ����м��ٵļ��ٶ�
	float dec;			//�ٶȿ����м��ٵļ��ٶ�
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;

	float output;
	float outputMin;
	float outputMax;
}VelCtrl_t;

/* ���λ�ÿ��ƽṹ�� */
typedef struct
{
	float refPos;
	float rawPos;
	
	float acc;			//λ�ÿ���ֻ�м��ٶ���Ҫ�м��ٶȣ����ٶȿ�����decһ��
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;
	
	float output;
	float outputMin;
	float outputMax;
}PosCtrl_t;

/* ����ṹ�� */
typedef struct
{
	MotorType_m motorType;
	
	ESCType_m escType;
	
	VelCtrl_t velCtrl;
	
	PosCtrl_t posCtrl;
}Motor_t;

void Motor_SetVel(VelCtrl_t *vel_t, float vel);

void Motor_SetPos(PosCtrl_t *pos_t, float pos);

void Motor_VelCtrlInit(Motor_t *motor, 
					   float acc, float dec, 
					   float kp, float ki, float kd, 
					   float outputMin, float outputMax);
					   
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax);

void Motor_PosCtrl(PosCtrl_t *pos);

void Motor_VelCtrl(VelCtrl_t *vel);

void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);

#endif
