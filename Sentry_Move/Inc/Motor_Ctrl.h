#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "main.h"
#include "macro.h"
#include "stm32f4xx.h"

/* ������Ʋ��� */
#define C620_CUR_MIN			-15000		//ʵ���ò���16384
#define C620_CUR_MAX			15000
#define C610_CUR_MIN			-15000
#define C610_CUR_MAX			15000

#define C620_POS_MIN			0			//���ص�λ��
#define C620_POS_MAX			8191
#define C620_POS_RANGE			C620_POS_MAX - C620_POS_MIN
#define C610_POS_MIN			0			//���ص�λ��
#define C610_POS_MAX			8191
#define C610_POS_RANGE			C610_POS_MAX - C610_POS_MIN

#define POS_CTRL_UNREADY		0u
#define POS_CTRL_READY			1u

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
	float refPos;		//λ�����������λ��
	float relaPos;		//���λ��
	
	float rawPos;		//��ǰλ��
	float rawPosLast;	//��һ�ε�λ��
	
	float acc;			//λ�ÿ���ֻ�м��ٶ���Ҫ�м��ٶȣ����ٶȿ�����decһ��
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;
	
	float output;
	float outputMin;
	float outputMax;
	
	uint8_t posReady;
}PosCtrl_t;

/* ����ṹ�� */
typedef struct
{
	MotorType_m motorType;
	
	ESCType_m escType;
	
	VelCtrl_t velCtrl;
	
	PosCtrl_t posCtrl;
}Motor_t;

extern uint8_t g_AimMode;					//��׼ģʽ��������̨
extern uint8_t g_MoveMode;					//�ƶ�ģʽ�����Ƶ���
extern uint8_t g_LoadMode;					//����ģʽ�����Ʋ���
extern uint8_t g_ShootMode;					//����ģʽ������Ħ����

void Motor_InitFlag(void);

void Motor_SetVel(VelCtrl_t *vel_t, float vel);

void Motor_SetPos(PosCtrl_t *pos_t, float pos);

void Motor_VelCtrlInit(Motor_t *motor, 
					   float acc, float dec, 
					   float kp, float ki, float kd);
					   
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax);

void Motor_PosCtrl(PosCtrl_t *pos_t);

void Motor_VelCtrl(VelCtrl_t *vel_t);

void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor);


void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num,
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg);

#endif
