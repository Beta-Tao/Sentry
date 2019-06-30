#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "main.h"
#include "stm32f4xx.h"
#include "can.h"
#include "DataScope_DP.h"

/* ������Ʋ��� */
#define C620_CUR_MIN			-11000		//ʵ���ò���16384
#define C620_CUR_MAX			11000
#define C610_CUR_MIN			-11000
#define C610_CUR_MAX			11000
#define GM6020_VOL_MIN			-20000
#define GM6020_VOL_MAX			20000

#define C620_POS_MIN			0			//���ص�λ��
#define C620_POS_MAX			8191
#define C620_POS_RANGE			C620_POS_MAX - C620_POS_MIN
#define C610_POS_MIN			0			//���ص�λ��
#define C610_POS_MAX			8191
#define C610_POS_RANGE			C610_POS_MAX - C610_POS_MIN
#define GM6020_POS_MIN			0
#define GM6020_POS_MAX			8191
#define GM6020_POS_RANGE		GM6020_POS_MAX - GM6020_POS_MIN

/* ģʽ���� */
#define SENTRY_DETECT_VEL			4000

/* CANͨѶID */
#define FIRST_FOUR_ID				0x200
#define SECOND_FOUR_ID				0x1FF
 
/* ����ͺ� */
typedef enum
{
	M_3508,
	M_2006,
	GM6020,
	SNAIL_2305,
}MotorType_e;

/* ������� */
typedef enum
{
	C620,
	C610,
	M_820R,
	GM_6020,
	SNAIL_430R,
}ESCType_e;

typedef enum
{
	ABS		= 0,
	RELA	= 1,
}PosCtrlType_e;

typedef struct
{
	float rawCur;
}CurCtrl_t;

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
	
	float velRatio;		//λ��ϵ������ʵ����ת��Ϊ���Ʊ�����ϵ��
}VelCtrl_t;

/* ���λ�ÿ��ƽṹ�� */
typedef struct
{
	float absPos;		//��ǰ����λ�ã���
	
	float refRelaPos;	//�������λ�ã���
	float detaPos;		//���η��������ת��
	
	float rawPos;		//��ǰλ�ã�ת�Ӷ���
	float rawPosLast;	//��һ�ε�λ�ã�ת�Ӷ���
	
	float posRange;		//������ص�λ�÷�Χ
	
	float posMax;		//ʵ��λ�õ����ֵ
	float posMin;		//ʵ��λ�õ���Сֵ
	
	float acc;			//λ�ÿ���ֻ�м��ٶ���Ҫ�м��ٶȣ����ٶȿ�����decһ��
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;
	
	float output;
	float outputMin;
	float outputMax;
	
	float posRatio;		//λ��ϵ������ʵ����ת��Ϊ���Ʊ�����ϵ��
}PosCtrl_t;

/* ����ṹ�� */
typedef struct
{
	MotorType_e motorType;
	
	ESCType_e escType;
	
	CurCtrl_t curCtrl;
	
	VelCtrl_t velCtrl;
	
	PosCtrl_t posCtrl;
}Motor_t;

void Motor_SetVel(VelCtrl_t *vel_t, float vel);

void Motor_SetPos(PosCtrl_t *pos_t, float pos, uint8_t type);

void Motor_VelCtrlInit(Motor_t *motor, 
					   float acc, float dec, 
					   float kp, float ki, float kd, float ratio);
					   
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax, float posMax, float posMin, float ratio);

void Motor_PosCtrl(PosCtrl_t *pos_t);

void Motor_VelCtrl(VelCtrl_t *vel_t);

void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor);
					   
void Motor_UpdatePosCtrl(PosCtrl_t *pos_t);

void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num,
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg);
									   
#endif
