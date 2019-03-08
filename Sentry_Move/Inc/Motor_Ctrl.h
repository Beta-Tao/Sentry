#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "main.h"
#include "stm32f4xx.h"
#include "can.h"
#include "DataScope_DP.h"

/* 电调控制参数 */
#define C620_CUR_MIN			-15000		//实际用不到16384
#define C620_CUR_MAX			15000
#define C610_CUR_MIN			-15000
#define C610_CUR_MAX			15000
#define GM6020_VOL_MIN			-3000
#define GM6020_VOL_MAX			3000

#define C620_POS_MIN			0			//返回的位置
#define C620_POS_MAX			8191
#define C620_POS_RANGE			C620_POS_MAX - C620_POS_MIN
#define C610_POS_MIN			0			//返回的位置
#define C610_POS_MAX			8191
#define C610_POS_RANGE			C610_POS_MAX - C610_POS_MIN
#define GM6020_POS_MIN			0
#define GM6020_POS_MAX			8191
#define GM6020_POS_RANGE		GM6020_POS_MAX - GM6020_POS_MIN

#define POS_CTRL_UNREADY		0u
#define POS_CTRL_READY			1u

/* 模式常量 */
#define SENTRY_DETECT_VEL			4000

/* CAN通讯ID */
#define FIRST_FOUR_ID				0x200
#define SECOND_FOUR_ID				0x1FF

/* 电机型号 */
typedef enum
{
	M_3508,
	M_2006,
	GM6020,
	SNAIL_2305,
}MotorType_e;

/* 电调类型 */
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

/* 电机速度控制结构体 */
typedef struct
{
	float refVel_Soft;
	float refVel;
	float rawVel;
	
	float acc;			//速度控制中加速的加速度
	float dec;			//速度控制中减速的加速度
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;

	float output;
	float outputMin;
	float outputMax;
}VelCtrl_t;

/* 电机位置控制结构体 */
typedef struct
{
	float absPos;		//当前绝对位置，度
	
	float refRelaPos;	//期望相对位置，度
	float rawPos;		//当前位置，转子读数
	float rawPosLast;	//上一次的位置，转子读数
	
	float posRange;
	
	float acc;			//位置控制只有减速段需要有加速度，和速度控制中dec一致
	
	float kp;
	float ki;
	float kd;
	float integ;
	float err, errLast;
	
	float output;
	float outputMin;
	float outputMax;
	
	float posRatio;		//位置系数，真实变量转换为控制变量的系数
	uint8_t posReady;
}PosCtrl_t;

typedef struct
{
	float rawCur;
}CurCtrl_t;

/* 电机结构体 */
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
					   float kp, float ki, float kd);
					   
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax, float ratio);

void Motor_PosCtrl(PosCtrl_t *pos_t);

void Motor_VelCtrl(VelCtrl_t *vel_t);

void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor);

void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num,
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg);
									   
#endif
