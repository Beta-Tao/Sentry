#ifndef _MOTOR_CTRL_H_
#define _MOTOR_CTRL_H_

#include "main.h"
#include "macro.h"
#include "stm32f4xx.h"

/* 电调控制参数 */
#define C620_CUR_MIN			-15000		//实际用不到16384
#define C620_CUR_MAX			15000
#define C610_CUR_MIN			-15000
#define C610_CUR_MAX			15000

#define C620_POS_MIN			0			//返回的位置
#define C620_POS_MAX			8191
#define C620_POS_RANGE			C620_POS_MAX - C620_POS_MIN
#define C610_POS_MIN			0			//返回的位置
#define C610_POS_MAX			8191
#define C610_POS_RANGE			C610_POS_MAX - C610_POS_MIN

#define POS_CTRL_UNREADY		0u
#define POS_CTRL_READY			1u

/* CAN通讯ID */
#define FIRST_FOUR_ID				0x200
#define SECOND_FOUR_ID				0x1FF

/* 电机型号 */
typedef enum
{
	M_3508,
	M_2006
}MotorType_m;

/* 电调类型 */
typedef enum
{
	C620,
	C610
}ESCType_m;

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
	float refPos;		//位置期望是相对位置
	float relaPos;		//相对位移
	
	float rawPos;		//当前位置
	float rawPosLast;	//上一次的位置
	
	float acc;			//位置控制只有减速段需要有加速度，和速度控制中dec一致
	
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

/* 电机结构体 */
typedef struct
{
	MotorType_m motorType;
	
	ESCType_m escType;
	
	VelCtrl_t velCtrl;
	
	PosCtrl_t posCtrl;
}Motor_t;

extern uint8_t g_AimMode;					//瞄准模式，控制云台
extern uint8_t g_MoveMode;					//移动模式，控制底盘
extern uint8_t g_LoadMode;					//供弹模式，控制拨盘
extern uint8_t g_ShootMode;					//发射模式，控制摩擦轮

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
