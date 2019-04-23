#ifndef _CHASSIS_CTRL_H_
#define _CHASSIS_CTRL_H_

#include "Motor_Ctrl.h"
#include "tim.h"

/* 底盘电机控制常量 */
#define CM_VEL_MIN				-900		//即转速
#define CM_VEL_MAX				900

#define CHASSIS_ACC		5
#define	CHASSIS_DEC		5
 
/* 总线ID */
#define CM_L_ID					0x201
#define CM_R_ID					0x202

/* 刚好不超功率的速度为 500.0f */
#define CHASSIS_DETECT_LOW_VEL			200.0f
#define CHASSIS_DETECT_NORMAL_VEL		320.0f
#define CHASSIS_DETECT_FAST_VEL			600.0f
#define CHASSIS_DODGE_VEL		320.0f

typedef enum
{
	CHASSIS_REMOTE			= 0,
	CHASSIS_STOP			= 1,  
	CHASSIS_DETECT_FAST		= 2,
	CHASSIS_DETECT_NORMAL	= 3,
	CHASSIS_DETECT_LOW		= 4,
	CHASSIS_DODGE			= 5,
	CHASSIS_DEBUG_VEL		= 6,
}ChassisMode_e;

typedef enum
{
	LEFT	= 2,
	RIGHT	= 0,
}ChassisDir_e;

//typedef struct
//{	
//	float leftDis;
//	
//	float rightDis;
//	
//	float revDis;
//}ChassisDis_t;

typedef struct
{
	volatile ChassisMode_e mode;
	
	Motor_t CM_Left;
	
	Motor_t CM_Right;
	
	volatile ChassisDir_e chassisDir;
	
	//ChassisDis_t chassisDis;
}Chassis_t;

extern Chassis_t sentryChassis;

void Chassis_CtrlInit(Chassis_t *chassis);

void Chassis_UpdateState(Chassis_t *chassis);
 
void Chassis_MotorCtrl(Motor_t *motor);

uint8_t Chassis_IsReverse(ChassisDir_e dir);

//void Chassis_GetDistance(TIM_HandleTypeDef *htim, Chassis_t *chassis);

#endif
