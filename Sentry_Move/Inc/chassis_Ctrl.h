#ifndef _CHASSIS_CTRL_H_
#define _CHASSIS_CTRL_H_

#include "Motor_Ctrl.h"
#include "tim.h"

/* 底盘电机控制常量 */
#define CM_VEL_MIN				-10000		//即转速
#define CM_VEL_MAX				10000

#define CHASSIS_ACC		500
#define	CHASSIS_DEC		500
 
/* 总线ID */
#define CM_L_ID					0x201
#define CM_R_ID					0x202

#define CHASSIS_DETECT_VEL		2000

typedef enum
{
	CHASSIS_REMOTE		= 0,
	CHASSIS_STOP		= 1,
	CHASSIS_DETECT		= 2,
	CHASSIS_DODGE		= 3,
	CHASSIS_DEBUG_VEL	= 4,
}ChassisMode_e;

typedef enum
{
	LEFT	= -1,
	RIGHT	= 1,
}ChassisDir_e;

typedef struct
{	
	float leftDis;
	
	float rightDis;
	
	float revDis;
}ChassisDis_t;

typedef struct
{
	ChassisMode_e mode;
	
	Motor_t CM_Left;
	
	Motor_t CM_Right;
	
	ChassisDir_e chassisDir;
	
	ChassisDis_t chassisDis;
}Chassis_t;

extern Chassis_t sentryChassis;

void Chassis_CtrlInit(Chassis_t *chassis);

void Chassis_UpdateState(Chassis_t *chassis);
 
void Chassis_MotorCtrl(Motor_t *motor);

void Chassis_GetDistance(TIM_HandleTypeDef *htim, Chassis_t *chassis);

#endif
