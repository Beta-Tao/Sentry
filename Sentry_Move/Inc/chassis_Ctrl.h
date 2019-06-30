#ifndef _CHASSIS_CTRL_H_
#define _CHASSIS_CTRL_H_

#include "Motor_Ctrl.h"
#include "tim.h"

/* 底盘电机控制常量 */
#define CM_VEL_MIN				-900		//即转速
#define CM_VEL_MAX				900

#define CHASSIS_ACC		4
#define	CHASSIS_DEC		3

/* 总线ID */
#define CM_L_ID					0x201
#define CM_R_ID					0x202

/* 刚好不超功率的速度为 480.0f */
#define CHASSIS_DETECT_LOW_VEL			200.0f
#define CHASSIS_DETECT_NORMAL_VEL		500.0f
#define CHASSIS_DETECT_FAST_VEL			600.0f
#define CHASSIS_DODGE_VEL				700.0f

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

typedef enum
{
	LEFT_STRAIGHT	 = 0,
	LEFT_CURVE		 = 1,
	MID				 = 2,
	RIGHT_CURVE		 = 3, 
	RIGHT_STRAIGHT	 = 4,
}ChassisPos_e;

typedef struct
{
	uint8_t trigLeftEdge;
	
	uint8_t trigRightEdge;
	
}ChassisTrig_t;

typedef struct
{
	volatile ChassisMode_e mode;
	
	Motor_t CM_Left;
	
	Motor_t CM_Right;
	
	ChassisTrig_t chassisTrig;
	
	volatile ChassisDir_e chassisDir;
	
	volatile ChassisPos_e chassisPos;
}Chassis_t;

extern Chassis_t sentryChassis;

void Chassis_CtrlInit(Chassis_t *chassis);

void Chassis_UpdateState(Chassis_t *chassis);
 
void Chassis_MotorCtrl(Motor_t *motor);

void Chassis_IsTrig(Chassis_t *chassis);
#endif
