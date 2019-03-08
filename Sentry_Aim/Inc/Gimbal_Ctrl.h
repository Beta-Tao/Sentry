#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-5000		//即转速 rpm
#define GM_YAW_VEL_MAX			5000

#define GM_YAW_ACC				2
#define	GM_YAW_DEC				2

#define GM_PITCH_VEL_MIN		-2500		//即转速 rpm
#define GM_PITCH_VEL_MAX		2500

#define GM_PITCH_ACC			2
#define	GM_PITCH_DEC			2

#define GM_YAW_INIT_VEL			150
#define GM_PITCH_INIT_VEL		1
 
/* 总线ID */
#define GM_YAW_ID					0x205
#define GM_PITCH_ID					0x206

typedef enum
{
	GIMBAL_REMOTE 		= 0,
	GIMBAL_STOP			= 1,
	GIMBAL_YAW_INIT		= 2,
	GIMBAL_PITCH_INIT	= 3,
	GIMBAL_TRACE		= 4,
	GIMBAL_DEBUG_VEL	= 5,
	GIMBAL_DEBUG_POS	= 6,
}GimbalMode_e;

typedef struct
{
	GimbalMode_e mode;
	Motor_t GM_Pitch;
	Motor_t GM_Yaw;
}Gimbal_t;

/* 云台电机 */
extern Gimbal_t sentryGimbal;

void Gimbal_CtrlInit(Gimbal_t *gimbal);

void Gimbal_UpdateState(Gimbal_t *gimbal);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
