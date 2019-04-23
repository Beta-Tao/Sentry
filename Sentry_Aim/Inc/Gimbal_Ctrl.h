#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-10000		//即转速 rpm
#define GM_YAW_VEL_MAX			10000

#define GM_YAW_ACC				0.8
#define	GM_YAW_DEC				0.8

#define GM_PITCH_VEL_MIN		-200		//即转速 rpm
#define GM_PITCH_VEL_MAX		200

#define GM_PITCH_ACC			2
#define	GM_PITCH_DEC			10

#define GM_YAW_MAX				0
#define GM_YAW_MIN				0

#define GM_PITCH_MAX			-3.0
#define GM_PITCH_MIN			-90.0

/* 云台模式常量 */
#define GM_YAW_INIT_VEL			100.0f
#define GM_PITCH_INIT_VEL		60.0f

#define GM_YAW_DETECT_VEL			120.0f
#define GM_PITCH_DETECT_VEL			200.0f

/* 总线ID */
#define GM_YAW_ID					0x205
#define GM_PITCH_ID					0x206

typedef enum
{
	GIMBAL_STOP			= 0,
	GIMBAL_REMOTE 		= 1,
	GIMBAL_YAW_INIT		= 2,
	GIMBAL_PITCH_INIT	= 3,
	GIMBAL_DETECT		= 4,
	GIMBAL_TRACE		= 5,
	GIMBAL_DEBUG_VEL	= 6,
	GIMBAL_DEBUG_POS	= 7,
}GimbalMode_e;

typedef struct
{
	volatile GimbalMode_e mode;
	
	Motor_t GM_Pitch;
	
	Motor_t GM_Yaw;
}Gimbal_t;

/* 云台电机 */
extern Gimbal_t sentryGimbal;

void Gimbal_CtrlInit(Gimbal_t *gimbal);

void Gimbal_UpdateState(Gimbal_t *gimbal);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
