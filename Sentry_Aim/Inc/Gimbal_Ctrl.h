#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-10000		//即转速 rpm
#define GM_YAW_VEL_MAX			10000

#define GM_YAW_ACC				5
#define	GM_YAW_DEC				2

#define GM_PITCH_VEL_MIN		-200		//即转速 rpm
#define GM_PITCH_VEL_MAX		200

#define GM_PITCH_ACC			5
#define	GM_PITCH_DEC			2

#define GM_YAW_MAX				0
#define GM_YAW_MIN				0

#define GM_PITCH_MAX			-3.0
#define GM_PITCH_MIN			-90.0

/* 云台模式常量 */
#define GM_YAW_INIT_VEL			110.0f
#define GM_PITCH_INIT_VEL		60.0f

#define GM_YAW_DETECT_VEL			180.0f
#define GM_PITCH_DETECT_VEL			100.0f
//yaw 110.0f pitch 120.0f

/* 总线ID */
#define GM_YAW_ID					0x205
#define GM_PITCH_ID					0x206

#define Yaw_GetCount		((int32_t)(sentryGimbal.GM_Yaw.posCtrl.absPos / 360.0f))

typedef enum
{
	LEFT	= 2,
	RIGHT	= 0,
}GimbalYawDir_e;

typedef enum
{
	UP		= 2,
	DOWN	= 0,
}GimbalPitchDir_e;

typedef enum
{
	GIMBAL_STOP			= 0,
	GIMBAL_REMOTE 		= 1,
	GIMBAL_YAW_INIT		= 2,
	GIMBAL_PITCH_INIT	= 3,
	GIMBAL_DETECT_WHOLE	= 4,
	GIMBAL_DETECT_AHEAD	= 5,
	GIMBAL_DETECT_BACK	= 6,
	GIMBAL_TRACE		= 7,
	GIMBAL_DEBUG_VEL	= 8,
	GIMBAL_DEBUG_POS	= 9,
}GimbalMode_e;

typedef struct
{
	uint8_t posReady;
	
	uint8_t yawDetectEdgeCnt;
	
	uint8_t yawOverRange;
	
	volatile GimbalYawDir_e yawDetectDir;
	
	volatile GimbalPitchDir_e pitchDetectDir;
}GimbalDetect_t;

typedef struct
{
	uint32_t lastTick;
	
	uint32_t tick;
	
	uint8_t isFirst;
	
	float wYaw[20];
	
	float wPitch[20];
	
	float avgWYaw;
	
	float avgWPitch;
	
	float lastAbsYaw;
	
	float lastAbsPitch;
	
	float yawFore;
	
	float pitchFore;
}GimbalFilter_t;

typedef struct
{
	volatile GimbalMode_e mode;
	
	GimbalDetect_t gimbalDetect;
	
	GimbalFilter_t gimbalFilter;
	
	Motor_t GM_Pitch;
	
	Motor_t GM_Yaw;
}Gimbal_t;

/* 云台电机 */
extern Gimbal_t sentryGimbal;

void Gimbal_CtrlInit(Gimbal_t *gimbal);

void Gimbal_InitDetect(Gimbal_t *gimbal);

void Gimbal_FilterInit(Gimbal_t *gimbal);

void Gimbal_UpdateState(Gimbal_t *gimbal);

void Gimbal_UpdateMasterTxData(Gimbal_t *gimbal);

void Gimbal_MotorCtrl(Motor_t *motor);

void Gimbal_TraceForecast(Gimbal_t *gimbal);

#endif
