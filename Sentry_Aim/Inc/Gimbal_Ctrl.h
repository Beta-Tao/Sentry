#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台控制方式 */
typedef enum
{
	IMU		= 0,
	MOTOR	= 1,
}GimbalCtrlType_e;

/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-600.0f		//即转速 rpm
#define GM_YAW_VEL_MAX			600.0f

#define GM_YAW_ACC				50.0f
#define	GM_YAW_DEC				30.0f

#define GM_PITCH_VEL_MIN		-600.0f		//即转速 rpm
#define GM_PITCH_VEL_MAX		600.0f

#define GM_PITCH_ACC			10.0f
#define	GM_PITCH_DEC			10.0f

#define GM_YAW_MAX				0.0f
#define GM_YAW_MIN				0.0f

#define GM_PITCH_MAX			0.0f
#define GM_PITCH_MIN			-50.0f

/* 基于电机安装位置的常量 */
#define GM_YAW_OFFSET			4578u
#define GM_PITCH_OFFSET			2741u

/* 云台模式常量 */
#define GM_YAW_INIT_VEL			110.0f
#define GM_PITCH_INIT_VEL		60.0f

#define GM_YAW_DETECT_VEL			180.0f
#define GM_PITCH_DETECT_VEL			100.0f

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
	
	uint8_t isStart;		//isFirst被置位后，逼近后再进行预测
	
	float wYaw[60];
	
	float wPitch[60];
	
	float distance[60];
	
	float avgWYaw;
	
	float avgWPitch;
	
	float avgDis;
	
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
	
	volatile GimbalCtrlType_e yawCtrlType;
	
	volatile GimbalCtrlType_e pitchCtrlType;
}Gimbal_t;

/* 云台电机 */
extern Gimbal_t sentryGimbal;

void Gimbal_CtrlInit(Gimbal_t *gimbal);

void Gimbal_InitDetect(Gimbal_t *gimbal);

void Gimbal_FilterInit(Gimbal_t *gimbal);

void Gimbal_UpdateState(Gimbal_t *gimbal);

void Gimbal_MotorCtrl(Motor_t *motor);

void Gimbal_UpdateLoadMode(void);

void Gimbal_TraceForecast(Gimbal_t *gimbal);

#endif
