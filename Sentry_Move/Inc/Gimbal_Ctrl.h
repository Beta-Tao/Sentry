#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台电机控制常量 */
/* 底盘电机控制常量 */
#define GM_VEL_MIN					-6000		//即转速
#define GM_VEL_MAX					6000
#define GM_PITCH_MIN				-90
#define GM_PITCH_MAX				90
#define GM_YAW_MIN					180
#define GM_YAW_MAX					-180

#define GIMBAL_MOTOR_ACC			1000		//控制周期是1ms，所以单位意义是每ms增加的转速
#define	GIMBAL_MOTOR_DEC			1000		//每ms减少的转速
#define GM_REMOTE_SENSITY			50

/* 总线ID */
#define GM_P_ID						0x203
#define GM_Y_ID						0x204

/* 云台电机 */
extern Motor_t GM_Pitch;
extern Motor_t GM_Yaw;

void Gimbal_CtrlInit(void);

void Gimbal_UpdateState(void);

void Gimbal_UpdateGMRef(void);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
