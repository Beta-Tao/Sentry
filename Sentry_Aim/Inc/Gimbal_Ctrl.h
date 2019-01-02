#ifndef _GIMBAL_CTRL_H_
#define _GIMBAL_CTRL_H_

#include "Motor_Ctrl.h"

/* 云台电机控制常量 */
#define GM_YAW_VEL_MIN			-800		//即转速 度/s
#define GM_YAW_VEL_MAX			800

#define GM_YAW_MOTOR_ACC		12
#define	GM_YAW_MOTOR_DEC		12

#define GM_PITCH_VEL_MIN		-300		//即转速 度/s
#define GM_PITCH_VEL_MAX		300

#define GM_PITCH_MOTOR_ACC		8
#define	GM_PITCH_MOTOR_DEC		8
 
/* 总线ID */
#define GM_YAW_ID					0x201
#define GM_PITCH_ID					0x202

/* 云台电机 */
extern Motor_t GM_Pitch;
extern Motor_t GM_Yaw;

void Gimbal_CtrlInit(void);

void Gimbal_UpdateRawValue(void);

void Gimbal_MotorCtrl(Motor_t *motor);

#endif
