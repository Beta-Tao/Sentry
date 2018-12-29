#ifndef _LOADER_CTRL_H_
#define _LOADER_CTRL_H_

#include "Motor_Ctrl.h"

/* 供弹电机控制常量 */
#define LM_VEL_MIN				-10000			//即转速
#define LM_VEL_MAX				10000

#define LOADER_MOTOR_ACC		300
#define	LOADER_MOTOR_DEC		300

/* 总线ID */
#define LM_ID					0x203

/* 底盘电机 */
extern Motor_t LM;			//Loader Motor

extern float g_LoadVel;		//供弹速度
extern uint8_t g_LoaderMode;

void Loader_CtrlInit(void);

void Loader_UpdateState(Motor_t *motor);

void Loader_MotorCtrl(Motor_t *motor);

#endif
