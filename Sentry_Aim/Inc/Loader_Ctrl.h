#ifndef _LOADER_CTRL_H_
#define _LOADER_CTRL_H_

#include "Motor_Ctrl.h"

/* 供弹电机控制常量 */
#define LM_VEL_MIN				-8000			//即转速
#define LM_VEL_MAX				8000

#define LOADER_ACC				1000
#define	LOADER_DEC				1000

#define LOADER_JAM_VEL			-100
#define LOADER_INIT_VEL			45

/* 总线ID */
#define LM_ID					0x207

typedef enum
{
	LOADER_INIT			= 0,
	LOADER_STOP			= 1,
	LOADER_RUN			= 2,
	LOADER_JAM			= 3,
	LOADER_DEBUG_VEL	= 4,
}LoaderMode_e;

typedef struct
{
	LoaderMode_e mode;
	
	float loadVel;
	
	Motor_t LM;
}Loader_t;

extern Loader_t sentryLoader;

void Loader_CtrlInit(Loader_t *loader);

void Loader_UpdateState(Loader_t *loader);

void Loader_MotorCtrl(Motor_t *motor);

LoaderMode_e Loader_JudgeJam(Loader_t *loader);

#endif
