#ifndef _LOADER_CTRL_H_
#define _LOADER_CTRL_H_

#include "Motor_Ctrl.h"

/* 供弹电机控制常量 */
#define LM_VEL_MIN				-1000			//即转速
#define LM_VEL_MAX				1000 

#define LOADER_ACC				4
#define	LOADER_DEC				4

#define LOADER_PS1				45

/* 总线ID */
#define LM_ID					0x203

typedef enum
{
	LOADER_STOP			= 0,
	LOADER_JAM,
	LOADER_RUN_PS1,
	LOADER_RUN_PS3,
	LOADER_RUN_PS5,
	LOADER_RUN_PS6,
	LOADER_RUN_PS8,
	LOADER_RUN_PS10,
	LOADER_RUN_PS15,
	LOADER_RUN_PS20,
	LOADER_DEBUG_VEL,
}LoaderMode_e;

typedef struct
{
	volatile LoaderMode_e lastMode;		//上一个状态，用于堵转后恢复之前状态
	
	volatile LoaderMode_e mode;
	
	Motor_t LM;
}Loader_t;

extern Loader_t sentryLoader;

void Loader_CtrlInit(Loader_t *loader);

void Loader_UpdateState(Loader_t *loader);

void Loader_MotorCtrl(Motor_t *motor);

uint8_t Loader_IsJammed(Loader_t *loader);

#endif
