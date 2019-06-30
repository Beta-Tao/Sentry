#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

/* 摩擦轮电机控制常量，待测试 */
#define FM_STOP		1000 - 1
#define FM_15MPS	1150
#define FM_20MPS	1200
#define FM_25MPS	1250
#define FM_30MPS	1290

typedef enum
{
	SHOOTER_CEASE	= 0,
	SHOOTER_OPEN_15MPS,
	SHOOTER_OPEN_20MPS,
	SHOOTER_OPEN_25MPS,
	SHOOTER_OPEN_30MPS,
}ShooterMode_e;

typedef struct
{
	uint32_t acc;
	
	uint32_t dec;
	
	uint32_t vel;
	
	volatile ShooterMode_e mode;
}Shooter_t;

/* 摩擦轮电机 */
extern Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter);

void Shooter_UpdateState(Shooter_t *shooter);

void Shooter_MotorCtrl(Shooter_t *shooter);

void Shooter_SetVel(Shooter_t *shooter, uint32_t vel);

void Shooter_LaserOn(void);

#endif
