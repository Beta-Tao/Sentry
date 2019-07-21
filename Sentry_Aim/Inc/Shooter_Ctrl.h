#ifndef _SHOOT_CTRL_H_
#define _SHOOT_CTRL_H_

#include "stm32f4xx_hal.h"
#include "Motor_Ctrl.h"

/* 摩擦轮电机控制常量，待测试 */
#define FM_STOP			0

#define FM_LEFT_15MPS	0
#define FM_LEFT_20MPS	0
#define FM_LEFT_25MPS	0
#define FM_LEFT_30MPS	-43000

#define FM_RIGHT_15MPS	0
#define FM_RIGHT_20MPS	0
#define FM_RIGHT_25MPS	0
#define FM_RIGHT_30MPS	43000

/* 加减速度 */
#define FM_LEFT_ACC		1000000
#define FM_LEFT_DEC		1000000
#define FM_RIGHT_ACC	1000000
#define FM_RIGHT_DEC	1000000

/* 总线ID */
#define FM_LEFT_ID		0x207
#define FM_RIGHT_ID		0x208

typedef enum
{
	SHOOTER_CEASE	= 0,
	SHOOTER_OPEN_15MPS,
	SHOOTER_OPEN_20MPS,
	SHOOTER_OPEN_25MPS,
	SHOOTER_OPEN_30MPS,
	SHOOTER_DEBUG,
}ShooterMode_e;

typedef struct
{
	uint32_t acc;
	
	uint32_t dec;
	
	Motor_t FM_LEFT;
	
	Motor_t FM_RIGHT;
	
	volatile ShooterMode_e mode;
}Shooter_t;

/* 摩擦轮电机 */
extern Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter);

void Shooter_UpdateState(Shooter_t *shooter);

void Shooter_MotorCtrl(Motor_t *motor);

void Shooter_LaserOn(void);

void Shooter_LaserOff(void);

#endif
