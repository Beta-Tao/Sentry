#ifndef _SENTRY_STRATEGY_H_
#define _SENTRY_STRATEGY_H_

#include "Master_Comm.h"

typedef enum
{
	DIS_ABOVE_5 = 0,
	DIS_BETWEEN_4_5,
	DIS_BETWEEN_3_4,
	DIS_BETWEEN_2_3,
	DIS_BETWEEN_1_2,
	DIS_BELOW_1,
}DisGrade_e;

typedef enum
{
	SHOOT_EQUI_FREQ = 0,	//等射频射击
	SHOOT_HIGH_FREQ,		//快速射击
	SHOOT_ABANDON,			//放弃射击
}ShootStrategy_e;			//该状态由视觉平台直接决定

typedef enum
{
	WHOLE				 = 0,
	HIDE_AIR_ATTACK		 = 1,
	HIDE_BRIGE_ATTACK	 = 2,
}ChassisRange_e;

typedef enum
{
	BELOW_POWER = 0,
	OVER_POWER = 1,
}ChassisState_e;

typedef enum
{
	NO_BULLET = 0,
	HAVE_BULLET,
}LoadState_e;

typedef enum
{
	NOT_OVER_SHOOT = 0,
	SHOOT_OVER_SHOOT,
}ShootState_e;

typedef struct
{
	volatile DisGrade_e disGrade;
	
	uint8_t chassisMode;
	
	uint8_t gimbalMode;
	
	uint8_t loaderMode;
	
	uint8_t shooterMode;
	
	volatile ChassisRange_e chassisRange;		//底盘移动范围
	
	volatile ChassisState_e chassisState;		//超功率标志位
	
	volatile LoadState_e loadState;		//空弹标志位
	
	volatile ShootState_e shootState;		//超热量标志位
}SentryST_t;

extern SentryST_t sentryST;

void Sentry_STInit(SentryST_t *st);

void Sentry_UpdateChassisST(SentryST_t *st);

void Sentry_UpdateGimbalST(SentryST_t *st);

void Sentry_UpdateLoaderST(SentryST_t *st);

void Sentry_UpdateShooterST(SentryST_t *st);

#endif
