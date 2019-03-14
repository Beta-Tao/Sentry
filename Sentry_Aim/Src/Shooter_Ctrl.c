#include "Shooter_Ctrl.h"
#include "Master_Comm.h"
#include "tim.h"

Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter)
{
	/* 初始化TIM1定时器 */
	TIM1_PWMInit();
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	
	sentryShooter.shootVel = FM_STOP;
	sentryShooter.mode = SHOOTER_CEASE;
}

void Shooter_MotorCtrl(Shooter_t *shooter)
{
	switch (shooter->mode)
	{
		case SHOOTER_CEASE:
			Shooter_SetVel(shooter, FM_STOP);
			break;
		default:
			break;
	}
}

void Shooter_UpdateState(Shooter_t *shooter)
{
	shooter->mode = masterData.shooterMode;
	shooter->shootVel = masterData.shootVel;
}

void Shooter_SetVel(Shooter_t *shooter, uint32_t vel)
{
	shooter->shootVel = vel;
	TIM1->CCR1 = vel;
	TIM1->CCR4 = vel;
}
