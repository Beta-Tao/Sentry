#include "Shooter_Ctrl.h"
#include "Master_Comm.h"
#include "tim.h"
#include "gpio.h"

Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter)
{	
	/* 初始化TIM1定时器 */
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_4);
	
	sentryShooter.mode = SHOOTER_CEASE;
	sentryShooter.acc = 2;
	sentryShooter.dec = 2;
	
	Shooter_LaserOn();
}

void Shooter_MotorCtrl(Shooter_t *shooter)
{
//	static uint32_t tick = 0, lastTick = 0, mode = 0;

//	switch (shooter->mode)
//	{
//		case SHOOTER_CEASE:
//			mode = SHOOTER_CEASE;
//			break;
//		case SHOOTER_OPEN_15MPS:
//		case SHOOTER_OPEN_20MPS:
//		case SHOOTER_OPEN_25MPS:
//		case SHOOTER_OPEN_30MPS:
//			if (mode != shooter->mode)		//速度需要变化
//			{
//				if (lastTick != 0)
//					lastTick = HAL_GetTick();
//				else
//				{
//					tick = HAL_GetTick();
//					if (tick - lastTick >= 1000)
//					{
//						if (mode > shooter->mode)		//速度减小
//							mode--;
//						else if (mode < shooter->mode)	//速度增大
//							mode++;
//						else
//							break;
//						tick = 0;
//						lastTick = 0;
//						
//						if (mode >= SHOOTER_OPEN_30MPS)
//							mode = SHOOTER_OPEN_30MPS;
//					}
//				}
//			}
//			else
//			{
//				tick = 0;
//				lastTick = 0;
//			}
//			break;
//		default:
//			break;
//	}
	
	switch (shooter->mode)
	{
		case SHOOTER_CEASE:
			Shooter_SetVel(shooter, FM_STOP);
			break;
		case SHOOTER_OPEN_15MPS:
			Shooter_SetVel(shooter, FM_15MPS);
			break;
		case SHOOTER_OPEN_20MPS:
			Shooter_SetVel(shooter, FM_20MPS);
			break;
		case SHOOTER_OPEN_25MPS:
			Shooter_SetVel(shooter, FM_25MPS);
			break;
		case SHOOTER_OPEN_30MPS:
			Shooter_SetVel(shooter, FM_30MPS);
			break;
		default:
			break;
	}
}

void Shooter_UpdateState(Shooter_t *shooter)
{
	shooter->mode = (ShooterMode_e)masterRxData.shooterMode;
}

void Shooter_SetVel(Shooter_t *shooter, uint32_t vel)
{
	static uint32_t tick = 0, lastTick = 0;
	ShooterMode_e mode = SHOOTER_CEASE;
	
	if ((mode == SHOOTER_CEASE) && (shooter->mode != SHOOTER_CEASE))
	{
		TIM1->CCR1 = vel;
		if (lastTick == 0)
			lastTick = HAL_GetTick();
		else
		{
			tick = HAL_GetTick();
			if (tick - lastTick >= 300)
			{
				TIM1->CCR4 = vel;
				mode = shooter->mode;
				tick = 0;
				lastTick = 0;
			}
		}
	}
	else
	{
		TIM1->CCR1 = vel;
		TIM1->CCR4 = vel;
		mode = shooter->mode;
		tick = 0;
		lastTick = 0;
	}
}

void Shooter_LaserOn(void)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
}
