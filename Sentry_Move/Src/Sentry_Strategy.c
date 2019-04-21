#include "Sentry_Strategy.h"
#include "Referee_Comm.h"
#include "Remote_Comm.h"
#include "Chassis_Ctrl.h"
#include "Gimbal_Ctrl.h"
#include "Loader_Ctrl.h"
#include "Shooter_Ctrl.h"
#include "string.h"

SentryST_t sentryST;

void Sentry_STInit(SentryST_t *st)
{
	st->disGrade = DIS_ABOVE_5;
	
	st->chassisMode = CHASSIS_DETECT_LOW;
	
	st->gimbalMode = GIMBAL_STOP;
	
	st->loaderMode = LOADER_STOP;
	
	st->shooterMode = SHOOTER_CEASE;
	
	st->chassisState = BELOW_POWER;
	
	st->loadState = HAVE_BULLET;
	
	st->shootState = NOT_OVER_SHOOT;
}

void Sentry_UpdateChassisST(SentryST_t *st)
{
//	if ((sentryState_t.remain_HP <= sentryState_t.max_HP * 0.3) ||
//		(PCComm.PCCommState == PC_COMM_DROP) || 
//		(st->shootState == NO_BULLET))		
//	/* 血量低于30%、PC端掉线或空弹时开启闪避模式 */
//		st->chassisMode = (st->chassisMode == CHASSIS_DETECT_FAST || st->chassisMode == CHASSIS_DETECT_LOW) ? 
//								st->chassisMode : CHASSIS_DETECT_FAST;
//	else
//	{
//		if (PCComm.PCData.isInSight == 1)
//			st->chassisMode = CHASSIS_STOP;
//	}

	/* 巡逻模式时根据功率缓冲进行高速和低速的切换 */
	if (st->chassisMode == CHASSIS_DETECT_FAST || st->chassisMode == CHASSIS_DETECT_LOW)
	{
		if (RefereeData_t.PowerHeatData_t.chassis_power_buffer <= 20)
		{
			st->chassisMode = CHASSIS_DETECT_LOW;
			st->chassisState = OVER_POWER;			//超功率
		}
		else
		{
			if (st->chassisState != OVER_POWER)				//之前没有超功率
			{
				st->chassisMode = CHASSIS_DETECT_FAST;
				st->chassisState = BELOW_POWER;
			}
			else							//之前超功率但是现在没有超，则要等待功率余量回复至0
			{
				if (RefereeData_t.PowerHeatData_t.chassis_power_buffer < 200)
				{
					st->chassisMode = CHASSIS_DETECT_LOW;
					st->chassisState = OVER_POWER;
				}
				else
				{
					st->chassisMode = CHASSIS_DETECT_FAST;
					st->chassisState = BELOW_POWER;
				}
			}
		}
	}
}

void Sentry_UpdateGimbalST(SentryST_t *st)
{
	st->gimbalMode = GIMBAL_TRACE;
}

void Sentry_UpdateLoaderST(SentryST_t *st)
{
	static uint16_t bulletCount = 0;
	
//	switch (st->shootST)
//	{
//		case SHOOT_EQUI_FREQ:		//均匀射击时冷却和热量持平
//		{
//			switch (st->shooterMode) 
//			{  
//				case SHOOTER_OPEN_15MPS:
//					st->loaderMode = LOADER_RUN_PS10;
//					break;
//				case SHOOTER_OPEN_20MPS:
//					st->loaderMode = LOADER_RUN_PS8;
//					break;
//				case SHOOTER_OPEN_25MPS:
//					st->loaderMode = LOADER_RUN_PS6;
//					break;
//				case SHOOTER_OPEN_30MPS:
//					st->loaderMode = LOADER_RUN_PS5;
//					break;
//				default:
//					break;
//			}
//			break;
//		}
//		case SHOOT_HIGH_FREQ:
//		{
//			if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 450)
//				st->loaderMode = LOADER_RUN_PS20;
//			else
//				st->loaderMode = LOADER_RUN_PS3;
//			break;
//		}
//		case SHOOT_ABANDON:
//		{
//			st->loaderMode = LOADER_STOP;
//			break;
//		}
//		default:
//			break;
//	}
	if ((st->loaderMode == LOADER_RUN_PS3) || (st->loaderMode == LOADER_RUN_PS5) ||
		(st->loaderMode == LOADER_RUN_PS6) || (st->loaderMode == LOADER_RUN_PS8) ||
		(st->loaderMode == LOADER_RUN_PS10) || (st->loaderMode == LOADER_RUN_PS20))
	{
		if (RefereeData_t.ShootData_t.bullet_speed <= 1.0f)		//没有子弹输出
			bulletCount++;
		else													//有数据时清空
		{
			bulletCount = 0;
			st->loadState = HAVE_BULLET;
			memset((void *)&(RefereeData_t.ShootData_t), 0, sizeof(RefereeData_t.ShootData_t));
		}
		
		if (bulletCount >= 5000)
		{
			bulletCount = 0;
			st->loadState = NO_BULLET;
		}
	}
	
	if (st->loadState == HAVE_BULLET)
	{
		if (PCComm.PCData.isTraced == 1)
		{
			switch (st->disGrade)
			{
				case DIS_BETWEEN_4_5:
				case DIS_BETWEEN_3_4:
					st->loaderMode = LOADER_RUN_PS5;
					break;
				case DIS_BETWEEN_2_3:
				case DIS_BETWEEN_1_2:
				case DIS_BELOW_1:
					if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 450)
					{
						if (st->shootState == NOT_OVER_SHOOT)			//当前小于450且之前没有超热量
							st->loaderMode = LOADER_RUN_PS10;
						else
						{
							if (RefereeData_t.PowerHeatData_t.shooter_heat0 == 0)
							{
								st->loaderMode = LOADER_RUN_PS10;
								st->shootState = NOT_OVER_SHOOT;
							}
							else
							{
								st->loaderMode = LOADER_RUN_PS3;
								st->shootState = SHOOT_OVER_SHOOT;
							}
						}
					}
					else
					{
						st->loaderMode = LOADER_RUN_PS3;
						st->shootState = SHOOT_OVER_SHOOT;
					}
					break;
				default:
					st->loaderMode = LOADER_STOP;
					break;
			}
		
			if (RefereeData_t.PowerHeatData_t.shooter_heat0 > 470)
				st->loaderMode = LOADER_STOP;
		}
		else
			st->loaderMode = LOADER_STOP;
	}
	else
		st->loaderMode = LOADER_STOP;
}

void Sentry_UpdateShooterST(SentryST_t *st)
{	
	/* 根据不同情况更新哨兵运动策略 */
	if (st->loadState == HAVE_BULLET)
	{
		if (st->disGrade != DIS_ABOVE_5)
		{
			switch (st->disGrade)
			{
				case DIS_ABOVE_5:
				case DIS_BETWEEN_4_5:
					st->shooterMode = SHOOTER_OPEN_30MPS;
					break;
				case DIS_BETWEEN_3_4:
					st->shooterMode = SHOOTER_OPEN_25MPS;
					break;
				case DIS_BETWEEN_2_3:
					st->shooterMode = SHOOTER_OPEN_20MPS;
					break;
				case DIS_BETWEEN_1_2:
				case DIS_BELOW_1:
					st->shooterMode = SHOOTER_OPEN_15MPS;
					break;
				default:
					break;
			}
		}
		else
			st->shooterMode = SHOOTER_CEASE;
	}
	else
		st->shooterMode = SHOOTER_CEASE;
}

void Dis_Reflush(uint8_t *data, uint8_t num, uint8_t len)
{
	uint8_t i;
	
	for (i = 0; i < len; i++)
	{
		if (i != num)
			*(data + i) = 0;
	}
}

void Sentry_GetDisGrade(SentryST_t *st)
{
	static uint8_t disCount[6] = {0};
	float dis = PCComm.PCData.distance;
	
//	if ((PCComm.PCData.isInSight) == 0)					//保证在看见的时候才有距离等级
//		st->disGrade = DIS_ABOVE_5;
//	else
//	{
		if (dis > 5000.0f)
			DisGrade_Update(DIS_ABOVE_5);
		else if ((dis > 4000.0f) && (dis < 5000.0f)) 
			DisGrade_Update(DIS_BETWEEN_4_5);
		else if ((dis > 3000.0f) && (dis < 4000.0f))
			DisGrade_Update(DIS_BETWEEN_3_4);
		else if ((dis > 2000.0f) && (dis < 3000.0f))
			DisGrade_Update(DIS_BETWEEN_2_3);
		else if ((dis > 1000.0f) && (dis < 2000.0f))
			DisGrade_Update(DIS_BETWEEN_1_2);
		else if ((dis < 1000.0f) && (dis > 0.0f))
			DisGrade_Update(DIS_BELOW_1);
		else
			return;
//	}
}
