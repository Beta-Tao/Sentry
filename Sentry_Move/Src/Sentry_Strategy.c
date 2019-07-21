#include "Sentry_Strategy.h"
#include "Referee_Comm.h"
#include "Remote_Comm.h"
#include "Chassis_Ctrl.h"
#include "Gimbal_Ctrl.h"
#include "Loader_Ctrl.h"
#include "Shooter_Ctrl.h"
#include "string.h"
#include "math.h"

SentryST_t sentryST;

void Sentry_STInit(SentryST_t *st)
{
	st->disGrade = DIS_BETWEEN_3_4;
	
	st->chassisMode = CHASSIS_STOP;
	
	st->gimbalMode = GIMBAL_DETECT_WHOLE;
	
	st->loaderMode = LOADER_STOP;
	
	st->shooterMode = SHOOTER_CEASE;
	
	st->chassisRange = WHOLE;
	
	st->chassisState = BELOW_POWER;
	
	st->loadState = HAVE_BULLET;
	
	st->shootState = NOT_OVER_SHOOT;
}

void Sentry_UpdateChassisST(SentryST_t *st)
{
	static uint32_t tick = 0, lastTick = 0;
	static uint16_t lastHP = 0;
	
	if (RefereeData_t.PowerHeatData_t.chassis_power_buffer <= 20)
		st->chassisState = OVER_POWER;
	else if (RefereeData_t.PowerHeatData_t.chassis_power_buffer == 200)
		st->chassisState = BELOW_POWER;
	else
		st->chassisState = st->chassisState;
	
	if ((sentryState_t.remain_HP <= (sentryState_t.max_HP * 0.3)) || 
		(st->loadState == NO_BULLET))
	/* 血量低于30%、PC端掉线或空弹时开启闪避模式 */
	{
//		st->chassisMode = (st->chassisMode == CHASSIS_DETECT_FAST || st->chassisMode == CHASSIS_DETECT_LOW) ? 
//								st->chassisMode : CHASSIS_DETECT_FAST;
		//st->chassisMode = CHASSIS_DODGE;
		switch (st->chassisState)
		{
			case OVER_POWER:
				st->chassisMode = CHASSIS_DETECT_LOW;
				break;
			case BELOW_POWER:
				st->chassisMode = CHASSIS_DETECT_NORMAL;
				break;
				//st->chassisMode = CHASSIS_DODGE;
			default:
				break;
		}
	}
	else
	{
		if (masterRxData.isInsight == 1)
		{
			if (lastTick == 0)		//第一次发现目标
			{
				lastTick = HAL_GetTick();
				st->chassisMode = CHASSIS_DETECT_LOW;
				lastHP = sentryState_t.remain_HP;
			}
			else
			{
				tick = HAL_GetTick();
				if (tick - lastTick <= 20000)		//20s以内的判断
				{
					if (lastHP - sentryState_t.remain_HP > 100)		//20s以内受到100点伤害
						st->chassisMode = CHASSIS_DETECT_NORMAL;
					else
						st->chassisMode = CHASSIS_DETECT_LOW;
				}
				else								//20s以后重新开始判断
				{
					lastTick = 0;
					tick = 0;
					st->chassisMode = CHASSIS_DETECT_LOW;
				}
			}
		}
		else
		{
			switch (st->chassisState)
			{
				case OVER_POWER:
					st->chassisMode = CHASSIS_DETECT_LOW;
					break;
				case BELOW_POWER:
					st->chassisMode = CHASSIS_DETECT_NORMAL;
					break;
				default:
					break;
			}
		}
	}
	
	/* 根据裁判系统数据更新哨兵巡逻范围 */
//	if ((RefereeData_t.otherRobotData_t.data_cmd_id == 0x200) && 
//		(RefereeData_t.otherRobotData_t.send_ID == (sentryState_t.side * 10 + 6)) && 
//		(RefereeData_t.otherRobotData_t.receiver_ID == (sentryState_t.side * 10 + 7)))
//		st->chassisRange = (ChassisRange_e)RefereeData_t.otherRobotData_t.flag;
	
	switch (st->chassisRange)
	{
		case WHOLE:
			if (sentryChassis.CM_Right.posCtrl.absPos > 2500000.0f)		//左边快到边缘
			{
				st->chassisMode = CHASSIS_DETECT_LOW;
				if (sentryChassis.chassisTrig.trigLeftEdge == 1)
				{
					if (RefereeData_t.PowerHeatData_t.chassis_power_buffer <= 100)
					{
						sentryChassis.chassisDir = NONE;
					}
					else
					{
						sentryChassis.chassisDir = RIGHT;
						sentryChassis.chassisTrig.trigLeftEdge = 0;
						sentryChassis.CM_Right.posCtrl.absPos = 0.0f;	//刷新
						st->chassisMode = CHASSIS_DETECT_NORMAL;
					}
				}
			}
//			else if ((sentryChassis.chassisPos == RIGHT_CURVE) || 
//						(sentryChassis.chassisPos == RIGHT_STRAIGHT))
//				sentryChassis.chassisDir = LEFT;
			else if (sentryChassis.CM_Right.posCtrl.absPos < -2500000.0f)
			{
				st->chassisMode = CHASSIS_DETECT_LOW;
				if (sentryChassis.chassisTrig.trigRightEdge == 1)
				{
					if (RefereeData_t.PowerHeatData_t.chassis_power_buffer <= 100)
					{
						sentryChassis.chassisDir = NONE;
					}
					else
					{
						sentryChassis.chassisDir = LEFT;
						sentryChassis.chassisTrig.trigRightEdge = 0;
						sentryChassis.CM_Right.posCtrl.absPos = 0.0f;
						st->chassisMode = CHASSIS_DETECT_NORMAL;
					}
				}
			}
			else
				break;
			break;
		case HIDE_AIR_ATTACK:
			switch (sentryChassis.chassisPos)
			{
				case LEFT_STRAIGHT:
				case LEFT_CURVE:
					sentryChassis.chassisDir = RIGHT;
					break;
				case RIGHT_STRAIGHT:
				case RIGHT_CURVE:
					sentryChassis.chassisDir = LEFT;
					break;
				default:
					break;
			}
			break;
		case HIDE_BRIGE_ATTACK:
			switch (sentryChassis.chassisPos)
			{
				case RIGHT_STRAIGHT:
					if (sentryChassis.chassisTrig.trigRightEdge == 1)
						sentryChassis.chassisDir = LEFT;
					break;
				default:
					sentryChassis.chassisDir = RIGHT;
					break;
			}
			break;
		default:
			break;
	}
}

void Sentry_UpdateGimbalST(SentryST_t *st)
{
//	if (sentryState_t.isAttacked == 1)
//	{
//		if (RefereeData_t.RobotHurt_t.armor_id == 0x0)
//			st->gimbalMode = GIMBAL_DETECT_BACK;
//		else if (RefereeData_t.RobotHurt_t.armor_id == 0x1)
//			st->gimbalMode = GIMBAL_DETECT_AHEAD;
//		else
//			st->gimbalMode = GIMBAL_DETECT_WHOLE;
//	}
//	else
		st->gimbalMode = GIMBAL_DETECT_WHOLE;
}

void Sentry_UpdateLoaderST(SentryST_t *st)
{
//	if (RefereeData_t.BulletRemaining_t.bullet_remaining_num != 0)		//没有发射完500发
//		st->loadState = HAVE_BULLET;
//	else
//		st->loadState = NO_BULLET;
	
	if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 360)
		st->shootState = NOT_OVER_SHOOT;
	else					//超过360开始顶着热量打击
		st->shootState = SHOOT_OVER_SHOOT;
	
	if (st->loadState == HAVE_BULLET)
	{
		switch (st->shootState)
		{
			case NOT_OVER_SHOOT:
				st->loaderMode = (LoaderMode_e)masterRxData.loaderMode;
				break;
			case SHOOT_OVER_SHOOT:
				if ((masterRxData.loaderMode == LOADER_RUN_PS8) ||
					(masterRxData.loaderMode == LOADER_RUN_PS10) ||
					(masterRxData.loaderMode == LOADER_RUN_PS15) ||
					(masterRxData.loaderMode == LOADER_RUN_PS20))
				{
					if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 420)
						st->loaderMode = LOADER_RUN_PS8;
					else
						st->loaderMode = LOADER_RUN_PS5;
				}
				else
					st->loaderMode = (LoaderMode_e)masterRxData.loaderMode;
				break;
			default:
				st->loaderMode = LOADER_STOP;
				break;
		}
	}
	else
		st->loaderMode = LOADER_STOP;
}

void Sentry_UpdateShooterST(SentryST_t *st)
{
	st->shooterMode = SHOOTER_OPEN_30MPS;
}
