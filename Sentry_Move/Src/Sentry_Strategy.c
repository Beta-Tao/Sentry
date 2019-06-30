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
	
	st->shooterMode = SHOOTER_OPEN_20MPS;
	
	st->chassisRange = WHOLE;
	
	st->chassisState = BELOW_POWER;
	
	st->loadState = HAVE_BULLET;
	
	st->shootState = NOT_OVER_SHOOT;
}

void Sentry_UpdateChassisST(SentryST_t *st)
{
	if (RefereeData_t.PowerHeatData_t.chassis_power_buffer <= 20)
		st->chassisState = OVER_POWER;
	else if (RefereeData_t.PowerHeatData_t.chassis_power_buffer == 200)
		st->chassisState = BELOW_POWER;
	else
		st->chassisState = st->chassisState;
	
	if ((sentryState_t.remain_HP <= (sentryState_t.max_HP * 0.3)) ||
		(PCRxComm.PCRxCommState == PC_COMM_DROP) || 
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
//		if (PCRxComm.PCData.isInSight == 1)
//			st->chassisMode = CHASSIS_DETECT_LOW;
//		else
//		{
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
//		}
	}
	
	/* 根据裁判系统数据更新哨兵巡逻范围 */
//	if ((RefereeData_t.otherRobotData_t.data_cmd_id == 0x200) && 
//		(RefereeData_t.otherRobotData_t.send_ID == (sentryState_t.side * 10 + 6)) && 
//		(RefereeData_t.otherRobotData_t.receiver_ID == (sentryState_t.side * 10 + 7)))
//		st->chassisRange = (ChassisRange_e)RefereeData_t.otherRobotData_t.flag;
	
	switch (st->chassisRange)
	{
		case WHOLE:
			if (sentryChassis.chassisTrig.trigLeftEdge == 1)
				sentryChassis.chassisDir = RIGHT;
			else if ((sentryChassis.chassisPos == RIGHT_CURVE) || 
						(sentryChassis.chassisPos == RIGHT_STRAIGHT))
				sentryChassis.chassisDir = LEFT;
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
	//static uint32_t tick = 0, lastTick = 0;
	
	if (PCRxComm.PCRxCommState == PC_COMM_DROP)
	{
		st->gimbalMode = GIMBAL_DETECT_WHOLE;
	}
	else
	{
		if (PCRxComm.PCData.isInSight == 1)
			st->gimbalMode = GIMBAL_TRACE;
		else
		{
			if (sentryState_t.isAttacked == 1)
			{
				if (RefereeData_t.RobotHurt_t.armor_id == 0x0)
					st->gimbalMode = GIMBAL_DETECT_BACK;
				else if (RefereeData_t.RobotHurt_t.armor_id == 0x1)
					st->gimbalMode = GIMBAL_DETECT_AHEAD;
				else
					st->gimbalMode = GIMBAL_DETECT_WHOLE;
			}
			else
				st->gimbalMode = GIMBAL_DETECT_WHOLE;
//			if (st->gimbalMode == GIMBAL_TRACE)		//丢失目标
//			{
//				if (lastTick == 0)
//					lastTick = HAL_GetTick();
//				else
//				{
//					tick = HAL_GetTick();
//					if (tick - lastTick >= 1500)
//					{
//						st->gimbalMode = GIMBAL_DETECT;
//						tick = 0;
//						lastTick = 0;
//					}
//				}
//			}
//			else
//			{
//				st->gimbalMode = GIMBAL_DETECT;
//				tick = 0;
//				lastTick = 0;
//			}
		}
	}
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
	if ((masterTxData.loaderMode == LOADER_RUN_PS3) || (masterTxData.loaderMode == LOADER_RUN_PS5) ||
		(masterTxData.loaderMode == LOADER_RUN_PS6) || (masterTxData.loaderMode == LOADER_RUN_PS8) ||
		(masterTxData.loaderMode == LOADER_RUN_PS10) || (masterTxData.loaderMode == LOADER_RUN_PS15) || 
		(masterTxData.loaderMode == LOADER_RUN_PS20))
	{
		if (RefereeData_t.ShootData_t.bullet_speed <= 1.0f)		//没有子弹输出
			bulletCount++;
		else													//有数据时清空
		{
			bulletCount = 0;
			st->loadState = HAVE_BULLET;
			memset((void *)&(RefereeData_t.ShootData_t), 0, sizeof(RefereeData_t.ShootData_t));
		}
		
		if (bulletCount >= 8000)
		{
			bulletCount = 0;
			st->loadState = NO_BULLET;
		} 
	}
	
	if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 360)
		st->shootState = NOT_OVER_SHOOT;
	else					//超过360开始均匀击打
		st->shootState = SHOOT_OVER_SHOOT;
	
	if (st->loadState == HAVE_BULLET)
	{
		if ((((masterRxData.yawErr < 3.0f) && (masterRxData.yawErr > -3.0f)) && 
			((masterRxData.pitchErr < 3.0f) && (masterRxData.pitchErr > -3.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 0) && 
			(PCRxComm.PCData.distance < 3.5f))
		{
			if (st->shootState == NOT_OVER_SHOOT)			//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS20;
			else
			{
				if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 420)
					st->loaderMode = LOADER_RUN_PS8;
				else
					st->loaderMode = LOADER_RUN_PS6;
			}
//			else
//			{
//				if (RefereeData_t.PowerHeatData_t.shooter_heat0 == 0)
//				{
//					st->loaderMode = LOADER_RUN_PS20;
//					st->shootState = NOT_OVER_SHOOT;
//				}
//				else
//				{
//					st->loaderMode = LOADER_RUN_PS3;
//					st->shootState = SHOOT_OVER_SHOOT;
//				}
//			}
		}
		else if ((((masterRxData.yawErr < 3.0f) && (masterRxData.yawErr > -3.0f)) && 
			((masterRxData.pitchErr < 3.0f) && (masterRxData.pitchErr > -3.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 0) && 
			(PCRxComm.PCData.distance > 3.5f) && (PCRxComm.PCData.distance < 6.0f))
		{
			if (st->shootState == NOT_OVER_SHOOT)			//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS10;
			else
			{
				if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 420)
					st->loaderMode = LOADER_RUN_PS8;
				else
					st->loaderMode = LOADER_RUN_PS6;
			}
		}
		else if ((((masterRxData.yawErr < 2.0f) && (masterRxData.yawErr > -2.0f)) && 
			((masterRxData.pitchErr < 2.0f) && (masterRxData.pitchErr > -2.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 0) && 
			(PCRxComm.PCData.distance > 6.5f))
		{
			if (st->shootState == NOT_OVER_SHOOT)			//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS5;
		}
		else if ((((masterRxData.yawErr < 5.0f) && (masterRxData.yawErr > -5.0f)) && 
			((masterRxData.pitchErr < 5.0f) && (masterRxData.pitchErr > -5.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 1) && 
			(PCRxComm.PCData.distance < 3.5f))
		{
			if (st->shootState == NOT_OVER_SHOOT)	 		//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS20;
			else
			{
				if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 420)
					st->loaderMode = LOADER_RUN_PS8;
				else
					st->loaderMode = LOADER_RUN_PS6;
			}
		}
		else if ((((masterRxData.yawErr < 4.0f) && (masterRxData.yawErr > -4.0f)) && 
			((masterRxData.pitchErr < 4.0f) && (masterRxData.pitchErr > -4.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 1) && 
			(PCRxComm.PCData.distance > 3.5f) && (PCRxComm.PCData.distance < 6.0f))
		{
			if (st->shootState == NOT_OVER_SHOOT)	 		//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS10;
			else
			{
				if (RefereeData_t.PowerHeatData_t.shooter_heat0 <= 420)
					st->loaderMode = LOADER_RUN_PS8;
				else
					st->loaderMode = LOADER_RUN_PS6;
			}
		}
		else if ((((masterRxData.yawErr < 3.0f) && (masterRxData.yawErr > -3.0f)) && 
			((masterRxData.pitchErr < 3.0f) && (masterRxData.pitchErr > -3.0f)) && 
			(PCRxComm.PCData.isFind == 1)) && (PCRxComm.PCData.isBig == 1) && 
			(PCRxComm.PCData.distance > 6.5f))
		{
			if (st->shootState == NOT_OVER_SHOOT)	 		//当前小于450且之前没有超热量
				st->loaderMode = LOADER_RUN_PS5;
		}
		else
			st->loaderMode = LOADER_STOP;
	}
}

void Sentry_UpdateShooterST(SentryST_t *st)
{
	st->shooterMode = SHOOTER_OPEN_20MPS;
//	/* 根据不同情况更新哨兵运动策略 */
//	if (st->loadState == HAVE_BULLET)
//	{
//		if (st->disGrade != DIS_ABOVE_5)
//		{
//			switch (st->disGrade)
//			{
//				case DIS_ABOVE_5:
//				case DIS_BETWEEN_4_5:
//					st->shooterMode = SHOOTER_OPEN_30MPS;
//					break;
//				case DIS_BETWEEN_3_4:
//					st->shooterMode = SHOOTER_OPEN_25MPS;
//					break;
//				case DIS_BETWEEN_2_3:
//					st->shooterMode = SHOOTER_OPEN_20MPS;
//					break;
//				case DIS_BETWEEN_1_2:
//				case DIS_BELOW_1:
//					st->shooterMode = SHOOTER_OPEN_15MPS;
//					break;
//				default:
//					break;
//			}
//		}
//		else
//			st->shooterMode = SHOOTER_CEASE;
//	}
//	else
//		st->shooterMode = SHOOTER_CEASE;
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
	float dis = PCRxComm.PCData.distance;
	
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
