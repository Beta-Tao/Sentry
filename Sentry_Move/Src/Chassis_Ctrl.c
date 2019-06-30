#include "Chassis_Ctrl.h"
#include "Sentry_Strategy.h"
#include "Remote_Comm.h"
#include "Referee_Comm.h"
#include "string.h"
#include "usart.h"
#include "stdlib.h"
#include "time.h"
#include "gpio.h"

Chassis_t sentryChassis;

uint8_t leftAboveTrig = 0, rightAboveTrig = 0;
ChassisDir_e lastDir = LEFT;
GPIO_PinState lastLeftAbovePinState = GPIO_PIN_SET, leftAbovePinState = GPIO_PIN_SET;
GPIO_PinState lastRightAbovePinState = GPIO_PIN_SET, rightAbovePinState = GPIO_PIN_SET;

float debugVel = 1150.0f;

/**
  * @brief	底盘控制初始化
  * @note	底盘控制模式以及底盘电机初始化
  * @retval	None
  */
void Chassis_CtrlInit(Chassis_t *chassis)
{
	Motor_t *left = &(chassis->CM_Left), *right = &(chassis->CM_Right);
	
	//底盘电机及电调类型初始化，底盘电机只需要速度闭环
	left->motorType = M_3508;
	left->escType = C620;
	Motor_VelCtrlInit(left, 
					  CHASSIS_ACC, CHASSIS_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  20, 0, 0, 									//kp, ki, kd 20 1.0 1.5
					  5.47394);
	chassis->CM_Left.posCtrl.posRatio = 38.91459;
	
	right->motorType = M_3508;
	right->escType = C620;
	Motor_VelCtrlInit(right, 
					  CHASSIS_ACC, CHASSIS_DEC, 			//acc, dec
					  20, 0, 0,									//kp, ki, kd 20 1.0 1.5
					  5.47394);
	chassis->CM_Right.posCtrl.posRatio = 38.91459;
	
	chassis->mode = CHASSIS_STOP;
	chassis->chassisDir = LEFT;
	
	chassis->chassisPos = MID;
	
	memset((void *)(&chassis->chassisTrig), 0, 2 * sizeof(uint8_t));
}

void Chassis_UpdateState(Chassis_t *chassis)
{
	/* 根据遥控器数据更新状态 */
	switch (RemoteComm.RemoteData.remote.s1)
	{
		case RC_SW_UP:		//当s1在上时，为自动模式
			sentryChassis.mode = (ChassisMode_e)sentryST.chassisMode;
			//sentryChassis.mode = CHASSIS_DETECT_NORMAL;
			//sentryChassis.mode = CHASSIS_DODGE;
			break;
		case RC_SW_MID:		//当s1在中时，为遥控模式
			sentryChassis.mode = CHASSIS_REMOTE;
			break;
		case RC_SW_DOWN:	//当s1在下时，为停止模式
			sentryChassis.mode = CHASSIS_STOP;
			break;
		default:
			break;
	}
	
	Chassis_IsTrig(chassis);
}

/**
  * @brief	底盘电机控制
  * @note	底盘电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @note	底盘只需要速度闭环
  * @retvel	None
  */
void Chassis_MotorCtrl(Motor_t *motor)
{
	static uint8_t num = 1;
	static uint32_t tick = 0, lastTick = 0;
	
	if (motor != &(sentryChassis.CM_Left) && motor != &(sentryChassis.CM_Right))
		return;
	
	switch (sentryChassis.mode)				//根据运动模式改变底盘速度
	{
		case CHASSIS_REMOTE:		//遥控模式则底盘速度和遥控器通道数值线性相关
			Motor_SetVel(&(motor->velCtrl), 
						(float)(-(RemoteComm.RemoteData.remote.ch2 - RC_CH_VALUE_OFFSET)) / RC_CH_VALUE_RANGE * CM_VEL_MAX);	//根据实际左右变换正负号
			if (RemoteComm.RemoteData.remote.ch2 > RC_CH_VALUE_OFFSET)
				sentryChassis.chassisDir = RIGHT;
			if (RemoteComm.RemoteData.remote.ch2 < RC_CH_VALUE_OFFSET)
				sentryChassis.chassisDir = LEFT;
			break;
		case CHASSIS_DETECT_FAST:		//低速巡逻，不超功率
			Motor_SetVel(&(motor->velCtrl), 
						CHASSIS_DETECT_FAST_VEL * ((int8_t)sentryChassis.chassisDir - 1));	//巡逻速度
			break;
		case CHASSIS_DETECT_NORMAL:		//高速巡逻，超功率
			Motor_SetVel(&(motor->velCtrl), 
						/*CHASSIS_DETECT_NORMAL_VEL*/debugVel * ((int8_t)sentryChassis.chassisDir - 1));	//巡逻速度
			break;
		case CHASSIS_DETECT_LOW:		//高速巡逻，超功率
			Motor_SetVel(&(motor->velCtrl), 
						CHASSIS_DETECT_LOW_VEL * ((int8_t)sentryChassis.chassisDir - 1));	//巡逻速度
			break;
		case CHASSIS_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);
			break;
		case CHASSIS_DODGE:				//闪避模式，向随机方向移动随机时间
			if (lastTick == 0)
				lastTick = HAL_GetTick();
			else
			{
				tick = HAL_GetTick();
				if (tick - lastTick >= num * 400)
				{
					if (RefereeData_t.PowerHeatData_t.chassis_power_buffer != 200)		//等待能量缓冲恢复
					{
						Motor_SetVel(&(motor->velCtrl), 0);
						break;
					}
					else
					{
						srand(HAL_GetTick());
						num = rand() % 3 + 2;		//至少移动两个身位
						//sentryChassis.chassisDir = (ChassisDir_e)(rand() % 2 * 2);
						sentryChassis.chassisDir = (ChassisDir_e)(abs(sentryChassis.chassisDir - 2));
						tick = 0;
						lastTick = 0;
					}
				}
			}
			Motor_SetVel(&(motor->velCtrl), CHASSIS_DODGE_VEL * ((int8_t)sentryChassis.chassisDir - 1)); 
			break;
		case CHASSIS_DEBUG_VEL:
			break;
		default:
			break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}

void Chassis_IsTrig(Chassis_t *chassis)
{
	static uint32_t trigTick[4], lastTrigTick[4];
//	static uint8_t leftAboveTrig = 0, rightAboveTrig = 0;
//	static ChassisDir_e lastDir = LEFT;
//	static GPIO_PinState lastLeftAbovePinState = GPIO_PIN_SET, leftAbovePinState = GPIO_PIN_SET;
//	static GPIO_PinState lastRightAbovePinState = GPIO_PIN_SET, rightAbovePinState = GPIO_PIN_SET;
	
	/* 发现左边缘 */
	if (HAL_GPIO_ReadPin(leftEdgeTrig_GPIO_Port, leftEdgeTrig_Pin) == GPIO_PIN_RESET)
	{
		if (lastTrigTick[0] == 0)						//第一次被触发
			lastTrigTick[0] = HAL_GetTick();			//记录第一次被触发的时间
		else										//已经触发过一次
		{
			trigTick[0] = HAL_GetTick();				//记录当前时间
			if ((trigTick[0] - lastTrigTick[0]) >= 5)		//被触发超过5ms
			{
				trigTick[0] = 0;
				lastTrigTick[0] = 0;
				
				chassis->chassisPos = LEFT_STRAIGHT;
				memset((void *)(&chassis->chassisTrig), 0, 2 * sizeof(uint8_t));
				chassis->chassisTrig.trigLeftEdge = 1;
				
				leftAboveTrig = 0;
				rightAboveTrig = 0;
			}
		}
	}
	else
	{
		lastTrigTick[0] = 0;		//未被触发时清零
		trigTick[0] = 0;
		chassis->chassisTrig.trigLeftEdge = 0;
	}
	
	/* 发现右边缘 */
	if (HAL_GPIO_ReadPin(rightEdgeTrig_GPIO_Port, rightEdgeTrig_Pin) == GPIO_PIN_RESET)
	{
		if (lastTrigTick[1] == 0)						//第一次被触发
			lastTrigTick[1] = HAL_GetTick();			//记录第一次被触发的时间
		else										//已经触发过一次
		{
			trigTick[1] = HAL_GetTick();				//记录当前时间
			if ((trigTick[1] - lastTrigTick[1]) >= 5)		//被触发超过5ms
			{
				trigTick[1] = 0;
				lastTrigTick[1] = 0;
				
				chassis->chassisPos = RIGHT_STRAIGHT;
				memset((void *)(&chassis->chassisTrig), 0, 2 * sizeof(uint8_t));
				chassis->chassisTrig.trigRightEdge = 1;
				
				leftAboveTrig = 0;
				rightAboveTrig = 0;
			}
		}
	}
	else
	{
		lastTrigTick[1] = 0;		//未被触发时清零
		trigTick[1] = 0;
		chassis->chassisTrig.trigRightEdge = 0;
	}
	
	/* 螺母判断 */
	
	leftAbovePinState = HAL_GPIO_ReadPin(leftAbove_GPIO_Port, leftAbove_Pin);
	rightAbovePinState = HAL_GPIO_ReadPin(rightAbove_GPIO_Port, rightAbove_Pin);
	
	/* 发现左螺母 */
	if (leftAbovePinState != lastLeftAbovePinState)
	{
		if (lastTrigTick[2] == 0)						//第一次被触发
			lastTrigTick[2] = HAL_GetTick();			//记录第一次被触发的时间
		else										//已经触发过一次
		{
			trigTick[2] = HAL_GetTick();				//记录当前时间
			if ((trigTick[2] - lastTrigTick[2]) >= 5)		//被触发超过5ms
			{
				trigTick[2] = 0;
				lastTrigTick[2] = 0;
				
				lastLeftAbovePinState = leftAbovePinState;
				
				if (leftAboveTrig == 0)					//首次计数
				{
					leftAboveTrig++;
					
					if (rightAboveTrig == 0)
						lastDir = chassis->chassisDir;				//记录方向
					else
					{
						if (lastDir != chassis->chassisDir)
							rightAboveTrig = 0;
					}
				}
				else
				{
					if (lastDir == chassis->chassisDir)
						leftAboveTrig++;					//方向相同则增加记录一次
					else
						leftAboveTrig--;					//方向不同则消去记录一次
				}
			}
		}
	}
	else
	{
		lastTrigTick[2] = 0;		//未被触发时清零
		trigTick[2] = 0;
	}
	
	/* 发现右螺母 */
	if (rightAbovePinState != lastRightAbovePinState)
	{
		if (lastTrigTick[3] == 0)						//第一次被触发
			lastTrigTick[3] = HAL_GetTick();			//记录第一次被触发的时间
		else										//已经触发过一次
		{
			trigTick[3] = HAL_GetTick();				//记录当前时间
			if ((trigTick[3] - lastTrigTick[3]) >= 5)		//被触发超过5ms
			{
				trigTick[3] = 0;
				lastTrigTick[3] = 0;
				
				lastRightAbovePinState = rightAbovePinState;
				
				if (rightAboveTrig == 0)					//首次计数
				{
					rightAboveTrig++;
					
					if (leftAboveTrig == 0)
						lastDir = chassis->chassisDir;				//记录方向
					else
					{
						if (lastDir != chassis->chassisDir)
							leftAboveTrig = 0;
					}
				}
				else
				{
					if (lastDir == chassis->chassisDir)
						rightAboveTrig++;					//方向相同则增加记录一次
					else
						rightAboveTrig--;					//方向不同则消去记录一次
				}
			}
		}
	}
	else
	{
		lastTrigTick[3] = 0;		//未被触发时清零
		trigTick[3] = 0;
	}
	
	/* 错误触发 */
	if ((rightAboveTrig == 1) && (leftAboveTrig > 0) && (leftAboveTrig < 4))
		leftAboveTrig = 0;
	
	if ((leftAboveTrig == 1) && (rightAboveTrig > 0) && (rightAboveTrig < 4))
		rightAboveTrig = 0;
	
	/* 整车都经过螺母 */
	if ((rightAboveTrig >= 4) && (leftAboveTrig >= 4))
	{
		rightAboveTrig = 0;
		leftAboveTrig = 0;
		
		switch (chassis->chassisDir)
		{
			case RIGHT:
				switch (chassis->chassisPos)
				{
					case RIGHT_STRAIGHT:
						chassis->chassisPos = RIGHT_STRAIGHT;
						break;
					case RIGHT_CURVE:
					case MID:
					case LEFT_CURVE:
					case LEFT_STRAIGHT:
						chassis->chassisPos++;
						break;
					default:
						break;
				}
				break;
			case LEFT:
				switch (chassis->chassisPos)
				{
					case RIGHT_STRAIGHT:
					case RIGHT_CURVE:
					case MID:
					case LEFT_CURVE:
						chassis->chassisPos--;
						break;
					case LEFT_STRAIGHT:
						chassis->chassisPos = LEFT_STRAIGHT;
						break;
					default:
						break;
				}
				break;
			default:
				break;
		}
		
		memset((void *)(&chassis->chassisTrig), 0, 2 * sizeof(uint8_t));
	}
}
