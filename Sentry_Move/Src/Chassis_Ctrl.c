#include "Chassis_Ctrl.h"
#include "Sentry_Strategy.h"
#include "Remote_Comm.h"
#include "string.h"
#include "usart.h"
#include "stdlib.h"
#include "gpio.h"

double dedogePos = 0.0f;

Chassis_t sentryChassis;

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
					  10, 0, 0, 									//kp, ki, kd 20 1.0 1.5
					  5.47394);
	chassis->CM_Left.posCtrl.posRatio = 38.91459;
	
	right->motorType = M_3508;
	right->escType = C620;
	Motor_VelCtrlInit(right, 
					  CHASSIS_ACC, CHASSIS_DEC, 			//acc, dec
					  10, 0, 0,									//kp, ki, kd 20 1.0 1.5
					  5.47394);
	chassis->CM_Right.posCtrl.posRatio = 38.91459;
	
	chassis->mode = CHASSIS_STOP;
	chassis->chassisDir = LEFT;
	
//	memset(&(chassis->chassisDis), 0, sizeof(chassis->chassisDis));
//	chassis->chassisDis.revDis = 200;
	
	//TIM5_CaptureInit();
}

void Chassis_UpdateState(Chassis_t *chassis)
{
	/* 根据遥控器数据更新状态 */
	switch (RemoteComm.RemoteData.remote.s1)
	{
		case RC_SW_UP:		//当s1在上时，为自动模式
			sentryChassis.mode = (ChassisMode_e)sentryST.chassisMode;
			//sentryChassis.mode = CHASSIS_DETECT_NORMAL;
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
	
	/* 如果是巡逻模式则通过测距模块判断边界 */
	switch (chassis->mode)
	{
		case CHASSIS_DETECT_FAST:
		case CHASSIS_DETECT_LOW:
		case CHASSIS_DETECT_NORMAL:
		case CHASSIS_DODGE:
			switch (chassis->chassisDir)
			{
				case LEFT:
//					if (chassis->chassisDis.leftDis <= chassis->chassisDis.revDis)
//						count++;
//					else
//						count = 0;
//					
//					if (count >= 15)
//					{
//						count = 0;
//						chassis->chassisDir = RIGHT;
//					}
					if (Chassis_IsReverse(LEFT) == 1)
						chassis->chassisDir = RIGHT;
					break;
				case RIGHT:
					if (Chassis_IsReverse(RIGHT) == 1)
						chassis->chassisDir = LEFT;
					break;
				default:
					break;
			}
			break;
		default:
			break;
	}
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
	uint8_t num = rand() % 3 + 1;
	static uint32_t tick = 0, lastTick = 0;
	
	if (motor != &(sentryChassis.CM_Left) && motor != &(sentryChassis.CM_Right))
		return;
	
	switch (sentryChassis.mode)				//根据运动模式改变底盘速度
	{
		case CHASSIS_REMOTE:		//遥控模式则底盘速度和遥控器通道数值线性相关
			Motor_SetVel(&(motor->velCtrl), 
						(float)(-(RemoteComm.RemoteData.remote.ch2 - RC_CH_VALUE_OFFSET)) / RC_CH_VALUE_RANGE * CM_VEL_MAX);	//根据实际左右变换正负号
			break;
		case CHASSIS_DETECT_FAST:		//低速巡逻，不超功率
			Motor_SetVel(&(motor->velCtrl), 
						CHASSIS_DETECT_FAST_VEL * ((int8_t)sentryChassis.chassisDir - 1));	//巡逻速度
			break;
		case CHASSIS_DETECT_NORMAL:		//高速巡逻，超功率
			Motor_SetVel(&(motor->velCtrl), 
						CHASSIS_DETECT_NORMAL_VEL * ((int8_t)sentryChassis.chassisDir - 1));	//巡逻速度
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
				if (tick - lastTick >= num * 1000)
				{
					num = rand() % 3 + 1;
					sentryChassis.chassisDir = (ChassisDir_e)(rand() % 2 * 2);
					tick = 0;
					lastTick = 0;
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

uint8_t Chassis_IsReverse(ChassisDir_e dir)
{
	static uint32_t trigTick = 0, lastTrigTick = 0;
	GPIO_PinState pinState = GPIO_PIN_SET;
	
	switch (dir)
	{
		case LEFT:
			pinState = HAL_GPIO_ReadPin(leftEdgeTrig_GPIO_Port, leftEdgeTrig_Pin);
			break;
		case RIGHT:
			pinState = HAL_GPIO_ReadPin(rightEdgeTrig_GPIO_Port, rightEdgeTrig_Pin);
			break;
		default:
			break;
	}
	
	if (pinState == GPIO_PIN_RESET)
	{
		if (lastTrigTick == 0)						//第一次被触发
			lastTrigTick = HAL_GetTick();			//记录第一次被触发的时间
		else										//已经触发过一次
		{
			trigTick = HAL_GetTick();				//记录当前时间
			if ((trigTick - lastTrigTick) >= 5)		//被触发超过5ms
			{
				trigTick = 0;
				lastTrigTick = 0;
				return 1;
			}
		}
	}
	else
	{
		lastTrigTick = 0;		//未被触发时清零
		trigTick = 0;
	}
	return 0;
}

//void Chassis_GetDistance(TIM_HandleTypeDef *htim, Chassis_t *chassis)
//{
//	static uint32_t leftCapValStart = 0, leftCapValEnd = 0, rightCapValStart = 0, rightCapValEnd = 0;
//	static uint8_t isLeftCapHigh = 0, isRightCapHigh = 0;
//	
//	if (htim->Instance->SR & 0x02)		//通道一出现捕获事件，即左边传感器
//	{
//		if (isLeftCapHigh == 1)
//		{
//			leftCapValEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
//			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
//			
//			if (leftCapValEnd < leftCapValStart)
//				chassis->chassisDis.leftDis = 
//					(float)(leftCapValEnd + htim->Init.Period - leftCapValStart) / 10;
//			else
//				chassis->chassisDis.leftDis = 
//					(float)(leftCapValEnd - leftCapValStart) / 10;
//			
//			isLeftCapHigh = 0;
//		}
//		else
//		{
//			leftCapValStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
//			isLeftCapHigh = 1;
//			
//			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
//			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
//		}
//	}
//	if (htim->Instance->SR & 0x04)		//通道二出现捕获事件，即右边传感器
//	{
//		if (isRightCapHigh == 1)
//		{
//			rightCapValEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
//			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
//			
//			if (rightCapValEnd < rightCapValStart)
//				chassis->chassisDis.rightDis = (float)(rightCapValEnd + htim->Init.Period - rightCapValStart) / 10;
//			else
//				chassis->chassisDis.rightDis = (float)(rightCapValEnd - rightCapValStart) / 10;
//			
//			isRightCapHigh = 0;
//		}
//		else
//		{
//			rightCapValStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
//			isRightCapHigh = 1;
//			
//			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
//			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
//		}
//	}
//}
