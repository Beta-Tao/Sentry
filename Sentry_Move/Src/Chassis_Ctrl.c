#include "Chassis_Ctrl.h"
#include "Remote_Comm.h"
#include "string.h"
#include "usart.h"

Chassis_t sentryChassis;

float detectVel = 2400;

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
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
	
	right->motorType = M_3508;
	right->escType = C620;
	Motor_VelCtrlInit(right, 
					  CHASSIS_ACC, CHASSIS_DEC, 			//acc, dec
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
	
	chassis->mode = CHASSIS_STOP;
	chassis->chassisDir = RIGHT;
	
	memset(&(chassis->chassisDis), 0, sizeof(chassis->chassisDis));
	chassis->chassisDis.revDis = 300;
}

void Chassis_UpdateState(Chassis_t *chassis)
{
	static uint8_t count = 0;
	
	switch (chassis->chassisDir)
	{
		case LEFT:
			if (chassis->chassisDis.leftDis <= chassis->chassisDis.revDis)
				count++;
			else
				count = 0;
			
			if (count >= 10)
			{
				count = 0;
				chassis->chassisDir = RIGHT;
			}
			break;
		case RIGHT:
			if (chassis->chassisDis.rightDis <= chassis->chassisDis.revDis)
				count++;
			else
				count = 0;
			
			if (count >= 5)
			{
				count = 0;
				chassis->chassisDir = LEFT;
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
	if (motor != &(sentryChassis.CM_Left) && motor != &(sentryChassis.CM_Right))
		return;
	
	if (sentryChassis.mode == CHASSIS_DEBUG_VEL)
		__HAL_UART_DISABLE_IT(&huart1, UART_IT_IDLE);
	else
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	
	switch (sentryChassis.mode)				//根据运动模式改变底盘速度
	{
		case CHASSIS_REMOTE:		//遥控模式则底盘速度和遥控器通道数值线性相关
			Motor_SetVel(&(motor->velCtrl), 
						(float)(-(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET)) / RC_CH_VALUE_RANGE * CM_VEL_MAX);	//根据实际左右变换正负号
			break;
		case CHASSIS_DETECT:		//巡逻模式则底盘速度为当前的巡逻速度值
			Motor_SetVel(&(motor->velCtrl), detectVel * sentryChassis.chassisDir);	//巡逻速度
			break;
		case CHASSIS_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);
			break;
		case CHASSIS_DEBUG_VEL:
			break;
		default:
			break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}

void Chassis_GetDistance(TIM_HandleTypeDef *htim, Chassis_t *chassis)
{
	static uint32_t leftCapValStart = 0, leftCapValEnd = 0, rightCapValStart = 0, rightCapValEnd = 0;
	static uint8_t isLeftCapHigh = 0, isRightCapHigh = 0;
	
	if (htim->Instance->SR & 0x02)		//通道一出现捕获事件，即左边传感器
	{
		if (isLeftCapHigh == 1)
		{
			leftCapValEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_RISING);
			
			if (leftCapValEnd < leftCapValStart)
				chassis->chassisDis.leftDis = 
					(float)(leftCapValEnd + htim->Init.Period - leftCapValStart) / 10;
			else
				chassis->chassisDis.leftDis = 
					(float)(leftCapValEnd - leftCapValStart) / 10;
			
			isLeftCapHigh = 0;
		}
		else
		{
			leftCapValStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			isLeftCapHigh = 1;
			
			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1);
			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_ICPOLARITY_FALLING);
		}
	}
	if (htim->Instance->SR & 0x04)		//通道二出现捕获事件，即右边传感器
	{
		if (isRightCapHigh == 1)
		{
			rightCapValEnd = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_RISING);
			
			if (rightCapValEnd < rightCapValStart)
				chassis->chassisDis.rightDis = (float)(rightCapValEnd + htim->Init.Period - rightCapValStart) / 10;
			else
				chassis->chassisDis.rightDis = (float)(rightCapValEnd - rightCapValStart) / 10;
			
			isRightCapHigh = 0;
		}
		else
		{
			rightCapValStart = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			isRightCapHigh = 1;
			
			TIM_RESET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2);
			TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_2, TIM_ICPOLARITY_FALLING);
		}
	}
}
