#include "Loader_Ctrl.h"
#include "gpio.h"
#include "Master_Comm.h"

Loader_t sentryLoader;

/**
  * @brief	供弹控制初始化
  * @note	供弹电机初始化 
  * @retval	None
  */
void Loader_CtrlInit(Loader_t *loader)
{
	//供弹电机及电调类型初始化
	loader->LM.motorType = M_2006;
	loader->LM.escType = C610;
	Motor_VelCtrlInit(&(loader->LM), 
					  LOADER_ACC, LOADER_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  9, 0.4, 0,  						//kp, ki, kd
					  6.00000);
	
	loader->mode = LOADER_INIT;
}

/**
  * @brief	更新供弹模式
  * @note	在供弹电机总线接收中断中
  * @retvel	None
  */
void Loader_UpdateState(Loader_t *loader)
{
	loader->mode = ((loader->mode == LOADER_INIT) || 
					(loader->mode == LOADER_JAM) ? loader->mode : (LoaderMode_e)masterRxData.loaderMode);
	
	static uint32_t covCount = 0;			//判断是否卡弹的计数位，避免误测以及启动转动时的误判
	switch(loader->mode)
	{
		case LOADER_RUN_PS3:
		case LOADER_RUN_PS5:
		case LOADER_RUN_PS6:
		case LOADER_RUN_PS8:
		case LOADER_RUN_PS10:
		case LOADER_RUN_PS15:
		case LOADER_RUN_PS20:
			if (Loader_IsJammed(loader) == 1)
			{
				loader->lastMode = loader->mode;		//暂存之前的状态
				loader->mode = LOADER_JAM;
			}
			break;
		case LOADER_JAM:							//卡弹状态
			covCount++;								//开始反转计数
			if (covCount >= 300)						//反转达到十个周期，则反转结束
			{
				loader->mode = loader->lastMode;
				covCount = 0;
			}
			break;
		case LOADER_INIT:				//添加判断堵转情况
			if (Loader_IsJammed(loader) == 1)
			{
				loader->lastMode = loader->mode;
				loader->mode = LOADER_JAM;
				break;
			}
		
			if (HAL_GPIO_ReadPin(ballTrig_GPIO_Port, ballTrig_Pin) == GPIO_PIN_SET)
				loader->mode = LOADER_STOP;
			break;
		case LOADER_STOP:
//			if (HAL_GPIO_ReadPin(ballTrig_GPIO_Port, ballTrig_Pin) == GPIO_PIN_RESET)	//空弹时上弹
//				loader->mode = LOADER_INIT;
			break;
		default:									//停转状态
			break;
	}
}

/**
  * @brief	供弹电机控制
  * @note	供弹电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @note	供弹电机只需要速度闭环
  * @retvel	None
  */
void Loader_MotorCtrl(Motor_t *motor)
{	
	if (motor != &(sentryLoader.LM))
		return;
	
	switch (sentryLoader.mode)
	{
		case LOADER_INIT:
			Motor_SetVel(&(motor->velCtrl), 2.0f * LOADER_PS1);
			break;
		case LOADER_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);
			break;
		case LOADER_JAM:				//堵转模式，则开始反转
			Motor_SetVel(&(motor->velCtrl), -10.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS3:
			Motor_SetVel(&(motor->velCtrl), 3.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS5:
			Motor_SetVel(&(motor->velCtrl), 5.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS6:
			Motor_SetVel(&(motor->velCtrl), 6.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS8:
			Motor_SetVel(&(motor->velCtrl), 8.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS10:
			Motor_SetVel(&(motor->velCtrl), 10.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS15:
			Motor_SetVel(&(motor->velCtrl), 15.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS20:
			Motor_SetVel(&(motor->velCtrl), 20.0f * LOADER_PS1);
			break;
		default:
			 break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}

uint8_t Loader_IsJammed(Loader_t *loader)
{
	static uint32_t jamCount;
	if (loader->LM.velCtrl.refVel != 0 && 
				loader->LM.velCtrl.rawVel <= 20 && loader->LM.velCtrl.rawVel >= -20)
										//供弹模式下期望转速不为0但实际转速较小，判断为堵转状态
	{
		jamCount++;
		if (jamCount >= 60)
		{
			jamCount = 0;
			return 1;
		}
	}
	else
		jamCount = 0;
	return 0;
}
