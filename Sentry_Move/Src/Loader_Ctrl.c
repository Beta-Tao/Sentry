#include "Loader_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t LM;		//供弹电机

float g_LoadVel;			//供弹速度
uint8_t g_LoaderMode;		//是否瞄准决定是否开始供弹

/**
  * @brief	供弹控制初始化
  * @note	供弹电机初始化 
  * @retval	None
  */
void Loader_CtrlInit(void)
{
	//供弹电机及电调类型初始化
	LM.motorType = M_2006;
	LM.escType = C610;
	Motor_VelCtrlInit(&LM, 
					  LOADER_MOTOR_ACC, LOADER_MOTOR_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  20, 1.0, 1.5 						//kp, ki, kd
					  );
	
	g_LoaderMode = SENTRY_LOAD_RUN;
	g_LoadVel = -4000;				//负数送弹
}

/**
  * @brief	更新供弹模式
  * @note	在供弹电机总线接收中断中
  * @retvel	None
  */
void Loader_UpdateState(Motor_t *motor)
{
	static uint8_t jamCount = 0, covCount = 0;			//判断是否卡弹的计数位，避免误测以及启动转动时的误判
	switch(g_LoaderMode)
	{
		case SENTRY_LOAD_RUN:							//供弹状态
			if (motor->velCtrl.refVel != 0 && 
				motor->velCtrl.rawVel <= 20 && motor->velCtrl.rawVel >= -20)
										//供弹模式下期望转速不为0但实际转速较小，判断为堵转状态
			{
				jamCount++;
				if (jamCount >= 5)
				{
					g_LoaderMode = SENTRY_LOAD_JAM;
					jamCount = 0;
				}
				break;
			}
			else
				jamCount = 0;
			break;
		case SENTRY_LOAD_JAM:							//卡弹状态
			covCount++;								//开始反转计数
			if (covCount >= 200)						//反转达到十个周期，则反转结束
			{
				g_LoaderMode = SENTRY_LOAD_RUN;
				covCount = 0;
				break;
			}
			break;
		default:									//停转状态
			break;
	}
}

/**
  * @brief	供弹电机控制
  * @note	供弹电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @retvel	None
  */
void Loader_MotorCtrl(Motor_t *motor)
{	
	switch (g_LoaderMode)
	{
		case SENTRY_LOAD_RUN:				//供弹模式
			Motor_SetVel(&(motor->velCtrl), g_LoadVel);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case SENTRY_LOAD_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);	
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case SENTRY_LOAD_JAM:				//堵转模式，则开始反转
			Motor_SetVel(&(motor->velCtrl), 500);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			 break;
	}
}
