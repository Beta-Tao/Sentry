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
	Motor_PosCtrlInit(&LM, 
					  LOADER_MOTOR_DEC,
					  0, 0, 0, 						//kp, ki, kd
					  LM_VEL_MIN, LM_VEL_MAX			//outputMin, outputMax
					  );
	
	g_LoaderMode = LOADER_STOP;
	g_LoadVel = -1000;
}

/**
  * @brief	更新供弹模式
  * @note	在供弹电机总线接收中断中
  * @retvel	None
  */
void Loader_UpdateState(Motor_t *motor)
{
	switch(g_LoaderMode)
	{
		case LOADER_RUN:							//供弹状态
			if (motor->velCtrl.refVel != 0 && motor->velCtrl.rawVel == 0)
										//供弹模式下望转速不为0但实际转速为0，判断为堵转状态
			{
				g_LoaderMode = LOADER_JAM;
				break;
			}
			break;
		case LOADER_STOP:							//停止状态		
			break;
		case LOADER_JAM:							//卡弹状态
			if (motor->posCtrl.posReady == POS_CTRL_READY)	//堵转模式位置到达预期，则继续供弹
			{
				g_LoaderMode = LOADER_RUN;
				break;
			}
			break;
		default:
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
		case LOADER_RUN:				//供弹模式
			Motor_SetVel(&(LM.velCtrl), g_LoadVel);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_STOP:
			Motor_SetVel(&(LM.velCtrl), 0);	
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_JAM:				//堵转模式，则位置闭环
			Motor_SetPos(&(LM.posCtrl), LOADER_MOTOR_REVPOS);
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			break;
	}
}
