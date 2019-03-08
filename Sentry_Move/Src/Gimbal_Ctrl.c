#include "Gimbal_Ctrl.h"

Gimbal_t sentryGimbal;

/**
  * @brief	底盘控制初始化
  * @note	底盘控制模式以及底盘电机初始化
  * @retval	None
  */
void Gimbal_CtrlInit(Gimbal_t *gimbal)
{
	Motor_t *yawMotor  = &(gimbal->GM_Yaw), 
			*pitchMotor = &(gimbal->GM_Pitch);
	//底盘电机及电调类型初始化，底盘电机只需要速度闭环
	yawMotor->motorType = M_3508;
	yawMotor->escType = C620;
	Motor_PosCtrlInit(yawMotor,
					  GM_YAW_ACC,
					  0.1, 0, 0,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, 1310.77901);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  20, 0.5, 0 								//kp, ki, kd 10 0.5 0
					  );

	pitchMotor->motorType = M_2006;
	pitchMotor->escType = C610;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC,
					  10.2, 0, 1000,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, 22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, 			//acc, dec
					  6, 0.3, 0		 									//kp, ki, kd
					  );
					  
	gimbal->mode = GIMBAL_INIT;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	switch (gimbal->mode)
	{
		case GIMBAL_INIT:
			break;
		default:
			break;
	}
}

/**
  * @brief	云台电机控制
  * @note	云台电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	if (motor != &(sentryGimbal.GM_Yaw) && motor != &(sentryGimbal.GM_Pitch))
		return;
	
	switch (sentryGimbal.mode)
	{
		case GIMBAL_STOP:							//停止状态则保持静止
			Motor_SetVel(&(motor->velCtrl), 0);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_REMOTE:							//遥控模式则进行速度闭环
			//期望速度信息直接在Master_Comm.c中进行更新
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_TRACE:
			//位置状态直接在Master_Comm.c中进行更新
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			break;
	}
}
