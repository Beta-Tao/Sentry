#include "Gimbal_Ctrl.h"
#include "AHRS_Update.h"

Motor_t GM_Yaw;		//Pitch轴电机
Motor_t GM_Pitch;			//Yaw轴电机

/**
  * @brief	底盘控制初始化
  * @note	底盘控制模式以及底盘电机初始化
  * @retval	None
  */
void Gimbal_CtrlInit(void)
{
	//底盘电机及电调类型初始化，底盘电机只需要速度闭环
	GM_Yaw.motorType = M_3508;
	GM_Yaw.escType = C620;
	Motor_PosCtrlInit(&GM_Yaw,
					  GM_YAW_MOTOR_ACC,
					  8, 0, 320,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX);
	Motor_VelCtrlInit(&GM_Yaw, 
					  GM_YAW_MOTOR_ACC, GM_YAW_MOTOR_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  10, 0.15, 0 								//kp, ki, kd 10 0.5 0
					  );
	 
	GM_Pitch.motorType = M_2006;
	GM_Pitch.escType = C610;
	Motor_PosCtrlInit(&GM_Pitch,
					  GM_PITCH_MOTOR_ACC,
					  10, 0, 200,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX);
	Motor_VelCtrlInit(&GM_Pitch, 
					  GM_PITCH_MOTOR_ACC, GM_PITCH_MOTOR_DEC, 			//acc, dec
					  6, 0.3, 0		 									//kp, ki, kd
					  );
}

void Gimbal_UpdateRawValue(void)
{
	GM_Yaw.posCtrl.rawPos = Gimbal_t.rawYaw;
	GM_Pitch.posCtrl.rawPos = Gimbal_t.rawPitch;
	
	/* 更新速度 */
	if (GM_Pitch.posCtrl.rawPos < -90)		//小于-90时芯片反向，Yaw轴与标准正方向相反
		GM_Yaw.velCtrl.rawVel = -Gimbal_t.rawVelZ;
	else
		GM_Yaw.velCtrl.rawVel = Gimbal_t.rawVelZ;
	
	GM_Pitch.velCtrl.rawVel = Gimbal_t.rawVelX;
}

/**
  * @brief	云台电机控制
  * @note	云台电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	switch (g_AimMode)
	{
		case SENTRY_STOP:							//停止状态则保持静止
			Motor_SetVel(&(motor->velCtrl), 0);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case SENTRY_REMOTE:							//遥控模式则进行速度闭环
		{
			/* 根据电机类型进行赋值 */
			if (motor == &GM_Yaw)
				Motor_SetVel(&(motor->velCtrl), Gimbal_t.refYawVel);
			else if (motor == &GM_Pitch)
				Motor_SetVel(&(motor->velCtrl), Gimbal_t.refPitchVel);
			else 
				break;
			
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		}
		case SENTRY_TRACE:
		{
			if (motor == &GM_Yaw)
				Motor_SetPos(&(motor->posCtrl), Gimbal_t.relaYaw);
			else if (motor == &GM_Pitch)
				Motor_SetPos(&(motor->posCtrl), Gimbal_t.relaPitch);
			else 
				break;
			
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		}
		default:									//其他状态进行位置闭环
			/*Motor_SetPos(&(GM_Yaw.posCtrl), Gimbal_t.relaYaw);
			Motor_PosCtrl(&(GM_Yaw.posCtrl));
			Motor_SetVel(&(GM_Yaw.velCtrl), GM_Yaw.posCtrl.output);
			Motor_VelCtrl(&(GM_Yaw.velCtrl));*/
			break;
	}
}
