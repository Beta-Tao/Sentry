#include "Gimbal_Ctrl.h"
#include "Master_Comm.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
#include "string.h"

//float p1 = 0.22, i1 = 0, d1 = 1000;		//pitchPos
float p1 = 0.0048, i1 = 0.0000705, d1 = 5000;
float p2 = 0.006, i2 = 0.0000705, d2 = 5000;
float p3 = 0.3, i3 = 0, d3 = 0;
float p4 = 0.4, i4 = 0, d4 = 20;
//float p1 = 0.0282, i1 = 0.0000705, d1 = 6.9;
//float p2 = 0.468, i2 = 0, d2 = 5;
Gimbal_t sentryGimbal;
//float pitchDetectVel = 10.0f;
//float yawDetectVel = 0;

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
					  0.008, 0, 0,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, GM_YAW_MAX, GM_YAW_MIN, 1310.77901);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  6, 0.03, 0, 								//kp, ki, kd 10 0.5 0
					  9.60160);

	pitchMotor->motorType = GM6020;
	pitchMotor->escType = GM_6020;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC,
					  0.6, 0, 15, 
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, GM_PITCH_MAX, GM_PITCH_MIN, 22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, 			//acc, dec
					  220, 8, 0,		 									//kp, ki, kd
					  0.16667);

	gimbal->mode = GIMBAL_YAW_INIT;
	
	Gimbal_InitDetect(gimbal);
	Gimbal_FilterInit(gimbal);
}

void Gimbal_InitDetect(Gimbal_t *gimbal)
{
	gimbal->gimbalDetect.yawOverRange = 0;
	gimbal->gimbalDetect.posReady = 0;
	gimbal->gimbalDetect.yawDetectDir = LEFT;
	gimbal->gimbalDetect.yawDetectEdgeCnt = 0;
}

void Gimbal_FilterInit(Gimbal_t *gimbal)
{
	gimbal->gimbalFilter.lastTick = HAL_GetTick();
	gimbal->gimbalFilter.tick = gimbal->gimbalFilter.lastTick;
	memset((void *)gimbal->gimbalFilter.wYaw, 0, sizeof(float) * 20);
	memset((void *)gimbal->gimbalFilter.wPitch, 0, sizeof(float) * 20);
	gimbal->gimbalFilter.lastAbsYaw = 0.0f;
	gimbal->gimbalFilter.lastAbsPitch = 0.0f;
	gimbal->gimbalFilter.yawFore = 0.0f;
	gimbal->gimbalFilter.pitchFore = 0.0f;
}

void Gimbal_UpdateMasterTxData(Gimbal_t *gimbal)
{
	masterTxData.yaw = gimbal->GM_Yaw.posCtrl.absPos;
	masterTxData.pitch = gimbal->GM_Pitch.posCtrl.absPos;
	masterTxData.yawErr = gimbal->GM_Yaw.posCtrl.refRelaPos;
	masterTxData.pitchErr = gimbal->GM_Pitch.posCtrl.refRelaPos;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	static uint8_t trigCount = 0;
	switch (gimbal->mode)
	{
		case GIMBAL_DETECT_AHEAD:
			if (masterRxData.gimbalMode == GIMBAL_TRACE ||
				masterRxData.gimbalMode == GIMBAL_REMOTE //||
				/*masterRxData.gimbalMode == GIMBAL_DETECT_BACK*/)
			{
				Gimbal_InitDetect(gimbal);
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
				break;
			}

			if (gimbal->gimbalDetect.yawDetectEdgeCnt > 3)
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
			break;
		case GIMBAL_DETECT_BACK:
			if (masterRxData.gimbalMode == GIMBAL_TRACE ||
				masterRxData.gimbalMode == GIMBAL_REMOTE //||
				/*masterRxData.gimbalMode == GIMBAL_DETECT_AHEAD*/)
			{
				Gimbal_InitDetect(gimbal);
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
				break;
			}
			
			if (gimbal->gimbalDetect.yawDetectEdgeCnt > 3)
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
			break;
		case GIMBAL_YAW_INIT:
			if (HAL_GPIO_ReadPin(YawInit_GPIO_Port, YawInit_Pin) == GPIO_PIN_RESET)
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount >= 3)
			{
				gimbal->mode = GIMBAL_PITCH_INIT;
				gimbal->GM_Yaw.posCtrl.absPos = 0;
				gimbal->GM_Yaw.posCtrl.refRelaPos = 0;
				trigCount = 0;
			}
			break;
		case GIMBAL_PITCH_INIT:
			if (gimbal->GM_Pitch.velCtrl.refVel != 0 && 
				(float)gimbal->GM_Pitch.velCtrl.rawVel / gimbal->GM_Pitch.velCtrl.velRatio <= 1 && 
					(float)gimbal->GM_Pitch.velCtrl.rawVel / gimbal->GM_Pitch.velCtrl.velRatio >= -1)
										//判断为堵转状态
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount >= 200)
			{ 
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
				gimbal->GM_Pitch.posCtrl.absPos = 0;
				gimbal->GM_Pitch.posCtrl.refRelaPos = 0;
				Motor_SetPos(&(gimbal->GM_Pitch.posCtrl), GM_PITCH_MAX, ABS);
				trigCount = 0;
			}
			break;
		default:
			Gimbal_InitDetect(gimbal);
			gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
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
	
	/* 更改PID参数 */
	switch (sentryGimbal.mode)
	{
		case GIMBAL_TRACE:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				if ((masterRxData.yawAngle > 2.0f) || (masterRxData.yawAngle < -2.0f))
					//Motor_SetPosPIDParam(&(motor->posCtrl), 0.007, 0.0000705, 8);
					Motor_SetPosPIDParam(&(motor->posCtrl), p1, i1, d1);
				else
					//Motor_SetPosPIDParam(&(motor->posCtrl), 0.0072, 0.0000705, 10);
					Motor_SetPosPIDParam(&(motor->posCtrl), p2, i2, d2);
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				if ((masterRxData.pitchAngle > 2.0f) || (masterRxData.pitchAngle < -2.0f))
					Motor_SetPosPIDParam(&(motor->posCtrl), p3, i3, d3);
				else
					Motor_SetPosPIDParam(&(motor->posCtrl), p4, i4, d4);
			}
			else
				break;
			break;
		default:
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetPosPIDParam(&(motor->posCtrl), 0.0282, 0.0000705, 6.9);
				//Motor_SetPosPIDParam(&(motor->posCtrl), p3, i3, d3);
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetPosPIDParam(&(motor->posCtrl), 0.468, 0, 5);
				//Motor_SetPosPIDParam(&(motor->posCtrl), p2, i2, d2);
			else
				break;
			break;
	}
	
	/* 根据模式更改控制方式 */
	switch (sentryGimbal.mode)
	{
		case GIMBAL_STOP:							//停止状态则保持静止，位置闭环
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_REMOTE:							//遥控模式则进行速度闭环
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetPos(&(motor->posCtrl), masterRxData.yawAngle, RELA);
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetPos(&(motor->posCtrl), masterRxData.pitchAngle, RELA);
			else
				break;
			
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_DETECT_WHOLE:							//云台巡逻模式
			if (motor == &(sentryGimbal.GM_Yaw))		//Yaw轴持续旋转
			{
				Motor_SetVel(&(motor->velCtrl), GM_YAW_DETECT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			if (motor == &(sentryGimbal.GM_Pitch))
			{
				if (motor->posCtrl.absPos >= -10.0f)
					sentryGimbal.gimbalDetect.pitchDetectDir = DOWN;
				if (motor->posCtrl.absPos <= -30.0f)
					sentryGimbal.gimbalDetect.pitchDetectDir = UP;
				
				Motor_SetVel(&(motor->velCtrl), 
						((int8_t)sentryGimbal.gimbalDetect.pitchDetectDir - 1) * GM_PITCH_DETECT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
				
//				if (sentryGimbal.GM_Yaw.posCtrl.absPos > 0.0f)
//				{
//					if (sentryGimbal.GM_Yaw.posCtrl.absPos >= (Yaw_GetCount * 360.0f + 90.0f) && 
//						sentryGimbal.GM_Yaw.posCtrl.absPos <= (Yaw_GetCount * 360.0f + 270.0f))		//后方
//						Motor_SetPos(&(motor->posCtrl), -13.0f, ABS);
//					else
//						Motor_SetPos(&(motor->posCtrl), -17.5f, ABS);			//前方
//				}
//				else
//				{
//					if (sentryGimbal.GM_Yaw.posCtrl.absPos <= (Yaw_GetCount * 360.0f - 90.0f) && 
//						sentryGimbal.GM_Yaw.posCtrl.absPos >= (Yaw_GetCount * 360.0f - 270.0f))		//后方
//						Motor_SetPos(&(motor->posCtrl), -13.0f, ABS);
//					else
//						Motor_SetPos(&(motor->posCtrl), -17.5f, ABS);			//前方 
//				}
//				
//				Motor_PosCtrl(&(motor->posCtrl));
//				Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
//				Motor_VelCtrl(&(motor->velCtrl));
			}
			break;
		case GIMBAL_DETECT_AHEAD:					//看前方
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				if ((motor->posCtrl.absPos > Yaw_GetCount * 360.0f + 30.0f) && 
					(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 180.0f) && 
					(sentryGimbal.gimbalDetect.posReady == 0))			//右转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = RIGHT;
					Motor_SetPos(&(motor->posCtrl), Yaw_GetCount * 360.0f, ABS);
					Motor_PosCtrl(&(motor->posCtrl));
					Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
					Motor_VelCtrl(&(motor->velCtrl));
				}
				else if((motor->posCtrl.absPos > Yaw_GetCount * 360.0f + 180.0f) && 
						(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 330.0f) &&
						(sentryGimbal.gimbalDetect.posReady == 0))		//左转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = LEFT;
					Motor_SetPos(&(motor->posCtrl), (Yaw_GetCount + 1) * 360.0f, ABS);
					Motor_PosCtrl(&(motor->posCtrl));
					Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
					Motor_VelCtrl(&(motor->velCtrl));
				}
				else
					sentryGimbal.gimbalDetect.posReady = 1;
				
				if (sentryGimbal.gimbalDetect.posReady == 1)
				{
					if (((motor->posCtrl.absPos > Yaw_GetCount * 360.0f + 325.0f) && 
						(motor->posCtrl.absPos < (Yaw_GetCount + 1) * 360.0f + 35.0f)) || 
						((motor->posCtrl.absPos > (Yaw_GetCount - 1) * 360.0f + 325.0f) && 
						(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 35.0f)))
						sentryGimbal.gimbalDetect.yawOverRange = 0;
					else
					{
						if (sentryGimbal.gimbalDetect.yawOverRange == 0)				//超过范围
						{
							sentryGimbal.gimbalDetect.yawDetectEdgeCnt++;
							if (sentryGimbal.gimbalDetect.yawDetectEdgeCnt > 3)		//巡检结束
							{
								sentryGimbal.gimbalDetect.posReady = 0;
								sentryGimbal.gimbalDetect.yawOverRange = 0;
							}
							else													//进行换向
							{
								sentryGimbal.gimbalDetect.yawDetectDir = 
									(GimbalYawDir_e)(abs(sentryGimbal.gimbalDetect.yawDetectDir - 2));
								sentryGimbal.gimbalDetect.yawOverRange = 1;
							}
						}
					}
					Motor_SetVel(&(motor->velCtrl), 
						((int8_t)sentryGimbal.gimbalDetect.yawDetectDir - 1) * GM_YAW_DETECT_VEL);
					Motor_VelCtrl(&(motor->velCtrl));
				}
			}
			if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPos(&(motor->posCtrl), -17.5f, ABS);
				Motor_PosCtrl(&(motor->posCtrl));
				Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			break;
		case GIMBAL_DETECT_BACK:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				if ((motor->posCtrl.absPos > Yaw_GetCount * 360.0f + 210.0f) && 
					(motor->posCtrl.absPos < (Yaw_GetCount + 1) * 360.0f) && 
					(sentryGimbal.gimbalDetect.posReady == 0))			//右转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = RIGHT;
					Motor_SetPos(&(motor->posCtrl), Yaw_GetCount * 360.0f + 180.0f, ABS);
					Motor_PosCtrl(&(motor->posCtrl));
					Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
					Motor_VelCtrl(&(motor->velCtrl));
				}
				else if((motor->posCtrl.absPos > Yaw_GetCount * 360.0f) && 
						(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 150.0f) &&
						(sentryGimbal.gimbalDetect.posReady == 0))		//左转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = LEFT;
					Motor_SetPos(&(motor->posCtrl), Yaw_GetCount * 360.0f + 180.0f, ABS);
					Motor_PosCtrl(&(motor->posCtrl));
					Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
					Motor_VelCtrl(&(motor->velCtrl));
				}
				else
					sentryGimbal.gimbalDetect.posReady = 1;
				
				if (sentryGimbal.gimbalDetect.posReady == 1)
				{
					if ((motor->posCtrl.absPos > Yaw_GetCount * 360.0f + 145.0f) && 
						(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 210.0f))
						sentryGimbal.gimbalDetect.yawOverRange = 0;
					else
					{
						if (sentryGimbal.gimbalDetect.yawOverRange == 0)				//超过范围
						{
							sentryGimbal.gimbalDetect.yawDetectEdgeCnt++;
							if (sentryGimbal.gimbalDetect.yawDetectEdgeCnt > 3)		//巡检结束
							{
								sentryGimbal.gimbalDetect.posReady = 0;
								sentryGimbal.gimbalDetect.yawOverRange = 0;
							}
							else													//进行换向
							{
								sentryGimbal.gimbalDetect.yawDetectDir = 
									(GimbalYawDir_e)(abs(sentryGimbal.gimbalDetect.yawDetectDir - 2));
								sentryGimbal.gimbalDetect.yawOverRange = 1;
							}
						}
					}
					Motor_SetVel(&(motor->velCtrl), 
						((int8_t)sentryGimbal.gimbalDetect.yawDetectDir - 1) * GM_YAW_DETECT_VEL);
					Motor_VelCtrl(&(motor->velCtrl));
				}
			}
			if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPos(&(motor->posCtrl), -13.0f, ABS);
				Motor_PosCtrl(&(motor->posCtrl));
				Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			break;
		case GIMBAL_TRACE:
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetPos(&(motor->posCtrl), masterRxData.yawAngle + sentryGimbal.gimbalFilter.yawFore, RELA);
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetPos(&(motor->posCtrl), masterRxData.pitchAngle + sentryGimbal.gimbalFilter.pitchFore, RELA);
			else
				break;
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_DEBUG_VEL:
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_DEBUG_POS: 
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_YAW_INIT:
			if (motor != &(sentryGimbal.GM_Yaw))
				break;
			else
			{
				Motor_SetVel(&(motor->velCtrl), GM_YAW_INIT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
				break;
			}
		case GIMBAL_PITCH_INIT:
			if (motor == &(sentryGimbal.GM_Yaw))		//Yaw轴已经初始化完成，进行位置闭环。
			{
				Motor_PosCtrl(&(motor->posCtrl));
				Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			else
			{
				Motor_SetVel(&(motor->velCtrl), GM_PITCH_INIT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			break;
		default:
			break;
	}
}

void Gimbal_TraceForecast(Gimbal_t *gimbal)
{
	GimbalFilter_t *filter = &(gimbal->gimbalFilter);
	float tmpWYaw = 0, tmpWPitch = 0;
	
	uint8_t i;
	
	if ((masterRxData.isFind == 1) && (gimbal->mode == GIMBAL_TRACE))		//发现目标则做预测处理
	{
		if (filter->isFirst == 0)				//对第一次发现目标进行处理
		{
			filter->tick = HAL_GetTick();		//获取时间
			
			/* 滤波器左移 */
			for (i = 0; i < 19; i++)
			{
				filter->wYaw[i] = filter->wYaw[i + 1];
				filter->wPitch[i] = filter->wPitch[i + 1];
			}
			
			/* 更新当前角速度 */
			filter->wYaw[19] = (gimbal->GM_Yaw.posCtrl.absPos + masterRxData.yawAngle - filter->lastAbsYaw) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			filter->wPitch[19] = (gimbal->GM_Pitch.posCtrl.absPos + masterRxData.pitchAngle - filter->lastAbsPitch) / 
								((float)(filter->tick - filter->lastTick) / 1000);
			
			/* 均值滤波 */
			for (i = 0; i < 20; i++)
			{
				tmpWYaw += filter->wYaw[i];
				tmpWPitch += filter->wPitch[i];
			}
			filter->avgWYaw = tmpWYaw / 20;
			filter->avgWPitch = tmpWPitch / 20;
			
			/* 补偿延时角 */
			filter->yawFore = filter->avgWYaw * (masterRxData.distance / 25.0f);
			filter->pitchFore = filter->avgWPitch * (masterRxData.distance / 25.0f);

			/* 存储数据 */
			filter->lastTick = filter->tick;
			filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos + masterRxData.yawAngle;
			filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos + masterRxData.pitchAngle;
		}
		else
		{
			filter->isFirst = 0;
			filter->avgWYaw = 0;
			filter->avgWPitch = 0;
			filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos + masterRxData.yawAngle;
			filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos + masterRxData.pitchAngle;
			filter->lastTick = HAL_GetTick();
			filter->tick = filter->lastTick;
			filter->pitchFore = 0;
			filter->yawFore = 0;
			for (i = 0; i < 19; i++)
			{
				filter->wYaw[i] = 0;
				filter->wPitch[i] = 0;
			}
		}
	}
	else
	{
		filter->isFirst = 1;
		filter->avgWYaw = 0;
		filter->avgWPitch = 0;
		filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos;
		filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos;
		filter->lastTick = HAL_GetTick();
		filter->tick = filter->lastTick;
		filter->pitchFore = 0;
		filter->yawFore = 0;
		for (i = 0; i < 19; i++)
		{
			filter->wYaw[i] = 0;
			filter->wPitch[i] = 0;
		}
	}
}
