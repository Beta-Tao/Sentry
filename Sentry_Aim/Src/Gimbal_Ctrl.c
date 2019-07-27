#include "Gimbal_Ctrl.h"
#include "Loader_Ctrl.h"
#include "Master_Comm.h"
#include "DataScope_DP.h"
#include "PC_Comm.h"
#include "usart.h"
#include "gpio.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "IMU_Comm.h"

//PIDParam_t yawVelPid = 					{300.0f, 20.0f, 100.0f};
//PIDParam_t pitchVelPid = 				{180.0f, 3.0f, 0.0f};

//PIDParam_t remoteYawPosPid = 			{0.6f, 0.0f, 0.5f};
//PIDParam_t detectYawPosPid = 			{0.0f, 0.0f, 0.0f};
////PIDParam_t traceStepYawPosPid = 		{0.85f, 0.05f, 9.0f};	//60000.0f
////PIDParam_t traceFollowYawPosPid = 		{2.3f, 0.04f, 7.0f};		//24000.0f

////10ms
//PIDParam_t traceStepYawPosPid = 		{0.85f, 0.05f, 9.0f};	//60000.0f
//PIDParam_t traceFollowYawPosPid = 		{2.3f, 0.04f, 7.0f};		//24000.0f

//PIDParam_t remotePitchPosPid = 			{-0.5f, 0.0f, 0.0f};
//PIDParam_t detectPitchPosPid = 			{0.0f, 0.0f, 0.0f};
//PIDParam_t traceStepPitchPosPid = 		{-1.0f, 0.0f, 0.0f};
//PIDParam_t traceFollowPitchPosPid = 	{-1.0f, 0.0f, 0.0f};

float detaYaw = 0.0f;

/* IMU控制 */
PIDParam_t yawVelPid = 					{-240.0f, -10.0f, 0.0f};
PIDParam_t yawTraceVelPid = 			{-70.0f, -0.4f, 0.0f};

PIDParam_t pitchVelPid = 				{180.0f, 3.0f, 0.0f};
PIDParam_t pitchTraceVelPid =			{180.0f, 3.0f, 0.0f};

PIDParam_t remoteYawPosPid = 			{11.0f, 0.0f, 0.0f};
PIDParam_t detectYawPosPid = 			{0.0f, 0.0f, 0.0f};

PIDParam_t traceStepYawPosPid = 		{19.0f, 0.0f, 200.0f};
PIDParam_t traceFollowYawPosPid = 		{30.0f, 20.0f, 200.0f};

PIDParam_t remotePitchPosPid = 			{-0.5f, 0.0f, 0.0f};
PIDParam_t detectPitchPosPid = 			{0.0f, 0.0f, 0.0f};

PIDParam_t traceStepPitchPosPid = 		{-1.0f, 0.0f, 0.0f};
PIDParam_t traceFollowPitchPosPid = 	{-1.0f, 0.0f, 0.0f};

Gimbal_t sentryGimbal;

float yawErr = 0.0f;
float pitchErr = 0.0f;

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
	yawMotor->motorType = GM6020;
	yawMotor->escType = GM_6020;
//	Motor_PosCtrlInit(yawMotor,
//					  GM_YAW_ACC, &remoteYawPosPid,
//					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, GM_YAW_MAX, GM_YAW_MIN, -22.75278);	//
//	Motor_VelCtrlInit(yawMotor, 
//					  GM_YAW_ACC, GM_YAW_DEC, &yawVelPid, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
//					  -0.16667);
	Motor_PosCtrlInit(yawMotor,
					  GM_YAW_ACC, &remoteYawPosPid,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, GM_YAW_MAX, GM_YAW_MIN, 1.0f);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, &yawVelPid, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  1.0f);
	
	pitchMotor->motorType = GM6020;
	pitchMotor->escType = GM_6020;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC, &remotePitchPosPid,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, GM_PITCH_MAX, GM_PITCH_MIN, -22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, &pitchVelPid,		//acc, dec
					  -0.16667);

	gimbal->mode = GIMBAL_YAW_INIT;
	gimbal->yawCtrlType = IMU;
	gimbal->pitchCtrlType = MOTOR;
	
	gimbal->gimbalTrace.yawTraceState = STEP;
	gimbal->gimbalTrace.pitchTraceState = STEP;
	gimbal->gimbalTrace.yawTraceTick = 0;
	gimbal->gimbalTrace.lastYawTraceTick = 0;
	gimbal->gimbalTrace.pitchTraceTick = 0;
	gimbal->gimbalTrace.lastPitchTraceTick = 0;
	
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
	gimbal->gimbalFilter.isFirst = 0;
	gimbal->gimbalFilter.isStart = 0;
	memset((void *)gimbal->gimbalFilter.wYaw, 0, sizeof(float) * 60);
	memset((void *)gimbal->gimbalFilter.wPitch, 0, sizeof(float) * 60);
	memset((void *)gimbal->gimbalFilter.distance, 0, sizeof(float) * 60);
	gimbal->gimbalFilter.lastAbsYaw = 0.0f;
	gimbal->gimbalFilter.lastAbsPitch = 0.0f;
	gimbal->gimbalFilter.yawFore = 0.0f;
	gimbal->gimbalFilter.pitchFore = 0.0f;
}

void Gimbal_TraceInit(Gimbal_t *gimbal)
{
	gimbal->gimbalTrace.yawTraceState = STEP;
	gimbal->gimbalTrace.yawTraceTick = 0;
	gimbal->gimbalTrace.lastYawTraceTick = 0;
	
	gimbal->gimbalTrace.pitchTraceState = STEP;
	gimbal->gimbalTrace.pitchTraceTick = 0;
	gimbal->gimbalTrace.lastPitchTraceTick = 0;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	static GimbalMode_e lastMode;
	
	switch (gimbal->mode)
	{
		case GIMBAL_TRACE:
			if (PCRxComm.PCData.isFind != 1)
				Gimbal_TraceInit(gimbal);
			break;
		default:
			Gimbal_TraceInit(gimbal);
			break;
	}
	
	switch (gimbal->mode)
	{
		case GIMBAL_DETECT_AHEAD:
		case GIMBAL_DETECT_BACK:
		case GIMBAL_DETECT_WHOLE:
		case GIMBAL_TRACE:
			if (masterRxData.gimbalMode == GIMBAL_REMOTE)
			{
				Gimbal_InitDetect(gimbal);
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
				break;
			}
			else if (PCRxComm.PCData.isInSight == 1)		//非遥控模式则为自动模式
			{
				Gimbal_InitDetect(gimbal);
				gimbal->mode = GIMBAL_TRACE;
//				if ((lastMode != GIMBAL_TRACE) && (gimbal->mode == GIMBAL_TRACE))
//				{
//					lastAbsPos = gimbal->GM_Yaw.posCtrl.absPos;
//				}
//				else
//				{
//					PCRxComm.PCData.yawAngle -= gimbal->GM_Yaw.posCtrl.absPos - lastAbsPos;
//					lastAbsPos = gimbal->GM_Yaw.posCtrl.absPos;
//				}
				break;
			}
			else
			{
				gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;

				if (gimbal->gimbalDetect.yawDetectEdgeCnt > 3)
					gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
			}
			break;
		case GIMBAL_YAW_INIT:
			gimbal->GM_Yaw.posCtrl.absPos = HIData.rawYaw;
			gimbal->GM_Yaw.posCtrl.refAbsPos = gimbal->GM_Yaw.posCtrl.absPos;
			gimbal->mode = GIMBAL_PITCH_INIT;
			break;
		case GIMBAL_PITCH_INIT:
			gimbal->GM_Pitch.posCtrl.absPos = (float)(gimbal->GM_Pitch.posCtrl.rawPos - GM_PITCH_OFFSET) / 
														gimbal->GM_Pitch.posCtrl.posRatio;
			gimbal->GM_Pitch.posCtrl.refAbsPos = gimbal->GM_Pitch.posCtrl.absPos;
			gimbal->mode = GIMBAL_STOP;
			break;
		case GIMBAL_DEBUG_VEL:
			Gimbal_InitDetect(gimbal);
			break;
		case GIMBAL_DEBUG_POS:
			Gimbal_InitDetect(gimbal);
			break;
		default:
			Gimbal_InitDetect(gimbal);
			gimbal->mode = (GimbalMode_e)masterRxData.gimbalMode;
			break;
	}
	
	if ((lastMode != GIMBAL_REMOTE) && (gimbal->mode == GIMBAL_REMOTE))		//切换回遥控模式,直接赋值
	{
		gimbal->GM_Yaw.posCtrl.refAbsPos = gimbal->GM_Yaw.posCtrl.absPos;
		gimbal->GM_Pitch.posCtrl.refAbsPos = gimbal->GM_Pitch.posCtrl.absPos;
	}
	lastMode = gimbal->mode;
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
	
	/* 根据模式更改控制方式 */
	switch (sentryGimbal.mode)
	{
		case GIMBAL_STOP:							//停止状态则保持静止，位置闭环
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
			else
				break;
			Motor_VelCtrl(motor, 0);
			break;
		case GIMBAL_REMOTE:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(remoteYawPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
				
				Motor_PosCtrl(motor, 
						sentryGimbal.GM_Yaw.posCtrl.refAbsPos + masterRxData.remoteYawAngle, ABS);
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(remotePitchPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
				
				Motor_PosCtrl(motor, 
						sentryGimbal.GM_Pitch.posCtrl.refAbsPos + masterRxData.remotePitchAngle, ABS);
			}
			else
				break;
			break;
		case GIMBAL_DETECT_WHOLE:							//云台巡逻模式
			if (motor == &(sentryGimbal.GM_Yaw))		//Yaw轴持续旋转
			{
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
				Motor_VelCtrl(motor, GM_YAW_DETECT_VEL);
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
				
				if (motor->posCtrl.absPos >= -5.0f)
					sentryGimbal.gimbalDetect.pitchDetectDir = DOWN;
				if (motor->posCtrl.absPos <= -30.0f)
					sentryGimbal.gimbalDetect.pitchDetectDir = UP;
				
				Motor_VelCtrl(motor, 
					((int8_t)sentryGimbal.gimbalDetect.pitchDetectDir - 1) * GM_PITCH_DETECT_VEL);
			}
			else
				break;
			break;
		case GIMBAL_DETECT_AHEAD:					//看前方
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(detectYawPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
				
				if ((motor->posCtrl.rawPos < 3896.0f) && 
					(motor->posCtrl.rawPos > 483.0f) && 
					(sentryGimbal.gimbalDetect.posReady == 0))			//右转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = RIGHT;
					Motor_PosCtrl(motor, Yaw_GetCount * 360.0f, ABS);
				}
				else if((motor->posCtrl.rawPos > 5260.0f) && 
						(motor->posCtrl.rawPos < 483.0f) &&
						(sentryGimbal.gimbalDetect.posReady == 0))		//左转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = LEFT;
					Motor_PosCtrl(motor, (Yaw_GetCount + 1) * 360.0f, ABS);
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
					Motor_VelCtrl(motor, 
						((int8_t)sentryGimbal.gimbalDetect.yawDetectDir - 1) * GM_YAW_DETECT_VEL);
				}
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(detectPitchPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
				Motor_PosCtrl(motor, -17.5f, ABS);
			}
			else
				break;
			break;
		case GIMBAL_DETECT_BACK:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(detectYawPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
				
				if ((motor->posCtrl.rawPos > Yaw_GetCount * 360.0f + 210.0f) && 
					(motor->posCtrl.absPos < (Yaw_GetCount + 1) * 360.0f) && 
					(sentryGimbal.gimbalDetect.posReady == 0))			//右转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = RIGHT;
					Motor_PosCtrl(motor, Yaw_GetCount * 360.0f + 180.0f, ABS);
				}
				else if((motor->posCtrl.absPos > Yaw_GetCount * 360.0f) && 
						(motor->posCtrl.absPos < Yaw_GetCount * 360.0f + 150.0f) &&
						(sentryGimbal.gimbalDetect.posReady == 0))		//左转
				{
					sentryGimbal.gimbalDetect.yawDetectDir = LEFT;
					Motor_PosCtrl(motor, Yaw_GetCount * 360.0f + 180.0f, ABS);
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
					Motor_VelCtrl(motor, ((int8_t)sentryGimbal.gimbalDetect.yawDetectDir - 1) * GM_YAW_DETECT_VEL);
				}
			}
			if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(detectPitchPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
				
				Motor_PosCtrl(motor, -13.0f, ABS);
			}
			break;
		case GIMBAL_TRACE:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				if (sentryGimbal.gimbalTrace.yawTraceState == STEP)
				{
					Motor_SetPosCtrlParam(&(motor->posCtrl), &(traceStepYawPosPid));
					Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawTraceVelPid));
					if (sentryGimbal.gimbalTrace.lastYawTraceTick == 0)
						sentryGimbal.gimbalTrace.lastYawTraceTick = HAL_GetTick();
					else
					{
						sentryGimbal.gimbalTrace.yawTraceTick = HAL_GetTick();
						if ((fabs(sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos) < 0.3f) &&
							(sentryGimbal.gimbalTrace.yawTraceTick - sentryGimbal.gimbalTrace.lastYawTraceTick > 180))
							sentryGimbal.gimbalTrace.yawTraceState = FOLLOW;
					}
				}
				else
				{
					Motor_SetPosCtrlParam(&(motor->posCtrl), &(traceFollowYawPosPid));
					Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawTraceVelPid));
				}
				Motor_PosCtrl(motor, PCRxComm.PCData.yawAngle + sentryGimbal.gimbalFilter.yawFore, RELA);
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				if (sentryGimbal.gimbalTrace.pitchTraceState == STEP)
				{
					Motor_SetPosCtrlParam(&(motor->posCtrl), &(traceStepPitchPosPid));
					Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchTraceVelPid));
					if (sentryGimbal.gimbalTrace.lastPitchTraceTick == 0)
						sentryGimbal.gimbalTrace.lastPitchTraceTick = HAL_GetTick();
					else
					{
						sentryGimbal.gimbalTrace.pitchTraceTick = HAL_GetTick();
						if ((fabs(sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos) < 0.3f) &&
							(sentryGimbal.gimbalTrace.pitchTraceTick - sentryGimbal.gimbalTrace.lastPitchTraceTick > 180))
							sentryGimbal.gimbalTrace.pitchTraceState = FOLLOW;
					}
				}
				else
				{
					Motor_SetPosCtrlParam(&(motor->posCtrl), &(traceFollowPitchPosPid));
					Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchTraceVelPid));
				}
				
				Motor_PosCtrl(motor, PCRxComm.PCData.pitchAngle + sentryGimbal.gimbalFilter.pitchFore, RELA);
			}
			else
				break;
			break;
		case GIMBAL_DEBUG_VEL:
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
			else
				break;
			Motor_RunVelPID(&(motor->velCtrl));
			break;
		case GIMBAL_DEBUG_POS:
			if (motor == &(sentryGimbal.GM_Yaw))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(remoteYawPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(yawVelPid));
			}
			else if (motor == &(sentryGimbal.GM_Pitch))
			{
				Motor_SetPosCtrlParam(&(motor->posCtrl), &(remotePitchPosPid));
				Motor_SetVelCtrlParam(&(motor->velCtrl), &(pitchVelPid));
			}
			else
				break;
			
			Motor_RunPosPID(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_RunVelPID(&(motor->velCtrl));
			break;
		default:
			break;
	}
}

void Gimbal_UpdateLoadMode(void)
{
	yawErr = sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos;
	pitchErr = sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos;
	
	//TODO: 更新供弹状态
	if ((sentryGimbal.mode == GIMBAL_TRACE) && (PCRxComm.PCData.isFind == 1))
	{
		if (PCRxComm.PCData.isBig == 0)				//小装甲板
		{
			if (PCRxComm.PCData.distance > 6.0f)
			{
				if ((fabs(sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos) < 2.0f) &&
					(fabs(sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos) < 2.0f))
					masterTxData.loaderMode = LOADER_RUN_PS5;
				else
					masterTxData.loaderMode = LOADER_STOP;
			}
			else
			{
				if ((fabs(sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos) < 2.5f) &&
					(fabs(sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos) < 2.5f))
					masterTxData.loaderMode = LOADER_RUN_PS15;
				else
					masterTxData.loaderMode = LOADER_STOP;
			}
		}
		else							//大装甲板
		{
			if (PCRxComm.PCData.distance > 6.0f)
			{
				if ((fabs(sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos) < 2.5f) &&
					(fabs(sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos) < 2.5f))
					masterTxData.loaderMode = LOADER_RUN_PS5;
				else
					masterTxData.loaderMode = LOADER_STOP;
			}
			else
			{
				if ((fabs(sentryGimbal.GM_Yaw.posCtrl.refAbsPos - sentryGimbal.GM_Yaw.posCtrl.absPos) < 3.0f) &&
					(fabs(sentryGimbal.GM_Pitch.posCtrl.refAbsPos - sentryGimbal.GM_Pitch.posCtrl.absPos) < 3.0f))
					masterTxData.loaderMode = LOADER_RUN_PS15;
				else
					masterTxData.loaderMode = LOADER_STOP;
			}
		}
	}
	else
		masterTxData.loaderMode = LOADER_STOP;
}

void Gimbal_TraceForecast(Gimbal_t *gimbal)
{
	GimbalFilter_t *filter = &(gimbal->gimbalFilter);
	float tmpWYaw = 0, tmpWPitch = 0, tmpDis = 0;
	static float lastDis = 0.0f;
	
	uint8_t i;
	
	if ((PCRxComm.PCData.isFind == 1) && (gimbal->mode == GIMBAL_TRACE))		//发现目标则做预测处理
	{
		if (filter->isFirst == 0)				//对第一次发现目标进行处理	跟随之后做预测
		{
//			if ((fabs(gimbal->GM_Yaw.posCtrl.refAbsPos - gimbal->GM_Yaw.posCtrl.absPos) < 1.0f) && 
//				(fabs(gimbal->GM_Pitch.posCtrl.refAbsPos - gimbal->GM_Pitch.posCtrl.absPos) < 1.0f) && 
//				(filter->isStart == 0))
//				filter->isStart = 1;
			
//			if (filter->isStart == 1)
//			{
				filter->tick = HAL_GetTick();		//获取时间
				
				/* 滤波器左移 */
				for (i = 0; i < 59; i++)
				{
					filter->wYaw[i] = filter->wYaw[i + 1];
					filter->wPitch[i] = filter->wPitch[i + 1];
					filter->distance[i] = filter->distance[i + 1];
				}
				
				/* 更新当前角速度 */
				
				detaYaw = gimbal->GM_Yaw.posCtrl.absPos + PCRxComm.PCData.yawAngle - filter->lastAbsYaw;

				filter->wYaw[59] = (gimbal->GM_Yaw.posCtrl.absPos + PCRxComm.PCData.yawAngle - filter->lastAbsYaw) / 
									((float)(filter->tick - filter->lastTick) / 1000);
				filter->wPitch[59] = (gimbal->GM_Pitch.posCtrl.absPos + PCRxComm.PCData.pitchAngle - filter->lastAbsPitch) / 
									((float)(filter->tick - filter->lastTick) / 1000);
				
				if (fabs(lastDis - PCRxComm.PCData.distance) < 0.5f)
					filter->distance[59] = PCRxComm.PCData.distance;
				else
					filter->distance[59] = lastDis;
				
				lastDis = filter->distance[59];
				
				/* 均值滤波 */
				for (i = 0; i < 60; i++)
				{
					tmpWYaw += filter->wYaw[i];
					tmpWPitch += filter->wPitch[i];
					tmpDis += filter->distance[i];
				}
				filter->avgWYaw = tmpWYaw / 60;
				filter->avgWPitch = tmpWPitch / 60;
				filter->avgDis = tmpDis / 60;
				
				if (filter->avgWYaw > (5.0f / filter->avgDis / 3.1415f * 180))
					filter->avgWYaw = 5.0f / filter->avgDis / 3.1415f * 180;
				if (filter->avgWYaw < -(5.0f / filter->avgDis / 3.1415f * 180))
					filter->avgWYaw = -(5.0f / filter->avgDis / 3.1415f * 180);
				
				/* 补偿延时角 */
				filter->yawFore = filter->avgWYaw * (filter->avgDis / 16.0f);
				filter->pitchFore = filter->avgWPitch * (filter->avgDis / 24.0f);
				
				if ((filter->yawFore < 0.2f) && (filter->yawFore > -0.2f))
					filter->yawFore = 0.0f;
				if (filter->yawFore > 11.0f)
					filter->yawFore = 11.0f;
				if (filter->yawFore < -11.0f)
					filter->yawFore = -11.0f;
				
				if ((filter->pitchFore < 0.4f) && (filter->pitchFore > -0.4f))
					filter->pitchFore = 0.0f;
				if (filter->pitchFore > 3.0f)
					filter->pitchFore = 3.0f;
				if (filter->pitchFore < -3.0f)
					filter->pitchFore = -3.0f;

				/* 存储数据 */
				filter->lastTick = filter->tick;
				filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos + PCRxComm.PCData.yawAngle;
				filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos + PCRxComm.PCData.pitchAngle;
//			}
		}
		else
		{
			filter->isFirst = 0;
			filter->isStart = 0;
			filter->avgWYaw = 0;
			filter->avgWPitch = 0;
			filter->avgDis = 0;
			lastDis = PCRxComm.PCData.distance;
			filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos + PCRxComm.PCData.yawAngle;
			filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos + PCRxComm.PCData.pitchAngle;
			filter->lastTick = HAL_GetTick();
			filter->tick = filter->lastTick;
			filter->pitchFore = 0;
			filter->yawFore = 0;
			for (i = 0; i < 60; i++)
			{
				filter->wYaw[i] = 0;
				filter->wPitch[i] = 0;
			}
		}
	}
	else
	{
		filter->isFirst = 1;
		filter->isStart = 0;
		filter->avgWYaw = 0;
		filter->avgWPitch = 0;
		filter->avgDis = 0;
		filter->lastAbsYaw = gimbal->GM_Yaw.posCtrl.absPos;
		filter->lastAbsPitch = gimbal->GM_Pitch.posCtrl.absPos;
		filter->lastTick = HAL_GetTick();
		filter->tick = filter->lastTick;
		filter->pitchFore = 0;
		filter->yawFore = 0;
		for (i = 0; i < 60; i++)
		{
			filter->wYaw[i] = 0;
			filter->wPitch[i] = 0;
		}
	}
}
