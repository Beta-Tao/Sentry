#include "Chassis_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t CM_Left;	//左轮电机
Motor_t CM_Right;	//右轮电机

int16_t chassisVel;			//遥控器通道2的值，直接作为底盘速度，控制电机的时候进行转换
int16_t chassisDir = -1;	//正负表示底盘运动方向

/**
  * @brief	底盘控制初始化
  * @note	底盘控制模式以及底盘电机初始化
  * @retval	None
  */
void Chassis_CtrlInit(void)
{
	//底盘电机及电调类型初始化，底盘电机只需要速度闭环
	CM_Left.motorType = M_3508;
	CM_Left.escType = C620;
	Motor_VelCtrlInit(&CM_Left, 
					  CHASSIS_MOTOR_ACC, CHASSIS_MOTOR_DEC, 	//acc, dec	控制周期是1ms，所以单位意义是每ms增加的转速
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
	
	CM_Right.motorType = M_3508;
	CM_Right.escType = C620;
	Motor_VelCtrlInit(&CM_Right, 
					  CHASSIS_MOTOR_ACC, CHASSIS_MOTOR_DEC, 			//acc, dec
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
}

/**
  * @brief	将遥控器通道值映射到底盘电机速度
  * @retval	None
  */
void Chassis_UpdateRef(void)
{
	Motor_SetVel(&CM_Left.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
	Motor_SetVel(&CM_Right.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
}

/**
  * @brief	底盘电机控制
  * @note	底盘电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @retvel	None
  */
void Chassis_MotorCtrl(Motor_t *motor)
{
	switch (g_MoveMode)				//根据运动模式改变底盘速度
	{
		case SENTRY_REMOTE:		//遥控模式则底盘速度和遥控器通道数值线性相关
			chassisVel = -(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET);	//根据实际左右变换正负号
			Chassis_UpdateRef();		//根据底盘速度更新底盘电机速度
			break;
		case SENTRY_DETECT:		//巡逻模式则底盘速度为当前的巡逻速度值
			chassisVel = SENTRY_DETECT_VEL * chassisDir;	//巡逻速度
			Chassis_UpdateRef();
			break;
		case SENTRY_STOP:
			chassisVel = 0;
			Chassis_UpdateRef();
		case SENTRY_DODGE:
			break;
		default:
			break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}
