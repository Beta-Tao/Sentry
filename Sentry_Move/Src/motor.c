#include "motor.h"
#include "macro.h"
#include "PID.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include <string.h>

volatile Motor_t chassisL;	//左轮电机
volatile Motor_t chassisR;	//右轮电机

int16_t chassisSpeedRef;	//遥控器通道2的值，直接作为底盘速度
int16_t chassisDir = -1;	//正负表示方向

/**
  * @brief	给电机赋期望速度值
  * @param	motor:	Motor_t结构体的指针
  * @param	speed:	预设的速度值
  * @retval	None
  * @note	注意电机速度方向和实际需要的方向是否相同
  */
void Motor_SetSpeed(volatile Motor_t *motor, int16_t speed)
{
	motor->refSpeed = speed;
}

/**
  * @brief	给电机赋期望位置值
  * @param	motor:	Motor_t结构体的指针
  * @param	pos:	预设的位置值
  * @note	注意电机转子位置范围
  * @retval	None
  */
void Motor_SetPos(Motor_t *motor, int16_t pos)
{
	motor->refPos = pos;
}

/**
  * @brief	将遥控器通道值映射到底盘电机速度
  * @retval	None
  */
void Motor_UpdateCMRef(void)
{
	Motor_SetSpeed(&chassisL, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
	Motor_SetSpeed(&chassisR, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
}

/**
  * @brief	底盘控制
  * @retval	None
  */
void Motor_CtrChassis(void)
{
	if (RemoteCtrlData.remote.ch2 < RC_CH_VALUE_MIN || 
			RemoteCtrlData.remote.ch2 > RC_CH_VALUE_MAX)	//遥控器没有打开的时候，底盘不动
	{
		chassisSpeedRef = 0;
		Remote_InitFlag();
	}
	else
	{
		switch (autoMode)
		{
			case SENTRY_REMOTE:		//遥控模式则底盘速度和遥控器通道数值线性相关
				chassisSpeedRef = -(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET);	//根据实际左右变换正负号
				Motor_UpdateCMRef();
				break;
			case SENTRY_DETECT:		//巡逻模式则底盘速度为当前的巡逻速度值
				chassisSpeedRef = SENTRY_DETECT_SPEED * chassisDir;
				Motor_UpdateCMRef();
				break;
		}
	}
}
