/**
  ******************************************************************************
  * @file       Remote_Ctrl.c
  * @brief      遥控器控制命令转换       
  ****************************************************************************
  */

#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "macro.h"
#include "motor.h"

uint8_t autoMode;	//自动模式标志位
uint8_t shootMode;	//发射及供弹模式标志位

/**
  * @brief	初始化控制相关参数
  * @param	None
  * @note	初始化为遥控模式
  * @retval	None
  */
void Ctrl_Init(void)
{
	autoMode = SENTRY_NOAUTO;
}
 
/**
  * @brief	对应的遥控器解码函数
  * @param	None
  * @retval	None
  */	
void Remote_Process(void)
{
	switch (RemoteCtrlData.remote.s1)
	{
		case RC_SW_UP:		//当s1在上时，为自动模式
			autoMode = SENTRY_AUTO;
			break;
		case RC_SW_MID:		//当s1在中时，为遥控模式
			autoMode = SENTRY_NOAUTO;
			break;
		case RC_SW_DOWN:	//当s1在下时，不做处理
			break;
	}
	switch (RemoteCtrlData.remote.s2)
	{
		case RC_SW_UP:							//当s2在上时，为停火模式
			shootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_MID:							//当s2在中时，为瞄准模式
			shootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_DOWN:						//当s2在下时，为开火模式
			shootMode = SENTRY_CEASE_FIRE;
			break;
	}
	
	/* 给底盘期望速度赋值 */
	chassisSpeedRef = RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET;
	//将底盘速度期望转换为两个电机的速度期望
	Motor_UpdateCMRef();
	/* 遥控云台的俯仰和方位角 */
}
