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
  * @brief	对应的遥控器解码函数
  * @param	None
  * @retval	None
  */	
void Remote_Process(void)
{
	switch (RemoteCtrlData.remote.s1)
	{
		case RC_SW_UP:		//当s1在上时，为巡逻模式
			autoMode = SENTRY_DETECT;
			break;
		case RC_SW_MID:		//当s1在中时，为遥控模式
			autoMode = SENTRY_REMOTE;
			break;
		case RC_SW_DOWN:	//当s1在下时，为躲避模式
			autoMode = SENTRY_DODGE;
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
}

void Remote_InitFlag(void)
{
	autoMode = SENTRY_REMOTE;			//初始化为遥控模式
	shootMode = SENTRY_CEASE_FIRE;		//初始化为停火模式
}
