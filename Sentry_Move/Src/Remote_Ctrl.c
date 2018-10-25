/**
  ******************************************************************************
  * @file       Remote_Ctrl.c
  * @brief      遥控器控制命令转换       
  ****************************************************************************
  */

#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "macro.h"

ChassisRef_t ChassisRef;      //底盘控制命令
uint8_t AutoMode;
float g_gear=50;
int shift_state;


extern int stop_flag;
/***
	*函数名称：Ctrl_Init
	*函数功能：初始化控制相关参数
	*入口参数：无
	*返回值  ：无
***/
void Ctrl_Init(void)
{
	AutoMode = NOAUTO;
}

/***
	*函数名称：Remode_Input_Judge
	*函数功能：判断控制信号输入类型
	*入口参数：无
	*返回值  ：无
***/
void Remode_Input_Judge(void)
{
	 switch(RemoteCtrlData.remote.s2)
	 {
	 case REMOTE_INPUT:      
	 {
		 Remote_Process();        
		 break;
	 }
	 case KEY_MOUSE_INPUT:   
	 {
		 Key_Mouse_Process();     
		 break;
	 }
	 }
 }
 
/***
	*函数名称：Remote_Process
	*函数功能：如果输入信号为遥控器，对应的解码函数
	*入口参数：无
	*返回值  ：无
***/	

void Remote_Process(void)
{
	switch(RemoteCtrlData.remote.s1)
	{
		case 2: 
			g_gear = 20;
			break;
		case 3:
			g_gear = 50;
			break;
		case 1: 
			g_gear = 80;
			break;
	}
	ChassisRef.forward_back_ref = RemoteCtrlData.remote.ch1 - (int16_t)REMOTE_STICK_OFFSET;
	ChassisRef.left_right_ref = RemoteCtrlData.remote.ch0 - (int16_t)REMOTE_STICK_OFFSET;
	ChassisRef.rotate_ref = RemoteCtrlData.remote.ch2 - (int16_t)REMOTE_STICK_OFFSET;
}

/***
	*函数名称：Key_Mouse_Process
	*函数功能：如果输入信号为键盘鼠标，对应的解码函数
	*入口参数：无
	*返回值  ：无
***/	
void Key_Mouse_Process(void)
{
	if ((RemoteCtrlData.key.v&1) == 1)//W按下
	{	
		if (ChassisRef.forward_back_ref < 660)	
			ChassisRef.forward_back_ref += 66;			
	}
	else if ((RemoteCtrlData.key.v&2) == 2)//S按下
	{
		if (ChassisRef.forward_back_ref > -660)	
			ChassisRef.forward_back_ref -= 66;
	}
	else
	{
		ChassisRef.forward_back_ref = 0;
	}
	if ((RemoteCtrlData.key.v&4) == 4)//A按下
	{
		if (ChassisRef.left_right_ref >- 660)	
			ChassisRef.left_right_ref -= 66;	
	}
	else if ((RemoteCtrlData.key.v&8) == 8)//D按下
	{
		if (ChassisRef.left_right_ref < 660)	
			ChassisRef.left_right_ref += 66;	
	}
	else
	{
		ChassisRef.left_right_ref = 0;
	}
	if ((RemoteCtrlData.key.v&64) == 64)//Q按下
	{
		if (ChassisRef.rotate_ref >-660)	
			ChassisRef.rotate_ref -= 66;	
	}
	
	else if ((RemoteCtrlData.key.v&128)==128)//E按下
	{
		if(ChassisRef.rotate_ref < 660)	
			ChassisRef.rotate_ref += 66;	
	}
	
	else
	{
		ChassisRef.rotate_ref = 0;
	}
	if ((RemoteCtrlData.key.v & 16) == 16)//shift 按下，挡位切换
	{	
		shift_state = 1;
	}
	else if (shift_state == 1 && (RemoteCtrlData.key.v & 16) != 16)
	{
		 g_gear += 10;
		 if(g_gear > 80)
			 g_gear = 50;
		 shift_state = 0;
	}
}
