/**
  ******************************************************************************
  * @file       Remote_Ctrl.c
  * @brief      ң������������ת��       
  ****************************************************************************
  */

#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "macro.h"

ChassisRef_t ChassisRef;      //���̿�������
uint8_t AutoMode;
float g_gear=50;
int shift_state;


extern int stop_flag;
/***
	*�������ƣ�Ctrl_Init
	*�������ܣ���ʼ��������ز���
	*��ڲ�������
	*����ֵ  ����
***/
void Ctrl_Init(void)
{
	AutoMode = NOAUTO;
}

/***
	*�������ƣ�Remode_Input_Judge
	*�������ܣ��жϿ����ź���������
	*��ڲ�������
	*����ֵ  ����
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
	*�������ƣ�Remote_Process
	*�������ܣ���������ź�Ϊң��������Ӧ�Ľ��뺯��
	*��ڲ�������
	*����ֵ  ����
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
	*�������ƣ�Key_Mouse_Process
	*�������ܣ���������ź�Ϊ������꣬��Ӧ�Ľ��뺯��
	*��ڲ�������
	*����ֵ  ����
***/	
void Key_Mouse_Process(void)
{
	if ((RemoteCtrlData.key.v&1) == 1)//W����
	{	
		if (ChassisRef.forward_back_ref < 660)	
			ChassisRef.forward_back_ref += 66;			
	}
	else if ((RemoteCtrlData.key.v&2) == 2)//S����
	{
		if (ChassisRef.forward_back_ref > -660)	
			ChassisRef.forward_back_ref -= 66;
	}
	else
	{
		ChassisRef.forward_back_ref = 0;
	}
	if ((RemoteCtrlData.key.v&4) == 4)//A����
	{
		if (ChassisRef.left_right_ref >- 660)	
			ChassisRef.left_right_ref -= 66;	
	}
	else if ((RemoteCtrlData.key.v&8) == 8)//D����
	{
		if (ChassisRef.left_right_ref < 660)	
			ChassisRef.left_right_ref += 66;	
	}
	else
	{
		ChassisRef.left_right_ref = 0;
	}
	if ((RemoteCtrlData.key.v&64) == 64)//Q����
	{
		if (ChassisRef.rotate_ref >-660)	
			ChassisRef.rotate_ref -= 66;	
	}
	
	else if ((RemoteCtrlData.key.v&128)==128)//E����
	{
		if(ChassisRef.rotate_ref < 660)	
			ChassisRef.rotate_ref += 66;	
	}
	
	else
	{
		ChassisRef.rotate_ref = 0;
	}
	if ((RemoteCtrlData.key.v & 16) == 16)//shift ���£���λ�л�
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
