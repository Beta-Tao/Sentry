#include "Master_Comm.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "Motor_Ctrl.h"
#include "usart.h"

unsigned char commOutputBuffer[25];

float relaYaw;
float relaPitch;
float refYawVel;
float refPitchVel;

void Comm_Float2Byte(float *target, unsigned char *buf, uint8_t loc)
{
	unsigned char *point;
    point = (unsigned char*)target;	  //得到float的地址
    buf[loc] = point[0];
    buf[loc + 1] = point[1];
    buf[loc + 2] = point[2];
    buf[loc + 3] = point[3];
}

void Comm_GenerateData(void)
{
	commOutputBuffer[0] = 0x50;								//帧头
	Comm_Float2Byte(&relaYaw, commOutputBuffer, 1);			//Yaw轴相对位移
	Comm_Float2Byte(&relaPitch, commOutputBuffer, 5);		//Pitch轴相对位移
	Comm_Float2Byte(&refYawVel, commOutputBuffer, 9);		//Yaw轴运动速度
	Comm_Float2Byte(&refPitchVel, commOutputBuffer, 13);	//Pitch轴运动速度
	commOutputBuffer[17] = g_AimMode;						//瞄准标志位
	commOutputBuffer[18] = g_LoadMode;						//供弹标志位
	commOutputBuffer[19] = g_ShootMode;						//发射标志位
}

void Comm_GetData(void)
{
	switch (g_AimMode)
	{
		case SENTRY_REMOTE:
			refYawVel = (-(float)((RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_YAW_VEL_MAX / 20));		//转速降低二十倍
			refPitchVel= ((float)((RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_PITCH_VEL_MAX / 20));
			break;
		case SENTRY_TRACE:
			relaYaw = 0;			//待定，需要从PC端获取数据
			relaPitch = 0;
			break;
		case SENTRY_STOP:
			refYawVel = 0;
			refPitchVel = 0;
			break;
		default:
			break;
	}
}

void Comm_SendData(void)
{
	Comm_GetData();
	
	Comm_GenerateData();
	
	HAL_UART_Transmit(&huart8, commOutputBuffer, COMM_FRAME_LEN, 200);
}
