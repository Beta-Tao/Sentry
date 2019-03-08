#include "Master_Comm.h"
#include "Remote_Comm.h"
#include "Motor_Ctrl.h"
#include "Gimbal_Ctrl.h"
#include "usart.h"
#include "PC_Comm.h"

unsigned char commOutputBuffer[COMM_FRAME_LEN];

float refRelaYaw;
float refRelaPitch;
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
	Comm_Float2Byte(&refRelaYaw, commOutputBuffer, 1);			//Yaw轴相对位移
	Comm_Float2Byte(&refRelaPitch, commOutputBuffer, 5);		//Pitch轴相对位移
	Comm_Float2Byte(&refYawVel, commOutputBuffer, 9);		//Yaw轴运动速度
	Comm_Float2Byte(&refPitchVel, commOutputBuffer, 13);	//Pitch轴运动速度
	commOutputBuffer[17] = sentryGimbal.mode;						//瞄准标志位
}

void Comm_GetData(void)
{
	switch (sentryGimbal.mode)
	{
		case GIMBAL_REMOTE:
			refYawVel = (-(float)((RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_YAW_VEL_MAX));
			refPitchVel= ((float)((RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_PITCH_VEL_MAX));
			break;
		case GIMBAL_TRACE:
			refRelaYaw = -PCAngle_t.relaYaw;			//从PC端获取数据
			refRelaPitch = -PCAngle_t.relaPitch;

			/* 处理数据完后清零，防止PC端掉线使得云台转动 */
			PCAngle_t.relaYaw = 0;
			PCAngle_t.relaPitch = 0;
			break;
		case  GIMBAL_STOP:
			refYawVel = 0;
			refPitchVel = 0;
			break;
		default:
			break;
	}
}

void Comm_SendData(void)
{ 
	uint8_t i;
	
	Comm_GetData();
	
	Comm_GenerateData();
	
	for (i = 0; i < COMM_FRAME_LEN; i++)
		HAL_UART_Transmit(&huart8, commOutputBuffer + i, 1, 10);
}
