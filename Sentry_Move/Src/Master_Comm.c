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
    point = (unsigned char*)target;	  //�õ�float�ĵ�ַ
    buf[loc] = point[0];
    buf[loc + 1] = point[1];
    buf[loc + 2] = point[2];
    buf[loc + 3] = point[3];
}

void Comm_GenerateData(void)
{
	commOutputBuffer[0] = 0x50;								//֡ͷ
	Comm_Float2Byte(&relaYaw, commOutputBuffer, 1);			//Yaw�����λ��
	Comm_Float2Byte(&relaPitch, commOutputBuffer, 5);		//Pitch�����λ��
	Comm_Float2Byte(&refYawVel, commOutputBuffer, 9);		//Yaw���˶��ٶ�
	Comm_Float2Byte(&refPitchVel, commOutputBuffer, 13);	//Pitch���˶��ٶ�
	commOutputBuffer[17] = g_AimMode;						//��׼��־λ
	commOutputBuffer[18] = g_LoadMode;						//������־λ
	commOutputBuffer[19] = g_ShootMode;						//�����־λ
}

void Comm_GetData(void)
{
	switch (g_AimMode)
	{
		case SENTRY_REMOTE:
			refYawVel = (-(float)((RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_YAW_VEL_MAX / 20));		//ת�ٽ��Ͷ�ʮ��
			refPitchVel= ((float)((RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / 
									RC_CH_VALUE_RANGE * GM_PITCH_VEL_MAX / 20));
			break;
		case SENTRY_TRACE:
			relaYaw = 0;			//��������Ҫ��PC�˻�ȡ����
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
