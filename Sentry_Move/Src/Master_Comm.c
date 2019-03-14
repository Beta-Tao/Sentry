#include "Master_Comm.h"
#include "usart.h"
#include "PC_Comm.h"
#include "string.h"
#include "Remote_Comm.h"

unsigned char commOutputBuffer[COMM_FRAME_LEN];

MasterData_t masterData;

void Comm_GenerateData(void)
{
	commOutputBuffer[0] = MASTER_FRAME_HEAD;								//帧头
	memcpy(commOutputBuffer + 1, &masterData, sizeof(MasterData_t));
}

void Comm_GetData(void)
{
	/* 根据遥控器数据更新云台状态 */
	switch (RemoteCtrlData.remote.s2)
	{
		case RC_SW_UP:							//当s2在上时，为追踪模式
			masterData.gimbalMode = GIMBAL_TRACE;
			break;
		case RC_SW_MID:							//当s2在中时，为遥控模式
			masterData.gimbalMode = GIMBAL_REMOTE;
			break;
		case RC_SW_DOWN:						//当s2在下时，为停止模式
			masterData.gimbalMode = GIMBAL_STOP;
			break;
		default:
			break;
	}
	
	switch (masterData.gimbalMode)
	{
		case GIMBAL_REMOTE:
			masterData.yawAngle = 
					(-(float)((RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 180);
			masterData.pitchAngle = 
					((float)((RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 45);
			masterData.posCtrlType = RELA;
			break;
		case GIMBAL_TRACE:
			
			masterData.yawAngle = -PCAngle_t.yawAngle;			//从PC端获取数据
			masterData.pitchAngle = -PCAngle_t.pitchAngle;
			masterData.posCtrlType = PCAngle_t.posCtrlType;

			/* 处理数据完后清零，防止PC端掉线使得云台转动 */
			PCAngle_t.yawAngle = 0;
			PCAngle_t.pitchAngle = 0;
			break;
		case  GIMBAL_STOP:
			masterData.yawAngle = 0;
			masterData.pitchAngle = 0;
			masterData.posCtrlType = RELA;
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
