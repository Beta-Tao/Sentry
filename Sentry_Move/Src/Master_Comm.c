#include "Master_Comm.h"
#include "usart.h"
#include "PC_Comm.h"
#include "string.h"
#include "Remote_Comm.h"
#include "Sentry_Strategy.h"

unsigned char commOutputBuffer[COMM_FRAME_LEN];

MasterData_t masterData;

void Master_CommInit(void)
{
	masterData.yawAngle = 0;
	masterData.pitchAngle = 0;
	masterData.posCtrlType = RELA; 
	masterData.gimbalMode = GIMBAL_STOP;
	masterData.loaderMode = LOADER_STOP;
	masterData.shooterMode = SHOOTER_CEASE;
}

void Master_GenerateData(void)
{
	commOutputBuffer[0] = MASTER_FRAME_HEAD;								//帧头
	memcpy(commOutputBuffer + 1, &masterData, sizeof(MasterData_t));
}

void Master_GetData(void)
{
	/* 根据遥控器数据更新云台状态 */
	switch (RemoteComm.RemoteData.remote.s2)
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
					(-(float)((RemoteComm.RemoteData.remote.ch0 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 90);
			masterData.pitchAngle = 
					((float)((RemoteComm.RemoteData.remote.ch1 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 25);
			masterData.posCtrlType = RELA;
			
			if (RemoteComm.RemoteData.remote.ch3 >= RC_CH_VALUE_OFFSET + 10)
				masterData.shooterMode = SHOOTER_OPEN_30MPS;
			else
				masterData.shooterMode = SHOOTER_CEASE;
			
			if (RemoteComm.RemoteData.remote.ch3 == RC_CH_VALUE_MAX)
				masterData.loaderMode = LOADER_RUN_PS3;
			else
				masterData.loaderMode = LOADER_STOP;
			break;
		case GIMBAL_TRACE:
			masterData.yawAngle = -PCComm.PCData.yawAngle;			//从PC端获取数据
			masterData.pitchAngle = -PCComm.PCData.pitchAngle;
			masterData.posCtrlType = PCComm.PCData.posCtrlType;
			
			masterData.loaderMode = sentryST.loaderMode;
			masterData.shooterMode = sentryST.shooterMode;
			break;
		case  GIMBAL_STOP:
			masterData.yawAngle = 0;
			masterData.pitchAngle = 0;
			masterData.posCtrlType = RELA;
		
			masterData.loaderMode = LOADER_STOP;
			masterData.shooterMode = SHOOTER_CEASE;
			break;
		default:
			break;
	}
}

void Master_SendData(void)
{
	uint8_t i;
	
	Master_GetData();
	
	Master_GenerateData();
	
	for (i = 0; i < COMM_FRAME_LEN; i++)
		HAL_UART_Transmit(&huart8, commOutputBuffer + i, 1, 10);
}
