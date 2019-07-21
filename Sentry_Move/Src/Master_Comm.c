#include "Master_Comm.h"
#include "usart.h"
#include "string.h"
#include "Remote_Comm.h"
#include "Sentry_Strategy.h"

uint8_t UART8_DMA_TX_BUF[BSP_UART8_DMA_TX_BUF_LEN];
uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
unsigned char commOutputBuffer[COMM_TX_FRAME_LEN];

MasterTxData_t masterTxData;
MasterRxData_t masterRxData;

uint32_t rx_data_len = 0;		//接收长度

void Master_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //开启不定长中断
	HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}

void Master_CommInit(void)
{
	masterTxData.remoteYawAngle = 0;
	masterTxData.remotePitchAngle = 0;
	masterTxData.gimbalMode = GIMBAL_STOP;
	masterTxData.shooterMode = SHOOTER_CEASE;
	
	masterRxData.loaderMode = 0;
}

void Master_GenerateData(void)
{
	commOutputBuffer[0] = MASTER_FRAME_HEAD;								//帧头
	memcpy(commOutputBuffer + 1, &masterTxData, sizeof(MasterTxData_t));
}

void Master_GetData(void)
{
	/* 根据遥控器数据更新云台状态 */
	switch (RemoteComm.RemoteData.remote.s2)
	{
		case RC_SW_UP:							//当s2在上时，为追踪模式
			masterTxData.gimbalMode = (GimbalMode_e)sentryST.gimbalMode;
			masterTxData.shooterMode = (ShooterMode_e)sentryST.shooterMode;
			break;
		case RC_SW_MID:							//当s2在中时，为遥控模式
			masterTxData.gimbalMode = GIMBAL_REMOTE;
		
			if (RemoteComm.RemoteData.remote.ch3 >= RC_CH_VALUE_OFFSET + 10)
				masterTxData.shooterMode = SHOOTER_OPEN_30MPS;
			else
				masterTxData.shooterMode = SHOOTER_CEASE;
			break;
		case RC_SW_DOWN:						//当s2在下时，为Debug模式
			masterTxData.gimbalMode =  (GimbalMode_e)sentryST.gimbalMode;
			
			if (RemoteComm.RemoteData.remote.ch3 >= RC_CH_VALUE_OFFSET + 10)
				masterTxData.shooterMode = SHOOTER_OPEN_30MPS;
			else
				masterTxData.shooterMode = SHOOTER_CEASE;
			break;
		default:
			break;
	}
	
	/* 更新数据 */
	switch (masterTxData.gimbalMode)
	{
		case GIMBAL_REMOTE:
			masterTxData.remoteYawAngle = 
					((float)(-(RemoteComm.RemoteData.remote.ch0 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 2);
			masterTxData.remotePitchAngle = 
					((float)((RemoteComm.RemoteData.remote.ch1 - RC_CH_VALUE_OFFSET) / RC_CH_VALUE_RANGE) * 2);
			break;
		case GIMBAL_TRACE:
			masterTxData.remoteYawAngle = 0.0f;
			masterTxData.remotePitchAngle = 0.0f;
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
	
	for (i = 0; i < COMM_TX_FRAME_LEN; i++)
		HAL_UART_Transmit(&huart8, commOutputBuffer + i, 1, 10);
}

void Master_RevData(void)
{
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);												//清除空闲中断的标志
		(void)UART8->SR;                                                             //清空SR寄存器
		(void)UART8->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //清除 DMA1_Steam6传输完成标志
		HAL_UART_DMAStop(&huart8);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_UART8_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == COMM_RX_FRAME_LEN && UART8_DMA_RX_BUF[0] == MASTER_FRAME_HEAD)                                           //判断数据是否为正确的数据长度
		{
			Master_Decode(UART8_DMA_RX_BUF);                                        //进入数据解码函数
		}
	}
}

void Master_Decode(uint8_t *pData)
{
	if (pData == NULL)		//数据错或帧头错误
	{
		return;
	}
	
	memcpy(&masterRxData, pData + 1, sizeof(MasterRxData_t));		//进行数据复制
}
