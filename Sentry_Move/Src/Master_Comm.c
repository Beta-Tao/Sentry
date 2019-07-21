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

uint32_t rx_data_len = 0;		//���ճ���

void Master_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //�����������ж�
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
	commOutputBuffer[0] = MASTER_FRAME_HEAD;								//֡ͷ
	memcpy(commOutputBuffer + 1, &masterTxData, sizeof(MasterTxData_t));
}

void Master_GetData(void)
{
	/* ����ң�������ݸ�����̨״̬ */
	switch (RemoteComm.RemoteData.remote.s2)
	{
		case RC_SW_UP:							//��s2����ʱ��Ϊ׷��ģʽ
			masterTxData.gimbalMode = (GimbalMode_e)sentryST.gimbalMode;
			masterTxData.shooterMode = (ShooterMode_e)sentryST.shooterMode;
			break;
		case RC_SW_MID:							//��s2����ʱ��Ϊң��ģʽ
			masterTxData.gimbalMode = GIMBAL_REMOTE;
		
			if (RemoteComm.RemoteData.remote.ch3 >= RC_CH_VALUE_OFFSET + 10)
				masterTxData.shooterMode = SHOOTER_OPEN_30MPS;
			else
				masterTxData.shooterMode = SHOOTER_CEASE;
			break;
		case RC_SW_DOWN:						//��s2����ʱ��ΪDebugģʽ
			masterTxData.gimbalMode =  (GimbalMode_e)sentryST.gimbalMode;
			
			if (RemoteComm.RemoteData.remote.ch3 >= RC_CH_VALUE_OFFSET + 10)
				masterTxData.shooterMode = SHOOTER_OPEN_30MPS;
			else
				masterTxData.shooterMode = SHOOTER_CEASE;
			break;
		default:
			break;
	}
	
	/* �������� */
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
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);												//��������жϵı�־
		(void)UART8->SR;                                                             //���SR�Ĵ���
		(void)UART8->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //��� DMA1_Steam6������ɱ�־
		HAL_UART_DMAStop(&huart8);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_UART8_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == COMM_RX_FRAME_LEN && UART8_DMA_RX_BUF[0] == MASTER_FRAME_HEAD)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			Master_Decode(UART8_DMA_RX_BUF);                                        //�������ݽ��뺯��
		}
	}
}

void Master_Decode(uint8_t *pData)
{
	if (pData == NULL)		//���ݴ��֡ͷ����
	{
		return;
	}
	
	memcpy(&masterRxData, pData + 1, sizeof(MasterRxData_t));		//�������ݸ���
}
