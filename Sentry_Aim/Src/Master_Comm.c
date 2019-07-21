#include "usart.h"
#include "string.h"
#include "Master_Comm.h"

uint8_t UART8_DMA_TX_BUF[BSP_UART8_DMA_TX_BUF_LEN];
uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
uint32_t rx_data_len = 0;		//���ν��ճ���
MasterRxData_t masterRxData;
MasterTxData_t masterTxData;
unsigned char commOutputBuffer[COMM_TX_FRAME_LEN];

void Master_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //�����������ж�
	HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}

void Master_CommInit(void)
{
	masterRxData.remoteYawAngle = 0;
	masterRxData.remotePitchAngle = 0;
	masterRxData.gimbalMode = 0;
	masterRxData.shooterMode = 0;
	
	masterTxData.loaderMode = 0;
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

void Master_GenerateData(void)
{
	commOutputBuffer[0] = MASTER_FRAME_HEAD;								//֡ͷ
	memcpy(commOutputBuffer + 1, &masterTxData, sizeof(MasterTxData_t));
}

void Master_SendData(void)
{
	uint8_t i;
	
	Master_GenerateData();
	
	for (i = 0; i < COMM_TX_FRAME_LEN; i++)
		HAL_UART_Transmit(&huart8, commOutputBuffer + i, 1, 10);
}
