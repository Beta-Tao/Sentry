#include "PC_Comm.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "string.h"

static uint32_t PCTick = 0;
PCComm_t PCComm;

uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                                       //�����������ж�
	HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);
}

void PC_CommInit(void)
{
	PCComm.PCData.yawAngle = 0;
	PCComm.PCData.pitchAngle = 0;
	PCComm.PCData.posCtrlType = RELA;
	
	PCComm.PCData.distance = 0;
	PCComm.PCData.isTraced = 0;
	PCComm.PCData.isInSight = 0;
	
	PCComm.PCCommState = PC_COMM_DROP;
}

void PC_Data_Receive(void)
{
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);                                           //��������жϵı�־
		(void)USART3->SR;                                                             //���SR�Ĵ���
		(void)USART3->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);                       //���DMA1_Steam1������ɱ�־
		HAL_UART_DMAStop(&huart3);                                                    //��������Ժ�رմ���DMA
		HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);  //��������
		if(USART3_DMA_RX_BUF[0] == START_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[1] == START_CHECK_SECOND && 
		   USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN - 2] == END_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN - 1] == END_CHECK_SECOND)			//У���ͷ��β
		{
			PCComm.PCCommState = PC_COMM_NORMAL;
 			PCTick = 0;											//���PCTick��ѭ��
			PC_Decode(USART3_DMA_RX_BUF);                                         //�������ݽ��뺯��
		}
	}
}

void PC_Decode(uint8_t *pData)
{
	memcpy(&(PCComm.PCData.yawAngle), pData + 2, sizeof(float));			//Yaw�������Ƕ�
	memcpy(&(PCComm.PCData.pitchAngle), pData + 6, sizeof(float));		//Pitch�������Ƕ�
	memcpy(&(PCComm.PCData.posCtrlType), pData + 10, sizeof(uint8_t));	//λ�ÿ��Ʊ�־λ
	
	memcpy(&(PCComm.PCData.distance), pData + 11, sizeof(float));			//Ŀ�����
	
	memcpy(&(PCComm.PCData.isTraced), pData + 15, sizeof(uint8_t));			//���ٱ�־λ
	memcpy(&(PCComm.PCData.isInSight), pData + 16, sizeof(uint8_t));
	
	DataScope_Debug(2, PCComm.PCData.yawAngle, PCComm.PCData.pitchAngle);
}

void PC_IsCommDrop(void)
{
	PCTick++;
	if (PCTick > PC_IT_CYCLE + 60) 
	{
		PCTick = 0;
		
		PCComm.PCData.yawAngle = 0;
		PCComm.PCData.pitchAngle = 0;
		PCComm.PCData.posCtrlType = RELA;
		
		PCComm.PCData.distance = 0;
		
		PCComm.PCData.isTraced = 0;
		PCComm.PCData.isInSight = 0;
		
		PCComm.PCCommState = PC_COMM_DROP;
	}
}
