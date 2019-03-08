#include "PC_Comm.h"
#include "usart.h"
#include "string.h"

PCFrame_t PCAngle_t;

uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                                       //�����������ж�
	HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);
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
		   USART3_DMA_RX_BUF[10] == END_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[11] == END_CHECK_SECOND)			//У���ͷ��β
		{
			PC_Decode(USART3_DMA_RX_BUF);                                         //�������ݽ��뺯��
		}
	}
}

void PC_Decode(uint8_t *pData)
{
	memcpy(&(PCAngle_t.relaYaw), pData + 2, sizeof(float));			//Yaw�������Ƕ�
	memcpy(&(PCAngle_t.relaPitch), pData + 6, sizeof(float));		//Pitch�������Ƕ�
}
