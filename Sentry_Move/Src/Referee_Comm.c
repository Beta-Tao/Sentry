#include "Referee_Comm.h"
#include "usart.h"

uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //�����������ж�
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}

void Referee_Data_Receive(void)
{	                                                          //���ν��ճ���
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //��������жϵı�־
		(void)USART6->SR;                                                             //���SR�Ĵ���
		(void)USART6->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //��� DMA2_Steam1������ɱ�־
		HAL_UART_DMAStop(&huart6);                                                    //��������Ժ�رմ���DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //��������
		if(USART6_DMA_RX_BUF[0] == 0xA5)                                              //�ж����ݰ�֡ͷ
		{
			Referee_Decode(USART6_DMA_RX_BUF);                                         //�������ݽ��뺯��
		}
	}
}

void Referee_Decode(uint8_t *pData)
{
	if (pData == NULL)
	{
		return;
	}
	
	
}
