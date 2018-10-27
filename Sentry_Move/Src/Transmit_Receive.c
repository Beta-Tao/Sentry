/**
  ******************************************************************************
  * @file       Transmit_Receive.c
  * @brief      ����ͨ�ŵĽ��ܷ����ļ�     
  ****************************************************************************
  */
#include "usart.h"
#include "can.h"
#include "Transmit_Receive.h"
#include "Remote_Decode.h"
#include "macro.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ���ң��������
 
/***
	*�������ƣ�RemotreCtl_Data_Receive
	*�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
	*��ڲ�������
	*����ֵ  ����
***/
void RemoteCtl_Data_Receive(void)
{
	uint32_t rx_data_len = 0;															//���ν��ճ���
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);												//��������жϵı�־
		(void)USART1->SR;                                                             //���SR�Ĵ���
		(void)USART1->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //��� DMA2_Steam2������ɱ�־
		HAL_UART_DMAStop(&huart1);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == RC_FRAME_LENGTH)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			RC_DataHandle(USART1_DMA_RX_BUF);                                        //�������ݽ��뺯��
		}
	}
}

/***
	*�������ƣ�RemotreCtl_Data_Receive
	*�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
	*��ڲ�������
	*����ֵ  ����
***/
void RemoteCtl_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //�����������ж�
	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
