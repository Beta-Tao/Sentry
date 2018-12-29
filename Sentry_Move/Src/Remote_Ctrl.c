/**
  * @file       Remote_Ctrl.c
  * @brief      ң������������ת��
  */
#include "usart.h"
#include "can.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "Motor_Ctrl.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //����һ���������ڴ�Ŵ�DMA���յ���ң��������

uint32_t rx_data_len = 0;

/**
  * @brief	��Ӧ��ң�������뺯��
  * @param	None
  * @retval	None
  */
void Remote_Process(void)
{
	if (isRevRemoteData == 0)						//ң����û�д򿪵�ʱ��ȫ������ֹͣ
	{
		g_MoveMode = SENTRY_STOP;
		g_AimMode = SENTRY_STOP;
	}
	else													//������ݿ���״̬�ı��˶�ģʽ
	{
		switch (RemoteCtrlData.remote.s1)
		{
			case RC_SW_UP:		//��s1����ʱ��ΪѲ��ģʽ
				g_MoveMode = SENTRY_DETECT;
				break;
			case RC_SW_MID:		//��s1����ʱ��Ϊң��ģʽ
				g_MoveMode = SENTRY_REMOTE;
				break;
			case RC_SW_DOWN:	//��s1����ʱ��Ϊ���ģʽ
				g_MoveMode = SENTRY_DODGE;
				break;
		}
		
		switch (RemoteCtrlData.remote.s2)
		{
			case RC_SW_UP:							//��s2����ʱ��Ϊ׷��ģʽ
				g_AimMode = SENTRY_TRACE;
				break;
			case RC_SW_MID:							//��s2����ʱ��Ϊң��ģʽ
				g_AimMode = SENTRY_REMOTE;
				break;
			case RC_SW_DOWN:						//��s2����ʱ��Ϊֹͣģʽ
				g_AimMode = SENTRY_STOP;
				break;
		}
		
		isRevRemoteData = 0;	//����������֮���־λ��0����ʾû�н��ܵ�����
	}
}

/**
  *�������ƣ�RemotreCtl_Data_Receive
  *�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
  *��ڲ�������
  *����ֵ  ����
  */
void RemoteCtl_Data_Receive(void)
{
	//uint32_t rx_data_len = 0;															//���ν��ճ���
	isRevRemoteData = 1;																//���յ�����
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

/**
  *�������ƣ�RemotreCtl_Data_Receive
  *�������ܣ�����ң�������ݣ�λ��USART1���жϺ�����
  *��ڲ�������
  *����ֵ  ����
  */
void RemoteCtl_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //�����������ж�
	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
