#include "Master_Comm.h"
#include "usart.h"
#include "Motor_Ctrl.h"
#include "AHRS_Update.h"
#include "string.h"

uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
uint32_t rx_data_len = 0;		//���ν��ճ���
uint8_t isRevData = 0;

void Comm_RevStart(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //�����������ж�
	HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}

void Comm_RevData(void)
{
	//uint32_t rx_data_len = 0;		//���ν��ճ���
	isRevData = 1;					//��ʾ���յ�������
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);												//��������жϵı�־
		(void)UART8->SR;                                                             //���SR�Ĵ���
		(void)UART8->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //��� DMA1_Steam6������ɱ�־
		HAL_UART_DMAStop(&huart8);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_UART8_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == COMM_FRAME_LEN)                                           //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			Comm_Decode(UART8_DMA_RX_BUF);                                        //�������ݽ��뺯��
		}
		isRevData = 0;
	}
}

void Comm_Decode(uint8_t *pData)
{
	
	if (pData == NULL || pData[0] != 0x50)		//���ݴ��֡ͷ����
	{
		return;
	}
	
	memcpy(&(Gimbal_t.relaYaw), pData + 1, 4);			//ע�����ﲻ��ʹ���������ַ�ʽ����ֱ�ӽ������������ѭ��
														//�²�����Ϊ�����ǿ��ת���ı���Դ�ַ���������
	
	//Gimbal_t.relaYaw = *((float *)(dataBuffer + 1));
	
	memcpy(&(Gimbal_t.relaPitch), pData + 5, 4);
	
	//Gimbal_t.relaPitch = *((float *)(dataBuffer + 5));
	
	memcpy(&(Gimbal_t.refYawVel), pData + 9, 4);
	
	//Gimbal_t.refYawVel = *((float *)(dataBuffer + 9));
	
	memcpy(&(Gimbal_t.refPitchVel), pData + 13, 4);
	
	//Gimbal_t.refPitchVel = *((float *)(dataBuffer + 13));
	
	g_AimMode = pData[17];
	
	g_LoadMode = pData[18];
	
	g_ShootMode = pData[19];
}
