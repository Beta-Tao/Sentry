#include "AHRS_Update.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

AHRS_t Gimbal_t;			//��̨�ṹ��

uint8_t USART6_DMA_RX_BUF[BSP_UART6_DMA_RX_BUF_LEN];

void AHRS_Init(void)
{
	
}

void AHRS_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_UART6_DMA_RX_BUF_LEN);
}

void AHRS_RevData(void)
{
	uint8_t rx_data_len = 0;
	
	if((__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);												//��������жϵı�־
		(void)USART6->SR;                                                             //���SR�Ĵ���
		(void)USART6->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);                       //��� DMA2_Steam2������ɱ�־
		HAL_UART_DMAStop(&huart6);                                                    //��������Ժ�رմ���DMA
		rx_data_len = BSP_UART6_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //��ȡ��һ����������С���ܳ���-�����ĳ��ȣ�
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_UART6_DMA_RX_BUF_LEN);  //��������
		if (rx_data_len == AHRS_FRAME_LEN * 2)                                       //�ж������Ƿ�Ϊ��ȷ�����ݳ���
		{
			AHRS_Decode(USART6_DMA_RX_BUF);                                        //�������ݽ��뺯��
		}
	}
}

void AHRS_Decode(uint8_t *pData)
{
	uint8_t frameLoc, frameCount;
	if (pData == NULL)	//���ݿ������
	{
		return;
	}
	
	for (frameCount = 0; frameCount < FRAME_NUM; frameCount++)
	{
		if (pData[frameCount * AHRS_FRAME_LEN] == 0x55)			//֡ͷ��ȷ
		{
			frameLoc = frameCount * AHRS_FRAME_LEN;				//ֻ��Ϊ��֮������
			switch(pData[frameLoc + 1])
			{
				case 0x53:			//�Ƕȣ�ע������ʵ���������
					Gimbal_t.rawPitch = (((pData[frameLoc + 3] << 8) | pData[frameLoc + 2]) / 
													32768.0f * 180);
					/* ���з������ݷ�Χ�ĵ��� */
					if (Gimbal_t.rawPitch > 180.0f)
						Gimbal_t.rawPitch -= 360.0f;
					Gimbal_t.rawPitch = -Gimbal_t.rawPitch;
					
					Gimbal_t.rawRoll = (((pData[frameLoc + 5] << 8) | pData[frameLoc + 4]) / 
													32768.0f * 180);
					Gimbal_t.rawYaw = (((pData[frameLoc + 7] << 8) | pData[frameLoc + 6]) / 
													32768.0f * 180);
					break;
				case 0x54:			//������
					Gimbal_t.rawMagX = (pData[frameLoc + 3] << 8) | pData[frameLoc + 2];
					Gimbal_t.rawMagY = (pData[frameLoc + 5] << 8) | pData[frameLoc + 4];
					Gimbal_t.rawMagZ = (pData[frameLoc + 7] << 8) | pData[frameLoc + 6];
					break;
				case 0x51:			//���ٶ�
					Gimbal_t.rawAccX = ((pData[frameLoc + 3] << 8) | pData[frameLoc + 2]) / 
											32768.0f * 2;
					Gimbal_t.rawAccY = ((pData[frameLoc + 5] << 8) | pData[frameLoc + 4]) / 
											32768.0f * 2;
					Gimbal_t.rawAccZ = ((pData[frameLoc + 7] << 8) | pData[frameLoc + 6]) / 
							 				32768.0f * 2;
					break;
				case 0x52:
					Gimbal_t.rawVelX = ((pData[frameLoc + 3] << 8) | pData[frameLoc + 2]) / 
													32768.0f * 2000;
					/* ���з������ݷ�Χ�ĵ��� */
					if (Gimbal_t.rawVelX > 2000.0f)
						Gimbal_t.rawVelX -= 4000.0f;
					Gimbal_t.rawVelX = -Gimbal_t.rawVelX;
					
					Gimbal_t.rawVelY = ((pData[frameLoc + 5] << 8) | pData[frameLoc + 4]) / 
													32768.0f * 2000;
					if (Gimbal_t.rawVelY > 2000.0f)
						Gimbal_t.rawVelY -= 4000.0f;
					Gimbal_t.rawVelZ = ((pData[frameLoc + 7] << 8) | pData[frameLoc + 6]) / 
													32768.0f * 2000;
					if (Gimbal_t.rawVelZ > 2000.0f)
						Gimbal_t.rawVelZ -= 4000.0f;
					break;
				default:
					break;
			}
		}
		else
			continue;
	}
}
