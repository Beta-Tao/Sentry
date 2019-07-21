#include "IMU_Comm.h"
#include "usart.h"
#include "CRC.h"

IMUData_t HIData;
uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];

void IMU_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //�����������ж�
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}

void IMU_CommInit(void)
{
	HIData.yaw = 0.0f;
	HIData.yawVel = 0.0f;
	HIData.lastRawYaw = HIData.rawYaw;
}

void IMU_Data_Receive(void)
{
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //��������жϵı�־
		(void)USART6->SR;                                                             //���SR�Ĵ���
		(void)USART6->DR;                                                             //���DR�Ĵ���
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);                       //���DMA1_Steam1������ɱ�־
		HAL_UART_DMAStop(&huart6);                                                    //��������Ժ�رմ���DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //��������
		if(USART6_DMA_RX_BUF[0] == PRE && USART6_DMA_RX_BUF[1] == TYPE)			//У��֡ͷ
		{
			IMU_Decode(USART6_DMA_RX_BUF);                                         //�������ݽ��뺯��
		}
	}
}

void IMU_Decode(uint8_t *pData)
{
	uint8_t frameHead, idLoc;
	uint16_t HIDataLen/*, HIDataCRC*/;
	for(frameHead = 0; frameHead < 20; frameHead++)
	{
		if(pData[frameHead] == 0x5A && pData[frameHead + 1] == 0xA5)
		{
			HIDataLen = (uint16_t)pData[frameHead + 2] | (uint16_t)(pData[frameHead + 3] << 8);
			if (HIDataLen > 15)
				break;
//			HIDataCRC = (uint16_t)pData[frameHead + 4] | (uint16_t)(pData[frameHead + 5] << 8);	//��У��
			for(idLoc = 0; idLoc < HIDataLen; idLoc++)
			{
				switch(pData[frameHead + 6 + idLoc])
				{
					case 0x90:
						idLoc += 1;
						break;
					case 0xA0:
						idLoc += 6;
						break;
					case 0xA5:
						idLoc += 6;
						break;
					case 0xB0:
						HIData.yawVel = 
							(float)(int16_t)(pData[frameHead + 6 + idLoc + 6] << 8 | pData[frameHead + 6 + idLoc + 5]) * 0.1f;
						idLoc += 6;
						break;
					case 0xC0:
						idLoc += 6;
						break;
					case 0xD0:
						HIData.rawYaw = 180.0f + 
							(float)(int16_t)(pData[frameHead + 6 + idLoc + 6] << 8 | pData[frameHead + 6 + idLoc + 5]) * 0.1f;
						idLoc += 6;
						
						HIData.detaRawYaw = HIData.rawYaw - HIData.lastRawYaw;
						if(HIData.detaRawYaw < -300)
							HIData.detaRawYaw += 360.0f;
						else if(HIData.detaRawYaw > 300)
							HIData.detaRawYaw -= 360.0f;
						
						HIData.yaw += HIData.detaRawYaw;
						HIData.lastRawYaw = HIData.rawYaw;
						break;
					case 0xD9:
						idLoc += 12;
						break;
					case 0xD1:
						idLoc += 16;
						break;
					case 0xF0:
						idLoc += 4;
						break;
					default:
						break;
				}
			}
		}
	}
}
