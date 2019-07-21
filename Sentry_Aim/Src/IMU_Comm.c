#include "IMU_Comm.h"
#include "usart.h"
#include "CRC.h"

IMUData_t HIData;
uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];

void IMU_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //开启不定长中断
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
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //清除空闲中断的标志
		(void)USART6->SR;                                                             //清空SR寄存器
		(void)USART6->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);                       //清除DMA1_Steam1传输完成标志
		HAL_UART_DMAStop(&huart6);                                                    //传输完成以后关闭串口DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //接受数据
		if(USART6_DMA_RX_BUF[0] == PRE && USART6_DMA_RX_BUF[1] == TYPE)			//校验帧头
		{
			IMU_Decode(USART6_DMA_RX_BUF);                                         //进入数据解码函数
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
//			HIDataCRC = (uint16_t)pData[frameHead + 4] | (uint16_t)(pData[frameHead + 5] << 8);	//不校验
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
