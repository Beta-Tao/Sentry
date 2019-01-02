#include "AHRS_Update.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

AHRS_t Gimbal_t;			//云台结构体

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
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);												//清除空闲中断的标志
		(void)USART6->SR;                                                             //清空SR寄存器
		(void)USART6->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_FLAG_TCIF1_5);                       //清除 DMA2_Steam2传输完成标志
		HAL_UART_DMAStop(&huart6);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_UART6_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_UART6_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == AHRS_FRAME_LEN * 2)                                       //判断数据是否为正确的数据长度
		{
			AHRS_Decode(USART6_DMA_RX_BUF);                                        //进入数据解码函数
		}
	}
}

void AHRS_Decode(uint8_t *pData)
{
	uint8_t frameLoc, frameCount;
	if (pData == NULL)	//数据空则出错
	{
		return;
	}
	
	for (frameCount = 0; frameCount < FRAME_NUM; frameCount++)
	{
		if (pData[frameCount * AHRS_FRAME_LEN] == 0x55)			//帧头正确
		{
			frameLoc = frameCount * AHRS_FRAME_LEN;				//只是为了之后代码简单
			switch(pData[frameLoc + 1])
			{
				case 0x53:			//角度，注意依据实际输出更改
					Gimbal_t.rawPitch = (((pData[frameLoc + 3] << 8) | pData[frameLoc + 2]) / 
													32768.0f * 180);
					/* 进行方向及数据范围的调整 */
					if (Gimbal_t.rawPitch > 180.0f)
						Gimbal_t.rawPitch -= 360.0f;
					Gimbal_t.rawPitch = -Gimbal_t.rawPitch;
					
					Gimbal_t.rawRoll = (((pData[frameLoc + 5] << 8) | pData[frameLoc + 4]) / 
													32768.0f * 180);
					Gimbal_t.rawYaw = (((pData[frameLoc + 7] << 8) | pData[frameLoc + 6]) / 
													32768.0f * 180);
					break;
				case 0x54:			//磁力计
					Gimbal_t.rawMagX = (pData[frameLoc + 3] << 8) | pData[frameLoc + 2];
					Gimbal_t.rawMagY = (pData[frameLoc + 5] << 8) | pData[frameLoc + 4];
					Gimbal_t.rawMagZ = (pData[frameLoc + 7] << 8) | pData[frameLoc + 6];
					break;
				case 0x51:			//加速度
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
					/* 进行方向及数据范围的调整 */
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
