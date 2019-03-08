#include "PC_Comm.h"
#include "usart.h"
#include "string.h"

PCFrame_t PCAngle_t;

uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                                       //开启不定长中断
	HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);
}

void PC_Data_Receive(void)
{
	if((__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);                                           //清除空闲中断的标志
		(void)USART3->SR;                                                             //清空SR寄存器
		(void)USART3->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart3_rx,DMA_FLAG_TCIF1_5);                       //清除DMA1_Steam1传输完成标志
		HAL_UART_DMAStop(&huart3);                                                    //传输完成以后关闭串口DMA
		HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);  //接受数据
		if(USART3_DMA_RX_BUF[0] == START_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[1] == START_CHECK_SECOND && 
		   USART3_DMA_RX_BUF[10] == END_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[11] == END_CHECK_SECOND)			//校验包头包尾
		{
			PC_Decode(USART3_DMA_RX_BUF);                                         //进入数据解码函数
		}
	}
}

void PC_Decode(uint8_t *pData)
{
	memcpy(&(PCAngle_t.relaYaw), pData + 2, sizeof(float));			//Yaw轴期望角度
	memcpy(&(PCAngle_t.relaPitch), pData + 6, sizeof(float));		//Pitch轴期望角度
}
