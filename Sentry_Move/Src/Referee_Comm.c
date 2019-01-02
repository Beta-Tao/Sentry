#include "Referee_Comm.h"
#include "usart.h"

uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);                                       //开启不定长中断
	HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);
}

void Referee_Data_Receive(void)
{	                                                          //本次接收长度
	if((__HAL_UART_GET_FLAG(&huart6,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart6);                                           //清除空闲中断的标志
		(void)USART6->SR;                                                             //清空SR寄存器
		(void)USART6->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx,DMA_FLAG_TCIF1_5);                       //清除 DMA2_Steam1传输完成标志
		HAL_UART_DMAStop(&huart6);                                                    //传输完成以后关闭串口DMA
		HAL_UART_Receive_DMA(&huart6, USART6_DMA_RX_BUF, BSP_USART6_DMA_RX_BUF_LEN);  //接受数据
		if(USART6_DMA_RX_BUF[0] == 0xA5)                                              //判断数据包帧头
		{
			Referee_Decode(USART6_DMA_RX_BUF);                                         //进入数据解码函数
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
