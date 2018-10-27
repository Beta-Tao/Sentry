/**
  ******************************************************************************
  * @file       Transmit_Receive.c
  * @brief      所有通信的接受发送文件     
  ****************************************************************************
  */
#include "usart.h"
#include "can.h"
#include "Transmit_Receive.h"
#include "Remote_Decode.h"
#include "macro.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的遥控器数据
 
/***
	*函数名称：RemotreCtl_Data_Receive
	*函数功能：接收遥控器数据，位于USART1的中断函数中
	*入口参数：无
	*返回值  ：无
***/
void RemoteCtl_Data_Receive(void)
{
	uint32_t rx_data_len = 0;															//本次接收长度
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);												//清除空闲中断的标志
		(void)USART1->SR;                                                             //清空SR寄存器
		(void)USART1->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_usart1_rx,DMA_FLAG_TCIF2_6);                       //清除 DMA2_Steam2传输完成标志
		HAL_UART_DMAStop(&huart1);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_USART1_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == RC_FRAME_LENGTH)                                           //判断数据是否为正确的数据长度
		{
			RC_DataHandle(USART1_DMA_RX_BUF);                                        //进入数据解码函数
		}
	}
}

/***
	*函数名称：RemotreCtl_Data_Receive
	*函数功能：接收遥控器数据，位于USART1的中断函数中
	*入口参数：无
	*返回值  ：无
***/
void RemoteCtl_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //开启不定长中断
	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
