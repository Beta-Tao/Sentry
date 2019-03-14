#include "usart.h"
#include "string.h"
#include "Master_Comm.h"

uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
uint32_t rx_data_len = 0;		//本次接收长度
MasterData_t masterData;

void Master_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //开启不定长中断
	HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}

void Master_RevData(void)
{
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);												//清除空闲中断的标志
		(void)UART8->SR;                                                             //清空SR寄存器
		(void)UART8->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //清除 DMA1_Steam6传输完成标志
		HAL_UART_DMAStop(&huart8);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_UART8_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == COMM_FRAME_LEN && UART8_DMA_RX_BUF[0] == MASTER_FRAME_HEAD)                                           //判断数据是否为正确的数据长度
		{
			Master_Decode(UART8_DMA_RX_BUF);                                        //进入数据解码函数
		}
	}
}

void Master_Decode(uint8_t *pData)
{
	if (pData == NULL)		//数据错或帧头错误
	{
		return;
	}
	
	memcpy(&masterData, pData + 1, sizeof(MasterData_t));		//进行数据复制
}
