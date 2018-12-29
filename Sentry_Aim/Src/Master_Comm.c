#include "Master_Comm.h"
#include "usart.h"
#include "Motor_Ctrl.h"
#include "AHRS_Update.h"
#include "string.h"

uint8_t UART8_DMA_RX_BUF[BSP_UART8_DMA_RX_BUF_LEN];
uint32_t rx_data_len = 0;		//本次接收长度
uint8_t isRevData = 0;

void Comm_RevStart(void)
{
	__HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);                                   //开启不定长中断
	HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);
}

void Comm_RevData(void)
{
	//uint32_t rx_data_len = 0;		//本次接收长度
	isRevData = 1;					//表示接收到了数据
	if((__HAL_UART_GET_FLAG(&huart8,UART_FLAG_IDLE)!=RESET)) 
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart8);												//清除空闲中断的标志
		(void)UART8->SR;                                                             //清空SR寄存器
		(void)UART8->DR;                                                             //清空DR寄存器
		__HAL_DMA_CLEAR_FLAG(&hdma_uart8_rx, DMA_FLAG_TCIF2_6);                       //清除 DMA1_Steam6传输完成标志
		HAL_UART_DMAStop(&huart8);                                                    //传输完成以后关闭串口DMA
		rx_data_len = BSP_UART8_DMA_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_uart8_rx); //获取这一次数据量大小（总长度-保留的长度）
		HAL_UART_Receive_DMA(&huart8, UART8_DMA_RX_BUF, BSP_UART8_DMA_RX_BUF_LEN);  //接受数据
		if (rx_data_len == COMM_FRAME_LEN)                                           //判断数据是否为正确的数据长度
		{
			Comm_Decode(UART8_DMA_RX_BUF);                                        //进入数据解码函数
		}
		isRevData = 0;
	}
}

void Comm_Decode(uint8_t *pData)
{
	
	if (pData == NULL || pData[0] != 0x50)		//数据错或帧头错误
	{
		return;
	}
	
	memcpy(&(Gimbal_t.relaYaw), pData + 1, 4);			//注意这里不能使用下面这种方式，会直接进入错误处理函数死循环
														//猜测是因为这里的强制转换改变了源字符串的属性
	
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
