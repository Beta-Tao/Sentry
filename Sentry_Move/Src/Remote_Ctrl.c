/**
  * @file       Remote_Ctrl.c
  * @brief      遥控器控制命令转换
  */
#include "usart.h"
#include "can.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "Motor_Ctrl.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的遥控器数据

uint32_t rx_data_len = 0;

/**
  * @brief	对应的遥控器解码函数
  * @param	None
  * @retval	None
  */
void Remote_Process(void)
{
	if (isRevRemoteData == 0)						//遥控器没有打开的时候，全部机构停止
	{
		g_MoveMode = SENTRY_STOP;
		g_AimMode = SENTRY_STOP;
	}
	else													//否则根据开关状态改变运动模式
	{
		switch (RemoteCtrlData.remote.s1)
		{
			case RC_SW_UP:		//当s1在上时，为巡逻模式
				g_MoveMode = SENTRY_DETECT;
				break;
			case RC_SW_MID:		//当s1在中时，为遥控模式
				g_MoveMode = SENTRY_REMOTE;
				break;
			case RC_SW_DOWN:	//当s1在下时，为躲避模式
				g_MoveMode = SENTRY_DODGE;
				break;
		}
		
		switch (RemoteCtrlData.remote.s2)
		{
			case RC_SW_UP:							//当s2在上时，为追踪模式
				g_AimMode = SENTRY_TRACE;
				break;
			case RC_SW_MID:							//当s2在中时，为遥控模式
				g_AimMode = SENTRY_REMOTE;
				break;
			case RC_SW_DOWN:						//当s2在下时，为停止模式
				g_AimMode = SENTRY_STOP;
				break;
		}
		
		isRevRemoteData = 0;	//处理完数据之后标志位置0，表示没有接受到数据
	}
}

/**
  *函数名称：RemotreCtl_Data_Receive
  *函数功能：接收遥控器数据，位于USART1的中断函数中
  *入口参数：无
  *返回值  ：无
  */
void RemoteCtl_Data_Receive(void)
{
	//uint32_t rx_data_len = 0;															//本次接收长度
	isRevRemoteData = 1;																//接收到数据
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

/**
  *函数名称：RemotreCtl_Data_Receive
  *函数功能：接收遥控器数据，位于USART1的中断函数中
  *入口参数：无
  *返回值  ：无
  */
void RemoteCtl_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);                                   //开启不定长中断
	HAL_UART_Receive_DMA(&huart1, USART1_DMA_RX_BUF, BSP_USART1_DMA_RX_BUF_LEN);
}
