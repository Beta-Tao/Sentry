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

uint8_t g_AutoMode;		//自动模式标志位
uint8_t g_ShootMode;	//发射及供弹模式标志位

/**
  * @brief	对应的遥控器解码函数
  * @param	None
  * @retval	None
  */
void Remote_Process(void)
{
	switch (RemoteCtrlData.remote.s1)
	{
		case RC_SW_UP:		//当s1在上时，为巡逻模式
			g_AutoMode = SENTRY_DETECT;
			break;
		case RC_SW_MID:		//当s1在中时，为遥控模式
			g_AutoMode = SENTRY_REMOTE;
			break;
		case RC_SW_DOWN:	//当s1在下时，为躲避模式
			g_AutoMode = SENTRY_DODGE;
			break;
	}
	switch (RemoteCtrlData.remote.s2)
	{
		case RC_SW_UP:							//当s2在上时，为停火模式
			g_ShootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_MID:							//当s2在中时，为瞄准模式
			g_ShootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_DOWN:						//当s2在下时，为开火模式
			g_ShootMode = SENTRY_CEASE_FIRE;
			break;
	}
}

/**
  * @brief	遥控器标志位初始化
  * @note	内含遥控模式以及停火模式
  * @param	None
  * @retval	None
  */
void Remote_InitFlag(void)
{
	g_AutoMode = SENTRY_REMOTE;			//初始化为遥控模式
	g_ShootMode = SENTRY_CEASE_FIRE;		//初始化为停火模式
}

/**
  *函数名称：RemotreCtl_Data_Receive
  *函数功能：接收遥控器数据，位于USART1的中断函数中
  *入口参数：无
  *返回值  ：无
  */
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
