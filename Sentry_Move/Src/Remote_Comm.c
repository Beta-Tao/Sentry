#include "Remote_Comm.h"
#include "usart.h"
#include "Motor_Ctrl.h"
#include "Chassis_Ctrl.h"
#include "Gimbal_Ctrl.h"

uint8_t USART1_DMA_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];  //定义一个数组用于存放从DMA接收到的遥控器数据

RemoteCtrl_t RemoteCtrlData;       //遥控器输入

/**
  * @brief	根据遥控器协议对进行接收到的数据进行处理
  * @param	pData:	一个指向8位数据的指针
  * @retval	None
  */
void RC_DataHandle(uint8_t *pData)
{
	if (pData == NULL)
    {
        return;
    }
	
	/* pData[0]为ch0的低8位，Data[1]的低3位ch0的高3位 */
	RemoteCtrlData.remote.ch0 = (uint16_t)(pData[0] | pData[1] << 8) & 0x07FF;
	
	/* pData[1]的高5位为ch1的低5位，pData[2]的低6位为ch1的高6位 */
	RemoteCtrlData.remote.ch1 = (uint16_t)(pData[1] >> 3 | pData[2] << 5) & 0x07FF;
	
	/* pData[2]的高2位为ch2的低2位, pData[3]为ch2的中8位，pData[4]的低1位为ch2的高1位 */
	RemoteCtrlData.remote.ch2 = (uint16_t)(pData[2] >> 6 | pData[3] << 2 | pData[4] << 10) & 0x07FF;
	
	/* pData[4]的高7位为ch3的低7位，pData[5]的低4位为ch3的高4位 */
	RemoteCtrlData.remote.ch3 = (uint16_t)(pData[4] >> 1 | pData[5] << 7) & 0x07FF;

	/* pData[5]的高2位为s1 */
	RemoteCtrlData.remote.s1  = ((pData[5] >> 6) & 0x03);
	
	/* pData[6]的6，7位为s2 */
	RemoteCtrlData.remote.s2  = ((pData[5] >> 4) & 0x03);
	
	/* pData[6],pData[7]为x */
	RemoteCtrlData.mouse.x    = (int16_t)(pData[6] | pData[7] << 8);
	
	/* pData[8],pData[9]为y */
	RemoteCtrlData.mouse.y    = (int16_t)(pData[8] | pData[9] << 8);
	
	/* pData[10],pData[11]为z */
	RemoteCtrlData.mouse.z    = (int16_t)(pData[10] | pData[11] << 8);
	
	/* pData[12]为左键 */
	RemoteCtrlData.mouse.press_l = pData[12];
	
	/* pData[13]为右键 */
	RemoteCtrlData.mouse.press_r = pData[13];
	
	/* pData[14],pData[15]为键盘值 */
	RemoteCtrlData.key.v 		 = (int16_t)(pData[14] | pData[15] << 8);
	
	/* pData[15], pData[16]为拨轮值 */
	RemoteCtrlData.clickwheel.ch = (uint16_t)(pData[16] | pData[17] << 8) & 0x07FF;
	
	/* 拨动开关进行解码 */
	Remote_Process();
}

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
			sentryChassis.mode = CHASSIS_DETECT;
			break;
		case RC_SW_MID:		//当s1在中时，为遥控模式
			sentryChassis.mode = CHASSIS_REMOTE;
			break;
		case RC_SW_DOWN:	//当s1在下时，为躲避模式
			sentryChassis.mode = CHASSIS_DODGE;
			break;
		default:
			break;
	}
	
	switch (RemoteCtrlData.remote.s2)
	{
		case RC_SW_UP:							//当s2在上时，为追踪模式
			sentryGimbal.mode = GIMBAL_TRACE;
			break;
		case RC_SW_MID:							//当s2在中时，为遥控模式
			sentryGimbal.mode = GIMBAL_REMOTE;
			//g_LoadMode = (g_LoadMode == SENTRY_LOAD_JAM) ? SENTRY_LOAD_JAM : SENTRY_LOAD_RUN;
			break;
		case RC_SW_DOWN:						//当s2在下时，为停止模式
			sentryGimbal.mode = GIMBAL_STOP;
			//g_LoadMode = SENTRY_LOAD_STOP;
			break;
		default:
			break;
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
	uint8_t rx_data_len;															//接收到数据
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
