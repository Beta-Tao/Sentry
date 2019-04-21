#include "PC_Comm.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "string.h"

static uint32_t PCTick = 0;
PCComm_t PCComm;

uint8_t USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN];

void PC_Data_Receive_Start(void)
{
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);                                       //开启不定长中断
	HAL_UART_Receive_DMA(&huart3, USART3_DMA_RX_BUF, BSP_USART3_DMA_RX_BUF_LEN);
}

void PC_CommInit(void)
{
	PCComm.PCData.yawAngle = 0;
	PCComm.PCData.pitchAngle = 0;
	PCComm.PCData.posCtrlType = RELA;
	
	PCComm.PCData.distance = 0;
	PCComm.PCData.isTraced = 0;
	PCComm.PCData.isInSight = 0;
	
	PCComm.PCCommState = PC_COMM_DROP;
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
		   USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN - 2] == END_CHECK_FIRST && 
		   USART3_DMA_RX_BUF[BSP_USART3_DMA_RX_BUF_LEN - 1] == END_CHECK_SECOND)			//校验包头包尾
		{
			PCComm.PCCommState = PC_COMM_NORMAL;
 			PCTick = 0;											//清空PCTick以循环
			PC_Decode(USART3_DMA_RX_BUF);                                         //进入数据解码函数
		}
	}
}

void PC_Decode(uint8_t *pData)
{
	memcpy(&(PCComm.PCData.yawAngle), pData + 2, sizeof(float));			//Yaw轴期望角度
	memcpy(&(PCComm.PCData.pitchAngle), pData + 6, sizeof(float));		//Pitch轴期望角度
	memcpy(&(PCComm.PCData.posCtrlType), pData + 10, sizeof(uint8_t));	//位置控制标志位
	
	memcpy(&(PCComm.PCData.distance), pData + 11, sizeof(float));			//目标距离
	
	memcpy(&(PCComm.PCData.isTraced), pData + 15, sizeof(uint8_t));			//跟踪标志位
	memcpy(&(PCComm.PCData.isInSight), pData + 16, sizeof(uint8_t));
	
	DataScope_Debug(2, PCComm.PCData.yawAngle, PCComm.PCData.pitchAngle);
}

void PC_IsCommDrop(void)
{
	PCTick++;
	if (PCTick > PC_IT_CYCLE + 60) 
	{
		PCTick = 0;
		
		PCComm.PCData.yawAngle = 0;
		PCComm.PCData.pitchAngle = 0;
		PCComm.PCData.posCtrlType = RELA;
		
		PCComm.PCData.distance = 0;
		
		PCComm.PCData.isTraced = 0;
		PCComm.PCData.isInSight = 0;
		
		PCComm.PCCommState = PC_COMM_DROP;
	}
}
