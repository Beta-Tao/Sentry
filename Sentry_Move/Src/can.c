/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "motor.h"
#include "PID.h"

static int8_t revCount = 0;
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_14TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = ENABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = ENABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 2);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */
  
	__HAL_CAN_ENABLE_IT(canHandle, CAN_IT_FMP0);
  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/**
  *	@brief	初始化CAN的过滤器
  *	@param	hcan:	CAN_HandleTypeDef结构体的指针
  *	@retval	None
  */
void CANFilter_Init(CAN_HandleTypeDef* hcan)
{
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	
	CAN_FilterConfTypeDef Canfilter;
	Canfilter.FilterMode = CAN_FILTERMODE_IDMASK;
	Canfilter.FilterScale = CAN_FILTERSCALE_32BIT;
	
	//filtrate any ID you want hereCanfilter.FilterIdHigh = 0x0000;
	Canfilter.FilterIdLow = 0x0000;
	Canfilter.FilterMaskIdHigh = 0x0000;
	Canfilter.FilterMaskIdLow = 0x0000;
	
	Canfilter.FilterFIFOAssignment = CAN_FilterFIFO0;	//send the message to FIFO0
	Canfilter.FilterActivation = ENABLE;
	Canfilter.BankNumber = 14;
	
	Canfilter.FilterNumber = 0;
	hcan->pTxMsg = &TxMessage;
	hcan->pRxMsg = &RxMessage;
	
	//CAN1 and CAN2 have same filter
	HAL_CAN_ConfigFilter(hcan, &Canfilter);
}

/**
  *	@brief	receive callback function
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	//如果是CAN1
	if (hcan == &hcan1)
	{
		if(hcan->pRxMsg->IDE == CAN_ID_STD && hcan->pRxMsg->RTR == CAN_RTR_DATA)
		{
			CAN_MotorRxMsgConv(hcan);
			if (hcan->pRxMsg->StdId == CHASSIS_L_ID)	//判断是否是底盘左电机ID
			{
				/* 计算PID，并通过CAN线给电机赋值 */
				PID_Calc(&CMSpeedPID_L, chassisL.rawRotateSpeed, chassisL.refRotateSpeed);
				CAN_MotorTxMsgConv(hcan, CMSpeedPID_L.output, CMSpeedPID_R.output, 0, 0);
				CAN_SendMsg(hcan, FIRST_FOUR_ID);
			}
			if (hcan->pRxMsg->StdId == CHASSIS_R_ID)	//判断是否是底盘右电机ID
			{
				PID_Calc(&CMSpeedPID_R, chassisR.rawRotateSpeed, chassisR.refRotateSpeed);
				CAN_MotorTxMsgConv(hcan, CMSpeedPID_L.output, CMSpeedPID_R.output, 0, 0);
				CAN_SendMsg(hcan, FIRST_FOUR_ID);
			}
			
			/* 通过串口发送调试数据 */
			revCount++;
			if (revCount >= 30)
			{
				PID_Debug(chassisL.rawRotateSpeed, chassisL.rawPos, 0,
						  chassisR.rawRotateSpeed, chassisR.rawPos, 0,
						  0, 0, 0, 
						  0);
				revCount = 0;
			}
		}
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);		//重新打开CAN中断
	}
}

/**
  *	@brief	从CAN线发送某个ID的信息，固定为RTR格式的数据帧
  *	@param	hcan:	CAN_HandleTypeDef结构体指针，给pTxMsg中的参数赋值
  *	@param	ID:		数据帧的ID值
  *	@retval	1 发送成功
  *			0 发送失败
  */
uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan, uint32_t ID)
{
	hcan->pTxMsg->StdId = ID;
	hcan->pTxMsg->ExtId = ID;
	hcan->pTxMsg->IDE = CAN_ID_STD;		//标准格式
	hcan->pTxMsg->RTR = CAN_RTR_DATA;	//数据帧
	hcan->pTxMsg->DLC = DLC_LEN;
	
	if(HAL_CAN_Transmit(hcan, 10) != HAL_OK)
		return 0;
	
	return 1;
}

/**
	*	@brief	转换发送给Motor的数据格式
	*	@param	ID1Msg message sent to the ID1 C610
	*	@param	ID2Msg message sent to the ID2 C610
	*	@param	ID3Msg message sent to the ID3 C610
	*	@param	ID4Msg message sent to the ID4 C610
	*	@retval None
	*/
void CAN_MotorTxMsgConv(CAN_HandleTypeDef* hcan, int16_t ID1Msg, int16_t ID2Msg, 
									   int16_t ID3Msg, int16_t ID4Msg)
{
	hcan->pTxMsg->Data[0] = (uint8_t)(ID1Msg >> 8);
	hcan->pTxMsg->Data[1] = (uint8_t)ID1Msg;
	
	hcan->pTxMsg->Data[2] = (uint8_t)(ID2Msg >> 8);
	hcan->pTxMsg->Data[3] = (uint8_t)ID2Msg;
	
	hcan->pTxMsg->Data[4] = (uint8_t)(ID3Msg >> 8);
	hcan->pTxMsg->Data[5] = (uint8_t)ID3Msg;
	
	hcan->pTxMsg->Data[6] = (uint8_t)(ID4Msg >> 8);
	hcan->pTxMsg->Data[7] = (uint8_t)ID4Msg;
}

/**
  *	@brief	转换发送给Motor的数据格式 
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
void CAN_MotorRxMsgConv(CAN_HandleTypeDef* hcan)
{
	/* 如果是从CAN1发送来的数据 */
	if (hcan == &hcan1)
	{
		switch(hcan->pRxMsg->StdId)
		{
		case CHASSIS_L_ID:
			/* 获取电机数据 */
			chassisL.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
			chassisL.posBuf[1] = chassisL.posBuf[0];
			chassisL.posBuf[0] = chassisL.rawPos;
			chassisL.rawRotateSpeed = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
			break;
		case CHASSIS_R_ID:
			chassisR.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
			chassisR.posBuf[1] = chassisR.posBuf[0];
			chassisR.posBuf[0] = chassisR.rawPos;
			chassisR.rawRotateSpeed = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
			break;
		}
	}
}



/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
