/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "Remote_Comm.h"
#include "Chassis_Ctrl.h"

/* 用于CAN配置，CubeMX生成的版本偏前 */
/*
  hcan1.Init.SJW = CAN_SJW_1TQ;
  hcan1.Init.BS1 = CAN_BS1_14TQ;
  hcan1.Init.BS2 = CAN_BS2_6TQ;
  hcan1.Init.TTCM = DISABLE;
  hcan1.Init.ABOM = DISABLE;
  hcan1.Init.AWUM = DISABLE;
  hcan1.Init.NART = ENABLE;
  hcan1.Init.RFLM = DISABLE;
  hcan1.Init.TXFP = ENABLE;
 */

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
    Error_Handler();
  }

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();
  
    __HAL_RCC_GPIOD_CLK_ENABLE();
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

void CAN_CommInit(CAN_HandleTypeDef* canHandle)
{
	__HAL_CAN_ENABLE_IT(canHandle, CAN_IT_FMP0);		//打开中断
	CANFilter_Init(canHandle);							//初始化过滤器
}

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

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
