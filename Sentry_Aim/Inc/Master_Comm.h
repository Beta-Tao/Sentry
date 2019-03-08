#ifndef _MASTER_COMM_H_
#define _MASTER_COMM_H_

#include "stm32f4xx_hal.h"

#define MASTER_FRAME_HEAD			0x50
#define BSP_UART8_DMA_RX_BUF_LEN 	18u		//注意此处只能使用19u，只能用相同的传输缓存大小保证数据帧格式正确
#define COMM_FRAME_LEN				BSP_UART8_DMA_RX_BUF_LEN	
//帧头 (float)refRelaYaw (float)refRelaPitch (float)refYawVel (float)refPitchVel
//(uint8_t)GimbalMode

void Master_Data_Receive_Start(void);

void Master_RevData(void);

void Master_Decode(uint8_t *pData);

#endif
