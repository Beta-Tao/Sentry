#ifndef _REOMTE__DECODE_H_
#define _REOMTE__DECODE_H_
#include "stm32f4xx_hal.h"
typedef struct
{
	struct
	{
		uint16_t ch0;//通道0
		uint16_t ch1;//通道1
		uint16_t ch2;//通道2
		uint16_t ch3;//通道3
		uint8_t  s1;//开关1
		uint8_t  s2;//开关2
	}remote;
	
	struct
	{
		int16_t  x;//鼠标x
		int16_t  y;//鼠标y
		int16_t  z;//鼠标z
		uint8_t  press_l;//鼠标左键
		uint8_t  press_r;//鼠标右键
	}mouse;
	
	struct
	{
		uint16_t v;//键盘
	}key;
}RemoteCtrl_t;



extern RemoteCtrl_t RemoteCtrlData;
void RC_DataHandle(uint8_t *pData);



#endif
