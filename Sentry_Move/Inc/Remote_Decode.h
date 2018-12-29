#ifndef _REOMTE__DECODE_H_
#define _REOMTE__DECODE_H_
#include "stm32f4xx_hal.h"

/**********************遥控器输入********************/
#define REMOTE_INPUT					1u
#define KEY_MOUSE_INPUT					3u
#define REMOTE_CHASIS_SPEED_TO_REF		1.197f          //1219*0.5/660f
#define REMOTE_PITCH_ANGLE_TO_REF		0.004f
#define REMOTE_YAW_ANGLE_TO_REF			0.004f
#define ROTATE_TO_REF					0.8f

#define RC_CH_VALUE_MIN					((uint16_t)364)
#define RC_CH_VALUE_OFFSET				((uint16_t)1024)
#define RC_CH_VALUE_MAX					((uint16_t)1684)
#define RC_CH_VALUE_RANGE				660.0f

#define RC_SW_UP						((uint16_t)1)
#define RC_SW_MID						((uint16_t)3)
#define RC_SW_DOWN						((uint16_t)2)

#define RC_FRAME_LENGTH					18u

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
	} remote;
	
	struct
	{
		int16_t  x;//鼠标x
		int16_t  y;//鼠标y
		int16_t  z;//鼠标z
		uint8_t  press_l;//鼠标左键
		uint8_t  press_r;//鼠标右键
	} mouse;
	
	struct
	{
		uint16_t v;//键盘
	} key;
	
}RemoteCtrl_t;

extern RemoteCtrl_t RemoteCtrlData;
extern uint8_t isRevRemoteData;
void RC_DataHandle(uint8_t *pData);

#endif
