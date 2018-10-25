#ifndef _REMOTE__CTRL_H
#define _REMOTE__CTRL_H


#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t forward_back_ref;
	int16_t left_right_ref;
	int16_t rotate_ref;
}ChassisRef_t;

extern ChassisRef_t ChassisRef;
extern uint8_t AutoMode;

void Remode_Input_Judge(void);
void Remote_Process(void);
void Key_Mouse_Process(void);
void Ctrl_Init(void);

#endif

