#ifndef _REMOTE_CTRL_H_
#define _REMOTE_CTRL_H_

#include "stm32f4xx_hal.h"

extern uint8_t autoMode;
extern uint8_t shootMode;

void Remote_Process(void);
void Ctrl_Init(void);

#endif
