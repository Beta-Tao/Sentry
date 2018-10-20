#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "main.h"
#include "stm32f4xx_hal.h"

typedef struct
{
	int16_t rawPos;
	int16_t posBuf;
}Motor_t;

#endif
