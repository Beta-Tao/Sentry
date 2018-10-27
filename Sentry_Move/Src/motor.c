#include "motor.h"
#include "macro.h"
#include <string.h>

volatile Motor_t chassisL;	//左轮电机
volatile Motor_t chassisR;	//右轮电机

int16_t chassisSpeedRef;

/**
  * @brief	给电机赋期望速度值
  * @param	motor:	Motor_t结构体的指针
  * @param	speed:	预设的速度值
  * @retval	None
  * @note	注意电机速度方向和实际需要的方向是否相同
  */
void Motor_SetRotateSpeed(Motor_t *motor, int16_t speed)
{
	motor->refRotateSpeed = speed;
}

/**
  * @brief	给电机赋期望位置值
  * @param	motor:	Motor_t结构体的指针
  * @param	pos:	预设的位置值
  * @note	注意电机转子位置范围
  * @retval	None
  */
void Motor_SetPos(Motor_t *motor, int16_t pos)
{
	motor->refPos = pos;
}

/**
  * @brief	将遥控器通道值映射到底盘电机速度
  * @retval	None
  */
void Motor_UpdateCMRef(void)
{
	chassisL.refRotateSpeed = (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX);
	chassisR.refRotateSpeed = (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX);
}
