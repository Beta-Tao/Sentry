#include "motor.h"
#include "macro.h"
#include <string.h>

volatile Motor_t chassisL;	//���ֵ��
volatile Motor_t chassisR;	//���ֵ��

int16_t chassisSpeedRef;
int16_t chassisSpeedDetect = 150;

/**
  * @brief	������������ٶ�ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetRotateSpeed(volatile Motor_t *motor, int16_t speed)
{
	motor->refRotateSpeed = speed;
}

/**
  * @brief	�����������λ��ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	pos:	Ԥ���λ��ֵ
  * @note	ע����ת��λ�÷�Χ
  * @retval	None
  */
void Motor_SetPos(Motor_t *motor, int16_t pos)
{
	motor->refPos = pos;
}

/**
  * @brief	��ң����ͨ��ֵӳ�䵽���̵���ٶ�
  * @retval	None
  */
void Motor_UpdateCMRef(void)
{
	Motor_SetRotateSpeed(&chassisL, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
	Motor_SetRotateSpeed(&chassisR, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
}
