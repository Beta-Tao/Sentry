#include "motor.h"
#include "macro.h"
#include <string.h>

volatile Motor_t chassisL;	//���ֵ��
volatile Motor_t chassisR;	//���ֵ��

int16_t chassisSpeedRef;

/**
  * @brief	������������ٶ�ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetRotateSpeed(Motor_t *motor, int16_t speed)
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
	chassisL.refRotateSpeed = (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX);
	chassisR.refRotateSpeed = (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX);
}
