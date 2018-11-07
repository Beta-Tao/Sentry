#include "motor.h"
#include "macro.h"
#include "PID.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include <string.h>

volatile Motor_t chassisL;	//���ֵ��
volatile Motor_t chassisR;	//���ֵ��

int16_t chassisSpeedRef;	//ң����ͨ��2��ֵ��ֱ����Ϊ�����ٶ�
int16_t chassisDir = -1;	//������ʾ����

/**
  * @brief	������������ٶ�ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetSpeed(volatile Motor_t *motor, int16_t speed)
{
	motor->refSpeed = speed;
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
	Motor_SetSpeed(&chassisL, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
	Motor_SetSpeed(&chassisR, 
						 (float)(chassisSpeedRef / RC_CH_VALUE_RANGE * CM_ROTATE_SPEED_MAX));
}

/**
  * @brief	���̿���
  * @retval	None
  */
void Motor_CtrChassis(void)
{
	if (RemoteCtrlData.remote.ch2 < RC_CH_VALUE_MIN || 
			RemoteCtrlData.remote.ch2 > RC_CH_VALUE_MAX)	//ң����û�д򿪵�ʱ�򣬵��̲���
	{
		chassisSpeedRef = 0;
		Remote_InitFlag();
	}
	else
	{
		switch (autoMode)
		{
			case SENTRY_REMOTE:		//ң��ģʽ������ٶȺ�ң����ͨ����ֵ�������
				chassisSpeedRef = -(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET);	//����ʵ�����ұ任������
				Motor_UpdateCMRef();
				break;
			case SENTRY_DETECT:		//Ѳ��ģʽ������ٶ�Ϊ��ǰ��Ѳ���ٶ�ֵ
				chassisSpeedRef = SENTRY_DETECT_SPEED * chassisDir;
				Motor_UpdateCMRef();
				break;
		}
	}
}
