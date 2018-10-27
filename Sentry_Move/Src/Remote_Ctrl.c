/**
  ******************************************************************************
  * @file       Remote_Ctrl.c
  * @brief      ң������������ת��       
  ****************************************************************************
  */

#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "macro.h"
#include "motor.h"

uint8_t autoMode;	//�Զ�ģʽ��־λ
uint8_t shootMode;	//���估����ģʽ��־λ

/**
  * @brief	��ʼ��������ز���
  * @param	None
  * @note	��ʼ��Ϊң��ģʽ
  * @retval	None
  */
void Ctrl_Init(void)
{
	autoMode = SENTRY_NOAUTO;
}
 
/**
  * @brief	��Ӧ��ң�������뺯��
  * @param	None
  * @retval	None
  */	
void Remote_Process(void)
{
	switch (RemoteCtrlData.remote.s1)
	{
		case RC_SW_UP:		//��s1����ʱ��Ϊ�Զ�ģʽ
			autoMode = SENTRY_AUTO;
			break;
		case RC_SW_MID:		//��s1����ʱ��Ϊң��ģʽ
			autoMode = SENTRY_NOAUTO;
			break;
		case RC_SW_DOWN:	//��s1����ʱ����������
			break;
	}
	switch (RemoteCtrlData.remote.s2)
	{
		case RC_SW_UP:							//��s2����ʱ��Ϊͣ��ģʽ
			shootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_MID:							//��s2����ʱ��Ϊ��׼ģʽ
			shootMode = SENTRY_CEASE_FIRE;
			break;
		case RC_SW_DOWN:						//��s2����ʱ��Ϊ����ģʽ
			shootMode = SENTRY_CEASE_FIRE;
			break;
	}
	
	/* �����������ٶȸ�ֵ */
	chassisSpeedRef = RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET;
	//�������ٶ�����ת��Ϊ����������ٶ�����
	Motor_UpdateCMRef();
	/* ң����̨�ĸ����ͷ�λ�� */
}
