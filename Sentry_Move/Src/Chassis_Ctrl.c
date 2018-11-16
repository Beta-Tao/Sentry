#include "Chassis_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t CM_Left;	//���ֵ��
Motor_t CM_Right;	//���ֵ��

int16_t chassisVel;			//ң����ͨ��2��ֵ��ֱ����Ϊ�����ٶȣ����Ƶ����ʱ�����ת��
int16_t chassisDir = -1;	//������ʾ�����˶�����

/**
  * @brief	���̿��Ƴ�ʼ��
  * @note	���̿���ģʽ�Լ����̵����ʼ��
  * @retval	None
  */
void Chassis_CtrlInit(void)
{
	//���̵����������ͳ�ʼ�������̵��ֻ��Ҫ�ٶȱջ�
	CM_Left.motorType = M_3508;
	CM_Left.escType = C620;
	Motor_VelCtrlInit(&CM_Left, 
					  CHASSIS_MOTOR_ACC, CHASSIS_MOTOR_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  20, 1.0, 1.5, 							//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX				//outputMin, outputMax
					  );
	
	CM_Right.motorType = M_3508;
	CM_Right.escType = C620;
	Motor_VelCtrlInit(&CM_Right, 
					  CHASSIS_MOTOR_ACC, CHASSIS_MOTOR_DEC, 			//acc, dec
					  20, 1.0, 1.5, 									//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX						//outputMin, outputMax
					  );
}

/**
  * @brief	��ң����ͨ��ֵӳ�䵽���̵���ٶ�
  * @retval	None
  */
void Chassis_UpdateCMRef(void)
{
	Motor_SetVel(&CM_Left.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
	Motor_SetVel(&CM_Right.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
}

/**
  * @brief	���µ���״̬�������˶�ģʽ�������ٶ�
  * @note	�ڶ�ʱ�����н��ж�ʱ���ж�
  * @retvel	None
  */
void Chassis_UpdateState(void)
{
	if (RemoteCtrlData.remote.ch2 < RC_CH_VALUE_MIN || 
			RemoteCtrlData.remote.ch2 > RC_CH_VALUE_MAX)	//ң����û�д򿪵�ʱ�򣬵��̲���
	{
		chassisVel = 0;
		Remote_InitFlag();
	}
	else
	{
		switch (g_AutoMode)
		{
			case SENTRY_REMOTE:		//ң��ģʽ������ٶȺ�ң����ͨ����ֵ�������
				chassisVel = -(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET);	//����ʵ�����ұ任������
				Chassis_UpdateCMRef();		//���ݵ����ٶȸ��µ��̵���ٶ�
				break;
			case SENTRY_DETECT:		//Ѳ��ģʽ������ٶ�Ϊ��ǰ��Ѳ���ٶ�ֵ
				chassisVel = SENTRY_DETECT_VEL * chassisDir;	//Ѳ���ٶ�
				Chassis_UpdateCMRef();
				break;
			case SENTRY_DODGE:
				break;
			default:
				break;
		}
	}
}

/**
  * @brief	���̵������
  * @note	���̵��ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @retvel	None
  */
 void Chassis_MotorCtrl(Motor_t *motor)
{
	Motor_VelCtrl(&(motor->velCtrl));
}
