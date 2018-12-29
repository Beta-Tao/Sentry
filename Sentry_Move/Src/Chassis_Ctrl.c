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
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
	
	CM_Right.motorType = M_3508;
	CM_Right.escType = C620;
	Motor_VelCtrlInit(&CM_Right, 
					  CHASSIS_MOTOR_ACC, CHASSIS_MOTOR_DEC, 			//acc, dec
					  10, 0, 0 									//kp, ki, kd 20 1.0 1.5
					  );
}

/**
  * @brief	��ң����ͨ��ֵӳ�䵽���̵���ٶ�
  * @retval	None
  */
void Chassis_UpdateRef(void)
{
	Motor_SetVel(&CM_Left.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
	Motor_SetVel(&CM_Right.velCtrl, (float)(chassisVel / RC_CH_VALUE_RANGE * CM_VEL_MAX));
}

/**
  * @brief	���̵������
  * @note	���̵��ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @retvel	None
  */
void Chassis_MotorCtrl(Motor_t *motor)
{
	switch (g_MoveMode)				//�����˶�ģʽ�ı�����ٶ�
	{
		case SENTRY_REMOTE:		//ң��ģʽ������ٶȺ�ң����ͨ����ֵ�������
			chassisVel = -(RemoteCtrlData.remote.ch2 - RC_CH_VALUE_OFFSET);	//����ʵ�����ұ任������
			Chassis_UpdateRef();		//���ݵ����ٶȸ��µ��̵���ٶ�
			break;
		case SENTRY_DETECT:		//Ѳ��ģʽ������ٶ�Ϊ��ǰ��Ѳ���ٶ�ֵ
			chassisVel = SENTRY_DETECT_VEL * chassisDir;	//Ѳ���ٶ�
			Chassis_UpdateRef();
			break;
		case SENTRY_STOP:
			chassisVel = 0;
			Chassis_UpdateRef();
		case SENTRY_DODGE:
			break;
		default:
			break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}
