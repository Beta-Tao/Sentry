#include "Gimbal_Ctrl.h"
#include "AHRS_Update.h"

Motor_t GM_Yaw;		//Pitch����
Motor_t GM_Pitch;			//Yaw����

float refVel = 0;

/**
  * @brief	���̿��Ƴ�ʼ��
  * @note	���̿���ģʽ�Լ����̵����ʼ��
  * @retval	None
  */
void Gimbal_CtrlInit(void)
{
	//���̵����������ͳ�ʼ�������̵��ֻ��Ҫ�ٶȱջ�
	GM_Yaw.motorType = M_3508;
	GM_Yaw.escType = C620;
	Motor_VelCtrlInit(&GM_Yaw, 
					  GM_YAW_MOTOR_ACC, GM_YAW_MOTOR_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  10, 0.5, 0 								//kp, ki, kd 10 0.5 0
					  );
	
	GM_Pitch.motorType = M_2006;
	GM_Pitch.escType = C610;
	Motor_VelCtrlInit(&GM_Pitch, 
					  GM_PITCH_MOTOR_ACC, GM_PITCH_MOTOR_DEC, 			//acc, dec
					  35, 0.1, 0		 									//kp, ki, kd
					  );
}

/**
  * @brief	��̨�������
  * @note	��̨���ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	switch (g_AimMode)
	{
		case SENTRY_STOP:							//ֹͣ״̬�򱣳־�ֹ
			Motor_SetVel(&(motor->velCtrl), refVel);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case SENTRY_REMOTE:							//ң��ģʽ������ٶȱջ�
		{
			/* ���ݵ�����ͽ��и�ֵ */
			if (motor == &GM_Yaw)
				Motor_SetVel(&(motor->velCtrl), Gimbal_t.refYawVel);
			else if (motor == &GM_Pitch)
				Motor_SetVel(&(motor->velCtrl), Gimbal_t.refPitchVel);
			else 
				break;
			
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		}
		default:									//����״̬����λ�ñջ�
			/*Motor_SetPos(&(GM_Yaw.posCtrl), Gimbal_t.relaYaw);
			Motor_PosCtrl(&(GM_Yaw.posCtrl));
			Motor_SetVel(&(GM_Yaw.velCtrl), GM_Yaw.posCtrl.output);
			Motor_VelCtrl(&(GM_Yaw.velCtrl));*/
			break;
	}
}
