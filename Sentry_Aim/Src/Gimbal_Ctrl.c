#include "Gimbal_Ctrl.h"
#include "AHRS_Update.h"

Motor_t GM_Yaw;		//Pitch����
Motor_t GM_Pitch;			//Yaw����

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
	Motor_PosCtrlInit(&GM_Yaw,
					  GM_YAW_MOTOR_ACC,
					  8, 0, 320,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX);
	Motor_VelCtrlInit(&GM_Yaw, 
					  GM_YAW_MOTOR_ACC, GM_YAW_MOTOR_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  10, 0.15, 0 								//kp, ki, kd 10 0.5 0
					  );
	 
	GM_Pitch.motorType = M_2006;
	GM_Pitch.escType = C610;
	Motor_PosCtrlInit(&GM_Pitch,
					  GM_PITCH_MOTOR_ACC,
					  10, 0, 200,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX);
	Motor_VelCtrlInit(&GM_Pitch, 
					  GM_PITCH_MOTOR_ACC, GM_PITCH_MOTOR_DEC, 			//acc, dec
					  6, 0.3, 0		 									//kp, ki, kd
					  );
}

void Gimbal_UpdateRawValue(void)
{
	GM_Yaw.posCtrl.rawPos = Gimbal_t.rawYaw;
	GM_Pitch.posCtrl.rawPos = Gimbal_t.rawPitch;
	
	/* �����ٶ� */
	if (GM_Pitch.posCtrl.rawPos < -90)		//С��-90ʱоƬ����Yaw�����׼�������෴
		GM_Yaw.velCtrl.rawVel = -Gimbal_t.rawVelZ;
	else
		GM_Yaw.velCtrl.rawVel = Gimbal_t.rawVelZ;
	
	GM_Pitch.velCtrl.rawVel = Gimbal_t.rawVelX;
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
			Motor_SetVel(&(motor->velCtrl), 0);
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
		case SENTRY_TRACE:
		{
			if (motor == &GM_Yaw)
				Motor_SetPos(&(motor->posCtrl), Gimbal_t.relaYaw);
			else if (motor == &GM_Pitch)
				Motor_SetPos(&(motor->posCtrl), Gimbal_t.relaPitch);
			else 
				break;
			
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
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
