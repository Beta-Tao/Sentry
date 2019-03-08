#include "Gimbal_Ctrl.h"

Gimbal_t sentryGimbal;

/**
  * @brief	���̿��Ƴ�ʼ��
  * @note	���̿���ģʽ�Լ����̵����ʼ��
  * @retval	None
  */
void Gimbal_CtrlInit(Gimbal_t *gimbal)
{
	Motor_t *yawMotor  = &(gimbal->GM_Yaw), 
			*pitchMotor = &(gimbal->GM_Pitch);
	//���̵����������ͳ�ʼ�������̵��ֻ��Ҫ�ٶȱջ�
	yawMotor->motorType = M_3508;
	yawMotor->escType = C620;
	Motor_PosCtrlInit(yawMotor,
					  GM_YAW_ACC,
					  0.1, 0, 0,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, 1310.77901);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  20, 0.5, 0 								//kp, ki, kd 10 0.5 0
					  );

	pitchMotor->motorType = M_2006;
	pitchMotor->escType = C610;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC,
					  10.2, 0, 1000,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, 22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, 			//acc, dec
					  6, 0.3, 0		 									//kp, ki, kd
					  );
					  
	gimbal->mode = GIMBAL_INIT;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	switch (gimbal->mode)
	{
		case GIMBAL_INIT:
			break;
		default:
			break;
	}
}

/**
  * @brief	��̨�������
  * @note	��̨���ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	if (motor != &(sentryGimbal.GM_Yaw) && motor != &(sentryGimbal.GM_Pitch))
		return;
	
	switch (sentryGimbal.mode)
	{
		case GIMBAL_STOP:							//ֹͣ״̬�򱣳־�ֹ
			Motor_SetVel(&(motor->velCtrl), 0);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_REMOTE:							//ң��ģʽ������ٶȱջ�
			//�����ٶ���Ϣֱ����Master_Comm.c�н��и���
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_TRACE:
			//λ��״ֱ̬����Master_Comm.c�н��и���
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			break;
	}
}
