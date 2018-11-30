#include "Gimbal_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t GM_Pitch;		//��̨Pitch����
Motor_t GM_Yaw;			//��̨Yaw����

/**
  * @brief	���̿��Ƴ�ʼ��
  * @note	���̿���ģʽ�Լ����̵����ʼ��
  * @retval	None
  */
void Gimbal_CtrlInit(void)
{
	/* ��̨�����������ͳ�ʼ������̨��Ҫλ�ñջ����ٶȱջ� */
	GM_Pitch.motorType = M_3508;
	GM_Pitch.escType = C620;
	Motor_VelCtrlInit(&GM_Pitch, 
					  GIMBAL_MOTOR_ACC, GIMBAL_MOTOR_DEC, 	//acc, dec
					  20, 1.0, 1.5, 							//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX				//outputMin, outputMax
					  );
	Motor_PosCtrlInit(&GM_Pitch, 
					  GIMBAL_MOTOR_DEC,						//λ�ÿ��Ƽ��ټ��ٶȺ��ٶȼ��ټ��ٶ�һ��
					  kp, ki, kd,
					  GM_VEL_MIN, GM_VEL_MAX);
	
	GM_Yaw.motorType = M_3508;
	GM_Yaw.escType = C620;
	Motor_VelCtrlInit(&GM_Yaw, 
					  GIMBAL_MOTOR_ACC, GIMBAL_MOTOR_DEC, 			//acc, dec
					  20, 1.0, 1.5, 									//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX						//outputMin, outputMax
					  );
	Motor_PosCtrlInit(&GM_Yaw, 
					  GIMBAL_MOTOR_DEC,
					  kp, ki, kd,
					  GM_VEL_MIN, GM_VEL_MAX);
}

/**
  * @brief	��ң����ͨ��ֵӳ�䵽��̨���λ��
  * @note	ң����������̨����˶���Pitch��Ӧͨ��0��Yaw��Ӧͨ��1
  * @retval	None
  */
void Gimbal_UpdateRef(void)
{ 
	Motor_SetPos(&GM_Pitch.posCtrl, 
				 GM_Pitch.posCtrl.refPos + 
				 (float)(RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / GM_REMOTE_SENSITY);
	Motor_SetPos(&GM_Yaw.posCtrl, 
				 GM_Yaw.posCtrl.refPos + 
				 (float)(RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / GM_REMOTE_SENSITY);
}

/**
  * @brief	������̨ģʽ
  * @note	�ڶ�ʱ�����н��ж�ʱ���ж�
  * @retvel	None
  */
void Gimbal_UpdateState(void)
{
	GM_Pitch.posCtrl.rawPos = g_Pitch;		//AHRS�õ��ķ�λ�Ǹ�ֵ��λ�ñջ�����
	GM_Yaw.posCtrl.rawPos = g_Yaw;
}

/**
  * @brief	���̵������
  * @note	���̵��ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	Motor_PosCtrl(&(motor->posCtrl));
	Motor_SetVel(&motor->velCtrl, motor->posCtrl.output);		//��λ�ñջ��������Ϊ�ٶȱջ�������
	Motor_VelCtrl(&(motor->velCtrl));
}
