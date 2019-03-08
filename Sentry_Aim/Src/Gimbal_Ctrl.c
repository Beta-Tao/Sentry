#include "Gimbal_Ctrl.h"
#include "usart.h"
#include "gpio.h"

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
					  0.01, 0, 0.83,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, 1310.77901);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  6, 0.15, 0, 								//kp, ki, kd 10 0.5 0
					  9.60160);

	pitchMotor->motorType = GM6020;
	pitchMotor->escType = GM_6020;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC,
					  3, 0, 235,
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, 22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, 			//acc, dec
					  80, 20, 0,		 									//kp, ki, kd
					  0.16667);

	gimbal->mode = GIMBAL_YAW_INIT;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	static uint8_t trigCount = 0;
	switch (gimbal->mode)
	{
		case GIMBAL_YAW_INIT:
			if (HAL_GPIO_ReadPin(YawInit_GPIO_Port, YawInit_Pin) == GPIO_PIN_RESET)
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount == 3)
			{
				gimbal->mode = GIMBAL_PITCH_INIT;
				gimbal->GM_Yaw.posCtrl.absPos = 0;
				gimbal->GM_Yaw.posCtrl.refRelaPos = 0;
				trigCount = 0;
			}
			break;
		case GIMBAL_PITCH_INIT:
			if (gimbal->GM_Pitch.velCtrl.refVel != 0 && 
				gimbal->GM_Pitch.velCtrl.rawVel <= 5 && gimbal->GM_Pitch.velCtrl.rawVel >= -5)
										//�ж�Ϊ��ת״̬
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount == 3)
			{
				gimbal->mode = GIMBAL_STOP;
				gimbal->GM_Pitch.posCtrl.absPos = 0;
				gimbal->GM_Pitch.posCtrl.refRelaPos = 0;
				trigCount = 0;
			}
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
		case GIMBAL_STOP:							//ֹͣ״̬�򱣳־�ֹ��λ�ñջ�
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
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
		case GIMBAL_DEBUG_VEL:
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_DEBUG_POS:
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
		case GIMBAL_YAW_INIT:
			if (motor != &(sentryGimbal.GM_Yaw))
				break;
			else
			{
				Motor_SetVel(&(motor->velCtrl), GM_YAW_INIT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
				break;
			}
		case GIMBAL_PITCH_INIT:
			if (motor == &(sentryGimbal.GM_Yaw))		//Yaw���Ѿ���ʼ����ɣ�����λ�ñջ���
			{
				Motor_PosCtrl(&(motor->posCtrl));
				Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
				Motor_VelCtrl(&(motor->velCtrl));
			}
			else
			{
				Motor_SetVel(&(motor->velCtrl), GM_PITCH_INIT_VEL);
				Motor_VelCtrl(&(motor->velCtrl));
				break;
			}
		default:
			break;
	}
}
