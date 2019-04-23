#include "Gimbal_Ctrl.h"
#include "Master_Comm.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "gpio.h"

Gimbal_t sentryGimbal;
float pitchDetectVel = GM_PITCH_DETECT_VEL;

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
					  0.3, 0, 60,
					  GM_YAW_VEL_MIN, GM_YAW_VEL_MAX, GM_YAW_MAX, GM_YAW_MIN, 1310.77901);	//
	Motor_VelCtrlInit(yawMotor, 
					  GM_YAW_ACC, GM_YAW_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  6, 0.01, 0, 								//kp, ki, kd 10 0.5 0
					  9.60160);

	pitchMotor->motorType = GM6020;
	pitchMotor->escType = GM_6020;
	Motor_PosCtrlInit(pitchMotor,
					  GM_PITCH_ACC,
					  0.6, 0, 50, 
					  GM_PITCH_VEL_MIN, GM_PITCH_VEL_MAX, GM_PITCH_MAX, GM_PITCH_MIN, 22.75278);
	Motor_VelCtrlInit(pitchMotor, 
					  GM_PITCH_ACC, GM_PITCH_DEC, 			//acc, dec
					  220, 2, 0,		 									//kp, ki, kd
					  0.16667);

	gimbal->mode = GIMBAL_YAW_INIT;
}

void Gimbal_UpdateState(Gimbal_t *gimbal)
{
	/* ����״̬ */
	gimbal->mode = (gimbal->mode == GIMBAL_YAW_INIT || 
					gimbal->mode == GIMBAL_PITCH_INIT) ? gimbal->mode : (GimbalMode_e)masterData.gimbalMode;
	
	static uint8_t trigCount = 0;
	switch (gimbal->mode)
	{
		case GIMBAL_YAW_INIT:
			if (HAL_GPIO_ReadPin(YawInit_GPIO_Port, YawInit_Pin) == GPIO_PIN_RESET)
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount >= 3)
			{
				gimbal->mode = GIMBAL_PITCH_INIT;
				gimbal->GM_Yaw.posCtrl.absPos = 0;
				gimbal->GM_Yaw.posCtrl.refRelaPos = 0;
				trigCount = 0;
			}
			break;
		case GIMBAL_PITCH_INIT:
			if (gimbal->GM_Pitch.velCtrl.refVel != 0 && 
				(float)gimbal->GM_Pitch.velCtrl.rawVel / gimbal->GM_Pitch.velCtrl.velRatio <= 1 && 
					(float)gimbal->GM_Pitch.velCtrl.rawVel / gimbal->GM_Pitch.velCtrl.velRatio >= -1)
										//�ж�Ϊ��ת״̬
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount >= 200)
			{ 
				gimbal->mode = GIMBAL_STOP;
				gimbal->GM_Pitch.posCtrl.absPos = 0;
				gimbal->GM_Pitch.posCtrl.refRelaPos = 0;
				Motor_SetPos(&(gimbal->GM_Pitch.posCtrl), GM_PITCH_MAX, ABS);
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
	static uint8_t overRange = 0;
	static float pitchDetectVel = GM_PITCH_DETECT_VEL;
	
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
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetPos(&(motor->posCtrl), masterData.yawAngle, RELA);
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetPos(&(motor->posCtrl), masterData.pitchAngle, RELA);
			else
				break;
			
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_DETECT:								//��̨Ѳ��ģʽ
			if (motor == &(sentryGimbal.GM_Yaw))		//Yaw�������ת
				Motor_SetVel(&(motor->velCtrl), GM_YAW_DETECT_VEL);
			if (motor == &(sentryGimbal.GM_Pitch))
			{
				if ((motor->posCtrl.absPos >= motor->posCtrl.posMax - 5.0f) || 
					(motor->posCtrl.absPos <= motor->posCtrl.posMin + 40.0f))
				{
					if (overRange == 0)
					{
						pitchDetectVel = -pitchDetectVel;
						overRange = 1;
					}
				}
				else
					overRange = 0;
				
				Motor_SetVel(&(motor->velCtrl), pitchDetectVel);
			}
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case GIMBAL_TRACE:
			if (motor == &(sentryGimbal.GM_Yaw))
				Motor_SetPos(&(motor->posCtrl), masterData.yawAngle, (PosCtrlType_e)masterData.posCtrlType);
			else if (motor == &(sentryGimbal.GM_Pitch))
				Motor_SetPos(&(motor->posCtrl), masterData.pitchAngle, (PosCtrlType_e)masterData.posCtrlType);
			else
				break;
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
			break;
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
			}
			break;
		default:
			break;
	}
}
