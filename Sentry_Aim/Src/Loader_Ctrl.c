#include "Loader_Ctrl.h"
#include "gpio.h"
#include "Master_Comm.h"

Loader_t sentryLoader;

/**
  * @brief	�������Ƴ�ʼ��
  * @note	���������ʼ�� 
  * @retval	None
  */
void Loader_CtrlInit(Loader_t *loader)
{
	//���������������ͳ�ʼ��
	loader->LM.motorType = M_2006;
	loader->LM.escType = C610;
	Motor_VelCtrlInit(&(loader->LM), 
					  LOADER_ACC, LOADER_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  9, 0.4, 0,  						//kp, ki, kd
					  6.00000);
	
	loader->mode = LOADER_INIT;
}

/**
  * @brief	���¹���ģʽ
  * @note	�ڹ���������߽����ж���
  * @retvel	None
  */
void Loader_UpdateState(Loader_t *loader)
{
	loader->mode = ((loader->mode == LOADER_INIT) || 
					(loader->mode == LOADER_JAM) ? loader->mode : (LoaderMode_e)masterRxData.loaderMode);
	
	static uint32_t covCount = 0;			//�ж��Ƿ񿨵��ļ���λ����������Լ�����ת��ʱ������
	switch(loader->mode)
	{
		case LOADER_RUN_PS3:
		case LOADER_RUN_PS5:
		case LOADER_RUN_PS6:
		case LOADER_RUN_PS8:
		case LOADER_RUN_PS10:
		case LOADER_RUN_PS15:
		case LOADER_RUN_PS20:
			if (Loader_IsJammed(loader) == 1)
			{
				loader->lastMode = loader->mode;		//�ݴ�֮ǰ��״̬
				loader->mode = LOADER_JAM;
			}
			break;
		case LOADER_JAM:							//����״̬
			covCount++;								//��ʼ��ת����
			if (covCount >= 300)						//��ת�ﵽʮ�����ڣ���ת����
			{
				loader->mode = loader->lastMode;
				covCount = 0;
			}
			break;
		case LOADER_INIT:				//����ж϶�ת���
			if (Loader_IsJammed(loader) == 1)
			{
				loader->lastMode = loader->mode;
				loader->mode = LOADER_JAM;
				break;
			}
		
			if (HAL_GPIO_ReadPin(ballTrig_GPIO_Port, ballTrig_Pin) == GPIO_PIN_SET)
				loader->mode = LOADER_STOP;
			break;
		case LOADER_STOP:
//			if (HAL_GPIO_ReadPin(ballTrig_GPIO_Port, ballTrig_Pin) == GPIO_PIN_RESET)	//�յ�ʱ�ϵ�
//				loader->mode = LOADER_INIT;
			break;
		default:									//ͣת״̬
			break;
	}
}

/**
  * @brief	�����������
  * @note	�������ֻ���ٶȿ���
  * @param	motor:	Motor_t�ṹ��ָ��
  * @note	�������ֻ��Ҫ�ٶȱջ�
  * @retvel	None
  */
void Loader_MotorCtrl(Motor_t *motor)
{	
	if (motor != &(sentryLoader.LM))
		return;
	
	switch (sentryLoader.mode)
	{
		case LOADER_INIT:
			Motor_SetVel(&(motor->velCtrl), 2.0f * LOADER_PS1);
			break;
		case LOADER_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);
			break;
		case LOADER_JAM:				//��תģʽ����ʼ��ת
			Motor_SetVel(&(motor->velCtrl), -10.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS3:
			Motor_SetVel(&(motor->velCtrl), 3.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS5:
			Motor_SetVel(&(motor->velCtrl), 5.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS6:
			Motor_SetVel(&(motor->velCtrl), 6.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS8:
			Motor_SetVel(&(motor->velCtrl), 8.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS10:
			Motor_SetVel(&(motor->velCtrl), 10.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS15:
			Motor_SetVel(&(motor->velCtrl), 15.0f * LOADER_PS1);
			break;
		case LOADER_RUN_PS20:
			Motor_SetVel(&(motor->velCtrl), 20.0f * LOADER_PS1);
			break;
		default:
			 break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}

uint8_t Loader_IsJammed(Loader_t *loader)
{
	static uint32_t jamCount;
	if (loader->LM.velCtrl.refVel != 0 && 
				loader->LM.velCtrl.rawVel <= 20 && loader->LM.velCtrl.rawVel >= -20)
										//����ģʽ������ת�ٲ�Ϊ0��ʵ��ת�ٽ�С���ж�Ϊ��ת״̬
	{
		jamCount++;
		if (jamCount >= 60)
		{
			jamCount = 0;
			return 1;
		}
	}
	else
		jamCount = 0;
	return 0;
}
