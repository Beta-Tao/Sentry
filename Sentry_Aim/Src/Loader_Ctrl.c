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
	
	loader->loadVel = 450;				//�����͵�
	loader->mode = LOADER_RUN;
}

/**
  * @brief	���¹���ģʽ
  * @note	�ڹ���������߽����ж���
  * @retvel	None
  */
void Loader_UpdateState(Loader_t *loader)
{
	loader->mode = (loader->mode == LOADER_INIT) ? LOADER_INIT : masterData.loaderMode;
	loader->loadVel = masterData.loadVel;
	
	static uint8_t covCount = 0, trigCount = 0;			//�ж��Ƿ񿨵��ļ���λ����������Լ�����ת��ʱ������
	switch(loader->mode)
	{
		case LOADER_RUN:							//����״̬
			if (Loader_JudgeJam(loader) == LOADER_JAM)
				loader->mode = LOADER_JAM;
			break;
		case LOADER_JAM:							//����״̬
			covCount++;								//��ʼ��ת����
			if (covCount >= 200)						//��ת�ﵽʮ�����ڣ���ת����
			{
				loader->mode = LOADER_RUN;
				covCount = 0;
			}
			break;
		case LOADER_INIT:				//����ж϶�ת���
			if (Loader_JudgeJam(loader) == LOADER_JAM)
			{
				loader->mode = LOADER_JAM;
				break;
			}
		
			if (HAL_GPIO_ReadPin(ballTrig_GPIO_Port, ballTrig_Pin) == GPIO_PIN_SET)
				trigCount++;
			else
				trigCount = 0;
			
			if (trigCount == 2)				//�ж�Ϊ���ӵ�������
			{
				loader->mode = LOADER_STOP;
				trigCount = 0;
			}
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
		case LOADER_RUN:				//����ģʽ
			Motor_SetVel(&(motor->velCtrl), sentryLoader.loadVel);
			break;
		case LOADER_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);
			break;
		case LOADER_JAM:				//��תģʽ����ʼ��ת
			Motor_SetVel(&(motor->velCtrl), LOADER_JAM_VEL);
			break;
		case LOADER_INIT:
			Motor_SetVel(&(motor->velCtrl), LOADER_INIT_VEL);
			break;
		default:
			 break;
	}
	Motor_VelCtrl(&(motor->velCtrl));
}

LoaderMode_e Loader_JudgeJam(Loader_t *loader)
{
	static uint8_t jamCount;
	if (loader->LM.velCtrl.refVel != 0 && 
				loader->LM.velCtrl.rawVel <= 20 && loader->LM.velCtrl.rawVel >= -20)
										//����ģʽ������ת�ٲ�Ϊ0��ʵ��ת�ٽ�С���ж�Ϊ��ת״̬
	{
		jamCount++;
		if (jamCount >= 10)
		{
			jamCount = 0;
			return LOADER_JAM;
		}
	}
	else
	{
		jamCount = 0;
	}
	return loader->mode;
}
