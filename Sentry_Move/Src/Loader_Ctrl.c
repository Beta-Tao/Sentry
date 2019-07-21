#include "Loader_Ctrl.h"
#include "Remote_Comm.h"
#include "Sentry_Strategy.h"

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
	
	loader->mode = LOADER_STOP;
}

/**
  * @brief	���¹���ģʽ
  * @note	�ڹ���������߽����ж���
  * @retvel	None
  */
void Loader_UpdateState(Loader_t *loader)
{
	if (loader->mode != LOADER_JAM)
	{
		switch (RemoteComm.RemoteData.remote.s2)
		{
			case RC_SW_UP:		//��s1����ʱ��Ϊ�Զ�ģʽ
				loader->mode = (LoaderMode_e)sentryST.loaderMode;
				//sentryChassis.mode = CHASSIS_DETECT_NORMAL;
				//sentryChassis.mode = CHASSIS_DODGE;
				break;
			case RC_SW_MID:		//��s1����ʱ��Ϊң��ģʽ
			case RC_SW_DOWN:	//��s1����ʱ��Ϊֹͣģʽ
				if (RemoteComm.RemoteData.remote.ch3 == RC_CH_VALUE_MAX)
					loader->mode = LOADER_RUN_PS10;
				else
					loader->mode = LOADER_STOP;
				break;
			default:
				break;
		}
	}
	
	static uint32_t covCount = 0;
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
			if (covCount >= 200)						//��ת�ﵽʮ�����ڣ���ת����
			{
				loader->mode = loader->lastMode;
				covCount = 0;
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
				loader->LM.velCtrl.rawVel <= 20 && loader->LM.velCtrl.rawVel >= -20 && 
				(loader->LM.curCtrl.rawCur > 2000 || loader->LM.curCtrl.rawCur < -2000))
										//����ģʽ������ת�ٲ�Ϊ0��ʵ��ת�ٽ�С���ж�Ϊ��ת״̬
	{
		jamCount++;
		if (jamCount >= 500)
		{
			jamCount = 0;
			return 1;
		}
	}
	else
		jamCount = 0;
	return 0;
}
