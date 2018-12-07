#include "Loader_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t LM;		//�������  

float g_LoadVel;			//�����ٶ�
uint8_t g_LoaderMode;		//�Ƿ���׼�����Ƿ�ʼ����

/**
  * @brief	�������Ƴ�ʼ��
  * @note	���������ʼ�� 
  * @retval	None
  */
void Loader_CtrlInit(void)
{
	//���������������ͳ�ʼ��
	LM.motorType = M_2006;
	LM.escType = C610;
	Motor_VelCtrlInit(&LM, 
					  LOADER_MOTOR_ACC, LOADER_MOTOR_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  20, 1.0, 1.5 						//kp, ki, kd
					  );
	
	g_LoaderMode = LOADER_RUN;
	g_LoadVel = -5000;				//�����͵�
}

/**
  * @brief	���¹���ģʽ
  * @note	�ڹ���������߽����ж���
  * @retvel	None
  */
void Loader_UpdateState(Motor_t *motor)
{
	static uint8_t jamCount = 0, covCount = 0;			//�ж��Ƿ񿨵��ļ���λ����������Լ�����ת��ʱ������
	switch(g_LoaderMode)
	{
		case LOADER_RUN:							//����״̬
			if (motor->velCtrl.refVel != 0 && motor->velCtrl.rawVel == 0)
										//����ģʽ������ת�ٲ�Ϊ0��ʵ��ת��Ϊ0���ж�Ϊ��ת״̬
			{
				jamCount++;
				if (jamCount >= 10)
				{
					g_LoaderMode = LOADER_JAM;
					jamCount = 0;
				}
				break;
			}
			else
				jamCount = 0;
			break;
		case LOADER_JAM:							//����״̬
			covCount++;								//��ʼ��ת����
			if (covCount >= 10)						//��ת�ﵽʮ�����ڣ���ת����
			{
				g_LoaderMode = LOADER_RUN;
				covCount = 0;
				break;
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
  * @retvel	None
  */
void Loader_MotorCtrl(Motor_t *motor)
{	
	switch (g_LoaderMode)
	{
		case LOADER_RUN:				//����ģʽ
			Motor_SetVel(&(motor->velCtrl), g_LoadVel);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_STOP:
			Motor_SetVel(&(motor->velCtrl), 0);	
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_JAM:				//��תģʽ����ʼ��ת
			Motor_SetVel(&(motor->velCtrl), 500);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			break;
	}
}
