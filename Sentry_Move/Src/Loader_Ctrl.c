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
	Motor_PosCtrlInit(&LM, 
					  LOADER_MOTOR_DEC,
					  0, 0, 0, 						//kp, ki, kd
					  LM_VEL_MIN, LM_VEL_MAX			//outputMin, outputMax
					  );
	
	g_LoaderMode = LOADER_STOP;
	g_LoadVel = -1000;
}

/**
  * @brief	���¹���ģʽ
  * @note	�ڹ���������߽����ж���
  * @retvel	None
  */
void Loader_UpdateState(Motor_t *motor)
{
	switch(g_LoaderMode)
	{
		case LOADER_RUN:							//����״̬
			if (motor->velCtrl.refVel != 0 && motor->velCtrl.rawVel == 0)
										//����ģʽ����ת�ٲ�Ϊ0��ʵ��ת��Ϊ0���ж�Ϊ��ת״̬
			{
				g_LoaderMode = LOADER_JAM;
				break;
			}
			break;
		case LOADER_STOP:							//ֹͣ״̬		
			break;
		case LOADER_JAM:							//����״̬
			if (motor->posCtrl.posReady == POS_CTRL_READY)	//��תģʽλ�õ���Ԥ�ڣ����������
			{
				g_LoaderMode = LOADER_RUN;
				break;
			}
			break;
		default:
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
			Motor_SetVel(&(LM.velCtrl), g_LoadVel);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_STOP:
			Motor_SetVel(&(LM.velCtrl), 0);	
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		case LOADER_JAM:				//��תģʽ����λ�ñջ�
			Motor_SetPos(&(LM.posCtrl), LOADER_MOTOR_REVPOS);
			Motor_PosCtrl(&(motor->posCtrl));
			Motor_SetVel(&(motor->velCtrl), motor->posCtrl.output);
			Motor_VelCtrl(&(motor->velCtrl));
			break;
		default:
			break;
	}
}
