#include "Motor_Ctrl.h"
#include "math.h"

uint8_t g_AimMode;					//��׼ģʽ��������̨
uint8_t g_MoveMode;					//�ƶ�ģʽ�����Ƶ���
uint8_t g_LoadMode;					//����ģʽ�����Ʋ���
uint8_t g_ShootMode;				//����ģʽ������Ħ����

void Motor_InitFlag(void)
{
	g_AimMode = SENTRY_STOP;					//��̨��ֹ
	g_MoveMode = SENTRY_STOP;					//���̾�ֹ
	g_LoadMode = SENTRY_LOAD_STOP;				//���̾�ֹ
	g_ShootMode = SENTRY_CEASE_FIRE;			//Ħ���־�ֹ
}

void Motor_VelCtrlInit(Motor_t *motor, 
					   float acc, float dec, 
					   float kp, float ki, float kd)
{
	motor->velCtrl.refVel = 0;
	motor->velCtrl.refVel_Soft = 0;
	motor->velCtrl.acc = acc;
	motor->velCtrl.dec = dec;
	
	motor->velCtrl.kp = kp;
	motor->velCtrl.ki = ki;
	motor->velCtrl.kd = kd;
	motor->velCtrl.integ = 0.0f;
	motor->velCtrl.err = 0.0f;
	motor->velCtrl.errLast = 0.0f;

	motor->velCtrl.output = 0.0f;
	
	if (motor->escType == C610)
	{
		motor->velCtrl.outputMin = C610_CUR_MIN;
		motor->velCtrl.outputMax = C610_CUR_MAX;
	}
	else if (motor->escType == C620)
	{
		motor->velCtrl.outputMin = C620_CUR_MIN;
		motor->velCtrl.outputMax = C620_CUR_MAX;
	}
	else
		return;
}

void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax)
{
	motor->posCtrl.refAbsPos = 0;
	
	motor->posCtrl.acc = acc;		//λ�û����ٶȺ��ٶȻ����ټ��ٶ�һ��
	
	motor->posCtrl.kp = kp;
	motor->posCtrl.ki = ki;
	motor->posCtrl.kd = kd;
	motor->posCtrl.integ = 0.0f;
	motor->posCtrl.err = 0.0f;
	motor->posCtrl.errLast = 0.0f;

	motor->posCtrl.output = 0.0f;
	motor->posCtrl.outputMin = outputMin;
	motor->posCtrl.outputMax = outputMax;
	
	motor->posCtrl.posReady = POS_CTRL_READY;
}

/**
  * @brief	������������ٶ�ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetVel(VelCtrl_t *vel_t, float vel)
{
	vel_t->refVel = vel;
	return;
}

/**
  * @brief	�����������λ��ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	pos_t:	Ԥ���λ��ֵ
  * @note	ע����ת��λ�÷�Χ����λ�ñջ���־λ�ĸ�λ
  * @retval	None
  */
void Motor_SetPos(PosCtrl_t *pos_t, float pos)
{
	pos_t->refAbsPos = pos;
	return;
}

/**
  * @brief	���е��λ�ÿ���
  * @note	�ڼ��ٶ�ʹ���˹̶����ٶȱƽ�
  *	@param	pos:	PosCtrl_t�ṹ���ָ�룬���λ�ÿ��ƽṹ���ָ��
  *	@retval	None
  */
void Motor_PosCtrl(PosCtrl_t *pos_t)
{
	float diff, detaPos;					//���λ�û���
	
	/* �����λת�� */
	detaPos = pos_t->rawPos - pos_t->rawPosLast;
	
	/* �����ϴ����� */
	pos_t->rawPosLast = pos_t->rawPos;
	
	if (detaPos > 180)	//��ת��һȦ
		pos_t->absPos += detaPos - 360;
	else if (detaPos < -180)	//��ת��һȦ
		pos_t->absPos += detaPos + 360;
	else
		pos_t->absPos += detaPos;
	
	/* �������ֵ��err���浱ǰ����errLast������һ�ε���� */
	pos_t->errLast = pos_t->err;
	
	pos_t->err = pos_t->refAbsPos - pos_t->absPos;

	/* ��֤��ǰ�Ŀ��Ƽ��� */
	//if (pos_t->err < 0.0f)
		//sign = -1.0f;
	
	/* �������ֵ��ע��ĩβ�����޷� */
	pos_t->integ += pos_t->err;
	if(pos_t->integ >= 10000)
		pos_t->integ = 10000;
	if(pos_t->integ <= -10000)
		pos_t->integ = -10000;
	
	diff = pos_t->err - pos_t->errLast;	//�������仯��
	
	/* ����ʽ��������PID��� */
	pos_t->output = pos_t->kp * pos_t->err + pos_t->ki * pos_t->integ + pos_t->kd * diff;
	//PID->output = kp * PID->err[0] + ki * PID->integ + kd * PID->diff;
	
	/* �ù̶����ٶȱƽ���ֵ�� 0.8Ϊ����ԣ����������������ֵ��ʵ��ֵ֮���ƫ�� */
	//refVel = sign * __sqrtf(2.0f * 0.8f * pos_t->acc * sign * pos_t->err);
	
	/* ����ӽ���ֵ���л���PID���� */
	//if (fabsf(refVel) < fabsf(pos_t->output))
		//pos_t->output = refVel;
	
	/* ����޷� */
	if(pos_t->output >= pos_t->outputMax)
		pos_t->output = pos_t->outputMax;
	if(pos_t->output <= pos_t->outputMin)
		pos_t->output = pos_t->outputMin;
}

/**
  * @brief	���е���ٶȿ���
  * @note	���ٶ��ù̶����ٶȿ��ƣ����ٶ�ǰ��ʹ�ù̶����ٶȼ��٣�����ʹ��PID�ƽ���ֵ
  *	@param	pos_t:	PosCtrl_t�ṹ���ָ�룬���λ�ÿ��ƽṹ���ָ��
  *	@retval	None
  */
void Motor_VelCtrl(VelCtrl_t *vel_t)
{
	float diff;
	
	/* �Ӽ���б�� */
	if (vel_t->refVel_Soft < (vel_t->refVel - vel_t->acc))		//��Ҫ���٣�ʹ�ü��ټ��ٶ�
		vel_t->refVel_Soft += vel_t->acc;
	else if (vel_t->refVel_Soft > (vel_t->refVel + vel_t->dec))	//��Ҫ���٣�ʹ�ü��ټ��ٶ�
		vel_t->refVel_Soft -= vel_t->dec;
	else													//�����Լ��ٶȷ�Χ��ʹ��PID����
		vel_t->refVel_Soft = vel_t->refVel;
	
	/* �ٶ�PID */
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel_Soft - vel_t->rawVel;		//ʹ��vel_t->refVel_Soft��Ϊ�ٶ�����
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	/* �����޷� */
	if (vel_t->integ >= 10000)
		vel_t->integ = 10000;
	if (vel_t->integ <= -10000)
		vel_t->integ = -10000;
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	/* ����޷� */
	if(vel_t->output >= vel_t->outputMax)
		vel_t->output = vel_t->outputMax;
	if(vel_t->output <= vel_t->outputMin)
		vel_t->output = vel_t->outputMin;
}

/**
  *	@brief	ת�����͸�Motor�����ݸ�ʽ 
  *	@param	hcan pointer to a CAN_HandleTypeDef structure that contains
  *         the configuration information for the specified CAN.
  *	@retval	None
  */
void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor)
{
	motor->posCtrl.rawPosLast = motor->posCtrl.rawPos;
	motor->posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
	
	motor->velCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
}

void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num, 
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg)
{
	CAN_MotorTxMsgConv(hcan, ID1Msg, ID2Msg, ID3Msg, ID4Msg);
	
	CAN_SendMsg(hcan, num);
}
