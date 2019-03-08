#include "Motor_Ctrl.h"
#include "math.h"

/**
  * @brief	����ٶȱջ�������ʼ��
  * @param	motor:	Motor_t�ṹ��ָ��
  * @param	acc:	�ٶ�б���еļ��ټ��ٶ�
  * @param	dec:	�ٶ�б���еļ��ټ��ٶ�
  * @param	kp, ki, kd:	PID����
  * @retval	None
  * @note	ע��ͬʱ��ʼ��PID�����������Լ��޷�����
  */
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
	
	switch (motor->escType)
	{
		case C610:
			motor->velCtrl.outputMin = C610_CUR_MIN;
			motor->velCtrl.outputMax = C610_CUR_MAX;
			break;
		case C620:
			motor->velCtrl.outputMin = C620_CUR_MIN;
			motor->velCtrl.outputMax = C620_CUR_MAX;
			break;
		case GM_6020:
			motor->velCtrl.outputMin = GM6020_VOL_MIN;
			motor->velCtrl.outputMax = GM6020_VOL_MAX;
			break;
		default:
			break;
	}
}

/**
  * @brief	���λ�ñջ�������ʼ��
  * @param	motor:	Motor_t�ṹ��ָ��
  * @param	acc:	�ٶ�б���еļ��ټ��ٶ�
  * @param	kp, ki, kd:	PID����
  * @param	outputMin, outputMax:	λ�ñջ��������Χ����Ӧ���ٶȱջ�������
  * @retval	None
  * @note	ע��ͬʱ��ʼ��PID�����������Լ��޷�����
  */
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax, float ratio)
{
	motor->posCtrl.refRelaPos = 0;
	
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

	switch (motor->escType)
	{
		case C610:
			motor->posCtrl.posRange = C610_POS_RANGE;
			break;
		case C620:
			motor->posCtrl.posRange = C620_POS_RANGE;
			break;
		case GM_6020:
			motor->posCtrl.posRange = GM6020_POS_RANGE;
			break;
		default:
			break;
	}
	
	motor->posCtrl.posRatio = ratio;
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
  * @brief	���������������λ��ֵ
  * @param	motor:	Motor_t�ṹ���ָ��
  * @param	pos_t:	Ԥ���λ��ֵ
  * @note	ע����ת��λ�÷�Χ����λ�ñջ���־λ�ĸ�λ
  * @retval	None
  */
void Motor_SetPos(PosCtrl_t *pos_t, float pos, uint8_t type)
{
	switch (type)
	{
		case RELA:
			pos_t->refRelaPos = pos;
			break;
		case ABS:
			pos_t->refRelaPos = pos - pos_t->absPos;
			break;
	}
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
	
	pos_t->rawPosLast = pos_t->rawPos;		//�����ϴ�����
	
	//�洢����λ��
	if (detaPos > pos_t->posRange / 2)	//��ת��һȦ
		detaPos -= pos_t->posRange;
	else if (detaPos < -pos_t->posRange / 2)	//��ת��һȦ
		detaPos += pos_t->posRange;
	else
		detaPos = detaPos;
	
	pos_t->refRelaPos -= detaPos / pos_t->posRatio;			//�������ת��
	pos_t->absPos += detaPos / pos_t->posRatio;				//���¾���ת��
	
	/* �������ֵ��err���浱ǰ����errLast������һ�ε���� */
	pos_t->errLast = pos_t->err;
	
	pos_t->err = pos_t->refRelaPos * pos_t->posRatio;
	
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
  *	@brief	ת��CAN�����ϵ�������ݸ�ʽ 
  *	@param	hcan:	CAN_HandleTypeDef�ṹ��ָ��
  * @param	motor:	Motor_t�ṹ��ָ��
  *	@retval	None
  */
void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor)
{
	motor->posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
	
	motor->velCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
	
	motor->curCtrl.rawCur = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
}

/**
  *	@brief	ͨ���̶���CAN���������ϵ��ĸ����������Ϣ
  *	@param	hcan	CAN_HandleTypeDef�ṹ��ָ��
  *	@param  num		��ΪFIRST_FOUR_ID�Լ�SECOND_FOUR_ID���������������ϵİ˸����
  * @param  IDxMsg	���ζ�Ӧ�������ϲ�ͬID�ĵ��
  *	@retval	None
  */
void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num, 
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg)
{
	CAN_MotorTxMsgConv(hcan, ID1Msg, ID2Msg, ID3Msg, ID4Msg);
	
	CAN_SendMsg(hcan, num);
}
