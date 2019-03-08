#include "Motor_Ctrl.h"
#include "math.h"

/**
  * @brief	电机速度闭环参数初始化
  * @param	motor:	Motor_t结构体指针
  * @param	acc:	速度斜坡中的加速加速度
  * @param	dec:	速度斜坡中的减速加速度
  * @param	kp, ki, kd:	PID参数
  * @retval	None
  * @note	注意同时初始化PID控制器参数以及限幅参数
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
  * @brief	电机位置闭环参数初始化
  * @param	motor:	Motor_t结构体指针
  * @param	acc:	速度斜坡中的加速加速度
  * @param	kp, ki, kd:	PID参数
  * @param	outputMin, outputMax:	位置闭环的输出范围，对应于速度闭环的输入
  * @retval	None
  * @note	注意同时初始化PID控制器参数以及限幅参数
  */
void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax, float ratio)
{
	motor->posCtrl.refRelaPos = 0;
	
	motor->posCtrl.acc = acc;		//位置环加速度和速度环减速加速度一致
	
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
  * @brief	给电机赋期望速度值
  * @param	motor:	Motor_t结构体的指针
  * @param	speed:	预设的速度值
  * @retval	None
  * @note	注意电机速度方向和实际需要的方向是否相同
  */
void Motor_SetVel(VelCtrl_t *vel_t, float vel)
{
	vel_t->refVel = vel;
	return;
}

/**
  * @brief	给电机赋期望绝对位置值
  * @param	motor:	Motor_t结构体的指针
  * @param	pos_t:	预设的位置值
  * @note	注意电机转子位置范围，及位置闭环标志位的复位
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
  * @brief	进行电机位置控制
  * @note	在减速段使用了固定加速度逼近
  *	@param	pos:	PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_PosCtrl(PosCtrl_t *pos_t)
{
	float diff, detaPos;					//相对位置缓存
	
	/* 求出相位转角 */
	detaPos = pos_t->rawPos - pos_t->rawPosLast;
	
	pos_t->rawPosLast = pos_t->rawPos;		//缓存上次数据
	
	//存储绝对位置
	if (detaPos > pos_t->posRange / 2)	//反转过一圈
		detaPos -= pos_t->posRange;
	else if (detaPos < -pos_t->posRange / 2)	//正转过一圈
		detaPos += pos_t->posRange;
	else
		detaPos = detaPos;
	
	pos_t->refRelaPos -= detaPos / pos_t->posRatio;			//更新相对转角
	pos_t->absPos += detaPos / pos_t->posRatio;				//更新绝对转角
	
	/* 计算误差值，err保存当前的误差，errLast保存上一次的误差 */
	pos_t->errLast = pos_t->err;
	
	pos_t->err = pos_t->refRelaPos * pos_t->posRatio;
	
	/* 计算积分值，注意末尾积分限幅 */
	pos_t->integ += pos_t->err;

	if(pos_t->integ >= 10000)
		pos_t->integ = 10000;
	if(pos_t->integ <= -10000)
		pos_t->integ = -10000;
	
	diff = pos_t->err - pos_t->errLast;	//计算误差变化率
	
	/* 绝对式方法计算PID输出 */
	pos_t->output = pos_t->kp * pos_t->err + pos_t->ki * pos_t->integ + pos_t->kd * diff;
	//PID->output = kp * PID->err[0] + ki * PID->integ + kd * PID->diff;
	
	/* 用固定加速度逼近终值， 0.8为积分裕量，用来削减积分值和实际值之间的偏差 */
	//refVel = sign * __sqrtf(2.0f * 0.8f * pos_t->acc * sign * pos_t->err);
	
	/* 如果接近终值则切换成PID控制 */
	//if (fabsf(refVel) < fabsf(pos_t->output))
		//pos_t->output = refVel;
	
	/* 输出限幅 */
	if(pos_t->output >= pos_t->outputMax)
		pos_t->output = pos_t->outputMax;
	if(pos_t->output <= pos_t->outputMin)
		pos_t->output = pos_t->outputMin;
}

/**
  * @brief	进行电机速度控制
  * @note	加速度用固定加速度控制，减速段前期使用固定加速度减速，后期使用PID逼近终值
  *	@param	pos_t:	PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_VelCtrl(VelCtrl_t *vel_t)
{
	float diff;
	
	/* 加减速斜坡 */
	if (vel_t->refVel_Soft < (vel_t->refVel - vel_t->acc))		//需要加速，使用加速加速度
		vel_t->refVel_Soft += vel_t->acc;
	else if (vel_t->refVel_Soft > (vel_t->refVel + vel_t->dec))	//需要减速，使用减速加速度
		vel_t->refVel_Soft -= vel_t->dec;
	else													//在线性加速度范围内使用PID调节
		vel_t->refVel_Soft = vel_t->refVel;
	
	/* 速度PID */
	vel_t->errLast = vel_t->err;
	vel_t->err = vel_t->refVel_Soft - vel_t->rawVel;		//使用vel_t->refVel_Soft作为速度期望
	diff = vel_t->err - vel_t->errLast;
	vel_t->integ += vel_t->err;
	 
	/* 积分限幅 */
	if (vel_t->integ >= 10000)
		vel_t->integ = 10000;
	if (vel_t->integ <= -10000)
		vel_t->integ = -10000;
		
	vel_t->output = vel_t->kp * vel_t->err + vel_t->ki * vel_t->integ + vel_t->kd * diff;
	
	/* 输出限幅 */
	if(vel_t->output >= vel_t->outputMax)
		vel_t->output = vel_t->outputMax;
	if(vel_t->output <= vel_t->outputMin)
		vel_t->output = vel_t->outputMin;
}

/**
  *	@brief	转换CAN总线上电机的数据格式 
  *	@param	hcan:	CAN_HandleTypeDef结构体指针
  * @param	motor:	Motor_t结构体指针
  *	@retval	None
  */
void Motor_CanRxMsgConv(CAN_HandleTypeDef *hcan, Motor_t *motor)
{
	motor->posCtrl.rawPos = (int16_t)(hcan->pRxMsg->Data[0] << 8 | hcan->pRxMsg->Data[1]);
	
	motor->velCtrl.rawVel = (int16_t)(hcan->pRxMsg->Data[2] << 8 | hcan->pRxMsg->Data[3]);
	
	motor->curCtrl.rawCur = (int16_t)(hcan->pRxMsg->Data[4] << 8 | hcan->pRxMsg->Data[5]);
}

/**
  *	@brief	通过固定的CAN总线总线上的四个电机发送信息
  *	@param	hcan	CAN_HandleTypeDef结构体指针
  *	@param  num		分为FIRST_FOUR_ID以及SECOND_FOUR_ID，用于区分总线上的八个电机
  * @param  IDxMsg	依次对应于总线上不同ID的电机
  *	@retval	None
  */
void Motor_CANSendMsg(CAN_HandleTypeDef* hcan, uint32_t num, 
					int16_t ID1Msg, int16_t ID2Msg, int16_t ID3Msg, int16_t ID4Msg)
{
	CAN_MotorTxMsgConv(hcan, ID1Msg, ID2Msg, ID3Msg, ID4Msg);
	
	CAN_SendMsg(hcan, num);
}
