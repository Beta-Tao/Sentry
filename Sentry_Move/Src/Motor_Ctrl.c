#include "Motor_Ctrl.h"
#include "DataScope_DP.h"
#include "Remote_Ctrl.h"
#include "Remote_Decode.h"
#include "usart.h"
#include "math.h"

void Motor_VelCtrlInit(Motor_t *motor, 
					   float acc, float dec, 
					   float kp, float ki, float kd, 
					   float outputMin, float outputMax)
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
	motor->velCtrl.outputMin = outputMin;
	motor->velCtrl.outputMax = outputMax;
}

void Motor_PosCtrlInit(Motor_t *motor, float acc, 
					   float kp, float ki, float kd,
					   float outputMin, float outputMax)
{
	motor->posCtrl.refPos = 0;
	
	motor->posCtrl.acc = acc;		//位置环加速度和速度环减速加速度一致
	
	motor->posCtrl.kp = kp;
	motor->posCtrl.ki = ki;
	motor->posCtrl.kd = kd;
	motor->posCtrl.integ = 0.0f;
	motor->posCtrl.err = 0.0f;
	motor->posCtrl.errLast = 0.0f;

	motor->posCtrl.output = 0.0f;
	motor->posCtrl.outputMax = outputMin;
	motor->posCtrl.outputMin = outputMax;
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
  * @brief	给电机赋期望位置值
  * @param	motor:	Motor_t结构体的指针
  * @param	pos:	预设的位置值
  * @note	注意电机转子位置范围
  * @retval	None
  */
void Motor_SetPos(PosCtrl_t *pos_t, float pos)
{
	pos_t->refPos = pos;
	return;
}

/**
  * @brief	进行电机位置控制
  * @note	在减速段使用了固定加速度逼近
  *	@param	pos:	PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_PosCtrl(PosCtrl_t *pos)
{
	float diff;
	float refVel;
	float sign = 1.0f;
	
	/* 计算误差值，err保存当前的误差，errLast保存上一次的误差 */
	pos->errLast = pos->err;
	pos->err = pos->refPos - pos->rawPos;
	
	/* 保证当前的控制极性 */
	if (pos->err < 0.0f)
		sign = -1.0f;
	
	/* 计算积分值，注意末尾积分限幅 */
	pos->integ += pos->err;
	if(pos->integ >= 10000)
		pos->integ = 10000;
	if(pos->integ <= -10000)
		pos->integ = -10000;
	
	diff = pos->err - pos->errLast;	//计算误差变化率
	
	/* 绝对式方法计算PID输出 */
	pos->output = pos->kp * pos->err + pos->ki * pos->integ + pos->kd * diff;
	//PID->output = kp * PID->err[0] + ki * PID->integ + kd * PID->diff;
	
	/* 用固定加速度逼近终值 */
	refVel = sign * __sqrtf(2.0f * 0.8f * pos->acc * sign * pos->err);
	
	/* 如果接近终值则切换成PID控制 */
	if (fabsf(refVel) < fabsf(pos->output))
		pos->output = refVel;
	
	/* 输出限幅 */
	if(pos->output >= pos->outputMax)
		pos->output = pos->outputMax;
	if(pos->output <= pos->outputMin)
		pos->output = pos->outputMin;
}

/**
  * @brief	进行电机速度控制
  * @note	加速度用固定加速度控制，减速段前期使用固定加速度减速，后期使用PID逼近终值
  *	@param	pos:	PosCtrl_t结构体的指针，电机位置控制结构体的指针
  *	@retval	None
  */
void Motor_VelCtrl(VelCtrl_t *vel)
{
	float diff;
	
	/* 加减速斜坡 */
	if (vel->refVel_Soft < (vel->refVel - vel->acc))		//需要加速，使用加速加速度
		vel->refVel_Soft += vel->acc;
	else if (vel->refVel_Soft > (vel->refVel + vel->acc))	//需要减速，使用减速加速度
		vel->refVel_Soft -= vel->dec;
	else													//在线性加速度范围内使用PID调节
		vel->refVel_Soft = vel->refVel;
	
	/* 速度PID */
	vel->err = vel->refVel_Soft - vel->rawVel;		//使用vel->refVel_Soft作为速度期望
	diff = vel->err - vel->errLast;
	vel->integ += vel->err;
	 
	/* 积分限幅 */
	if (vel->integ >= 10000)
		vel->integ = 10000;
	if (vel->integ >= -10000)
		vel->integ = -10000;
	vel->output = vel->kp * vel->err + vel->ki * vel->integ + vel->kd * diff;
	
	/* 输出限幅 */
	if(vel->output >= vel->outputMax)
		vel->output = vel->outputMax;
	if(vel->output <= vel->outputMin)
		vel->output = vel->outputMin;
}

/**
  * @brief	串口调试电机控制参数
  *	@param	data1~10:	共十个通道的数据
  * @note	通过串口7发送
  *	@retval	None
  */
void CtrlDebug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10)
{
	unsigned char Send_Count;
	int i;

	DataScope_Get_Channel_Data(data1, 1); //将数据 1.0  写入通道 1
    DataScope_Get_Channel_Data(data2, 2); //将数据 2.0  写入通道 2
    DataScope_Get_Channel_Data(data3, 3); //将数据 3.0  写入通道 3
    DataScope_Get_Channel_Data(data4, 4); //将数据 4.0  写入通道 4
	DataScope_Get_Channel_Data(data5, 5); //将数据 5.0  写入通道 5
    DataScope_Get_Channel_Data(data6, 6); //将数据 6.0  写入通道 6
	DataScope_Get_Channel_Data(data7, 7); //将数据 7.0  写入通道 7
    DataScope_Get_Channel_Data(data8, 8); //将数据 8.0  写入通道 8
	DataScope_Get_Channel_Data(data9, 9); //将数据 9.0  写入通道 9
    DataScope_Get_Channel_Data(data10, 10); //将数据 10.0 写入通道 10

	Send_Count = DataScope_Data_Generate(10); //生成10个通道的格式化帧数据，返回帧数据长度
	
	//HAL_UART_Transmit(&huart8, DataScope_OutPut_Buffer, Send_Count, 50);
	for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕  
	{
		HAL_UART_Transmit(&huart7, DataScope_OutPut_Buffer + i, 1, 50);
	}
}
