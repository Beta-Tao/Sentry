#include "PID.h"
#include "DataScope_DP.h"
#include "usart.h"
#include "macro.h"

PID_t CMPosPID_L = //左轮电机位置PID	chassis wheel
	{	0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		CM_ROTATE_SPEED_MAX, CM_ROTATE_SPEED_MIN,	//outputMax, outputMin
	};
PID_t CMSpeedPID_L = //左轮电机速度PID
	{	20, 1.0, 1.5,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		CM_CURRENT_MAX, CM_CURRENT_MIN,	//outputMax, outputMin
	};
PID_t CMPosPID_R = //右轮电机位置PID
	{	0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		CM_ROTATE_SPEED_MAX, CM_ROTATE_SPEED_MIN,	//outputMax, outputMin
	};
PID_t CMSpeedPID_R = //右轮电机速度PID
	{	20, 1.0, 1.5,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		CM_CURRENT_MAX, CM_CURRENT_MIN	//outputMax, outputMin
	};

/**
  * @brief	利用Motor_t结构体中的转速或者位置计算PID控制输出值，可以是速度PID，也可以是位置PID
  * @param	PID:	PID_t结构体的指针
  *	@param	motor:	Motor_t结构体的指针
  *	@retval	None
  */
void PID_Calc(PID_t *PID, float fdb, float ref)
{	
	/* 计算误差值，err[0]保存当前的误差，err[1]保存上一次的误差 */
	PID->err[1] = PID->err[0];
	PID->err[0] = ref - fdb;
	
	/* 计算积分值，注意末尾积分限幅 */
	PID->integ += PID->err[0];
	if(PID->integ >= 1000)
		PID->integ = 1000;
	if(PID->integ <= -1000)
		PID->integ = -1000;
	
	PID->diff = PID->err[0] - PID->err[1];	//计算误差变化率
	
	/* 绝对式方法计算PID输出 */
	PID->output = PID->kp * PID->err[0] + PID->ki * PID->integ + PID->kd * PID->diff;
	//PID->output = kp * PID->err[0] + ki * PID->integ + kd * PID->diff;
	
	/* PID输出限幅 */
	if(PID->output >= PID->outputMax)
		PID->output = PID->outputMax;
	if(PID->output <= PID->outputMin)
		PID->output = PID->outputMin;
}

void PID_Debug(float data1, float data2, float data3, float data4, 
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
