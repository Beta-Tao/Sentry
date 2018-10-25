#include "PID.h"
#include "DataScope_DP.h"
#include "usart.h"

PID_t CWPosPID_R = //���ֵ��λ��PID	chassis wheel
	{	0, 0,		//ref, fdb
		0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		16384, -16384,	//outputMax, outputMin
		POSITION //PIDType
	};
PID_t CWSpeedPID_R = //���ֵ���ٶ�PID
	{	0, 0,		//ref, fdb
		0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		4000, -4000,	//outputMax, outputMin
		POSITION //PIDType
	};
PID_t CWPosPID_L = //���ֵ��λ��PID
	{	0, 0,		//ref, fdb
		0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		16384, -16384,	//outputMax, outputMin
		POSITION //PIDType
	};
PID_t CWSpeedPID_L = //���ֵ���ٶ�PID
	{	0, 0,		//ref, fdb
		0, 0, 0,	//kp, ki, kd
		0, 0,		//diff, integ
		0, 0,	//err[0]->error_now	error[1]->error_last
		0,		//output
		4000, -4000,	//outputMax, outputMin
		POSITION //PIDType
	};

/**
  * @brief	����Motor_t�ṹ���е�ת�ٻ���λ�ü���PID�������ֵ���������ٶ�PID��Ҳ������λ��PID
  * @param	PID:	PID_t�ṹ���ָ��
  *	@param	motor:	Motor_t�ṹ���ָ��
  *	@retval	None
  */
void PID_Calc(PID_t *PID, volatile Motor_t *motor)
{
	/* �ж���ʲô���͵�PID���Ի�ȡ��ͬ�Ĳο�ֵ*/
	if (PID->PIDType == POSITION)
		PID->fdb = motor->rawPos;
	else if (PID->PIDType == SPEED)
		PID->fdb = motor->rawRotateSpeed;
	
	/* �������ֵ��err[0]���浱ǰ����err[1]������һ�ε���� */
	PID->err[1] = PID->err[0];
	PID->err[0] = PID->ref - PID->fdb;
	
	/* �������ֵ��ע��ĩβ�����޷� */
	PID->integ += PID->err[0];
	if(PID->integ >= 1000)
		PID->integ = 1000;
	if(PID->integ <= -1000)
		PID->integ = -1000;
	
	PID->diff = PID->err[0] - PID->err[1];	//�������仯��
	
	/* ����ʽ��������PID��� */
	PID->output = PID->kp * PID->err[0] + PID->ki * PID->integ + PID->kd * PID->diff;
	//PID->output = kp * PID->err[0] + ki * PID->integ + kd * PID->diff;
	
	/* PID����޷� */
	if(PID->output >= PID->outputMax)
		PID->output = PID->outputMax;
	if(PID->output <= PID->outputMin)
		PID->output = PID->outputMin;
}

/**
  * @brief	������ٶȸ�ֵ
  * @param	PID:	PID_t�ṹ���ָ��
  * @param	speed:	Ԥ����ٶ�ֵ
  * @retval	None
  * @note	ע�����ٶȷ����ʵ����Ҫ�ķ����Ƿ���ͬ
  */
void Motor_SetSpeed(PID_t *PID, float speed)
{
	if (PID->PIDType == SPEED)
		PID->ref = speed;
}

/**
  * @brief	�����λ�ø�ֵ
  * @param	PID:	PID_t�ṹ���ָ��
  * @param	pos:	Ԥ���λ��ֵ
  * @retval	None
  */
void Motor_SetPosition(PID_t *PID, float pos)
{
	if (PID->PIDType == POSITION)
		PID->ref = pos;
}

void PID_Debug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10)
{
	unsigned char Send_Count;
	int i;

	DataScope_Get_Channel_Data(data1, 1); //������ 1.0  д��ͨ�� 1
    DataScope_Get_Channel_Data(data2, 2); //������ 2.0  д��ͨ�� 2
    DataScope_Get_Channel_Data(data3, 3); //������ 3.0  д��ͨ�� 3
    DataScope_Get_Channel_Data(data4, 4); //������ 4.0  д��ͨ�� 4
	DataScope_Get_Channel_Data(data5, 5); //������ 5.0  д��ͨ�� 5
    DataScope_Get_Channel_Data(data6, 6); //������ 6.0  д��ͨ�� 6
	DataScope_Get_Channel_Data(data7, 7); //������ 7.0  д��ͨ�� 7
    DataScope_Get_Channel_Data(data8, 8); //������ 8.0  д��ͨ�� 8
	DataScope_Get_Channel_Data(data9, 9); //������ 9.0  д��ͨ�� 9
    DataScope_Get_Channel_Data(data10, 10); //������ 10.0 д��ͨ�� 10

	Send_Count = DataScope_Data_Generate(10); //����10��ͨ���ĸ�ʽ��֡���ݣ�����֡���ݳ���
	
	//HAL_UART_Transmit(&huart8, DataScope_OutPut_Buffer, Send_Count, 50);
	for( i = 0 ; i < Send_Count; i++)  //ѭ������,ֱ���������  
	{
		HAL_UART_Transmit(&huart7, DataScope_OutPut_Buffer + i, 1, 50);
	}
}