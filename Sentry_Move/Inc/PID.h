#ifndef _PID_H_
#define _PID_H_

#include "main.h"
#include "motor.h"

typedef enum
{
	POSITION,
	SPEED
}PID_m;

typedef struct
{
	float ref;
	float fdb;
	float kp;
	float ki;
	float kd;
	float diff;
	float integ;
	float err[2];	//err[0]->error_now	error[1]->error_last
	
	float output;
	float outputMax;
	float outputMin;
	
	PID_m PIDType;
}PID_t;

extern PID_t CWPosPID_R;		//���ֵ��λ��PID	chassis wheel
extern PID_t CWSpeedPID_R;		//���ֵ���ٶ�PID
extern PID_t CWPosPID_L;		//���ֵ��λ��PID
extern PID_t CWSpeedPID_L;		//���ֵ���ٶ�PID	

void PID_Calc(PID_t *PID, volatile Motor_t *motor);
void Motor_SetSpeed(PID_t *PID, float speed);
void Motor_SetPosition(PID_t *PID, float pos);
void PID_Debug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);

#endif
