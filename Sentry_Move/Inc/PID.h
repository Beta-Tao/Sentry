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

extern PID_t CWPosPID_R;		//左轮电机位置PID	chassis wheel
extern PID_t CWSpeedPID_R;		//左轮电机速度PID
extern PID_t CWPosPID_L;		//右轮电机位置PID
extern PID_t CWSpeedPID_L;		//右轮电机速度PID	

void PID_Calc(PID_t *PID, volatile Motor_t *motor);
void Motor_SetSpeed(PID_t *PID, float speed);
void Motor_SetPosition(PID_t *PID, float pos);
void PID_Debug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);

#endif
