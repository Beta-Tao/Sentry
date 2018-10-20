#ifndef _PID_H_
#define _PID_H_

#include <main.h>

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

extern PID_t CWPosPID_R;
extern PID_t CWSpeedPID_R;
extern PID_t CWPosPID_L;
extern PID_t CWSpeedPID_L;

void PID_TrigCalc(PID_Regulator_t *PID, volatile Encoder *M);
void Trigger_SetSpeed(PID_Regulator_t *pid, float speed);
void Trigger_SetPosition(PID_Regulator_t *pid, float position);
void Trigger_Debug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);

#endif