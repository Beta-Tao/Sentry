#ifndef _PID_H_
#define _PID_H_

#include "main.h"

typedef struct
{
	float kp;
	float ki;
	float kd;
	float diff;
	float integ;
	float err[2];	//err[0]->error_now	error[1]->error_last
	
	float output;
	float outputMax;
	float outputMin;
}PID_t;

extern PID_t CMPosPID_L;		//左轮电机位置PID	chassis wheel
extern PID_t CMSpeedPID_L;		//左轮电机速度PID
extern PID_t CMPosPID_R;		//右轮电机位置PID
extern PID_t CMSpeedPID_R;		//右轮电机速度PID	

void PID_Calc(PID_t *PID, float fdb, float ref);
void PID_Debug(float data1, float data2, float data3, float data4, 
				float data5, float data6, float data7, float data8, 
				float data9, float data10);

#endif
