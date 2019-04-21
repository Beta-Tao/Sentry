#ifndef _SENTRY_STRATEGY_H_
#define _SENTRY_STRATEGY_H_

#define DisGrade_Update(grade) 						\
		do{											\
			disCount[grade]++;						\
			Dis_Reflush(disCount, grade, 6);		\
			if (disCount[grade] >= 20)				\
			{										\
				disCount[grade] = 0;				\
				st->disGrade = grade;				\
			}										\
		}while(0)

#include "Master_Comm.h"
#include "PC_Comm.h"

typedef enum
{
	DIS_ABOVE_5 = 0,
	DIS_BETWEEN_4_5,
	DIS_BETWEEN_3_4,
	DIS_BETWEEN_2_3,
	DIS_BETWEEN_1_2,
	DIS_BELOW_1,
}DisGrade_e;

typedef enum
{
	SHOOT_EQUI_FREQ = 0,	//����Ƶ���
	SHOOT_HIGH_FREQ,		//�������
	SHOOT_ABANDON,			//�������
}ShootStrategy_e;			//��״̬���Ӿ�ƽֱ̨�Ӿ���

typedef enum
{
	BELOW_POWER = 0,
	OVER_POWER = 1,
}ChassisState_e;

typedef enum
{
	NO_BULLET = 0,
	HAVE_BULLET,
}LoadState_e;

typedef enum
{
	NOT_OVER_SHOOT = 0,
	SHOOT_OVER_SHOOT,
}ShootState_e;

typedef struct
{
	volatile DisGrade_e disGrade;
	
	uint8_t chassisMode;
	
	uint8_t gimbalMode;
	
	uint8_t loaderMode;
	
	uint8_t shooterMode;
	
	volatile ChassisState_e chassisState;		//�����ʱ�־λ
	
	volatile LoadState_e loadState;		//�յ���־λ
	
	volatile ShootState_e shootState;		//��������־λ
}SentryST_t;

extern SentryST_t sentryST;

void Sentry_STInit(SentryST_t *st);

void Sentry_UpdateChassisST(SentryST_t *st);

void Sentry_UpdateGimbalST(SentryST_t *st);

void Sentry_UpdateLoaderST(SentryST_t *st);

void Sentry_UpdateShooterST(SentryST_t *st);

void Dis_Reflush(uint8_t *data, uint8_t num, uint8_t len);

void Sentry_GetDisGrade(SentryST_t *st);

#endif
