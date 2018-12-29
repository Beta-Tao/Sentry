#ifndef _AHRS_GET_DATA_
#define _AHRS_GET_DATA_
/**
  **********************
  * �ṹ�嶨��
  **********************
  */
#include "stm32f4xx.h"

#define BSP_UART6_DMA_RX_BUF_LEN 	50u
#define AHRS_FRAME_LEN				11u
#define FRAME_NUM					3u		//��Ҫ���������֡��

typedef struct
{
	/* ʵ��ֵ */
	float rawAccX;
	
	float rawAccY;
	
	float rawAccZ;

	float rawMagX;
	
	float rawMagY;
	
	float rawMagZ;
	
	float rawPitch;
	
	float rawYaw;
	
	float rawRoll;
	
	/* ����ֵ */
	float relaYaw;
	
	float relaPitch;
	
	float refYawVel;
	
	float refPitchVel;
}AHRS_t;

extern AHRS_t Gimbal_t;

void AHRS_Init(void);

void AHRS_Data_Receive_Start(void);

void AHRS_RevData(void);

void AHRS_UpdateRef(void);

void AHRS_Decode(uint8_t *pData);

#endif
