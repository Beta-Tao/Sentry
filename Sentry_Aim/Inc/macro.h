#ifndef _MACRO_H_
#define _MACRO_H_

#include "stm32f4xx_hal.h"

/* ģʽ���� */
#define SENTRY_DETECT_VEL			4000

/* ����ģʽ���ƶ�����׼���� */
#define SENTRY_REMOTE				0u		//ң��
#define SENTRY_STOP					1u		//ֹͣ

/* �ƶ�ģʽ */
#define SENTRY_DETECT				2u		//���
#define SENTRY_DODGE				3u		//����

/* ��׼ģʽ */
#define SENTRY_TRACE				4u		//׷��

/* ����ģʽ */
#define SENTRY_CEASE_FIRE			5u		//ͣ��
#define SENTRY_OPEN_FIRE			6u		//����

/* ����ģʽ */
#define SENTRY_LOAD_STOP			7u		//ֹͣ����
#define SENTRY_LOAD_RUN				8u		//��ʼ����
#define SENTRY_LOAD_JAM				9u			//����

#endif
