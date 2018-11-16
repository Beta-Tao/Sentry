/**********���ļ��������ֺ궨��*****/

#ifndef _MACRO_H_
#define _MACRO_H_

/*********************ͨ�Ų�����*********************/
#define DEBUS_BDR						100000u
#define NORMAL_BDR						115200u

/*********************��ʱ������*********************/
#define BREAK_1MS						999u
#define BREAK_4MS						3999u
#define BREAK_100MS						99999u

/***********************����ϵͳ����******************/
#define FRAME_HEADER_LENGTH				5
#define CMD_ID_LENGTH					2

/*�����������ƴ�������λ1-6*/
#define LED1_LIGHT						(0x80)
#define LED2_LIGHT						(0x40)
#define LED3_LIGHT						(0x20)
#define LED4_LIGHT						(0x10)
#define LED5_LIGHT						(0x08)
#define LED6_LIGHT						(0x04)
//������յ�������������
/***********************��̨���******************/
#define RATE_BUF_SIZE					6

#endif
