#ifndef _REFEREE_COMM_H_
#define _REFEREE_COMM_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"

#define BSP_USART6_DMA_RX_BUF_LEN 128u
#define BSP_USART6_DMA_TX_BUF_LEN 30u

#define FRAME_HEADER_LEN	sizeof(FrameHeader_t)
#define CMD_ID_LEN					2u	//�����볤��
#define CRC16_LEN					2u	//CRC16
#define GAME_ROBOT_STATE_LEN		8u
#define ROBOT_HURT_LEN				1u
#define SHOOT_DATA_LEN				6u
#define POWER_HEAT_DATA_LEN			20u
#define RFID_DETECT_LEN				2u
#define GAME_RESULT_LEN				1u
#define BUFF_MUSK_LEN				2u
#define GAME_ROBOT_POS_LEN			16u
#define SHOW_DATA_LEN				13u


#define FRAME_HEADER_SOF		0xA5

#define GAME_ROBOT_STATE_CMD_ID			0x0001
#define ROBOT_HURT_CMD_ID				0x0002
#define SHOOT_DATA_CMD_ID				0x0003
#define POWER_HEAT_DATA_CMD_ID		 	0x0004
#define RFID_DETECT_CMD_ID				0x0005
#define GAME_RESULT_CMD_ID				0x0006
#define BUFF_MUSK_CMD_ID				0x0007
#define GAME_ROBOT_POS_CMD_ID			0x0008
#define SHOW_DATA_CMD_ID				0x0100

typedef __packed struct
{
	uint8_t		sof;			//֡��ʼ�ֽ�
	uint16_t	dataLength;		//���ݶ�Data����
	uint8_t		seq;			//�����
	uint8_t		crc8;			//֡ͷCRC8У��
}FrameHeader_t;				//FrameHeader

typedef __packed struct
{
    uint16_t	stageRemianTime; //��ǰ�׶�ʣ��ʱ��
    uint8_t		gameProgress;    //��ǰ�����׶�
    uint8_t		robotLevel;      //�����˵�ǰ�ȼ�
    uint16_t	remainHP;        //�����˵�ǰѪ��
    uint16_t	maxHP;           //��������Ѫ��
}extGameRobotState_t;			//����������״̬

typedef __packed struct
{
    uint8_t	armorType  : 4;  //���仯����Ϊװ���˺�ʱ����ʶװ��ID
    uint8_t	hurtType   : 4;  //Ѫ���仯����
}extRobotHurt_t;             //�˺�����

typedef __packed struct
{
    uint8_t	bulletType;      	//��������
    uint8_t	bulletFreq;      	//������Ƶ
    float	bulletSpeed;		//��������
}extShootData_t;              	//ʵʱ�����Ϣ

typedef __packed struct
{
	float		chassisVolt;		//���������ѹ
	float		chassisCurrent;		//�����������
	float		chassisPower;		//�����������
	float		chassisPowerBuffer;	//���̹��ʻ���
	uint16_t	shooterHeat0;		//17mmǹ������
	uint16_t	shooterHeat1;		//42mmǹ������
}extPowerHeatData_t;				//ʵʱ������������

typedef __packed struct
{
	uint8_t cardType;		//������
	uint8_t cardIdx;		//�������ţ�������������
}extRfidDetect_t;			//���ؽ�������

typedef __packed struct
{
	uint8_t winner;			//�������
}extGameResult_t;			//����ʤ������

typedef __packed struct
{
	uint16_t buffMusk;		//Buff���ͣ�1��ʾ��Ч
}extBuffMusk_t;				//Buff��ȡ����

typedef __packed struct
{
	float x;			//λ��X����ֵ
	float y;			//λ��Y����ֵ
	float z;			//λ��Z����ֵ
	float yaw;			//ǹ�ڳ���Ƕ�ֵ
}extGameRobotPos_t;		//������λ�ó�����Ϣ

typedef __packed struct
{ 
	float data1;		//�Զ�������1
	float data2;		//�Զ�������2
	float data3;		//�Զ�������3
	uint8_t mask;		//�Զ�������4
}extShowData_t;			//�������Զ�������

typedef __packed struct
{
	extGameRobotState_t	GameRobotState_t;
	
	extRobotHurt_t		RobotHurt_t;
	
	extShootData_t		ShootData_t;
	
	extPowerHeatData_t	PowerHeatData_t;
	
	extRfidDetect_t		RfidDetect_t;
	
	extGameResult_t		GameResult_t;
	
	extBuffMusk_t		BuffMusk_t;
	
	extGameRobotPos_t	GameRobotPos_t;
	
	extShowData_t		ShowData_t;

}extRefereeData_t;

extern extRefereeData_t RefereeData_t;

extern uint8_t USART6_DMA_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];
extern uint8_t USART6_DMA_TX_BUF[BSP_USART6_DMA_TX_BUF_LEN];

void Referee_Data_Receive_Start(void);

void Referee_Data_Receive(void);

void Referee_Decode(uint8_t *pData);

#endif
