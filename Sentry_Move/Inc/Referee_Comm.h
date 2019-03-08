#ifndef _REFEREE_COMM_H_
#define _REFEREE_COMM_H_

#include "stm32f4xx_hal.h"
#include "CRC.h"

#define BSP_USART6_DMA_RX_BUF_LEN 128u
#define BSP_USART6_DMA_TX_BUF_LEN 30u

#define FRAME_HEADER_LEN	sizeof(FrameHeader_t)
#define CMD_ID_LEN					2u	//命令码长度
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
	uint8_t		sof;			//帧起始字节
	uint16_t	dataLength;		//数据段Data长度
	uint8_t		seq;			//包序号
	uint8_t		crc8;			//帧头CRC8校验
}FrameHeader_t;				//FrameHeader

typedef __packed struct
{
    uint16_t	stageRemianTime; //当前阶段剩余时间
    uint8_t		gameProgress;    //当前比赛阶段
    uint8_t		robotLevel;      //机器人当前等级
    uint16_t	remainHP;        //机器人当前血量
    uint16_t	maxHP;           //机器人满血量
}extGameRobotState_t;			//比赛机器人状态

typedef __packed struct
{
    uint8_t	armorType  : 4;  //若变化类型为装甲伤害时，标识装甲ID
    uint8_t	hurtType   : 4;  //血量变化类型
}extRobotHurt_t;             //伤害数据

typedef __packed struct
{
    uint8_t	bulletType;      	//弹丸类型
    uint8_t	bulletFreq;      	//弹丸射频
    float	bulletSpeed;		//弹丸射速
}extShootData_t;              	//实时射击信息

typedef __packed struct
{
	float		chassisVolt;		//底盘输出电压
	float		chassisCurrent;		//底盘输出电流
	float		chassisPower;		//底盘输出功率
	float		chassisPowerBuffer;	//底盘功率缓冲
	uint16_t	shooterHeat0;		//17mm枪口热量
	uint16_t	shooterHeat1;		//42mm枪口热量
}extPowerHeatData_t;				//实时功率热量数据

typedef __packed struct
{
	uint8_t cardType;		//卡类型
	uint8_t cardIdx;		//卡索引号，用于区分区域
}extRfidDetect_t;			//场地交互数据

typedef __packed struct
{
	uint8_t winner;			//比赛结果
}extGameResult_t;			//比赛胜负数据

typedef __packed struct
{
	uint16_t buffMusk;		//Buff类型，1表示有效
}extBuffMusk_t;				//Buff获取数据

typedef __packed struct
{
	float x;			//位置X坐标值
	float y;			//位置Y坐标值
	float z;			//位置Z坐标值
	float yaw;			//枪口朝向角度值
}extGameRobotPos_t;		//机器人位置朝向信息

typedef __packed struct
{ 
	float data1;		//自定义数据1
	float data2;		//自定义数据2
	float data3;		//自定义数据3
	uint8_t mask;		//自定义数据4
}extShowData_t;			//参赛队自定义数据

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
