/**********本文件包含各种宏定义*****/

#ifndef _MACRO_H_
#define _MACRO_H_

/*********************通信波特率*********************/
#define DEBUS_BDR						100000u
#define NORMAL_BDR						115200u

/*********************定时器周期*********************/
#define BREAK_1MS						999u
#define BREAK_4MS						3999u
#define BREAK_100MS						99999u

/**********************遥控器输入********************/
#define REMOTE_INPUT					1u
#define KEY_MOUSE_INPUT					3u
#define REMOTE_CHASIS_SPEED_TO_REF		1.197f          //1219*0.5/660f
#define REMOTE_PITCH_ANGLE_TO_REF		0.004f
#define REMOTE_YAW_ANGLE_TO_REF			0.004f
#define ROTATE_TO_REF					0.8f

#define RC_CH_VALUE_MIN					((uint16_t)364)
#define RC_CH_VALUE_OFFSET				((uint16_t)1024)
#define RC_CH_VALUE_MAX					((uint16_t)1684)
#define RC_CH_VALUE_RANGE				660.0f

#define RC_SW_UP						((uint16_t)1)
#define RC_SW_MID						((uint16_t)3)
#define RC_SW_DOWN						((uint16_t)2)

#define RC_FRAME_LENGTH					18u

/************************电调最值*********************/
#define CM_POSITION_MAX				8191u
#define CM_POSITION_MIN				0
#define CM_CURRENT_MAX				15000
#define CM_CURRENT_MIN				-15000
#define CM_ROTATE_SPEED_MAX			6000
#define CM_ROTATE_SPEED_MIN			-6000

/************************各模式常量*********************/


/************************模式flag*********************/
#define SENTRY_REMOTE						0u
#define SENTRY_DETECT						1u
#define SENTRY_DODGE						2u

#define SENTRY_CEASE_FIRE					0u
#define SENTRY_AIM							1u
#define SENTRY_OPEN_FIRE					2u

/**********************云台角度最大值*****************/
#define YAW_FORWARDANGLE				180    //该值需要根据具体角度确定
#define PITCH_FORWARDANGLE				-28    //该值需要根据具体角度确定
#define YAW_LMAXANGLE					5
#define YAW_RMAXANGLE					-65  
#define PITCH_UMAXANGLE					10
#define PITCH_DMAXANGLE					-60

/***********************裁判系统数据******************/
#define FRAME_HEADER_LENGTH				5
#define CMD_ID_LENGTH					2

/*定义操作界面灯从右至左位1-6*/
#define LED1_LIGHT						(0x80)
#define LED2_LIGHT						(0x40)
#define LED3_LIGHT						(0x20)
#define LED4_LIGHT						(0x10)
#define LED5_LIGHT						(0x08)
#define LED6_LIGHT						(0x04)
//点亮多盏灯请进行异或操作
/***********************云台电机******************/
#define RATE_BUF_SIZE					6

#endif
