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
