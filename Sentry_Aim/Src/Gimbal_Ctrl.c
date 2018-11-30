#include "Gimbal_Ctrl.h"
#include "Remote_Decode.h"
#include "Remote_Ctrl.h"

Motor_t GM_Pitch;		//云台Pitch轴电机
Motor_t GM_Yaw;			//云台Yaw轴电机

/**
  * @brief	底盘控制初始化
  * @note	底盘控制模式以及底盘电机初始化
  * @retval	None
  */
void Gimbal_CtrlInit(void)
{
	/* 云台电机及电调类型初始化，云台需要位置闭环和速度闭环 */
	GM_Pitch.motorType = M_3508;
	GM_Pitch.escType = C620;
	Motor_VelCtrlInit(&GM_Pitch, 
					  GIMBAL_MOTOR_ACC, GIMBAL_MOTOR_DEC, 	//acc, dec
					  20, 1.0, 1.5, 							//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX				//outputMin, outputMax
					  );
	Motor_PosCtrlInit(&GM_Pitch, 
					  GIMBAL_MOTOR_DEC,						//位置控制减速加速度和速度减速加速度一致
					  kp, ki, kd,
					  GM_VEL_MIN, GM_VEL_MAX);
	
	GM_Yaw.motorType = M_3508;
	GM_Yaw.escType = C620;
	Motor_VelCtrlInit(&GM_Yaw, 
					  GIMBAL_MOTOR_ACC, GIMBAL_MOTOR_DEC, 			//acc, dec
					  20, 1.0, 1.5, 									//kp, ki, kd
					  C620_CUR_MIN, C620_CUR_MAX						//outputMin, outputMax
					  );
	Motor_PosCtrlInit(&GM_Yaw, 
					  GIMBAL_MOTOR_DEC,
					  kp, ki, kd,
					  GM_VEL_MIN, GM_VEL_MAX);
}

/**
  * @brief	将遥控器通道值映射到云台电机位置
  * @note	遥控器控制云台相对运动，Pitch对应通道0，Yaw对应通道1
  * @retval	None
  */
void Gimbal_UpdateRef(void)
{ 
	Motor_SetPos(&GM_Pitch.posCtrl, 
				 GM_Pitch.posCtrl.refPos + 
				 (float)(RemoteCtrlData.remote.ch0 - RC_CH_VALUE_OFFSET) / GM_REMOTE_SENSITY);
	Motor_SetPos(&GM_Yaw.posCtrl, 
				 GM_Yaw.posCtrl.refPos + 
				 (float)(RemoteCtrlData.remote.ch1 - RC_CH_VALUE_OFFSET) / GM_REMOTE_SENSITY);
}

/**
  * @brief	更新云台模式
  * @note	在定时器七中进行定时器中断
  * @retvel	None
  */
void Gimbal_UpdateState(void)
{
	GM_Pitch.posCtrl.rawPos = g_Pitch;		//AHRS得到的方位角赋值给位置闭环反馈
	GM_Yaw.posCtrl.rawPos = g_Yaw;
}

/**
  * @brief	底盘电机控制
  * @note	底盘电机只有速度控制
  * @param	motor:	Motor_t结构体指针
  * @retvel	None
  */
void Gimbal_MotorCtrl(Motor_t *motor)
{
	Motor_PosCtrl(&(motor->posCtrl));
	Motor_SetVel(&motor->velCtrl, motor->posCtrl.output);		//将位置闭环的输出作为速度闭环的输入
	Motor_VelCtrl(&(motor->velCtrl));
}
