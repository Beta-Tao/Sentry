#include "Shooter_Ctrl.h"
#include "Master_Comm.h"
#include "gpio.h"

// ע�����ҵ������
PIDParam_t leftFMVelPID = {11.0f, 0.004f, 0.0f};
PIDParam_t rightFMVelPID = {11.0f, 0.0f, 0.0f};
Shooter_t sentryShooter;

void Shooter_CtrlInit(Shooter_t *shooter)
{
	Motor_t *leftMotor  = &(shooter->FM_LEFT), 
			*rightMotor = &(shooter->FM_RIGHT);
	//���̵����������ͳ�ʼ�������̵��ֻ��Ҫ�ٶȱջ�
	leftMotor->motorType = M_3508;
	leftMotor->escType = C620;
	Motor_VelCtrlInit(leftMotor, 
					  FM_LEFT_ACC, FM_LEFT_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  &leftFMVelPID, 								//kp, ki, kd 10 0.5 0
					  0.16667);

	rightMotor->motorType = M_3508;
	rightMotor->escType = C620;
	Motor_VelCtrlInit(rightMotor, 
					  FM_RIGHT_ACC, FM_RIGHT_DEC, 	//acc, dec	����������1ms�����Ե�λ������ÿms���ӵ�ת��
					  &rightFMVelPID, 								//kp, ki, kd 10 0.5 0
					  0.16667);

	shooter->mode = SHOOTER_CEASE;
}

void Shooter_MotorCtrl(Motor_t *motor)
{
	switch (sentryShooter.mode)
	{
		case SHOOTER_CEASE:
			Shooter_LaserOff();
			break;
		default:
			Shooter_LaserOn();
			break;
	}
		
	if (motor == &(sentryShooter.FM_LEFT))
	{	
		Motor_SetVelCtrlParam(&(motor->velCtrl), &leftFMVelPID);
		//TODO: ���Ը�����ʱĦ����ת��
		switch (sentryShooter.mode)
		{
			case SHOOTER_CEASE:
				Motor_VelCtrl(motor, FM_STOP);
				break;
			case SHOOTER_OPEN_15MPS:
				break;
			case SHOOTER_OPEN_20MPS:
				break;
			case SHOOTER_OPEN_25MPS:
				break;
			case SHOOTER_OPEN_30MPS:
				Motor_VelCtrl(motor, FM_LEFT_30MPS);
				break;
			default:
				break;
		}
	}
	else
	{
		Motor_SetVelCtrlParam(&(motor->velCtrl), &rightFMVelPID);
		
		switch (sentryShooter.mode)
		{
			case SHOOTER_CEASE:
				Motor_VelCtrl(motor, FM_STOP);
				break;
			case SHOOTER_OPEN_15MPS:
				break;
			case SHOOTER_OPEN_20MPS:
				break;
			case SHOOTER_OPEN_25MPS:
				break;
			case SHOOTER_OPEN_30MPS:
				Motor_VelCtrl(motor, FM_RIGHT_30MPS);
				break;
			default:
				break;
		}
	}
}

void Shooter_UpdateState(Shooter_t *shooter)
{
	shooter->mode = (ShooterMode_e)masterRxData.shooterMode;
}

//void Shooter_SetVel(Shooter_t *shooter, uint32_t vel)
//{
//	static uint32_t tick = 0, lastTick = 0;
//	ShooterMode_e mode = SHOOTER_CEASE;
//	
//	if ((mode == SHOOTER_CEASE) && (shooter->mode != SHOOTER_CEASE))
//	{
//		//TODO: ������һ��Ħ����
//		if (lastTick == 0)
//			lastTick = HAL_GetTick();
//		else
//		{
//			tick = HAL_GetTick();
//			if (tick - lastTick >= 300)
//			{
//				//TODO: �����ڶ���Ħ����
//				mode = shooter->mode;
//				tick = 0;
//				lastTick = 0;
//			}
//		}
//	}
//	else
//	{
//		//TODO: ��������Ħ����
//		mode = shooter->mode;
//		tick = 0;
//		lastTick = 0;
//	}
//}

void Shooter_LaserOn(void)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_SET);
}

void Shooter_LaserOff(void)
{
	HAL_GPIO_WritePin(Laser_GPIO_Port, Laser_Pin, GPIO_PIN_RESET);
}
