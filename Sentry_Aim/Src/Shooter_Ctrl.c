#include "Shooter_Ctrl.h"

Shooter_t sentryShooter;

//void Shooter_CtrlInit(Shooter_t *shooter)
//{
//	Motor_t *left = &(shooter->FM_Left), *right = &(shooter->FM_Right);
//	
//	left->motorType = SNAIL_2305;
//	left->escType = M_820R;
//	Motor_VelCtrlInit(left, 
//					  SHOOTER_ACC, SHOOTER_DEC, 
//					  0, 0, 0);
//	
//	g_ShootMode = SENTRY_CEASE_FIRE;
//	g_ShootVel = 0;		//Ðèµ÷Õû
//}

//void Shooter_MotorCtrl(Motor_t *motor)
//{
//	switch (g_ShootMode)
//	{
//		case SENTRY_CEASE_FIRE:
//			Motor_SetVel(&(motor->velCtrl), 0);
//			Motor_VelCtrl(&(motor->velCtrl));
//			Laser_Ctrl(LASER_OFF);
//			break;
//		case SENTRY_OPEN_FIRE:
//			Motor_SetVel(&(motor->velCtrl), g_ShootVel);
//			Motor_VelCtrl(&(motor->velCtrl));
//			Laser_Ctrl(LASER_ON);
//			break;
//		default:
//			break;
//	}
//}
