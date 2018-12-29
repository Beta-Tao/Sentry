#ifndef _MACRO_H_
#define _MACRO_H_

#include "stm32f4xx_hal.h"

/* 模式常量 */
#define SENTRY_DETECT_VEL			4000

/* 控制模式，移动和瞄准共享 */
#define SENTRY_REMOTE				0u		//遥控
#define SENTRY_STOP					1u		//停止

/* 移动模式 */
#define SENTRY_DETECT				2u		//侦查
#define SENTRY_DODGE				3u		//闪避

/* 瞄准模式 */
#define SENTRY_TRACE				4u		//追踪

/* 发射模式 */
#define SENTRY_CEASE_FIRE			5u		//停火
#define SENTRY_OPEN_FIRE			6u		//开火

/* 供弹模式 */
#define SENTRY_LOAD_STOP			7u		//停止供弹
#define SENTRY_LOAD_RUN				8u		//开始供弹
#define SENTRY_LOAD_JAM				9u			//卡弹

#endif
