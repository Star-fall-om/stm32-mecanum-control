#ifndef CHASSIS_PORT_EXAMPLE_H
#define CHASSIS_PORT_EXAMPLE_H

#include "FreeRTOS.h"

/* 外设已初始化、调度器尚未启动时调用。 */
void Chassis_PortInit(void);
/* 收到人工启动指令后，从动作调用任务等待定位并开始保持。 */
BaseType_t Chassis_PortPrepareMotion(void);

#endif
