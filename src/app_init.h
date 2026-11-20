/* 应用级一次性初始化，不包含CubeMX生成的外设初始化。 */
#ifndef __APP_INIT_H
#define __APP_INIT_H

/* 调度前初始化CAN、惯导及现有定位轮归零链，随后挂USART2接收。 */
void Chassis_AppInit(void);

#endif
