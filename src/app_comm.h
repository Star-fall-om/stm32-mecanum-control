/* USART2惯导字节缓冲与解析入口。 */
#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "main.h"

/* USART2惯导入口，任务侧解析，中断侧只接收和重挂。 */
void Comm_IMUInit(void);
void Comm_IMURxComplete(UART_HandleTypeDef *huart);
void Comm_IMUError(UART_HandleTypeDef *huart);
/* 通信任务每1ms解析收到的惯导字节；归零时丢弃积压与半帧。 */
void Comm_Process(void);
void Comm_ResetIMU(void);

#endif
