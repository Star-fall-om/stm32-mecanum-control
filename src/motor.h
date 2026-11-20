#ifndef MECANUM_MOTOR_H
#define MECANUM_MOTOR_H

#include "main.h"

/* 0x36为多圈位置。驱动器使用65536计数/圈，符号与幅值分开编码。 */
typedef struct
{
  int64_t position;
  uint32_t position_seq, position_ms; // 合法应答序号及接收时刻。
} Motor_Feedback;

/* 下标使用底盘轮序，转速单位rpm。 */
typedef struct
{
  uint16_t speed;
  uint8_t address, direction, acceleration;
} Motor_WheelCommand;

void Motor_Init(void);
uint32_t Motor_GetVelocityGeneration(void);
/* 递增取消代次，撤销邮箱中仍待发送的速度帧。 */
void Motor_CancelVelocity(void);
/* 轮换整组发送起点，四轮共用约1 ms的邮箱等待预算。 */
HAL_StatusTypeDef Motor_VelocityGroup(const Motor_WheelCommand command[4],
                                     uint8_t start_index, uint32_t generation);
/* 通信任务每拍调用；约20 ms轮询四轮，每拍最多查询一个电机。 */
void Motor_Process(void);
HAL_StatusTypeDef Motor_GetFeedback(uint8_t address, Motor_Feedback *feedback);
/* 从HAL的FIFO0回调转发，只有CAN1的帧由本驱动处理。 */
void Motor_OnCanRxFifo0(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef Motor_Enable(uint8_t address, uint8_t enable);
/* 发送0xFE停止帧。HAL_OK表示帧进入CAN邮箱。 */
HAL_StatusTypeDef Motor_Stop(uint8_t address);

#endif
