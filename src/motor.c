/* 四轮Emm CAN驱动，任务发送命令，FIFO0中断发布实时位置。 */
#include "motor.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static SemaphoreHandle_t motor_mutex;
static volatile Motor_Feedback motor_feedback[4];  // 下标0~3对应地址1~4。
static uint32_t motor_query_ms;
static volatile uint32_t motor_velocity_generation;
static uint32_t motor_velocity_mailboxes;  // 记录待发速度帧占用的硬件邮箱。
static uint8_t motor_velocity_dwt_ready;
static uint8_t motor_query_index;                 // 未成功发送时保留游标，下次继续查询这一轴

/* 创建发送互斥量，在任务启动前调用一次。 */
void Motor_Init(void)
{
  uint32_t cycle_start;
  uint8_t probe;

  /* 单线程初始化时创建；所有任务共用一个发送锁，中断接收不取这个锁。 */
  motor_mutex = xSemaphoreCreateMutex();
  if (motor_mutex == NULL) Error_Handler();
  /* 启用微秒预算计时，不清已有 CYCCNT，不占外设计时器。 */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U)
  {
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    cycle_start = DWT->CYCCNT;
    for (probe = 0U; probe < 16U; probe++) __NOP();
    motor_velocity_dwt_ready = DWT->CYCCNT != cycle_start;
  }
}

uint32_t Motor_GetVelocityGeneration(void)
{
  /* 单个32位代次直接读取；递增和邮箱撤销仍在同一临界区。 */
  return motor_velocity_generation;
}

/* 调用方处于短临界区；分类可能保留已完成帧，只以硬件pending状态为准。 */
static uint32_t Motor_PendingVelocity(void)
{
  uint32_t pending = 0U, mailbox;
  for (mailbox = CAN_TX_MAILBOX0; mailbox <= CAN_TX_MAILBOX2; mailbox <<= 1U)
    if ((motor_velocity_mailboxes & mailbox) != 0U &&
        HAL_CAN_IsTxMessagePending(&hcan1, mailbox) != 0U)
      pending |= mailbox;
  return pending;
}

void Motor_CancelVelocity(void)
{
  uint32_t pending;
  taskENTER_CRITICAL();
  motor_velocity_generation++;
  pending = Motor_PendingVelocity();
  /* 只提出撤销请求，不等待完成；STOP提交前还要再次检查pending。 */
  if (pending != 0U) (void)HAL_CAN_AbortTxRequest(&hcan1, pending);
  taskEXIT_CRITICAL();
}

/* 调用者持有发送锁。地址进入扩展帧ID，命令的其余字节作为数据。 */
static HAL_StatusTypeDef Motor_SendLocked(const uint8_t *command, uint8_t length,
                                          uint32_t wait_ms, uint32_t generation)
{
  CAN_TxHeaderTypeDef header = {0};
  uint8_t data[8] = {0};
  uint32_t wait_count = 0U, mailbox, index;
  uint32_t min_free = wait_ms == 0U ? 1U : 3U;
  uint8_t velocity = command[1] == 0xF6U;
  uint8_t stop = command[1] == 0xFEU;
  HAL_StatusTypeDef status;

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < min_free && wait_count < wait_ms)
  {
    vTaskDelay(pdMS_TO_TICKS(1U));
    wait_count++;
  }
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < min_free) return HAL_BUSY;

  header.ExtId = (uint32_t)command[0] << 8;
  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.DLC = (uint32_t)length - 1U;
  for (index = 0U; index < header.DLC; index++) data[index] = command[index + 1U];

  /* 取消检查、提交帧和邮箱分类放在同一个临界区。 */
  taskENTER_CRITICAL();
  if (velocity && generation != motor_velocity_generation)
  {
    taskEXIT_CRITICAL();
    return HAL_ERROR;
  }
  if (stop && Motor_PendingVelocity() != 0U)
  {
    taskEXIT_CRITICAL();
    return HAL_BUSY;
  }
  status = HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox);
  if (status == HAL_OK)
  {
    motor_velocity_mailboxes &= ~mailbox;
    if (velocity) motor_velocity_mailboxes |= mailbox;
  }
  taskEXIT_CRITICAL();
  return status == HAL_OK ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef Motor_VelocityGroup(const Motor_WheelCommand command[4],
                                     uint8_t start_index, uint32_t generation)
{
  uint32_t wait_us = 1000U;  // 四轮共用预算，调参只改这里，不是每轮各等 1 ms
  uint32_t budget_cycles = (SystemCoreClock / 1000000U) * wait_us;
  uint32_t cycle_start, last_cycle, cycle_now;
  uint32_t poll_count = 0U, poll_limit = 4096U;  // 计数器异常时仍保证循环有限
  uint8_t offset, index, stalled = 0U, clock_ready;
  HAL_StatusTypeDef status = HAL_OK, wheel_status;

  if (command == NULL || start_index >= 4U) return HAL_ERROR;
  for (index = 0U; index < 4U; index++)
  {
    if (command[index].address < 1U || command[index].address > 4U ||
        command[index].direction > 1U || command[index].speed > 5000U)
      return HAL_ERROR;
  }
  if (motor_mutex == NULL) return HAL_ERROR;
  if (generation != motor_velocity_generation) return HAL_ERROR;
  /* 只尝试一次取锁，让持锁的通信任务继续运行。 */
  if (xSemaphoreTake(motor_mutex, 0U) != pdTRUE)
  {
    if (generation != motor_velocity_generation) return HAL_ERROR;
    return HAL_BUSY;
  }

  clock_ready = motor_velocity_dwt_ready && budget_cycles != 0U &&
    (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U &&
    (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U;
  cycle_start = last_cycle = DWT->CYCCNT;
  for (offset = 0U; offset < 4U; offset++)
  {
    uint8_t data[8];
    index = (uint8_t)((start_index + offset) % 4U);
    data[0] = command[index].address;
    data[1] = 0xF6U;
    data[2] = command[index].direction;
    data[3] = (uint8_t)(command[index].speed >> 8);
    data[4] = (uint8_t)command[index].speed;
    data[5] = command[index].acceleration;
    data[6] = 0U;  // sync=0，不改现有逐轮立即执行协议
    data[7] = 0x6BU;

    /* 只等待硬件释放邮箱；中断保持开启，整个批次仍持有同一发送锁。 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U && clock_ready)
    {
      if (generation != motor_velocity_generation) break;
      cycle_now = DWT->CYCCNT;
      if ((uint32_t)(cycle_now - cycle_start) >= budget_cycles) break;
      stalled = cycle_now == last_cycle ? (uint8_t)(stalled + 1U) : 0U;
      last_cycle = cycle_now;
      poll_count++;
      if (stalled >= 16U || poll_count >= poll_limit)
      {
        clock_ready = 0U;  // 运行中计数停止也退回立即尝试，不无限等待
      }
    }

    /* 取消校验和最终单帧提交连在一起；临界区内绝不等待空邮箱。 */
    taskENTER_CRITICAL();
    if (generation != motor_velocity_generation)
    {
      taskEXIT_CRITICAL();
      status = HAL_ERROR;
      break;
    }
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
      wheel_status = HAL_TIMEOUT;
    }
    else
    {
      wheel_status = Motor_SendLocked(data, sizeof(data), 0U, generation);
      if (wheel_status != HAL_OK && wheel_status != HAL_BUSY)
      {
        taskEXIT_CRITICAL();
        status = HAL_ERROR; /* 真正发送错误不能被后续轮的邮箱忙覆盖。 */
        break;
      }
    }
    taskEXIT_CRITICAL();
    if (wheel_status != HAL_OK) status = HAL_BUSY;
  }
  (void)xSemaphoreGive(motor_mutex);
  return status;
}

static HAL_StatusTypeDef Motor_Send(const uint8_t *command, uint8_t length)
{
  HAL_StatusTypeDef status;
  uint32_t generation = Motor_GetVelocityGeneration();
  if (motor_mutex == NULL) return HAL_ERROR;
  if (xSemaphoreTake(motor_mutex, pdMS_TO_TICKS(50U)) != pdTRUE) return HAL_BUSY;
  status = Motor_SendLocked(command, length, 50U, generation);
  (void)xSemaphoreGive(motor_mutex);
  return status;
}

void Motor_Process(void)
{
  const uint8_t address[4] = {2U, 1U, 3U, 4U};  // 按底盘轮序轮询四个电机。
  uint32_t now = HAL_GetTick();
  uint8_t command[3] = {address[motor_query_index], 0x36, 0x6B};
  uint32_t query_period_ms = 20U;  // 从每轮首条成功查询起计时。
  HAL_StatusTypeDef result;
  if (motor_query_index == 0U && (uint32_t)(now - motor_query_ms) < query_period_ms) return;
  /* 锁忙时留到下一通信拍。 */
  if (motor_mutex == NULL || xSemaphoreTake(motor_mutex, 0U) != pdTRUE)
    return;
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3U)
  {
    (void)xSemaphoreGive(motor_mutex);
    return;  // 尚有运动包未发完，不把低 ID 查询塞到它前面
  }
  /* 每次最多发一条，不占满三个邮箱；失败不换轴，防止繁忙时漏查某个电机。 */
  result = Motor_SendLocked(command, sizeof(command), 0U, 0U);
  if (result == HAL_OK)
  {
    if (motor_query_index == 0U) motor_query_ms = now;
    motor_query_index++;
    if (motor_query_index >= 4U) motor_query_index = 0U;
  }
  (void)xSemaphoreGive(motor_mutex);
}

HAL_StatusTypeDef Motor_GetFeedback(uint8_t address, Motor_Feedback *feedback)
{
  uint8_t index;
  if (feedback == NULL || address < 1U || address > 4U) return HAL_ERROR;
  index = (uint8_t)(address - 1U);
  /* CAN1 ISR 优先级 5，可由这个临界区屏蔽，保证 64 位计数及序号来自同一帧。 */
  taskENTER_CRITICAL();
  *feedback = motor_feedback[index];
  taskEXIT_CRITICAL();
  return HAL_OK;
}

void Motor_OnCanRxFifo0(CAN_HandleTypeDef *hcan)
{
  CAN_RxHeaderTypeDef header;
  uint8_t data[8];
  uint32_t magnitude;
  int64_t count;
  uint8_t index;
  if (hcan != &hcan1) return;
  /* FIFO 深度只有 3：每次只取一帧，未读完时 HAL 会再次触发中断。 */
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) return;
  if (header.IDE != CAN_ID_EXT || header.RTR != CAN_RTR_DATA || header.DLC != 7U ||
      header.ExtId < 0x100U || header.ExtId > 0x400U || (header.ExtId & 0xFFU) != 0U ||
      data[0] != 0x36U || data[1] > 1U || data[6] != 0x6BU)
    return;
  index = (uint8_t)((header.ExtId >> 8) - 1U);
  magnitude = ((uint32_t)data[2] << 24) | ((uint32_t)data[3] << 16) |
              ((uint32_t)data[4] << 8) | data[5];
  count = (int64_t)magnitude;
  if (data[1] == 1U) count = -count;  // 符号和32位幅值分别编码。
  motor_feedback[index].position = count;
  motor_feedback[index].position_ms = HAL_GetTick();
  motor_feedback[index].position_seq++;
}

/* 底盘使用立即执行的使能命令。 */
HAL_StatusTypeDef Motor_Enable(uint8_t address, uint8_t enable)
{
  uint8_t command[6] = {address, 0xF3, 0xAB, enable, 0U, 0x6B};
  if (address < 1U || address > 4U) return HAL_ERROR;
  return Motor_Send(command, sizeof(command));
}

/* 提交单轮停止帧，由上层先关闭速度输出。 */
HAL_StatusTypeDef Motor_Stop(uint8_t address)
{
  uint8_t command[5] = {address, 0xFE, 0x98, 0U, 0x6B};
  if (address < 1U || address > 4U) return HAL_ERROR;
  return Motor_Send(command, sizeof(command));
}
