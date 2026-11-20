/* USART2惯导接收：中断保存字节与时间，通信任务每1ms解析。 */
#include "app_comm.h"
#include "usart.h"
#include "imu.h"
#include "FreeRTOS.h"
#include "task.h"

/* USART2中断只生产字节和接收时间，通信任务负责解析。 */
static volatile uint8_t imu_buffer[128];
static volatile uint32_t imu_rx_ms[128];
static volatile uint16_t imu_head, imu_tail;
static volatile uint8_t imu_lost;
static uint8_t uart2_byte;

/* 接收时间与字节先写入同一槽，最后发布写下标。 */
static void Comm_PutByte(void)
{
  uint16_t next = (uint16_t)(imu_head + 1U);
  if (next >= 128U) next = 0U;
  if (next == imu_tail)
  {
    imu_lost = 1U;
    return;
  }
  imu_buffer[imu_head] = uart2_byte;
  imu_rx_ms[imu_head] = HAL_GetTick();
  imu_head = next;
}

void Comm_Process(void)
{
  uint8_t byte, received;
  uint16_t processed;
  uint32_t rx_ms;

  /* 每拍最多处理一缓冲，接收中断继续补入字节时也要给控制任务让出CPU。 */
  for (processed = 0U; processed < 128U; processed++)
  {
    /* 与USART2中断及归零请求互斥，保证字节和时间来自同一槽。 */
    taskENTER_CRITICAL();
    if (imu_lost)
    {
      imu_tail = imu_head;
      imu_lost = 0U;
      IMU_ResetReceiver();
    }
    received = imu_tail != imu_head;
    if (received)
    {
      byte = imu_buffer[imu_tail];
      rx_ms = imu_rx_ms[imu_tail];
      imu_tail++;
      if (imu_tail >= 128U) imu_tail = 0U;
    }
    taskEXIT_CRITICAL();
    if (!received) break;
    IMU_ProcessByte(byte, rx_ms);
  }
}

/* 调度前挂接USART2单字节接收。 */
void Comm_IMUInit(void)
{
  HAL_StatusTypeDef result;

  imu_head = imu_tail = 0U;
  imu_lost = 0U;
  result = HAL_UART_Receive_IT(&huart2, &uart2_byte, 1U);
  if (result != HAL_OK)
  {
    Error_Handler();
  }
}

/* USART2 中断入口只入缓冲和重挂；到达时间仍由 Comm_PutByte 同槽记录。 */
void Comm_IMURxComplete(UART_HandleTypeDef *huart)
{
  (void)huart;
  Comm_PutByte();
  (void)HAL_UART_Receive_IT(&huart2, &uart2_byte, 1U);
}

/* 错误只作废残帧并重挂接收，不改变航向基线和底盘输出。 */
void Comm_IMUError(UART_HandleTypeDef *huart)
{
  imu_lost = 1U;
  (void)huart;
  (void)HAL_UART_Receive_IT(&huart2, &uart2_byte, 1U);
}

void Comm_ResetIMU(void)
{
  taskENTER_CRITICAL();
  imu_tail = imu_head;
  imu_lost = 0U;
  IMU_ResetReceiver();  // 软件归零由调用者完成，通信错误不会触碰角度基线。
  taskEXIT_CRITICAL();
}
