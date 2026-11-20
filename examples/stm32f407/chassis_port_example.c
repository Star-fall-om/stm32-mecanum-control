#include "chassis_port_example.h"
#include "app_comm.h"
#include "app_init.h"
#include "chassis.h"
#include "motor.h"
#include "moving_control.h"
#include "usart.h"

/* 在CubeMX初始化CAN1、USART2、TIM1/TIM4后调用，再启动调度器。 */
void Chassis_PortInit(void)
{
  Chassis_AppInit();
  MovingControl_Init();
  if (xTaskCreate(chassis_comm_task, "ChassisComm", 512, NULL, 44, NULL) != pdPASS ||
      xTaskCreate(control_task, "ControlTask", 512, NULL, 45, NULL) != pdPASS ||
      xTaskCreate(moving_control_task, "MovingControl", 2048, NULL, 41, NULL) != pdPASS)
    Error_Handler();
}

/* 上层收到人工启动命令后，从唯一的动作调用任务进入。 */
BaseType_t Chassis_PortPrepareMotion(void)
{
  uint8_t ready;
  Chassis_InitializeIdle();
  Chassis_OdomReset();
  /* 请求归零会清除就绪标志，等通信任务建立并发布新基线。 */
  do
  {
    vTaskDelay(pdMS_TO_TICKS(5U));
    taskENTER_CRITICAL();
    ready = chassis_position.ready;
    taskEXIT_CRITICAL();
  } while (!ready);
  return Chassis_HoldCurrent();
}

/* 如果宿主已经定义这些HAL回调，把转发语句合并到现有实现。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  Motor_OnCanRxFifo0(hcan);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) Comm_IMURxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2) Comm_IMUError(huart);
}
