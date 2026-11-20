/* 应用模块一次性初始化。 */
/*
 * 文件职责：把 CubeMX 之外的应用层初始化集中在一处，顺序固定。
 * 所属层次：main创建任务前调用，集中初始化CAN和惯导归零所需模块。
 * 使用外设：CAN1、TIM1/TIM4定位轮、USART2惯导。
 * 运行上下文：调度器启动前，单线程执行；此时中断已使能，但还没有任务。
 * 已知边界：只做应用层初始化，CubeMX 生成的外设初始化不在这里。
 */
#include "app_init.h"
#include "app_comm.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "encoder.h"
#include "ops9_control.h"

/* CAN过滤器、启动和FIFO0通知配置沿用现有协议。 */
static void App_CANInit(void)
{
  CAN_FilterTypeDef filter;

  /* 过滤器全 0 掩码 = 放行所有 ID，本工程只需要收 EMM/V5 的扩展帧，
     不做筛选，所以设置的是通收而不是按地址过滤。 */
  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0U;
  filter.FilterIdLow = 0U;
  filter.FilterMaskIdHigh = 0U;
  filter.FilterMaskIdLow = 0U;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = ENABLE;
  /* 从过滤器组 14 开始留给从模式，本工程只用 0 号组。 */
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  /* CAN 接收只做短帧解析；位置等待及查询都在任务上下文执行。 */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }
}

/* 保留现有定位轮与OPS9启动归零依赖，最后挂接USART2接收。 */
void Chassis_AppInit(void)
{
  Motor_Init();
  IMU_Init();
  Encoder_init();
  Control_init(1U); // 本地 OPS9 继续参与惯导启动归零和航向发布。
  App_CANInit();
  Comm_IMUInit();
}
