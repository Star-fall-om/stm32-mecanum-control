# STM32 麦克纳姆轮底盘控制

面向 STM32F407 和 FreeRTOS 的四轮麦克纳姆底盘控制模块。上层可提交绝对位姿、相对转角或折线路径。底盘用 HWT101 航向、双定位轮与四个电机的 CAN 位置反馈估计位姿，再以约 5 ms 的周期更新四轮速度。

| 文件 | 职责 |
| --- | --- |
| [chassis.c](src/chassis.c) | 麦轮速度解算、反馈融合、坐标修正和 CAN 输出门控 |
| [moving_control.c](src/moving_control.c) | 点到点移动、定时转向、轨迹跟踪和到点保持 |
| [path_trajectory.c](src/path_trajectory.c) | 折线圆角化、航向曲线、速度包络和可选净空检查 |
| [imu.c](src/imu.c)、[encoder.c](src/encoder.c)、[ops9_control.c](src/ops9_control.c) | 惯导航向、双轮编码器采样和 OPS9 双定位轮里程计 |
| [motor.c](src/motor.c) | 四个 Emm 驱动器的使能、轮速、停车与多圈位置查询 |

## 硬件接线

| 外设 | 原车配置 |
| --- | --- |
| CAN1 | PA11 RX、PA12 TX，经 CAN 收发器接电机。500 kbit/s、扩展帧、FIFO0。APB1 为 42 MHz 时，预分频 7、BS1 9 TQ、BS2 2 TQ |
| USART2 / HWT101 | PD6 RX、PD5 TX，115200、8N1。HWT101 同时输出 `0x52` 角速度帧与 `0x53` 角度帧，启动归零需要两类帧 |
| 右定位轮 | TIM1 CH1/CH2，PE9/PE11 |
| 左定位轮 | TIM4 CH1/CH2，PD12/PD13 |
| 可选校准键 | PE4，高有效，下拉。启用后可在车辆静止时触发 HWT101 硬件零偏校准 |

两个编码器定时器用 TI12 模式，计数周期为 65535。原车定位轮为 4096 计数/圈、半径 29 mm。电机反馈为 65536 计数/圈。底盘轮序对应 CAN 地址 `{2, 1, 3, 4}`，基准方向 `{0, 1, 0, 1}`。更换车辆时，先抬轮低速确认单轮方向，再确认定位轮安装方向。

## 接入 STM32CubeMX 工程

1. 将 `src/` 中的 C 文件加入目标工程，把该目录加入头文件搜索路径。宿主工程提供 STM32F4 HAL、标准数学库和 CubeMX 生成的 `main.h`、`can.h`、`usart.h`、`tim.h`，以及 `hcan1`、`huart2`、`htim1`、`htim4` 和 `Error_Handler()`。
2. 将 FreeRTOS 配为 1 kHz tick，启用动态分配、互斥量、任务通知和任务删除、挂起、延时接口。示例优先级为动作子任务 46、输出 45、通信 44、管理 41，`configMAX_PRIORITIES` 至少为 47，`configUSE_PORT_OPTIMISED_TASK_SELECTION` 设为 0。HAL 毫秒时基单独使用 TIM7。
3. 使能 CAN1 RX0 和 USART2 中断。示例中两者的抢占优先级与 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY` 均为 5，使任务侧临界区能够与收包回调互斥。保留 HAL IRQ 入口，并从 HAL 回调转发惯导收包、串口错误和 CAN FIFO0。
4. 将 [接入示例](examples/stm32f407/chassis_port_example.c) 和同目录头文件加入工程。先初始化外设，再调用 `Chassis_PortInit()`，最后用宿主的 FreeRTOS 启动入口启动调度器。已有 HAL 回调时，将示例中的转发语句并入原回调。

输出任务启动时会使能驱动器。上层收到人工启动指令后，从同一个 FreeRTOS 任务串行调用底盘接口。示例中的 `Chassis_PortPrepareMotion()` 先建立静止状态，再等待软件归零后的定位就绪，最后锁定当前位姿。首次使用可从短距离直行开始。

```c
/* 上层收到人工启动指令后调用。初始位姿为(0, 0, 0)。 */
static BaseType_t RunShortMove(void)
{
  BaseType_t moved;
  HAL_StatusTypeDef stopped;
  if (Chassis_PortPrepareMotion() != pdPASS) return pdFAIL;
  moved = MoveTo(0, 100, 0.0f);
  stopped = Chassis_Stop();
  return moved == pdPASS && stopped == HAL_OK ? pdPASS : pdFAIL;
}
```

## 运动接口与参数

位置单位为 mm，航向单位为度。归零时，世界坐标的 +X 对应车身右侧，+Y 对应前方，正航向为逆时针。

| 接口 | 行为 |
| --- | --- |
| `MoveTo(x, y, heading)` | 平移与转向叠加。转向曲线结束且 XY 各轴误差小于 5 mm、航向误差小于 2° 后切换到保持 |
| `MoveTurnTo(x, y, heading)` | 平移到末段距离后再开始转向，完成窗口与 `MoveTo` 相同 |
| `MovePath(points, count, heading, approach_mm)` | 跟踪 2 至 24 个折线点生成的连续轨迹。首点填实际出发位置，`approach_mm` 指定末段预留直线长度 |
| `Turn(delta_deg, duration_ms)` | 保持起始 XY 并按指定时长转向，到时切换到最终位姿保持。实际角度可从位置反馈读取 |
| `Chassis_Stop()` | 动作返回后退出保持、撤销待发旧速度并提交四轮停止帧。返回值表示帧提交状态 |

在 [chassis_config.c](src/chassis_config.c) 修改反馈源、平移限速和增益。三种反馈模式共用 HWT101 航向，XY 可选 OPS9、四轮编码器或两者融合。驱动轮几何尺寸在 [chassis.h](src/chassis.h)，轮序和极性在 [chassis.c](src/chassis.c)，定位轮半径和安装补偿在 [ops9_control.c](src/ops9_control.c)。

默认按开阔场地生成连续轨迹。需要约束边界或障碍时，在 `chassis_path_clearance` 中提供 `pose_clear` 回调与车身半对角线 `body_radius_mm`。回调按给定位置、航向和额外净空检查车身外廓，可通过返回 0 拒绝候选轨迹。净空检查用于规划 `MovePath`，其余动作的可通行范围由调用方安排。

本发布版通过 C 语法与接口静态检查。整工程链接、台架测试和整车联调待验证。

## 许可证

[MIT](LICENSE)
