#ifndef MECANUM_CHASSIS_H
#define MECANUM_CHASSIS_H

#include "main.h"
#include <FreeRTOS.h>

/* 驱动轮与车身共用尺寸，单位 mm；轴距、轮距均为完整轮心距离。 */
#define CHASSIS_WHEEL_DIAMETER_MM 100.0f
#define CHASSIS_WHEELBASE_MM      175.0f
#define CHASSIS_TRACK_MM          220.0f
/* 电机转数 / 车轮转数；当前四轮直连。 */
#define CHASSIS_GEAR_RATIO        1.0f

/* 只切换地图XY的位移来源；三种模式的航向均使用现有IMU。 */
typedef enum
{
  CHASSIS_FEEDBACK_MIXED = 1,  // OPS9高频位置叠加驱动轮位移修正。
  CHASSIS_FEEDBACK_OPS9 = 2,   // 仅使用OPS9定位轮解算的XY。
  CHASSIS_FEEDBACK_MOTOR = 3   // 仅累计四个底盘电机编码器解算的XY。
} ChassisFeedbackSource;

typedef struct
{
  float x, y, yaw;
  uint8_t ready;  // 完整原点建立后置位，不代表当前反馈在线。
} Chassis_Position;
extern Chassis_Position chassis_position;

/* 只读融合快照：XY单位mm；区间位移均在区间中间航向的车身坐标内。
 * weight_x/y为驱动轮占比，左右OPS9定位轮始终同权；航向仍只来自IMU。
 * 纯电机模式权重为1、修正量为0，XY独立累计；enabled只表示混合模式。
 * baseline_ready只表示已对齐基线，不用于判断反馈是否在线。 */
typedef struct
{
  uint32_t interval_seq, common_time_ms, interval_ms;
  float ops_x_mm, ops_y_mm;
  float ops_dx_mm, ops_dy_mm, motor_dx_mm, motor_dy_mm;
  float weight_x, weight_y;
  float correction_x_mm, correction_y_mm;
  uint8_t enabled, baseline_ready;
} Chassis_FusionDiagnostics;

#ifdef __cplusplus
extern "C" {
#endif

/* 反馈源由接入工程的底盘配置定义。 */
extern const ChassisFeedbackSource chassis_feedback_source;

/* 仅启动准备调用：建立空闲状态并下发正常零速，不用于运行中止。 */
void Chassis_InitializeIdle(void);
/* 提交四轮停止帧并退出保持。由同一运动调用任务在动作返回后调用。 */
HAL_StatusTypeDef Chassis_Stop(void);
/* 按顶部所选来源发布位置反馈，位置单位mm；仅通信任务调用。 */
void Chassis_OdomProcess(void);
/* 任务侧原子读取融合诊断，不消费反馈、不发送CAN查询。 */
void Chassis_GetFusionDiagnostics(Chassis_FusionDiagnostics *snapshot);
/* 请求共享IMU/OPS9归零并清除所选来源的积分基线，新原点就绪后恢复闭环。 */
void Chassis_OdomReset(void);
/* 空闲或安全保持时请求里程计应用XY；保持目标同步平移，不改变航向。
 * 0表示拒绝，非零序号供业务等待确认。 */
uint32_t Chassis_RequestMapXY(float x_mm, float y_mm);
uint8_t Chassis_MapXYApplied(uint32_t sequence);
void Chassis_UpdatePosition(float x_mm, float y_mm, float yaw_deg);
extern volatile float chassis_x_mm, chassis_y_mm, chassis_yaw_deg;
extern volatile uint8_t chassis_odom_ready;
extern volatile uint32_t chassis_odom_time_ms;

/* 输出任务内部使用：平移与航向分开缩放，保留原代次和四轮契约。 */
HAL_StatusTypeDef Chassis_OutputVelocity(int32_t x, int32_t y, int32_t yaw,
                       uint8_t acceleration, uint8_t ramp, float ratio, uint32_t generation);

#ifdef __cplusplus
}
#endif
#endif
