/* 本地 HWT101 航向输入和角度 P 控制。 */
/*
 * 硬件：USART2(PD5 TX / PD6 RX)，115200 8N1，接 HWT101。
 * 输入：0x55 帧头的 11 字节 0x53 航向帧和 0x52 角速度帧，单位度/度每秒。
 * 运行上下文：CommTask 解析；位置环与独立转向按 5 ms 配置周期使用角度 P。
 * 已知边界：不做姿态融合或反馈超时判断，两类运动互斥由调用者保证。
 */
#ifndef __APP_IMU_H
#define __APP_IMU_H

#include "main.h"

/* 沿用 OPS9 的传感器状态，仅通信任务读写；两个合法帧序号分别推进。 */
typedef struct
{
  uint8_t get_data_flag, get_rate_flag;
  float yaw, yaw_rate;
  uint32_t yaw_seq, rate_seq;
  uint32_t yaw_time_ms, rate_time_ms;
  uint32_t origin_seq;
} IMU_para_t;

/* 角度 P 参数和最近一次输出，轮速单位 rpm。 */
typedef struct
{
  float P_Pos;
  float out_max;
  float output;
} PID_Position;


#ifdef __cplusplus
extern "C" {
#endif

/* 两组角度 P 参数分开保存；P_Pos 为 rpm/度，out_max 为 rpm。 */
extern PID_Position PD_Pos_Rotate_Cascade_1;
extern PID_Position PD_Rotate;

/* 最短角差为度，kp 为 rpm/度，返回限幅后的浮点轮速；发送处向零取整。 */
float IMU_AngleP(float error, float kp, float rpm_max);
/* 输入角度为度，输出直接为 rpm；保留角速度参数以兼容接口，但不参与控制。 */
float Angle_In_Move(float target_angle, float now_rotate_angle, float Wz);

/* 初始化。清零帧下标、零点、航向、角速度和帧序号。 */
void IMU_Init(void);
/* 通信任务每轮轮询PE4：零偏采集、恢复正常模式及保存；不改变航向零点。 */
/* 配置为1时，使用PE4高有效按键执行HWT101零偏校准。 */
extern const uint8_t chassis_imu_bias_button_enabled;
void IMU_CalibrationProcess(void);
/* 串口屏只读校准阶段；空闲返回NULL，显示任务即可恢复原来的页面内容。 */
const char *IMU_CalibrationText(void);
/* 到达时间来自接收中断；通信任务校验完整帧，不将任务空跑当作新航向。 */
void IMU_ProcessByte(uint8_t byte, uint32_t rx_ms);
void IMU_ResetReceiver(void);  // 只丢弃半帧，不改变已建立的航向零点。
void IMU_zero_angle(void);  // 仅通信任务调用，下一合法角度帧建立本地 OPS9 零点。
const IMU_para_t *get_imu_para_point(void);
/* OPS9定位供屏幕及底盘位置闭环，内部位置单位米，航向单位度。 */
extern volatile float ops9_x_m, ops9_y_m, ops9_yaw_deg;
extern volatile uint32_t ops9_position_time_ms;
extern volatile uint8_t ops9_position_ready;
/* 把当前航向记为零点，之后 imu_yaw 表示相对该点的角度。 */
void IMU_SetZero(void);

/* 相对零点的航向角，单位度，范围 (-180, 180]。
 * 生产者为通信任务的 IMU_ProcessByte，消费者为临时转向任务。 */
extern volatile float imu_yaw;

/* OPS9及底盘闭环使用的地图航向：本地归零后折到(-180,180]度，逆时针为正。
 * CommTask 先解析再供两路解算使用；IMU_SetZero只调整独立转向角，不改地图航向。 */
extern volatile float imu_raw_yaw;
extern volatile uint32_t imu_yaw_seq;
extern volatile uint32_t imu_yaw_time_ms;

/* Z 轴角速度，单位 °/s，保留与航向同号的原值，仅用于遥测观察。 */
extern volatile float imu_yaw_rate;
extern volatile uint32_t imu_rate_seq, imu_rate_time_ms;

/* 诊断专用：合法0x53/0x52帧的Z轴int16原码，不经过本机软件归零。 */
extern volatile int16_t imu_rx_yaw_raw, imu_rx_rate_raw;

#ifdef __cplusplus
}
#endif

#endif
