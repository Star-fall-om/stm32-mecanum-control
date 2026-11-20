/* USART2 接收 HWT101 航向/角速度；航向用角度 P，角速度仅观察。 */
/*
 * 文件职责：沿用 OPS9 的 HWT101 帧解析和连续航向，供本地定位与底盘使用。
 * 所属层次：传感器与航向计算层；动作生命周期由 moving_control 管理。
 * 关键外设：USART2(PD5 TX / PD6 RX)，115200 8N1。
 * 运行上下文：USART2 中断先收缓冲，通信任务再逐字节调用 IMU_ProcessByte；
 *            直行和转向任务按 5 ms 配置周期调用角度 P。
 * 已知边界：不做姿态融合或反馈超时判断，两类动作由管理任务串行执行。
 */
#include "imu.h"
#include "ops9_control.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

/* 两组角度 P 分开调：P_Pos 为 rpm/度，out_max 为转向轮速上限 rpm。
 * 增益均为 1 rpm/度；直行限 50 rpm，独立转向限 100 rpm，角速度只观察。 */
PID_Position PD_Pos_Rotate_Cascade_1 = {1.0f, 50.0f, 0.0f};
PID_Position PD_Rotate = {1.0f, 100.0f, 0.0f}; // 独立转向上限翻倍，直行航向上限不变。

/* 角度 P 直接输出车轮 rpm；无死区、积分、微分和角速度反馈。 */
float IMU_AngleP(float error, float kp, float rpm_max)
{
  float output;
  if (!isfinite(error) || !isfinite(kp) || !isfinite(rpm_max) ||
      kp < 0.0f || rpm_max < 0.0f) return 0.0f;
  error = fmodf(error, 360.0f);
  if (error > 180.0f) error -= 360.0f;
  else if (error <= -180.0f) error += 360.0f;
  output = kp * error;
  if (!isfinite(output)) return 0.0f;
  if (output > rpm_max) output = rpm_max;
  else if (output < -rpm_max) output = -rpm_max;
  return output;
}

float Angle_In_Move(float target_angle, float now_rotate_angle, float Wz)
{
  (void)Wz;  // 保留旧接口，角速度仅观察，不参与控制。
  PD_Pos_Rotate_Cascade_1.output = IMU_AngleP(target_angle - now_rotate_angle,
    PD_Pos_Rotate_Cascade_1.P_Pos, PD_Pos_Rotate_Cascade_1.out_max);
  return PD_Pos_Rotate_Cascade_1.output;
}

/* 相对零点的航向角，单位度，范围 (-180, 180]，供独立转向使用。 */
volatile float imu_yaw;
/* OPS9使用本地归零后的地图航向，不减独立转向的imu_zero。 */
volatile float imu_raw_yaw;
volatile uint32_t imu_yaw_seq;
volatile uint32_t imu_yaw_time_ms;
volatile uint32_t imu_rate_seq, imu_rate_time_ms;

/* 保存最近一帧的有符号原码，便于核对归零和角度展开。 */
volatile int16_t imu_rx_yaw_raw, imu_rx_rate_raw;

/* Z 轴角速度，单位 °/s，保留已确认与航向同号的传感器原始正负号。 */
volatile float imu_yaw_rate;

/* 底盘航向零点，单位度；本地启动归零后的首次航向记录，断线重连保留。 */
static float imu_zero;
static uint8_t imu_zero_ready;
/* 完整保留 OPS9 的连续角算法，串口缓存改由现有通信任务喂入。 */
static IMU_para_t IMU_para;
static uint8_t imu_frame[11];
static uint8_t imu_frame_index;
static float imu_last_raw_yaw;

typedef enum
{
  IMU_CAL_IDLE = 0,
  IMU_CAL_SEND,
  IMU_CAL_SAVE_WAIT,
  IMU_CAL_RECOVER_UNLOCK,
  IMU_CAL_RECOVER_NORMAL,
  IMU_CAL_RECOVER_WAIT
} IMU_CalibrationState;

/* 仅通信任务读写；result 记录命令流程结果，不代表实测零偏已经改善。 */
static struct
{
  IMU_CalibrationState state;
  HAL_StatusTypeDef result;
  uint32_t step_ms, key_changed_ms;
  uint8_t step, key_pressed, key_armed;
} imu_calibration;

/* PE4/KEY0 高有效。轮询不依赖 EXTI4，也不改变 CubeMX 已有配置。
 * 按下、松开各稳定20ms；忙时不排队，结束后重新确认松开才能重触发。 */
/* KEY0已配置下拉：松开为低，按下为高；初始化采样也使用同一极性。 */
void IMU_CalibrationProcess(void)
{
  static const uint8_t commands[5][5] =
  {
    {0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U}, /* 解锁。 */
    {0xFFU, 0xAAU, 0x48U, 0x01U, 0x00U}, /* 开始自动获取零偏。 */
    {0xFFU, 0xAAU, 0x69U, 0x88U, 0xB5U}, /* 20秒后重新解锁。 */
    {0xFFU, 0xAAU, 0x48U, 0x00U, 0x00U}, /* 恢复正常数据模式。 */
    {0xFFU, 0xAAU, 0x00U, 0x00U, 0x00U}  /* 保存当前参数和零偏。 */
  };
  static const uint32_t waits_ms[5] = {0U, 200U, 20000U, 200U, 200U};
  uint32_t now = HAL_GetTick();
  uint8_t pressed;
  HAL_StatusTypeDef result;

  if (!chassis_imu_bias_button_enabled) return;
  pressed = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET;
  if (pressed != imu_calibration.key_pressed)
  {
    imu_calibration.key_pressed = pressed;
    imu_calibration.key_changed_ms = now;
  }
  if (imu_calibration.state == IMU_CAL_IDLE)
  {
    if ((uint32_t)(now - imu_calibration.key_changed_ms) < 20U) return;
    if (!pressed)
    {
      imu_calibration.key_armed = 1U;
      return;
    }
    if (!imu_calibration.key_armed) return;
    imu_calibration.key_armed = 0U;
    imu_calibration.result = HAL_BUSY;
    imu_calibration.step = 0U;
    imu_calibration.step_ms = now;
    imu_calibration.state = IMU_CAL_SEND;
  }
  /* 忙时的松开和按下均不作为下一轮请求，避免结束后自动补做。 */
  imu_calibration.key_armed = 0U;
  if (imu_calibration.state == IMU_CAL_SEND)
  {
    if ((uint32_t)(now - imu_calibration.step_ms) <
        waits_ms[imu_calibration.step]) return;
    /* 每轮最多发送一帧；只短暂等待UART发送，长等待全部通过时间状态完成。 */
    result = HAL_UART_Transmit(&huart2, (uint8_t *)commands[imu_calibration.step], 5U, 5U);
    imu_calibration.step_ms = HAL_GetTick();
    if (result != HAL_OK)
    {
      imu_calibration.result = result;
      imu_calibration.state = IMU_CAL_RECOVER_UNLOCK;
      return;  /* 失败后不再继续保存，只尽力退出校准模式。 */
    }
    imu_calibration.step++;
    if (imu_calibration.step == 5U)
      imu_calibration.state = IMU_CAL_SAVE_WAIT;
    return;
  }
  if ((uint32_t)(now - imu_calibration.step_ms) < 200U) return;
  if (imu_calibration.state == IMU_CAL_RECOVER_UNLOCK)
  {
    /* 即使恢复解锁发送失败，下一步仍尝试正常模式；保留首次失败结果。 */
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)commands[0], 5U, 5U);
    imu_calibration.step_ms = HAL_GetTick();
    imu_calibration.state = IMU_CAL_RECOVER_NORMAL;
  }
  else if (imu_calibration.state == IMU_CAL_RECOVER_NORMAL)
  {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)commands[3], 5U, 5U);
    imu_calibration.step_ms = HAL_GetTick();
    imu_calibration.state = IMU_CAL_RECOVER_WAIT;
  }
  else
  {
    if (imu_calibration.state == IMU_CAL_SAVE_WAIT)
      imu_calibration.result = HAL_OK;
    imu_calibration.state = IMU_CAL_IDLE;
    imu_calibration.key_changed_ms = now;
  }
}


/* 短临界区内读取校准状态，返回供界面显示的简短文本。 */
const char *IMU_CalibrationText(void)
{
  IMU_CalibrationState state;
  uint8_t step;

  taskENTER_CRITICAL();
  state = imu_calibration.state;
  step = imu_calibration.step;
  taskEXIT_CRITICAL();
  if (state == IMU_CAL_IDLE) return NULL;
  if (state == IMU_CAL_SEND)
  {
    if (step < 2U) return "IMU BIAS\\rStarting";
    if (step == 2U) return "IMU BIAS\\rCalibrating\\rKeep still";
    return "IMU BIAS\\rSaving";
  }
  if (state == IMU_CAL_SAVE_WAIT) return "IMU BIAS\\rSaving";
  return "IMU BIAS\\rTX error\\rRecovering";
}

void IMU_ResetReceiver(void)
{
  imu_frame_index = 0U;
}



/* OPS9本地定位：内部位置单位米，由底盘模块换算为毫米供位置闭环使用。 */
volatile float ops9_x_m, ops9_y_m, ops9_yaw_deg;
volatile uint32_t ops9_position_time_ms;
volatile uint8_t ops9_position_ready;

/* 连续角可能超过多圈，先取余再统一到 (-180, 180]，不能只加减一次 360。 */
static float IMU_WrapYaw(float yaw)
{
  yaw = fmodf(yaw, 360.0f);
  if (yaw > 180.0f)
    yaw -= 360.0f;
  else if (yaw <= -180.0f)
    yaw += 360.0f;
  return yaw;
}

/* 初始化接收状态、航向零点和定位快照。 */
void IMU_Init(void)
{
  /* 两组角度 P 只清输出，保留当前调参值。 */
  taskENTER_CRITICAL();
  PD_Pos_Rotate_Cascade_1.output = PD_Rotate.output = 0.0f;
  taskEXIT_CRITICAL();
  imu_zero = 0.0f;
  imu_zero_ready = 0U;
  memset(&IMU_para, 0, sizeof(IMU_para));
  imu_frame_index = 0U;
  imu_last_raw_yaw = 0.0f;
  ops9_x_m = ops9_y_m = 0.0f;
  ops9_yaw_deg = 0.0f;
  ops9_position_time_ms = 0U;
  ops9_position_ready = 0U;
  imu_yaw = 0.0f;
  imu_raw_yaw = 0.0f;
  imu_yaw_seq = 0U;
  imu_yaw_time_ms = 0U;
  imu_rate_seq = imu_rate_time_ms = 0U;
  imu_yaw_rate = 0.0f;
  imu_rx_yaw_raw = imu_rx_rate_raw = 0;
  memset(&imu_calibration, 0, sizeof(imu_calibration));
  if (chassis_imu_bias_button_enabled)
    imu_calibration.key_pressed = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_4) == GPIO_PIN_SET;
  imu_calibration.key_changed_ms = HAL_GetTick();
  imu_calibration.key_armed = 1U;
}


/* 沿用 OPS9 的 11 字节校验与坏帧重同步；0x53 为角度、0x52 为校准后 Z 角速度。
 * 不主动设置传感器输出频率或硬件归零；断流仍保留最近一次测量。 */
void IMU_ProcessByte(uint8_t byte, uint32_t rx_ms)
{
  uint8_t index, checksum = 0U;
  int16_t raw;
  float yaw, delta;

  if(imu_frame_index == 0U && byte != 0x55U) return;
  imu_frame[imu_frame_index++] = byte;
  if(imu_frame_index < 11U) return;
  for(index = 0U; index < 10U; index++)
    checksum = (uint8_t)(checksum + imu_frame[index]);
  if(checksum == imu_frame[10] &&
     (imu_frame[1] == 0x53U || imu_frame[1] == 0x52U))
  {
    raw = (int16_t)((uint16_t)imu_frame[6] | ((uint16_t)imu_frame[7] << 8));
    if(imu_frame[1] == 0x53U)
    {
      imu_rx_yaw_raw = raw;  // 原始Z轴角度：原码乘180/32768得到度。
      yaw = (float)raw * 180.0f / 32768.0f;
      if(IMU_para.get_data_flag == 0U)
      {
        IMU_para.yaw = 0.0f;
        IMU_para.get_data_flag = 1U;
      }
      else
      {
        delta = yaw - imu_last_raw_yaw;
        if(delta > 180.0f) delta -= 360.0f;
        else if(delta < -180.0f) delta += 360.0f;
        IMU_para.yaw += delta;
      }
      imu_last_raw_yaw = yaw;
      IMU_para.yaw_time_ms = rx_ms;
      IMU_para.yaw_seq++;
      /* 首次本地归零完成后才供底盘使用；序号只在真实合法航向帧上推进。 */
      if(Control_StartupZeroDone() && IMU_para.get_rate_flag)
      {
        taskENTER_CRITICAL();
        yaw = IMU_WrapYaw(IMU_para.yaw);
        if(!imu_zero_ready)
        {
          imu_zero = yaw;
          imu_zero_ready = 1U;
        }
        imu_raw_yaw = yaw;
        imu_yaw = IMU_WrapYaw(yaw - imu_zero);
        imu_yaw_time_ms = rx_ms;
        imu_yaw_seq++;
        taskEXIT_CRITICAL();
      }
    }
    else
    {
      imu_rx_rate_raw = raw;  // 原始Z轴角速度：原码乘2000/32768得到度/秒。
      IMU_para.yaw_rate = (float)raw * 2000.0f / 32768.0f;
      IMU_para.get_rate_flag = 1U;
      IMU_para.rate_time_ms = rx_ms;
      IMU_para.rate_seq++;
      taskENTER_CRITICAL();
      imu_yaw_rate = IMU_para.yaw_rate;
      imu_rate_time_ms = rx_ms;
      imu_rate_seq++;
      taskEXIT_CRITICAL();
    }
    imu_frame_index = 0U;
    return;
  }

  // 坏包只重同步，不更新测量；保留下一帧头之后的字节，支持跨任务拍拼帧。
  for(index = 1U; index < 11U; index++)
    if(imu_frame[index] == 0x55U) break;
  imu_frame_index = (uint8_t)(11U - index);
  if(imu_frame_index != 0U)
    memmove(imu_frame, &imu_frame[index], imu_frame_index);
}

void IMU_zero_angle(void)
{
  // 仅通信任务重置展开基线，已测得的角速度和合法帧序号保留。
  IMU_para.get_data_flag = 0U;
  IMU_para.origin_seq++;
  IMU_para.yaw = 0.0f;
  imu_last_raw_yaw = 0.0f;
  taskENTER_CRITICAL();
  imu_zero = 0.0f;
  imu_zero_ready = 0U;
  // 控制缓存与序号一起保留，下一合法帧才发布新原点，避免旧序号配上新角度。
  taskEXIT_CRITICAL();
}

const IMU_para_t *get_imu_para_point(void)
{
  return &IMU_para;
}

/* 手动重建本机航向零点，不改变原始角度、角速度和接收序号。 */
void IMU_SetZero(void)
{
  taskENTER_CRITICAL();
  /* 从未减零点的角度记录，重复归零不能再使用已经减过零点的 imu_yaw。 */
  imu_zero = imu_raw_yaw;
  imu_yaw = IMU_WrapYaw(imu_raw_yaw - imu_zero);
  if (imu_yaw_seq != 0U)
    imu_zero_ready = 1U;
  taskEXIT_CRITICAL();
}
