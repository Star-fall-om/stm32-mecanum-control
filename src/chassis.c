/* 麦轮运动学和步进电机组合。 */
/*
 * 文件职责：把底盘速度指令分解到四个轮，再交给 motor.c 发 CAN。
 * 所属层次：底盘运动层；上层入口是全场定位与速度接口，下层出口是 motor.h。
 * 关键外设：CAN1 上的四个 EMM/V5 步进电机，从站地址见下方地址表。
 * 运行上下文：全部在任务上下文调用，轮速整组有界等待邮箱，主动停车仍按原方式发送。
 * 单位约定：全场定位坐标单位 mm，航向单位度，电机速度沿用驱动器量纲。
 * 已知边界：里程计通过 Chassis_UpdatePosition 发布地图定位，不做反馈失联停车。
 */
#include "chassis.h"   // 本模块对外接口
#include "motor.h"     // Motor_VelocityGroup / Motor_Stop
#include "moving_control.h"  // 动作门控与速度消息
#include <math.h>      // fabsf / fmodf / cosf / sinf
#include "FreeRTOS.h"  // taskENTER_CRITICAL / taskEXIT_CRITICAL 宏
#include "task.h"      // 临界区宏实现
#include "ops9_control.h"
#include <string.h>     // 仅清融合状态，不改硬件编码器计数

/* 上电默认零位。定位任务建立基线后才置位ready。 */
Chassis_Position chassis_position;

/* 输出门控、逐轮下发和麦轮速度解算。 */
static uint8_t Chassis_PositionActive(void);
static HAL_StatusTypeDef Chassis_SendWheel(int32_t wheel[4],
                                           uint8_t acceleration, uint8_t ramp, uint32_t generation);
HAL_StatusTypeDef Chassis_OutputVelocity(int32_t x, int32_t y, int32_t yaw,
                       uint8_t acceleration, uint8_t ramp, float ratio, uint32_t generation);

/* 关闭输出并撤销旧代次，避免保持指令覆盖停止命令。 */
static uint32_t Chassis_CloseOutput(void)
{
  uint32_t generation;
  taskENTER_CRITICAL();
  chassis_mode = 0U;
  target_seq++;
  Motor_CancelVelocity();
  xQueueReset(Move_CMD_Queue);
  generation = Motor_GetVelocityGeneration();
  taskEXIT_CRITICAL();
  return generation;
}

void Chassis_InitializeIdle(void)
{
  uint32_t generation = Chassis_CloseOutput();
  (void)Chassis_OutputVelocity(0, 0, 0, 0U, 0U, 1.0f, generation);
}

HAL_StatusTypeDef Chassis_Stop(void)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t address;
  (void)Chassis_CloseOutput();
  for (address = 1U; address <= 4U; address++)
  {
    HAL_StatusTypeDef result = Motor_Stop(address);
    if (status == HAL_OK && result != HAL_OK) status = result;
  }
  return status;
}

HAL_StatusTypeDef Chassis_OutputVelocity(int32_t x, int32_t y, int32_t yaw,
                       uint8_t acceleration, uint8_t ramp, float ratio, uint32_t generation)
{
  int32_t wheel[4] = {x + y - yaw, -x + y + yaw,
                     -x + y - yaw, x + y + yaw};
  uint8_t index;
  if (ramp)
  {
    for (index = 0U; index < 4U; index++)
    {
      int32_t turn = (index & 1U) ? yaw : -yaw;
      /* 平移仍按原比例取整；航向已经是 rpm，取出后再叠加，不重复缩放。 */
      wheel[index] = (int32_t)((wheel[index] - turn) * ratio) + turn;
    }
  }
  return Chassis_SendWheel(wheel, acceleration, ramp, generation);
}

/* 把解算好的四轮转速逐轮下发到 CAN。
 * 四种转速正负号在这里拆成「方向 + 幅值」：负速度在基准转向上取反，单轮幅值截到 5000。
 * 每拍轮换起点，四轮共用 1000 us 邮箱等待预算；取锁忙不等待，超时下拍发最新值。
 * ramp=1 先检查门控，等待期间通过取消代次响应停车，不自动停车、不缓存旧命令。 */
static HAL_StatusTypeDef Chassis_SendWheel(int32_t wheel[4],
                                           uint8_t acceleration, uint8_t ramp, uint32_t generation)
{
  const uint8_t address[4] = {2U, 1U, 3U, 4U};           // 物理轮序 → CAN 从站地址
  const uint8_t normal_direction[4] = {0U, 1U, 0U, 1U};  // 四轮基准转向
  static uint8_t next_start;
  Motor_WheelCommand command[4];
  /* 底层批量发送接口要求结果数组，调用方不再保存逐轮调试统计。 */
  uint8_t index, start_index;

  /* 提前记录代次；准备数据期间若停车，批量接口也会拒绝旧批次。 */
  taskENTER_CRITICAL();
  if (ramp && !Chassis_PositionActive())
  {
    taskEXIT_CRITICAL();
    return HAL_ERROR;
  }
  start_index = next_start;
  next_start = (uint8_t)((next_start + 1U) % 4U);
  taskEXIT_CRITICAL();

  for (index = 0U; index < 4U; index++)
  {
    uint8_t direction = normal_direction[index];
    int32_t magnitude = wheel[index] < 0 ? -wheel[index] : wheel[index];  // 驱动器只收方向和幅值
    uint16_t speed;

    // 发送前确认位置环仍使能，主动停车后不再发剩余轮速
    if (ramp && !Chassis_PositionActive()) return HAL_ERROR;
    if (wheel[index] < 0)
    {
      direction = (uint8_t)(1U - direction);   // 负速度 → 基准转向取反
    }
    if (magnitude > 5000) magnitude = 5000;    // 单轮转速上限，防止叠加后超出电机能力
    speed = (uint16_t)magnitude;
    command[index].address = address[index];
    command[index].direction = direction;
    command[index].speed = speed;
    command[index].acceleration = acceleration;
  }

  return Motor_VelocityGroup(command, start_index, generation);
}

/* 平移、原地转向与保持共用输出门控。 */
static uint8_t Chassis_PositionActive(void)
{
  uint8_t active;
  taskENTER_CRITICAL();
  active = chassis_mode == 1U || chassis_mode == 2U ||
           chassis_mode == CHASSIS_MODE_HOLD;
  taskEXIT_CRITICAL();
  return active;
}


/* 把任意角度归一到 (-180, 180]，避免目标角与反馈角相差 360° 时算出巨大误差。 */
static float Chassis_Angle(float angle)
{
  angle = fmodf(angle, 360.0f);          // 先折到 (-360, 360)
  if (angle > 180.0f) angle -= 360.0f;
  if (angle < -180.0f) angle += 360.0f;
  return angle;
}

/* 统一反馈快照：地图坐标单位毫米，逆时针航向为正。
 * 仅由CommTask按所选来源发布给位置环；目标仍由上层明确下达。
 * ready 只表示已建立原点，本版不据此判断反馈是否中断。 */
volatile float chassis_x_mm, chassis_y_mm;
volatile float chassis_yaw_deg;
volatile uint8_t chassis_odom_ready;
/* 仅供屏幕判断定位是否更新，不改变位置环或失联控制。 */
volatile uint32_t chassis_odom_time_ms;
static volatile uint8_t odom_reset_pending;
static float odom_correct_x, odom_correct_y;
static uint32_t odom_correct_request;
static volatile uint32_t odom_correct_applied;
static uint8_t odom_correct_pending;
static uint8_t odom_correct_mode;
static uint32_t odom_correct_target_seq, odom_correct_generation;

/* 工作状态只由通信任务改写；其它任务只能读取最终发布的完整快照。 */
static Chassis_FusionDiagnostics fusion, fusion_published;
/* 纯电机地图XY单独积分，不能通过OPS9位置加修正量间接实现。 */
static float motor_x_mm, motor_y_mm;

/* 试用调参集中在这里：前后直行多信驱动轮，横移和旋转更多保留OPS9。
 * 两只定位轮不单独调权，OPS9原有轮序、半径和旋转补偿保持原样。 */
#define FUSION_MOTOR_HISTORY 8U
#define FUSION_REFERENCE_HISTORY 128U
#define FUSION_COUNTS_PER_TURN 65536.0f
#define FUSION_SPEED_FLOOR_MM_S 20.0f
#define FUSION_WEIGHT_TAU_MS 100.0f
#define FUSION_FORWARD_BASE 0.20f
#define FUSION_FORWARD_GAIN 0.55f
#define FUSION_FORWARD_TURN_REDUCTION 0.75f
#define FUSION_LATERAL_BASE 0.15f
#define FUSION_LATERAL_TURN_REDUCTION (2.0f / 3.0f)
#define FUSION_DEG_TO_RAD 0.01745329252f

typedef struct
{
  int64_t count;
  uint32_t ms;
} Fusion_MotorPoint;

/* 插值结果保留整数基点和小范围浮点偏移，不能先把多圈累计值转成float。 */
typedef struct
{
  int64_t base;
  float offset;
} Fusion_MotorValue;

typedef struct
{
  uint32_t ms;
  float x, y, yaw;  // 混合时保存OPS地图XY；纯电机只存IMU连续航向，XY置零。
} Fusion_ReferencePoint;

static const uint8_t fusion_address[4] = {2U, 1U, 3U, 4U};
static const int8_t fusion_sign[4] = {1, -1, 1, -1};
static Fusion_MotorPoint fusion_motor_history[4][FUSION_MOTOR_HISTORY];
static uint8_t fusion_motor_next[4], fusion_motor_count[4];
static uint32_t fusion_motor_seq[4];
static Fusion_ReferencePoint fusion_reference_history[FUSION_REFERENCE_HISTORY], fusion_previous_reference;
static uint16_t fusion_reference_next, fusion_reference_count;
static Fusion_MotorValue fusion_previous_motor[4];

/* 同毫秒更新覆盖末槽；序号未变的回包不再入历史。 */
static void Fusion_CaptureMotor(uint8_t index)
{
  Motor_Feedback feedback;
  uint8_t slot;
  (void)Motor_GetFeedback(fusion_address[index], &feedback);
  if (!feedback.position_seq || feedback.position_seq == fusion_motor_seq[index]) return;
  fusion_motor_seq[index] = feedback.position_seq;
  slot = (uint8_t)((fusion_motor_next[index] + FUSION_MOTOR_HISTORY - 1U) % FUSION_MOTOR_HISTORY);
  if (!fusion_motor_count[index] || fusion_motor_history[index][slot].ms != feedback.position_ms)
  {
    slot = fusion_motor_next[index];
    fusion_motor_next[index] = (uint8_t)((slot + 1U) % FUSION_MOTOR_HISTORY);
    if (fusion_motor_count[index] < FUSION_MOTOR_HISTORY) fusion_motor_count[index]++;
  }
  fusion_motor_history[index][slot].count = feedback.position;
  fusion_motor_history[index][slot].ms = feedback.position_ms;
}

static void Fusion_CaptureReference(void)
{
  const IMU_para_t *imu = get_imu_para_point();
  uint8_t motor_only = chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR;
  uint32_t ms = motor_only ? imu->yaw_time_ms : ops9_position_time_ms;
  uint16_t slot = (uint16_t)((fusion_reference_next + FUSION_REFERENCE_HISTORY - 1U) % FUSION_REFERENCE_HISTORY);
  if (!fusion_reference_count || fusion_reference_history[slot].ms != ms)
  {
    slot = fusion_reference_next;
    fusion_reference_next = (uint16_t)((slot + 1U) % FUSION_REFERENCE_HISTORY);
    if (fusion_reference_count < FUSION_REFERENCE_HISTORY) fusion_reference_count++;
  }
  /* 混合沿用原OPS采样时刻；纯电机按实际IMU帧时刻采航向，不取OPS位移。 */
  fusion_reference_history[slot].ms = ms;
  fusion_reference_history[slot].x = motor_only ? 0.0f : fusion.ops_x_mm;
  fusion_reference_history[slot].y = motor_only ? 0.0f : fusion.ops_y_mm;
  fusion_reference_history[slot].yaw = imu->yaw;
}

/* 只在已有样本之间插值，不外推。所有遍历和浮点运算均在临界区外。 */
static uint8_t Fusion_MotorAt(uint8_t index, uint32_t ms, Fusion_MotorValue *value)
{
  uint8_t count = fusion_motor_count[index], i, slot;
  Fusion_MotorPoint before, after;
  float ratio;
  if (!count) return 0U;
  slot = (uint8_t)((fusion_motor_next[index] + FUSION_MOTOR_HISTORY - count) % FUSION_MOTOR_HISTORY);
  before = fusion_motor_history[index][slot];
  if ((int32_t)(ms - before.ms) < 0) return 0U;
  if (ms == before.ms) { value->base = before.count; value->offset = 0.0f; return 1U; }
  for (i = 1U; i < count; i++)
  {
    slot = (uint8_t)((slot + 1U) % FUSION_MOTOR_HISTORY);
    after = fusion_motor_history[index][slot];
    if ((int32_t)(ms - after.ms) <= 0)
    {
      ratio = (float)(ms - before.ms) / (float)(after.ms - before.ms);
      value->base = before.count;
      value->offset = (float)(after.count - before.count) * ratio;
      return 1U;
    }
    before = after;
  }
  return 0U;
}

static uint8_t Fusion_ReferenceAt(uint32_t ms, Fusion_ReferencePoint *value)
{
  uint16_t i, slot;
  Fusion_ReferencePoint before, after;
  float ratio;
  if (!fusion_reference_count) return 0U;
  slot = (uint16_t)((fusion_reference_next + FUSION_REFERENCE_HISTORY - fusion_reference_count) % FUSION_REFERENCE_HISTORY);
  before = fusion_reference_history[slot];
  if ((int32_t)(ms - before.ms) < 0) return 0U;
  if (ms == before.ms) { *value = before; return 1U; }
  for (i = 1U; i < fusion_reference_count; i++)
  {
    slot = (uint16_t)((slot + 1U) % FUSION_REFERENCE_HISTORY);
    after = fusion_reference_history[slot];
    if ((int32_t)(ms - after.ms) <= 0)
    {
      ratio = (float)(ms - before.ms) / (float)(after.ms - before.ms);
      value->ms = ms;
      value->x = before.x + (after.x - before.x) * ratio;
      value->y = before.y + (after.y - before.y) * ratio;
      value->yaw = before.yaw + (after.yaw - before.yaw) * ratio;
      return 1U;
    }
    before = after;
  }
  return 0U;
}

static void Fusion_ProcessInterval(void)
{
  Fusion_MotorValue motor[4];
  Fusion_ReferencePoint reference;
  uint32_t common = 0U, latest;
  uint8_t i, slot;
  float wheel[4], dx, dy, dt, inv_dt, delta_yaw, half_yaw, heading, sine, cosine, chord;
  float vf, vl, vr, sum, forward, turn, alpha, wx, wy, error_x, error_y;
  float scale = 3.14159265359f * CHASSIS_WHEEL_DIAMETER_MM /
                (FUSION_COUNTS_PER_TURN * CHASSIS_GEAR_RATIO);

  Fusion_CaptureReference();
  for (i = 0U; i < 4U; i++) Fusion_CaptureMotor(i);
  for (i = 0U; i < 4U; i++)
  {
    if (!fusion_motor_count[i]) return;
    slot = (uint8_t)((fusion_motor_next[i] + FUSION_MOTOR_HISTORY - 1U) % FUSION_MOTOR_HISTORY);
    latest = fusion_motor_history[i][slot].ms;
    if (i == 0U || (int32_t)(latest - common) < 0) common = latest;
  }
  /* 纯电机将IMU也纳入共同时间，四轮与航向均只在已有样本内插值。 */
  if (chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR &&
      (int32_t)(common - get_imu_para_point()->yaw_time_ms) > 0)
    common = get_imu_para_point()->yaw_time_ms;
  /* 共同时间不前进就不再消费，四轮先对齐到同一时刻才比较位移。 */
  if (fusion.baseline_ready && (int32_t)(common - fusion.common_time_ms) <= 0) return;
  if (!Fusion_ReferenceAt(common, &reference)) return;
  for (i = 0U; i < 4U; i++)
    if (!Fusion_MotorAt(i, common, &motor[i])) return;

  if (fusion.baseline_ready)
  {
    dt = (float)(common - fusion.common_time_ms);
    inv_dt = 1000.0f / dt;
    for (i = 0U; i < 4U; i++)
    {
      /* 先做int64计数差，再加小范围插值差，保留大累计计数下的单计数变化。 */
      wheel[i] = ((float)(motor[i].base - fusion_previous_motor[i].base) +
                  motor[i].offset - fusion_previous_motor[i].offset) * scale * fusion_sign[i];
    }
    dx = (wheel[0] - wheel[1] - wheel[2] + wheel[3]) * 0.25f;
    dy = (wheel[0] + wheel[1] + wheel[2] + wheel[3]) * 0.25f;
    delta_yaw = (reference.yaw - fusion_previous_reference.yaw) * FUSION_DEG_TO_RAD;
    half_yaw = delta_yaw * 0.5f;
    heading = (fusion_previous_reference.yaw * FUSION_DEG_TO_RAD) + half_yaw;
    sine = sinf(heading); cosine = cosf(heading);
    /* 近零角用sinc的展开式；转动时用弦长补偿，再按中间航向投影到地图。 */
    chord = fabsf(half_yaw) < 0.0001f ? 1.0f - half_yaw * half_yaw / 6.0f : sinf(half_yaw) / half_yaw;
    fusion.motor_dx_mm = dx * chord; fusion.motor_dy_mm = dy * chord;
    if (chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR)
    {
      /* 独立累计四轮位移的地图投影；不读OPS地图XY，也不叠加融合修正。 */
      motor_x_mm += cosine * fusion.motor_dx_mm - sine * fusion.motor_dy_mm;
      motor_y_mm += sine * fusion.motor_dx_mm + cosine * fusion.motor_dy_mm;
    }
    else
    {
      /* 混合模式保留原有权重、平滑及OPS高频位置修正算法。 */
      error_x = reference.x - fusion_previous_reference.x;
      error_y = reference.y - fusion_previous_reference.y;
      fusion.ops_dx_mm = cosine * error_x + sine * error_y;
      fusion.ops_dy_mm = -sine * error_x + cosine * error_y;

      vf = fabsf(dy) * inv_dt; vl = fabsf(dx) * inv_dt;
      /* 用区间连续航向差取得同一时间段的IMU平均角速度，不混入轮式转角。 */
      vr = fabsf(delta_yaw) * inv_dt * (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f;
      sum = vf + vl + vr + FUSION_SPEED_FLOOR_MM_S;
      forward = vf / sum; turn = vr / sum;
      wy = (FUSION_FORWARD_BASE + FUSION_FORWARD_GAIN * forward) *
           (1.0f - FUSION_FORWARD_TURN_REDUCTION * turn);
      wx = FUSION_LATERAL_BASE * (1.0f - FUSION_LATERAL_TURN_REDUCTION * turn);
      alpha = dt / (FUSION_WEIGHT_TAU_MS + dt);
      fusion.weight_x += alpha * (wx - fusion.weight_x);
      fusion.weight_y += alpha * (wy - fusion.weight_y);
      error_x = fusion.weight_x * (fusion.motor_dx_mm - fusion.ops_dx_mm);
      error_y = fusion.weight_y * (fusion.motor_dy_mm - fusion.ops_dy_mm);
      fusion.correction_x_mm += cosine * error_x - sine * error_y;
      fusion.correction_y_mm += sine * error_x + cosine * error_y;
    }
    fusion.interval_ms = common - fusion.common_time_ms;
    fusion.interval_seq++;
  }
  /* 首次齐全只建基线，不把启动前的累计转数当成车身位移。 */
  for (i = 0U; i < 4U; i++) fusion_previous_motor[i] = motor[i];
  fusion_previous_reference = reference;
  fusion.common_time_ms = common;
  fusion.baseline_ready = 1U;
}

static void Fusion_Reset(void)
{
  uint8_t i;
  Motor_Feedback feedback;
  fusion_reference_next = fusion_reference_count = 0U;
  for (i = 0U; i < 4U; i++)
  {
    fusion_motor_next[i] = fusion_motor_count[i] = 0U;
    /* 消费当前缓存序号，重定位后只采新回包，旧区间不得修正新坐标。 */
    (void)Motor_GetFeedback(fusion_address[i], &feedback);
    fusion_motor_seq[i] = feedback.position_seq;
  }
  /* 历史槽位由计数作废，不逐槽清内存；这里只清小型工作快照。 */
  memset(&fusion, 0, sizeof(fusion));
  fusion.enabled = chassis_feedback_source == CHASSIS_FEEDBACK_MIXED;
  if (chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR)
    fusion.weight_x = fusion.weight_y = 1.0f;
  fusion.ops_x_mm = ops9_x_m * 1000.0f;
  fusion.ops_y_mm = ops9_y_m * 1000.0f;
  /* 归零时承接占位原点；地图回写后承接已发布的新坐标，不恢复旧累计值。 */
  motor_x_mm = chassis_x_mm;
  motor_y_mm = chassis_y_mm;
  taskENTER_CRITICAL();
  fusion_published = fusion;
  taskEXIT_CRITICAL();
}

void Chassis_GetFusionDiagnostics(Chassis_FusionDiagnostics *snapshot)
{
  if (snapshot == NULL) return;
  taskENTER_CRITICAL();
  *snapshot = fusion_published;
  taskEXIT_CRITICAL();
}

/* 请求与应用都检查同一动作边界，防止重定位覆盖急停或下一条路线。 */
static uint8_t Chassis_CanCorrectXY(void)
{
  return chassis_position.ready &&
      (chassis_mode == 0U || chassis_mode == CHASSIS_MODE_HOLD) &&
      StraightTask_Handle == NULL && TurnTask_Handle == NULL &&
      uxQueueMessagesWaiting(Move_Queue) == 0U &&
      uxQueueMessagesWaiting(MoveComplete_Queue) == 0U;
}

uint32_t Chassis_RequestMapXY(float x_mm, float y_mm)
{
  uint32_t sequence = 0U;
  if (!isfinite(x_mm) || !isfinite(y_mm)) return 0U;
  taskENTER_CRITICAL();
  if (!odom_reset_pending && !odom_correct_pending && chassis_odom_ready &&
      Chassis_CanCorrectXY())
  {
    odom_correct_x = x_mm; odom_correct_y = y_mm;
    odom_correct_mode = chassis_mode;
    odom_correct_target_seq = target_seq;
    odom_correct_generation = Motor_GetVelocityGeneration();
    odom_correct_request++;
    if (!odom_correct_request) odom_correct_request++;
    sequence = odom_correct_request;
    odom_correct_pending = 1U;
  }
  taskEXIT_CRITICAL();
  return sequence;
}

uint8_t Chassis_MapXYApplied(uint32_t sequence)
{
  return sequence != 0U && sequence == odom_correct_applied;
}

/* 可由其它任务请求；实际清零和基准记录由 CommTask 完成，避免两任务同时改积分量。 */
void Chassis_OdomReset(void)
{
  taskENTER_CRITICAL();
  /* 反馈源和位置环一起重置；实际OPS归零仍由通信任务执行。 */
  Control_RequestZero();
  odom_reset_pending = 1U;
  odom_correct_pending = 0U;
  chassis_position.ready = 0U;  // 请求时立即关闭旧定位输出，等新原点发布再发车
  taskEXIT_CRITICAL();
}

/* OPS9、IMU与CAN反馈本拍处理后调用，按唯一开关发布位置反馈。
 * OPS9和纯电机XY各自积分，只有所选结果进入位置环，避免相互回灌。 */
void Chassis_OdomProcess(void)
{
  uint8_t corrected = 0U;
  float yaw;
  taskENTER_CRITICAL();
  if (odom_reset_pending)
  {
    odom_reset_pending = 0U;
    chassis_odom_ready = 0U;
    chassis_odom_time_ms = 0U;
    chassis_x_mm = 0.0f;
    chassis_y_mm = 0.0f;
    chassis_yaw_deg = 0.0f;
    chassis_position.x = chassis_x_mm;
    chassis_position.y = chassis_y_mm;
    chassis_position.yaw = chassis_yaw_deg;
    chassis_position.ready = 0U;
    taskEXIT_CRITICAL();
    Fusion_Reset();
    return;
  }
  if (!ops9_position_ready)
  {
    /* 沿用共享IMU归零流程；此标志等待角度/角速度齐全，不依赖定位轮运动。 */
    chassis_odom_ready = 0U;
    chassis_position.ready = 0U;
    taskEXIT_CRITICAL();
    Fusion_Reset();
    return;
  }
  taskEXIT_CRITICAL();

  /* 历史采样、插值、运动学、三角函数均不屏蔽中断；CAN快照只短暂取锁。 */
  fusion.enabled = chassis_feedback_source == CHASSIS_FEEDBACK_MIXED;
  fusion.ops_x_mm = ops9_x_m * 1000.0f;
  fusion.ops_y_mm = ops9_y_m * 1000.0f;
  if (chassis_feedback_source != CHASSIS_FEEDBACK_OPS9) Fusion_ProcessInterval();
  yaw = Chassis_Angle(ops9_yaw_deg);

  taskENTER_CRITICAL();
  /* 计算期间若收到归零请求，旧一拍不再发布，下次通信拍统一重建基线。 */
  if (odom_reset_pending)
  {
    taskEXIT_CRITICAL();
    return;
  }

  /* 纯电机首次必须等四轮和IMU对齐；回写后已有明确位置，重采基线期间保留它。 */
  if (chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR &&
      !fusion.baseline_ready && !chassis_odom_ready)
  {
    fusion_published = fusion;
    taskEXIT_CRITICAL();
    return;
  }

  /* 坐标修正与保持目标、融合反馈在同一临界区内生效。 */
  if (odom_correct_pending && chassis_odom_ready)
  {
    if (!Chassis_CanCorrectXY() || chassis_mode != odom_correct_mode ||
        target_seq != odom_correct_target_seq ||
        Motor_GetVelocityGeneration() != odom_correct_generation ||
        !MovingControl_ShiftMapXY(odom_correct_x - chassis_position.x,
                                 odom_correct_y - chassis_position.y))
    {
      /* 交接期间发生取消或新动作时，只作废请求，不恢复保持或改写坐标。 */
      odom_correct_pending = 0U;
      taskEXIT_CRITICAL();
      return;
    }
    Control_SetMapXY(odom_correct_x * 0.001f, odom_correct_y * 0.001f);
    /* 纯电机累计位置使用同一坐标基准。 */
    motor_x_mm = odom_correct_x;
    motor_y_mm = odom_correct_y;
    fusion.correction_x_mm = fusion.correction_y_mm = 0.0f;
    corrected = 1U;
    odom_correct_applied = odom_correct_request;
    odom_correct_pending = 0U;
  }
  if (chassis_feedback_source == CHASSIS_FEEDBACK_MOTOR)
  {
    chassis_x_mm = motor_x_mm;
    chassis_y_mm = motor_y_mm;
    if (fusion.baseline_ready) chassis_odom_time_ms = fusion.common_time_ms;
  }
  else
  {
    /* 纯OPS9不进入四轮积分；只有混合模式叠加现有驱动轮修正。 */
    chassis_x_mm = ops9_x_m * 1000.0f;
    chassis_y_mm = ops9_y_m * 1000.0f;
    if (chassis_feedback_source == CHASSIS_FEEDBACK_MIXED)
    {
      chassis_x_mm += fusion.correction_x_mm;
      chassis_y_mm += fusion.correction_y_mm;
    }
    chassis_odom_time_ms = ops9_position_time_ms;
  }
  chassis_yaw_deg = ops9_yaw_deg;
  chassis_odom_ready = 1U;
  chassis_position.x = chassis_x_mm;
  chassis_position.y = chassis_y_mm;
  chassis_position.yaw = yaw;
  chassis_position.ready = 1U;
  if (!corrected) fusion_published = fusion;
  taskEXIT_CRITICAL();
  /* 同一通信任务尚未进入下一拍；先丢弃旧历史，再采集重定位后的新基线。 */
  if (corrected) Fusion_Reset();
}




/* 反馈入口：定位模块每有一帧新的全场坐标就调用一次。
 * 坐标与就绪标志一起发布；更新频率不决定位置环的 5 ms 配置周期。
 * 这里不做失联判断，也不因反馈内容自动停车。 */
void Chassis_UpdatePosition(float x_mm, float y_mm, float yaw_deg)
{
  yaw_deg = Chassis_Angle(yaw_deg);
  taskENTER_CRITICAL();
  chassis_position.x = x_mm;
  chassis_position.y = y_mm;
  chassis_position.yaw = yaw_deg;
  /* 归零请求可能抢占正在计算的旧一拍，旧结果不能重新打开输出门。 */
  if (odom_reset_pending == 0U) chassis_position.ready = 1U;
  taskEXIT_CRITICAL();
}




/* 5 ms输出任务只消费当前动作代次的速度或保持指令。 */
void control_task(void *argument)
{
  TickType_t wake = xTaskGetTickCount();
  move_cmd_t command;
  HAL_StatusTypeDef hold_result;
  uint8_t address, active;
  (void)argument;
  for (address = 1U; address <= 4U; address++)
    (void)Motor_Enable(address, 1U);
  for (;;)
  {
    hold_result = MovingControl_HoldTick(&command);
    if (hold_result == HAL_OK || xQueueReceive(Move_CMD_Queue, &command, 0) == pdTRUE)
    {
      taskENTER_CRITICAL();
      active = chassis_mode != 0U && command.target_seq == target_seq &&
               command.generation == Motor_GetVelocityGeneration() &&
               (!command.ramp || chassis_position.ready);
      taskEXIT_CRITICAL();
      if (active)
        (void)Chassis_OutputVelocity(command.x, command.y, command.yaw,
            command.acceleration, command.ramp, command.ratio, command.generation);
    }
    /* 超期后不突发补发旧节拍，下一拍仍取当前目标。 */
    /* 跳过错过的整拍，仍回到原5ms相位；避免每次超期额外再等完整5ms。 */
    wake += ((TickType_t)(xTaskGetTickCount()-wake) / pdMS_TO_TICKS(5U)) * pdMS_TO_TICKS(5U);
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
  }
}
