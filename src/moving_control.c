#include "moving_control.h"
#include "chassis.h"
#include "motor.h"
#include "imu.h"
#include "app_comm.h"
#include "encoder.h"
#include "ops9_control.h"
#include <math.h>

QueueHandle_t Move_Queue, Move_CMD_Queue, MoveComplete_Queue;
TaskHandle_t MovingControlTask_Handle, StraightTask_Handle, TurnTask_Handle;
volatile uint8_t chassis_mode;
volatile uint32_t target_seq;
static uint8_t in_pos;
static uint32_t curve_start_ms;
static uint8_t curve_started;
static PathTrajectory move_path; // 单一调用任务串行使用，队列只存动作参数。

/* 子任务完成后只发通知，管理任务统一回收句柄和参数。 */
static void move_task_finish(uint32_t notify)
{
  xTaskNotify(MovingControlTask_Handle, notify, eSetValueWithOverwrite);
  for (;;) vTaskSuspend(NULL);
}

void MovingControl_Init(void)
{
  Move_Queue = xQueueCreate(100, sizeof(MoveTaskParams_t));
  /* 轮速队列只保留最新命令。 */
  Move_CMD_Queue = xQueueCreate(1, sizeof(move_cmd_t));
  MoveComplete_Queue = xQueueCreate(8, sizeof(general_notify_t));
  if (Move_Queue == NULL || Move_CMD_Queue == NULL || MoveComplete_Queue == NULL)
    Error_Handler();
}

/* 同一调用任务先等到本次结果，再提交下一动作。 */
static BaseType_t move_execute(const MoveTaskParams_t *params)
{
  general_notify_t event;
  uint8_t ready;
  taskENTER_CRITICAL();
  ready = chassis_position.ready;
  taskEXIT_CRITICAL();
  if (!ready || xQueueSend(Move_Queue, params, 0U) != pdPASS)
    return pdFAIL;
  if (xQueueReceive(MoveComplete_Queue, &event, portMAX_DELAY) != pdTRUE)
    return pdFAIL;
  return event.cmd == MOVE_COMPLETE_EVENT ? pdPASS : pdFAIL;
}

BaseType_t MoveTo(int32_t x_mm, int32_t y_mm, float heading_deg)
{
  MoveTaskParams_t params = {0};
  params.cmd = StraightForward;
  params.point.x = x_mm;
  params.point.y = y_mm;
  params.point.target_angle = heading_deg;
  return move_execute(&params);
}

BaseType_t MoveTurnTo(int32_t x_mm, int32_t y_mm, float heading_deg)
{
  MoveTaskParams_t params = {0};
  params.cmd = MOVE_TURN_TO;
  params.point.x = x_mm;
  params.point.y = y_mm;
  params.point.target_angle = heading_deg;
  return move_execute(&params);
}

/* 转向曲线用秒计算，预算再向上对齐到现有 5 ms 输出周期。 */
typedef struct
{
  float acceleration, peak_rpm, rise_s, flat_s, total_s;
  uint32_t budget_ms;
} TurnProfile;

/* 每个动作独立保存转向进度，移动和原地转向复用算法，不共享运行状态。 */
typedef struct
{
  TurnProfile profile;
  float initial_error, last_output;
  uint32_t previous_elapsed;
} TurnMotion;

/* 移动计时沿用原曲线；转向另记起点，延迟触发不消耗其加减速时间。 */
typedef struct
{
  TurnMotion turn;
  float start_heading;
  uint32_t turn_start_ms;
  uint8_t turn_started;
  uint8_t use_fine_ratio; // 动作移交到保持后，继续使用同一末端增益。
} MoveMotion;

/* 动作结束后复制目标和曲线状态，任务退出不会丢失三轴保持目标。 */
static struct
{
  float x, y, yaw;
  MoveMotion motion;
  uint32_t sequence, generation;
} chassis_hold;

static BaseType_t Turn_BuildProfile(float delta_deg, uint32_t duration_ms,
                                  TurnProfile *profile)
{
  const float peak_rpm = 100.0f;       // 90°、180°共用峰值轮速。
  const float ramp_angle_ratio = 0.2f; // 90°加速、减速各占 20% 转角。
  float radius = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f;
  float reference_area = CHASSIS_GEAR_RATIO * radius * 90.0f /
                         (3.0f * CHASSIS_WHEEL_DIAMETER_MM);
  float area;

  if (!isfinite(delta_deg) || fabsf(delta_deg) > 180.0f)
    return pdFAIL;
  /* 面积单位 rpm·s；固定加速度由 90°基准推导，不随请求角度改变。 */
  area = reference_area * fabsf(delta_deg) / 90.0f;
  profile->acceleration = peak_rpm * peak_rpm /
                          (2.0f * ramp_angle_ratio * reference_area);
  profile->peak_rpm = peak_rpm;
  profile->rise_s = peak_rpm / profile->acceleration;
  profile->flat_s = area / peak_rpm - profile->rise_s;
  if (profile->flat_s < 0.0f)
  {
    /* 短角度使用三角形，不强行达到 100 rpm，也不改变加速度。 */
    profile->rise_s = sqrtf(area / profile->acceleration);
    profile->peak_rpm = profile->acceleration * profile->rise_s;
    profile->flat_s = 0.0f;
  }
  profile->total_s = 2.0f * profile->rise_s + profile->flat_s;
  profile->budget_ms = (uint32_t)ceilf(profile->total_s * 1000.0f / 5.0f) * 5U;
  return duration_ms >= profile->budget_ms ? pdPASS : pdFAIL;
}

/* 在调用任务中检查车身扫掠，5ms任务不做候选搜索。 */
BaseType_t MovePathCanPlan(const PathPoint *points, uint16_t count, float start_heading,
                          float heading_deg, float approach_mm)
{
  TurnProfile profile;
  PathMotionConfig motion;
  const ChassisMotionConfig *config = &chassis_motion_config;
  float scale = config->speed_ratio * 3.14159265f * CHASSIS_WHEEL_DIAMETER_MM /
                (60.0f * CHASSIS_GEAR_RATIO);
  float deg_per_rpm = 3.0f * CHASSIS_WHEEL_DIAMETER_MM /
                     (CHASSIS_GEAR_RATIO * (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f);
  if (Turn_BuildProfile(0.0f, UINT32_MAX, &profile) != pdPASS) return pdFAIL;
  /* 只在接口边界换算物理单位；轨迹层只处理物理量。 */
  motion.cruise = config->XYVmax * scale;
  motion.omega_max = 100.0f * deg_per_rpm;
  motion.alpha_max = profile.acceleration * deg_per_rpm;
  motion.rise_s = config->rise_ms * 0.001f; motion.brake_mm = config->brake_mm;
  motion.end_speed = config->end_speed * config->fine_ratio * scale;
  motion.speed_curve = config->speed_curve;
  return Path_PlanClear(&move_path, points, count, approach_mm, start_heading, heading_deg,
                        &motion, &chassis_path_clearance) ? pdPASS : pdFAIL;
}

BaseType_t MovePath(const PathPoint *points, uint16_t count, float heading_deg, float approach_mm)
{
  MoveTaskParams_t params = {0};
  float heading;
  taskENTER_CRITICAL(); heading = chassis_position.yaw; taskEXIT_CRITICAL();
  /* 调用任务完成几何预检；5ms动作任务只采样已验收的轨迹。 */
  if (MovePathCanPlan(points, count, heading, heading_deg, approach_mm) != pdPASS) return pdFAIL;
  params.cmd = MOVE_PATH;
  params.point.x = (int)lroundf(points[count - 1U].x);
  params.point.y = (int)lroundf(points[count - 1U].y);
  params.point.target_angle = heading_deg;
  return move_execute(&params);
}

/* 使用实际经过时间取解析曲线，不依赖任务恰好每5ms被调度。 */
static float Turn_ProfileRpm(const TurnProfile *profile, float elapsed_s)
{
  if (elapsed_s < profile->rise_s)
    return profile->acceleration * elapsed_s;
  if (elapsed_s < profile->rise_s + profile->flat_s)
    return profile->peak_rpm;
  if (elapsed_s < profile->total_s)
    return profile->acceleration * (profile->total_s - elapsed_s);
  return 0.0f;
}

/* 对同一转速曲线解析积分；rpm·s按原车轮/车身几何换算成转角。 */
static float Turn_ProfileAngle(const TurnProfile *profile, float elapsed_s)
{
  float area, decel_s;
  float radius = (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f;
  if (elapsed_s < 0.0f) elapsed_s = 0.0f;
  if (elapsed_s > profile->total_s) elapsed_s = profile->total_s;
  if (elapsed_s < profile->rise_s)
    area = 0.5f * profile->acceleration * elapsed_s * elapsed_s;
  else
  {
    area = 0.5f * profile->peak_rpm * profile->rise_s;
    if (elapsed_s < profile->rise_s + profile->flat_s)
      area += profile->peak_rpm * (elapsed_s - profile->rise_s);
    else
    {
      decel_s = elapsed_s - profile->rise_s - profile->flat_s;
      area += profile->peak_rpm * (profile->flat_s + decel_s) -
              0.5f * profile->acceleration * decel_s * decel_s;
    }
  }
  return area * 3.0f * CHASSIS_WHEEL_DIAMETER_MM / (CHASSIS_GEAR_RATIO * radius);
}

/* 同一转向输出可作为纯旋转，也可叠加到移动的 x/y 指令中。 */
static float Turn_Output(TurnMotion *motion, float error, uint32_t elapsed)
{
  float direction = motion->initial_error < 0.0f ? -1.0f : 1.0f;
  float output, step_limit, reference_error;
  const float rpm_max = 100.0f;

  if (!isfinite(error)) return error;
  error = fmodf(error, 360.0f);
  if (error > 180.0f) error -= 360.0f;
  if (error < -180.0f) error += 360.0f;
  reference_error = error;
  output = 0.0f;
  if ((float)elapsed < motion->profile.total_s * 1000.0f)
  {
    /* 最终角差减去规划剩余转角，得到本拍规划航向相对实测航向的误差。
     * 曲线执行期间，实测角度扰动也参与本拍控制。 */
    reference_error = error - (motion->initial_error - direction *
      Turn_ProfileAngle(&motion->profile, (float)elapsed * 0.001f));
    output = direction * Turn_ProfileRpm(&motion->profile, (float)elapsed * 0.001f);
  }
  taskENTER_CRITICAL();
  output += IMU_AngleP(reference_error, PD_Rotate.P_Pos, PD_Rotate.out_max);
  taskEXIT_CRITICAL();
  if ((float)elapsed >= motion->profile.total_s * 1000.0f)
  {
    /* 角度 P 把 -180 折为 +180，半圈平局仍按动作初始角差选方向。 */
    if (fabsf(error) == 180.0f) output = direction * fabsf(output);
  }
  if (!isfinite(output)) return output;
  if (output > rpm_max) output = rpm_max;
  else if (output < -rpm_max) output = -rpm_max;
  /* 曲线与 P 收尾均按实际间隔限制速度变化，反向纠偏也经过减速。 */
  step_limit = motion->profile.acceleration *
               (float)(elapsed - motion->previous_elapsed) * 0.001f;
  if (output > motion->last_output + step_limit) output = motion->last_output + step_limit;
  else if (output < motion->last_output - step_limit) output = motion->last_output - step_limit;
  motion->previous_elapsed = elapsed;
  motion->last_output = output;
  return output;
}

BaseType_t Turn(float delta_deg, uint32_t duration_ms)
{
  MoveTaskParams_t params = {0};
  TurnProfile profile;
  if (delta_deg == 0.0f) return pdPASS;  // 零角度不启动动作，也不等待计时。
  /* 预算不足或角度非法时不入队，不先转半段再因总时长截断。 */
  if (Turn_BuildProfile(delta_deg, duration_ms, &profile) != pdPASS)
    return pdFAIL;
  params.cmd = TURN;
  params.turn_delta_deg = delta_deg;
  params.duration_ms = duration_ms;
  return move_execute(&params);
}

/* 沿用原停车复位：先关输出、推进代次，再清到位和曲线状态。 */
static float Chassis_Angle(float angle)
{
  angle = fmodf(angle, 360.0f);          // 先折到 (-360, 360)
  if (angle > 180.0f) angle -= 360.0f;
  if (angle < -180.0f) angle += 360.0f;
  return angle;
}

static void numerical_limit(float *value, float max, float min, float dead_zone)
{
  if (*value > dead_zone) *value += min;
  else if (*value < -dead_zone) *value -= min;
  if (*value > max) *value = max;
  else if (*value < -max) *value = -max;
}

static HAL_StatusTypeDef chassis_move(float x, float y, float z,
                                      float px, float py, float angle,
                                      uint32_t command_seq, uint32_t generation,
                                      MoveMotion *motion, uint8_t late_turn,
                                      move_cmd_t *output)
{
  const ChassisMotionConfig *config = &chassis_motion_config;
  float mKpx = config->mKpx, mKpy = config->mKpy;
  float fine_ratio = motion->use_fine_ratio ? config->fine_ratio : 1.0f;
  float XYVmax = config->XYVmax, XYVmin = config->XYVmin;
  float speed_ratio = config->speed_ratio;
  uint8_t speed_curve = config->speed_curve;
  float rise_ms = config->rise_ms, brake_mm = config->brake_mm;
  float end_speed = config->end_speed;
  const float turn_start_distance_mm = config->turn_start_distance_mm;
  float devx = px - x, devy = py - y;      // 位置偏差：反馈减目标
  float devz = Chassis_Angle(z - angle);   // 角度偏差：先归一再相减，避免绕圈
  float rad = angle * 3.141f / 180.0f;     // 用当前航向把场地系偏差旋转到车体系
  float vy1 = cosf(rad) * mKpy * devy;
  float vx2 = sinf(rad) * mKpy * devy;
  float vy2 = sinf(rad) * mKpx * devx;
  float vx1 = cosf(rad) * mKpx * devx;
  float vz, move_x, move_y, move_speed, start_limit, brake_limit, speed_limit;
  uint32_t elapsed_ms, now_ms, turn_elapsed;
  int32_t vx, vy, yaw;
  TurnMotion *turn = &motion->turn;

  /* 换目标或停车的复位与本拍计算互斥，避免旧目标重新写入输出。 */
  taskENTER_CRITICAL();
  if ((chassis_mode != 1U && chassis_mode != 2U && chassis_mode != CHASSIS_MODE_HOLD) ||
      command_seq != target_seq || generation != Motor_GetVelocityGeneration())
  {
    taskEXIT_CRITICAL();
    return HAL_ERROR;
  }
  now_ms = HAL_GetTick();
  if (!isfinite(devx) || !isfinite(devy) || !isfinite(devz) || !isfinite(angle))
  {
    taskEXIT_CRITICAL();
    return HAL_ERROR;
  }
  if (!curve_started)
  {
    /* 首次有效定位才确定相对转角；MoveTo 的输入仍是地图绝对航向。 */
    if (!isfinite(devz) ||
        Turn_BuildProfile(late_turn ? 0.0f : devz, UINT32_MAX, &turn->profile) != pdPASS)
    {
      taskEXIT_CRITICAL();
      return HAL_ERROR;
    }
    turn->initial_error = late_turn ? 0.0f : devz;
    turn->last_output = 0.0f;
    turn->previous_elapsed = 0U;
    motion->start_heading = angle;
    motion->turn_start_ms = now_ms;
    motion->turn_started = late_turn ? 0U : 1U;
    curve_start_ms = now_ms;
    curve_started = 1U;
  }
  elapsed_ms = (uint32_t)(now_ms - curve_start_ms);
  if (late_turn && !motion->turn_started &&
      hypotf(devx, devy) <= turn_start_distance_mm)
  {
    /* 仅触发一次，用这一拍的实际航向规划，越过门槛后不再反复重建。 */
    if (Turn_BuildProfile(devz, UINT32_MAX, &turn->profile) != pdPASS)
    {
      taskEXIT_CRITICAL();
      return HAL_ERROR;
    }
    turn->initial_error = devz;
    turn->previous_elapsed = 0U;
    /* 保留上一拍保持航向的轮速，首次取曲线时仍受相同加速度限制。 */
    motion->turn_start_ms = now_ms;
    motion->turn_started = 1U;
  }
  turn_elapsed = (uint32_t)(now_ms - motion->turn_start_ms);
  vz = Turn_Output(turn, motion->turn_started ? devz :
                   Chassis_Angle(motion->start_heading - angle), turn_elapsed);
  taskEXIT_CRITICAL();
  if (!isfinite(vz)) return HAL_ERROR;

  numerical_limit(&vy1, XYVmax, XYVmin, 5.0f);
  numerical_limit(&vx2, XYVmax, XYVmin, 5.0f);
  numerical_limit(&vy2, XYVmax, XYVmin, 5.0f);
  numerical_limit(&vx1, XYVmax, XYVmin, 5.0f);
  move_x = -(vx1 + vx2);          // 合成车体系平移速度，负号沿用原坐标系约定
  move_y = -(vy1 - vy2);
  /* 末端补偿按配置倍率放大，随后仍服从共用平移上限及起步曲线。 */
  move_x *= fine_ratio; move_y *= fine_ratio;
  move_speed = hypotf(move_x, move_y);
  end_speed *= fine_ratio; // 末端限速与位置补偿使用同一倍率。
  start_limit = XYVmax * Path_CurveValue(speed_curve, (float)elapsed_ms / rise_ms);
  brake_limit = end_speed + (XYVmax - end_speed) *
                Path_CurveValue(speed_curve, hypotf(devx, devy) / brake_mm);
  /* 起步按时间、减速按剩距取所选曲线上限；短行程取较低值，位置环持续收尾。 */
  speed_limit = fminf(start_limit, brake_limit);
  if (move_speed > speed_limit && move_speed > 0.0f)
  {
    move_x *= speed_limit / move_speed;  // 平移向量整体限速，保持原行驶方向
    move_y *= speed_limit / move_speed;
  }
  vx = (int32_t)move_x;
  vy = (int32_t)move_y;
  yaw = (int32_t)vz;
  /* 用最新定位判断，一次进窗就置位，直到换目标或主动停车才清零。
   * 换目标或主动停止后，这一拍旧目标的结果不能写回新的到位状态。 */
  taskENTER_CRITICAL();
  if (chassis_mode == 1U && command_seq == target_seq)
  {
    if (fabsf(devx) < 5.0f && fabsf(devy) < 5.0f && fabsf(devz) < 2.0f &&
        motion->turn_started && (float)turn_elapsed >= turn->profile.total_s * 1000.0f)
      in_pos = 1U;               // 两个移动接口共用严格窗口，仍须等旋转曲线结束。
  }
  taskEXIT_CRITICAL();
  {
    move_cmd_t command = {0};
    command.x = vx; command.y = vy; command.yaw = yaw;
    command.ratio = speed_ratio; command.ramp = 1U;
    command.target_seq = command_seq; command.generation = generation;
    if (output != NULL)
    {
      *output = command;  // 保持由输出任务直接消费，不把自身速度反复排入队列。
      return HAL_OK;
    }
    return xQueueOverwrite(Move_CMD_Queue, &command) == pdPASS ? HAL_OK : HAL_BUSY;
  }  // acceleration=0 关闭驱动器加减速，只用上面的行驶曲线
}

/* 切换时撤销旧轮速、推进目标代次；不发送STOP，下一拍直接接续保持。 */
static BaseType_t Chassis_EnterHold(float x, float y, float yaw, const MoveMotion *motion,
                                   uint32_t sequence, uint32_t generation, uint8_t mode)
{
  taskENTER_CRITICAL();
  if (chassis_mode != mode || target_seq != sequence ||
      Motor_GetVelocityGeneration() != generation || !chassis_position.ready)
  {
    taskEXIT_CRITICAL();
    return pdFAIL;
  }
  chassis_hold.x = x; chassis_hold.y = y; chassis_hold.yaw = yaw;
  chassis_hold.motion = *motion;
  target_seq++;
  Motor_CancelVelocity();
  xQueueReset(Move_CMD_Queue);
  chassis_hold.sequence = target_seq;
  chassis_hold.generation = Motor_GetVelocityGeneration();
  chassis_mode = CHASSIS_MODE_HOLD;
  taskEXIT_CRITICAL();
  return pdPASS;
}

BaseType_t MovingControl_ShiftMapXY(float dx, float dy)
{
  /* 调用者必须同时锁住定位发布，确保目标和反馈坐标在同一拍平移。 */
  if (chassis_mode == 0U) return pdPASS;
  if (chassis_mode != CHASSIS_MODE_HOLD || chassis_hold.sequence != target_seq ||
      chassis_hold.generation != Motor_GetVelocityGeneration() ||
      !isfinite(dx) || !isfinite(dy)) return pdFAIL;
  chassis_hold.x += dx; chassis_hold.y += dy;
  target_seq++;
  Motor_CancelVelocity();
  xQueueReset(Move_CMD_Queue);
  chassis_hold.sequence = target_seq;
  chassis_hold.generation = Motor_GetVelocityGeneration();
  return pdPASS;
}

BaseType_t Chassis_HoldCurrent(void)
{
  Chassis_Position position;
  MoveMotion motion = {0};
  uint32_t sequence, generation, now;
  BaseType_t result;
  taskENTER_CRITICAL();
  position = chassis_position;
  sequence = target_seq; generation = Motor_GetVelocityGeneration();
  if (!position.ready || chassis_mode != 0U ||
      !isfinite(position.x) || !isfinite(position.y) || !isfinite(position.yaw) ||
      StraightTask_Handle != NULL || TurnTask_Handle != NULL ||
      uxQueueMessagesWaiting(Move_Queue) != 0U ||
      Turn_BuildProfile(0.0f, UINT32_MAX, &motion.turn.profile) != pdPASS)
  {
    taskEXIT_CRITICAL();
    return pdFAIL;
  }
  now = HAL_GetTick();
  motion.start_heading = position.yaw;
  motion.turn_start_ms = now;
  motion.turn_started = 1U;
  result = Chassis_EnterHold(position.x, position.y, position.yaw, &motion,
                             sequence, generation, 0U);
  if (result == pdPASS)
  {
    curve_started = 1U;
    curve_start_ms = now;
  }
  taskEXIT_CRITICAL();
  return result;
}

HAL_StatusTypeDef MovingControl_HoldTick(move_cmd_t *command)
{
  Chassis_Position position;
  MoveMotion motion;
  float x, y, yaw;
  uint32_t sequence, generation;
  HAL_StatusTypeDef result;
  if (command == NULL) return HAL_ERROR;
  taskENTER_CRITICAL();
  if (chassis_mode != CHASSIS_MODE_HOLD || !chassis_position.ready)
  {
    taskEXIT_CRITICAL();
    return HAL_BUSY;
  }
  position = chassis_position;
  x = chassis_hold.x; y = chassis_hold.y; yaw = chassis_hold.yaw;
  motion = chassis_hold.motion;
  sequence = chassis_hold.sequence; generation = chassis_hold.generation;
  taskEXIT_CRITICAL();
  result = chassis_move(x, y, yaw, position.x, position.y, position.yaw,
                         sequence, generation, &motion, 0U, command);
  taskENTER_CRITICAL();
  /* 计算期间若已停车或换目标，既不回写旧状态，也不把取消当成输出故障。 */
  if (chassis_mode != CHASSIS_MODE_HOLD || target_seq != sequence ||
      Motor_GetVelocityGeneration() != generation)
    result = HAL_BUSY;
  else if (result == HAL_OK)
    chassis_hold.motion = motion;
  taskEXIT_CRITICAL();
  return result;
}

/* 连续路径叠加轨迹前馈和位置反馈，经过拐点时保持跟踪。 */
static HAL_StatusTypeDef path_move_tick(PathProgress *progress, float dt,
                                        const Chassis_Position *position, MoveMotion *motion,
                                        uint32_t sequence, uint32_t generation, move_cmd_t *command)
{
  PathSample sample;
  const ChassisMotionConfig *config = &chassis_motion_config;
  float scale = config->speed_ratio * 3.14159265f * CHASSIS_WHEEL_DIAMETER_MM /
                (60.0f * CHASSIS_GEAR_RATIO);
  float deg_per_rpm = 3.0f * CHASSIS_WHEEL_DIAMETER_MM /
                     (CHASSIS_GEAR_RATIO * (CHASSIS_WHEELBASE_MM + CHASSIS_TRACK_MM) * 0.5f);
  float wx, wy, speed, limit, angle, output, change, correction_x, correction_y;
  if (!isfinite(position->x) || !isfinite(position->y) || !isfinite(position->yaw)) return HAL_ERROR;
  Path_Advance(&move_path, progress, dt);
  Path_Sample(&move_path, progress->s, &sample);
  /* 最小补偿只加在位置纠偏上，零误差与轨迹前馈都不额外加速。 */
  correction_x = config->mKpx * (sample.x - position->x);
  correction_y = config->mKpy * (sample.y - position->y);
  numerical_limit(&correction_x, config->XYVmax, config->XYVmin, 5.0f);
  numerical_limit(&correction_y, config->XYVmax, config->XYVmin, 5.0f);
  /* 前馈换回内部速度量，平移比例仅由轮速混合器施加一次。 */
  wx = sample.tx * progress->speed / scale + correction_x;
  wy = sample.ty * progress->speed / scale + correction_y;
  /* 位置P也服从前瞻减速包络，不能在弯前把已降低的前馈重新抬高。
   * 参考走完整段后单独保留小速度纠偏，避免零前馈把终点误差锁住。 */
  limit = fminf(config->XYVmax, Path_SpeedLimit(&move_path, progress->s) / scale);
  if (progress->s >= move_path.length)
  {
    /* 只加快参考走完后的末端纠偏，前馈和弯道限速维持原值。 */
    float fine_ratio = motion->use_fine_ratio ? config->fine_ratio : 1.0f;
    wx *= fine_ratio; wy *= fine_ratio;
    limit = fminf(config->XYVmax, config->end_speed * fine_ratio);
  }
  /* 位置反馈也服从起步曲线，避免首拍纠偏绕过统一的起步时间。 */
  limit = fminf(limit, config->XYVmax * Path_CurveValue(config->speed_curve,
      progress->elapsed * 1000.0f / config->rise_ms));
  speed = hypotf(wx, wy);
  if (speed > limit) { wx *= limit / speed; wy *= limit / speed; }
  angle = position->yaw * 3.14159265f / 180.0f;
  command->x = (int32_t)(cosf(angle) * wx + sinf(angle) * wy);
  command->y = (int32_t)(cosf(angle) * wy - sinf(angle) * wx);
  output = sample.heading_d1 * progress->speed / deg_per_rpm +
           IMU_AngleP(Chassis_Angle(sample.heading - position->yaw), PD_Rotate.P_Pos, PD_Rotate.out_max);
  output = fmaxf(-100.0f, fminf(output, 100.0f));
  change = motion->turn.profile.acceleration * dt;
  output = fmaxf(motion->turn.last_output - change, fminf(output, motion->turn.last_output + change));
  motion->turn.last_output = output;
  motion->turn.previous_elapsed = (uint32_t)(progress->elapsed * 1000.0f);
  command->yaw = (int32_t)output;
  command->ratio = config->speed_ratio; command->ramp = 1U; command->acceleration = 0U;
  command->target_seq = sequence; command->generation = generation;
  return HAL_OK;
}

static void follow_path_task(void *argument)
{
  MoveTaskParams_t value = *(MoveTaskParams_t *)argument;
  PathProgress progress = {0};
  MoveMotion motion = {0};
  Chassis_Position position;
  uint32_t sequence = target_seq, generation = Motor_GetVelocityGeneration();
  uint32_t start = HAL_GetTick(), previous = start;
  taskENTER_CRITICAL();
  position = chassis_position;
  taskEXIT_CRITICAL();
  if (!position.ready || Turn_BuildProfile(0.0f, UINT32_MAX, &motion.turn.profile) != pdPASS)
    move_task_finish(MOVE_NOTIFY_ERROR);
  /* 不重建全程航向，否则会覆盖已通过车身净空检查的局部转向窗口。 */
  motion.start_heading = position.yaw; motion.turn_start_ms = start; motion.turn_started = 1U;
  motion.use_fine_ratio = 1U;
  taskENTER_CRITICAL();
  if (chassis_mode == 1U && target_seq == sequence)
  { curve_started = 1U; curve_start_ms = start; }
  taskEXIT_CRITICAL();
  for (;;)
  {
    move_cmd_t command = {0};
    uint32_t now = HAL_GetTick();
    float dt = (float)(now - previous) * 0.001f;
    previous = now;
    taskENTER_CRITICAL();
    position = chassis_position;
    if (chassis_mode != 1U || sequence != target_seq || generation != Motor_GetVelocityGeneration())
    { taskEXIT_CRITICAL(); move_task_finish(MOVE_NOTIFY_ERROR); }
    taskEXIT_CRITICAL();
    if (position.ready)
    {
      if (path_move_tick(&progress, dt, &position, &motion, sequence, generation, &command) != HAL_OK)
        move_task_finish(MOVE_NOTIFY_ERROR);
      if (progress.s >= move_path.length &&
          fabsf(position.x - value.point.x) < 5.0f && fabsf(position.y - value.point.y) < 5.0f &&
          fabsf(Chassis_Angle(value.point.target_angle - position.yaw)) < 2.0f)
      {
        if (Chassis_EnterHold(value.point.x, value.point.y, value.point.target_angle,
                              &motion, sequence, generation, 1U) != pdPASS)
          move_task_finish(MOVE_NOTIFY_ERROR);
        move_task_finish(MOVE_NOTIFY_STRAIGHT);
      }
      /* 与旧移动相同，发布前重查代次；主动停止后旧任务不能重开输出。 */
      taskENTER_CRITICAL();
      if (chassis_mode != 1U || sequence != target_seq || generation != Motor_GetVelocityGeneration())
      { taskEXIT_CRITICAL(); move_task_finish(MOVE_NOTIFY_ERROR); }
      (void)xQueueOverwrite(Move_CMD_Queue, &command);
      taskEXIT_CRITICAL();
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void straight_line_task(void *argument)
{
  MoveTaskParams_t value = *(MoveTaskParams_t *)argument;
  uint32_t command_seq = target_seq;
  uint32_t generation = Motor_GetVelocityGeneration();
  MoveMotion motion = {0}; // 每段移动独立记录保持航向、末段触发和转向进度。

  motion.use_fine_ratio = 1U;
  for (;;)
  {
    Chassis_Position position;
    taskENTER_CRITICAL();
    position = chassis_position;
    taskEXIT_CRITICAL();
    if (chassis_mode != 1U || command_seq != target_seq)
    {
      move_task_finish(MOVE_NOTIFY_ERROR);
    }
    if (position.ready)
    {
      if (chassis_move(value.point.x, value.point.y, value.point.target_angle,
                       position.x, position.y, position.yaw, command_seq, generation,
                       &motion, value.cmd == MOVE_TURN_TO, NULL) == HAL_ERROR)
      {
        if (chassis_mode != 1U || command_seq != target_seq)
          move_task_finish(MOVE_NOTIFY_ERROR);
        move_task_finish(MOVE_NOTIFY_ERROR);
      }
      if (in_pos)
      {
        /* 到点后将目标交给保持环继续纠偏。 */
        if (Chassis_EnterHold(value.point.x, value.point.y, value.point.target_angle,
                              &motion, command_seq, generation, 1U) != pdPASS)
          move_task_finish(MOVE_NOTIFY_ERROR);
        move_task_finish(MOVE_NOTIFY_STRAIGHT);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* 总时长契约不变；期间XY守住起点，曲线和角度反馈同时控制，返回后继续保持。 */
static void rotate_still_task(void *argument)
{
  MoveTaskParams_t value = *(MoveTaskParams_t *)argument;
  Chassis_Position origin, position;
  float target;
  uint32_t duration_ms = value.duration_ms;
  uint32_t start_ms, elapsed;
  uint32_t command_seq = target_seq;
  uint32_t generation = Motor_GetVelocityGeneration();
  float initial_error;
  MoveMotion motion = {0};
  uint32_t notify = MOVE_NOTIFY_TURN;

  taskENTER_CRITICAL();
  origin = chassis_position;
  initial_error = value.turn_delta_deg;
  target = Chassis_Angle(origin.yaw + initial_error); // 与XY保持共用同一地图航向。
  PD_Rotate.output = 0.0f;
  taskEXIT_CRITICAL();
  if (!origin.ready || !isfinite(origin.x) || !isfinite(origin.y) || !isfinite(target) ||
      Turn_BuildProfile(initial_error, duration_ms, &motion.turn.profile) != pdPASS)
  {
    move_task_finish(MOVE_NOTIFY_ERROR);
  }
  motion.turn.initial_error = initial_error;
  start_ms = HAL_GetTick();
  motion.start_heading = origin.yaw;
  motion.turn_start_ms = start_ms;
  motion.turn_started = 1U;
  taskENTER_CRITICAL();
  if (chassis_mode == 2U && command_seq == target_seq)
  {
    curve_started = 1U;
    curve_start_ms = start_ms;
  }
  taskEXIT_CRITICAL();

  for (;;)
  {
    if (chassis_mode != 2U || command_seq != target_seq)
    {
      move_task_finish(MOVE_NOTIFY_ERROR);
    }
    elapsed = (uint32_t)(HAL_GetTick() - start_ms);
    if (elapsed >= duration_ms) break;
    taskENTER_CRITICAL();
    position = chassis_position;
    taskEXIT_CRITICAL();
    /* 原地转向也复用位置环；XY锁住起始点，航向每拍跟随转向曲线纠偏。 */
    if (position.ready && chassis_move(origin.x, origin.y, target,
          position.x, position.y, position.yaw, command_seq, generation,
          &motion, 0U, NULL) == HAL_ERROR)
    {
      if (chassis_mode != 2U || command_seq != target_seq)
        move_task_finish(MOVE_NOTIFY_ERROR);
      notify = MOVE_NOTIFY_ERROR;
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  if (notify != MOVE_NOTIFY_ERROR && Chassis_EnterHold(origin.x, origin.y, target, &motion,
                             command_seq, generation, 2U) != pdPASS)
    notify = MOVE_NOTIFY_ERROR;
  move_task_finish(notify);
}

/* 动作分发与参数回收放在一起；增加动作时在这里增加分支。 */
void moving_control_task(void *argument)
{
  /* 管理任务保管当前参数，子任务入口立即复制；回收子任务后才接收下一条。 */
  MoveTaskParams_t request;
  uint32_t notify_value;
  general_notify_t event;
  BaseType_t created;
  (void)argument;
  MovingControlTask_Handle = xTaskGetCurrentTaskHandle();
  for (;;)
  {
    if (StraightTask_Handle == NULL && TurnTask_Handle == NULL)
    {
      if (xQueueReceive(Move_Queue, &request, pdMS_TO_TICKS(10)) != pdTRUE) continue;
      event.action = request.cmd;
      /* 正常换目标仍撤销旧速度批次，保证输出只有一个控制源。 */
      taskENTER_CRITICAL();
      Motor_CancelVelocity();
      xQueueReset(Move_CMD_Queue);
      target_seq++;
      chassis_mode = (request.cmd == StraightForward || request.cmd == MOVE_TURN_TO || request.cmd == MOVE_PATH) ? 1U : 2U;
      in_pos = curve_started = 0U;
      PD_Pos_Rotate_Cascade_1.output = 0.0f;
      taskEXIT_CRITICAL();
      created = pdFAIL;
      if (request.cmd == StraightForward || request.cmd == MOVE_TURN_TO)
        created = xTaskCreate(straight_line_task, "Straight", 1024, &request, 46, &StraightTask_Handle);
      else if (request.cmd == MOVE_PATH)
        created = xTaskCreate(follow_path_task, "Path", 1024, &request, 46, &StraightTask_Handle);
      else if (request.cmd == TURN)
        created = xTaskCreate(rotate_still_task, "RotateStill", 1024, &request, 46, &TurnTask_Handle);
      if (created == pdPASS) continue;
      notify_value = MOVE_NOTIFY_ERROR;
    }
    else if (xTaskNotifyWait(0, UINT32_MAX, &notify_value, pdMS_TO_TICKS(10)) != pdTRUE)
      continue;

    /* 局部失败与完成共用资源回收，不发STOP、不锁住下一动作。 */
    event.action = request.cmd;
    event.cmd = notify_value == MOVE_NOTIFY_ERROR ? MOVE_ERROR_EVENT : MOVE_COMPLETE_EVENT;
    if (StraightTask_Handle != NULL) vTaskDelete(StraightTask_Handle);
    if (TurnTask_Handle != NULL) vTaskDelete(TurnTask_Handle);
    StraightTask_Handle = TurnTask_Handle = NULL;
    (void)xQueueSend(MoveComplete_Queue, &event, 0);
  }
}

/* 1 ms通信任务负责惯导解析、定位轮积分和CAN反馈查询。 */
void chassis_comm_task(void *argument)
{
  TickType_t wake = xTaskGetTickCount();
  (void)argument;
  for (;;)
  {
    Comm_Process();
    IMU_CalibrationProcess();
    Encoder_task();
    Control_task(2U);
    Motor_Process();
    Chassis_OdomProcess();
    /* 一拍超期也实际休眠1ms；优先级提高后不能连续追赶并挤占输出任务。 */
    if ((TickType_t)(xTaskGetTickCount()-wake) >= pdMS_TO_TICKS(1U))
      wake = xTaskGetTickCount();
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1U));
  }
}
