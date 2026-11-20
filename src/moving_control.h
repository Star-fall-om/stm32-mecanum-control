#ifndef __MOVING_CONTROL_H
#define __MOVING_CONTROL_H

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "path_trajectory.h"

/* 平移和末段转向的调参入口。 */
typedef struct
{
  float mKpx, mKpy;                // 世界坐标X/Y位置增益。
  float fine_ratio;                // 运动到点时的位置纠偏倍率。
  float XYVmax, XYVmin;            // 内部平移上限和位置环最小补偿。
  float speed_ratio;               // 由内部平移量换算为轮速。
  uint8_t speed_curve;             // 1梯形、2 S形、3正弦。
  float rise_ms, brake_mm;         // 起步时间ms、终点减速距离mm。
  float end_speed;                 // 末端基础限速。
  float turn_start_distance_mm;    // MoveTurnTo开始转向的剩余距离。
} ChassisMotionConfig;
extern const ChassisMotionConfig chassis_motion_config;
extern const PathClearance chassis_path_clearance;

typedef enum { StraightForward, TURN, MOVE_TURN_TO, MOVE_PATH } MoveCmd_t;
typedef struct { int32_t x, y; float target_angle; } Point_t;
typedef struct
{
  MoveCmd_t cmd;
  Point_t point;
  float turn_delta_deg; // TURN的相对转角，度，逆时针为正。
  uint32_t duration_ms;
} MoveTaskParams_t;

/* 保留平移缩放、位置门控和取消代次，不在队列里存放参数指针。 */
typedef struct
{
  int32_t x, y, yaw;
  float ratio;
  uint32_t target_seq, generation;
  uint8_t acceleration, ramp;
} move_cmd_t;

enum { MOVE_COMPLETE_EVENT = 1, MOVE_ERROR_EVENT = 3 };
enum { MOVE_NOTIFY_STRAIGHT = 1, MOVE_NOTIFY_TURN = 3, MOVE_NOTIFY_ERROR };
typedef struct { uint32_t cmd; MoveCmd_t action; } general_notify_t;

extern QueueHandle_t Move_Queue, Move_CMD_Queue, MoveComplete_Queue;
extern TaskHandle_t MovingControlTask_Handle, StraightTask_Handle, TurnTask_Handle;
enum { CHASSIS_MODE_HOLD = 4 };
/* 0=空闲，1=平移动作，2=定时转向，4=位姿保持。 */
extern volatile uint8_t chassis_mode;
extern volatile uint32_t target_seq;

void MovingControl_Init(void);
/* 启动归零并定位就绪后调用；只接受空闲状态，锁定当前地图位姿。 */
BaseType_t Chassis_HoldCurrent(void);
/* 仅供5ms输出任务调用：HAL_OK给出本拍保持速度，HAL_BUSY表示本拍不输出。 */
HAL_StatusTypeDef MovingControl_HoldTick(move_cmd_t *command);
/* 仅供里程计重设临界区调用；平移保持目标并撤销旧代次输出，不改变物理目标。 */
BaseType_t MovingControl_ShiftMapXY(float dx, float dy);
/* 运动接口由同一任务串行调用。绝对坐标用mm，航向用度，逆时针为正。 */
BaseType_t MoveTo(int32_t x_mm, int32_t y_mm, float heading_deg);
/* 进入末段距离后才启动转向。到位窗口和MoveTo相同。 */
BaseType_t MoveTurnTo(int32_t x_mm, int32_t y_mm, float heading_deg);
/* 跟踪折线点生成的连续轨迹。首点是当前位置，approach_mm是末端预留直线长度。 */
BaseType_t MovePath(const PathPoint *points, uint16_t count, float heading_deg, float approach_mm);
/* 用同一静态工作区预检路径，不提交动作。 */
BaseType_t MovePathCanPlan(const PathPoint *points, uint16_t count, float start_heading,
                          float heading_deg, float approach_mm);
/* 定时原地转向并保持起点XY。返回时切换到最终位姿保持。 */
BaseType_t Turn(float delta_deg, uint32_t duration_ms);

void moving_control_task(void *argument);
void control_task(void *argument);
void chassis_comm_task(void *argument);

#endif
