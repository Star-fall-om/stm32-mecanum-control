#include "chassis.h"
#include "imu.h"
#include "moving_control.h"

/* XY使用双定位轮和四轮电机反馈融合，航向来自HWT101。 */
const ChassisFeedbackSource chassis_feedback_source = CHASSIS_FEEDBACK_MIXED;

/* 保留原车的平移控制参数，换车时从低速重新调试。 */
const ChassisMotionConfig chassis_motion_config = {
  .mKpx = 2.3f,
  .mKpy = 2.3f,
  .fine_ratio = 2.0f,
  .XYVmax = 900.0f,
  .XYVmin = 5.0f,
  .speed_ratio = 0.238f,
  .speed_curve = 3U,
  .rise_ms = 1200.0f,
  .brake_mm = 650.0f,
  .end_speed = 60.0f,
  .turn_start_distance_mm = 300.0f
};

/* 默认在开阔场地规划。需要约束边界时，由接入工程提供净空检查。 */
const PathClearance chassis_path_clearance = { .pose_clear = NULL };

/* 接好PE4高有效按键后可改为1，启用HWT101硬件零偏校准。 */
const uint8_t chassis_imu_bias_button_enabled = 0U;
