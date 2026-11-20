#ifndef PATH_TRAJECTORY_H
#define PATH_TRAJECTORY_H

#include <stdint.h>

/* 轨迹使用固定工作区，最多接收24个折线点。 */
#define PATH_MAX_POINTS 24u
#define PATH_MAX_SEGMENTS (2u * PATH_MAX_POINTS)

typedef struct { float x, y; } PathPoint;
typedef struct
{
  float s, length, x, y, tx, ty, cx, cy, curvature;
} PathSegment;
typedef struct
{
  float cruise, omega_max, alpha_max; // 物理速度mm/s、角速度及角加速度。
  float rise_s, brake_mm, end_speed; // 共用起步秒数、减速距离及末端速度包络。
  uint8_t speed_curve;              // 1梯形、2 S形、3正弦。
} PathMotionConfig;

typedef struct
{
  PathSegment segment[PATH_MAX_SEGMENTS];
  uint16_t count;
  float length, align_s;
  float heading_start, heading_delta;
  float cruise, road_limit, acceleration, braking;
  float heading_begin_s, turn_acceleration, omega_max, alpha_max, deviation;
  float rise_s, brake_mm, end_speed;
  uint8_t speed_curve;
} PathTrajectory;
typedef struct
{
  float x, y, tx, ty, curvature;
  float heading, heading_d1, heading_d2;
} PathSample;
typedef struct { float s, speed, elapsed; } PathProgress;

/* 纯几何：坐标/弧长为mm，最大偏离指到原折线的最短距离。 */
int Path_Build(PathTrajectory *path, const PathPoint *points, uint16_t count,
               float deviation_mm, float approach_mm);
/* 航向为度；角速度/角加速度为度/s、度/s²，平移速度为mm/s。 */
int Path_SetMotion(PathTrajectory *path, float start_heading, float final_heading,
                   const PathMotionConfig *motion);
/* 两类平移控制共用同一条归一化曲线，progress限制在0到1。 */
float Path_CurveValue(uint8_t curve, float progress);
void Path_Sample(const PathTrajectory *path, float s, PathSample *sample);
float Path_SpeedLimit(const PathTrajectory *path, float s);
void Path_Advance(const PathTrajectory *path, PathProgress *progress, float dt);
/* 宿主按车身外廓和额外扫掠余量检查位姿。坐标与间隙用mm，航向用度。 */
typedef int (*PathPoseCheck)(float x, float y, float heading, float margin);
typedef struct
{
  PathPoseCheck pose_clear;
  float body_radius_mm;  // 车身中心至最远角点的距离。
  float margin_mm;       // 外部障碍与车身之间的基础间隙。
} PathClearance;
int Path_ValidateClearance(const PathTrajectory *path, const PathClearance *clearance);
/* 在圆角上安排局部转向，必要时缩小圆角；不合格时只返回失败，不发布运动。 */
int Path_PlanClear(PathTrajectory *path, const PathPoint *points, uint16_t count,
                   float approach_mm, float start_heading, float final_heading,
                   const PathMotionConfig *motion, const PathClearance *clearance);

#endif
