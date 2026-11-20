#include "path_trajectory.h"
#include <math.h>

/* 几何与运动均使用物理单位；不依赖HAL、RTOS或电机驱动。 */
static float path_clamp(float x, float low, float high)
{
  return fmaxf(low, fminf(x, high));
}

static int path_line(PathTrajectory *path, float x, float y, float ex, float ey)
{
  PathSegment *seg;
  float length = hypotf(ex - x, ey - y);
  if (length < 0.001f) return 1;
  if (path->count >= PATH_MAX_SEGMENTS) return 0;
  seg = &path->segment[path->count++];
  seg->s = path->length; seg->length = length;
  seg->x = x; seg->y = y;
  seg->tx = (ex - x) / length; seg->ty = (ey - y) / length;
  seg->cx = seg->cy = seg->curvature = 0.0f;
  path->length += length;
  return 1;
}

int Path_Build(PathTrajectory *path, const PathPoint *points, uint16_t count,
               float deviation_mm, float approach_mm)
{
  PathPoint p[PATH_MAX_POINTS];
  float cut[PATH_MAX_POINTS] = {0};
  uint16_t i, n = 0U;
  float x, y, last_arc_end = 0.0f;
  if (path == 0 || points == 0 || count < 2U || count > PATH_MAX_POINTS ||
      !isfinite(deviation_mm) || deviation_mm <= 0.0f ||
      !isfinite(approach_mm) || approach_mm < 0.0f) return 0;
  path->count = 0U; path->length = path->align_s = 0.0f;
  path->deviation = deviation_mm;
  for (i = 0U; i < count; i++)
  {
    float ax, ay, bx, by;
    if (!isfinite(points[i].x) || !isfinite(points[i].y)) return 0;
    if (n && hypotf(points[i].x - p[n - 1U].x, points[i].y - p[n - 1U].y) < 0.001f)
      continue;
    /* 合并同向共线点，使末端200mm约束作用于整条进站直线。 */
    if (n >= 2U)
    {
      ax = p[n - 1U].x - p[n - 2U].x; ay = p[n - 1U].y - p[n - 2U].y;
      bx = points[i].x - p[n - 1U].x; by = points[i].y - p[n - 1U].y;
      if (fabsf(ax * by - ay * bx) <= 0.00001f * hypotf(ax, ay) * hypotf(bx, by) &&
          ax * bx + ay * by > 0.0f) n--;
    }
    p[n++] = points[i];
  }
  if (n < 2U) return 0;
  for (i = 1U; i + 1U < n; i++)
  {
    float ax = p[i].x - p[i - 1U].x, ay = p[i].y - p[i - 1U].y;
    float bx = p[i + 1U].x - p[i].x, by = p[i + 1U].y - p[i].y;
    float la = hypotf(ax, ay), lb = hypotf(bx, by);
    float dot = path_clamp((ax * bx + ay * by) / (la * lb), -1.0f, 1.0f);
    float cross = (ax * by - ay * bx) / (la * lb);
    float half = atan2f(fabsf(cross), dot) * 0.5f;
    float cut_limit;
    /* 精确折返无法用相切圆角连续通过；不把几何折返当作180度车头转向。 */
    if (fabsf(cross) < 0.000001f)
    {
      if (dot < 0.0f) return 0;
      continue;
    }
    /* 等价于R=偏离/(1-cos半角)再乘tan半角，避免近共线时相减丢精度。 */
    cut_limit = deviation_mm * (1.0f + cosf(half)) / (sinf(half) * cosf(half));
    cut[i] = fminf(cut_limit, 0.45f * fminf(la, lb));
    if (i + 2U == n)
    {
      if (lb <= approach_mm) return 0;
      cut[i] = fminf(cut[i], lb - approach_mm);
    }
  }
  x = p[0].x; y = p[0].y;
  for (i = 1U; i + 1U < n; i++)
  {
    float ax = p[i].x - p[i - 1U].x, ay = p[i].y - p[i - 1U].y;
    float bx = p[i + 1U].x - p[i].x, by = p[i + 1U].y - p[i].y;
    float la = hypotf(ax, ay), lb = hypotf(bx, by);
    float ux = ax / la, uy = ay / la, vx = bx / lb, vy = by / lb;
    float angle, radius, sign, ex, ey;
    PathSegment *seg;
    if (cut[i] == 0.0f)
    {
      if (!path_line(path, x, y, p[i].x, p[i].y)) return 0;
      x = p[i].x; y = p[i].y;
      continue;
    }
    angle = atan2f(fabsf(ux * vy - uy * vx), ux * vx + uy * vy);
    radius = cut[i] * cosf(angle * 0.5f) / sinf(angle * 0.5f);
    sign = ux * vy - uy * vx > 0.0f ? 1.0f : -1.0f;
    ex = p[i].x - ux * cut[i]; ey = p[i].y - uy * cut[i];
    if (!path_line(path, x, y, ex, ey) || path->count >= PATH_MAX_SEGMENTS) return 0;
    seg = &path->segment[path->count++];
    seg->s = path->length; seg->length = radius * angle;
    seg->x = ex; seg->y = ey; seg->tx = ux; seg->ty = uy;
    seg->cx = ex - sign * uy * radius; seg->cy = ey + sign * ux * radius;
    seg->curvature = sign / radius;
    path->length += seg->length;
    last_arc_end = path->length;
    x = p[i].x + vx * cut[i]; y = p[i].y + vy * cut[i];
  }
  if (!path_line(path, x, y, p[n - 1U].x, p[n - 1U].y)) return 0;
  /* 最后圆角出口即进站直线起点；纯直线路线在末端保留段前完成航向。 */
  path->align_s = last_arc_end > 0.0f ? last_arc_end : fmaxf(0.0f, path->length - approach_mm);
  return path->count != 0U;
}

static int path_heading_limits(PathTrajectory *path)
{
  float span = path->align_s - path->heading_begin_s;
  float d1, d2;
  path->road_limit = path->cruise;
  path->turn_acceleration = path->acceleration;
  if (fabsf(path->heading_delta) < 0.0001f) return 1;
  if (span < 0.001f) return 0;
  d1 = 1.875f * fabsf(path->heading_delta) / span;
  d2 = 5.77350269f * fabsf(path->heading_delta) / (span * span);
  /* 两项分别分配一半角加速度：θ''·v²及θ'·a。只限制转向窗口内的平移。 */
  path->road_limit = fminf(path->cruise, fminf(path->omega_max / d1, sqrtf(path->alpha_max * 0.5f / d2)));
  path->turn_acceleration = path->alpha_max * 0.5f / d1;
  return 1;
}

float Path_CurveValue(uint8_t curve, float progress)
{
  if (progress <= 0.0f) return 0.0f;
  if (progress >= 1.0f) return 1.0f;
  if (curve == 1U) return progress;
  if (curve == 2U)
    return progress * progress * progress *
           (10.0f + progress * (-15.0f + 6.0f * progress));
  return (1.0f - cosf(3.14159265f * progress)) * 0.5f;
}

int Path_SetMotion(PathTrajectory *path, float start_heading, float final_heading,
                   const PathMotionConfig *motion)
{
  float slope;
  if (!isfinite(start_heading) || !isfinite(final_heading) || motion->cruise <= 0.0f ||
      motion->omega_max <= 0.0f || motion->alpha_max <= 0.0f) return 0;
  path->heading_start = start_heading;
  path->heading_delta = fmodf(final_heading - start_heading, 360.0f);
  if (path->heading_delta > 180.0f) path->heading_delta -= 360.0f;
  if (path->heading_delta <= -180.0f) path->heading_delta += 360.0f;
  path->cruise = motion->cruise; path->road_limit = motion->cruise;
  path->heading_begin_s = 0.0f;
  path->omega_max = motion->omega_max; path->alpha_max = motion->alpha_max;
  path->rise_s = motion->rise_s; path->brake_mm = motion->brake_mm;
  path->end_speed = motion->end_speed; path->speed_curve = motion->speed_curve;
  /* 由所选曲线的最大斜率换算加速度，预检与执行使用同一套起步参数。 */
  slope = path->speed_curve == 1U ? 1.0f : path->speed_curve == 2U ? 1.875f : 1.57079633f;
  path->acceleration = path->cruise * slope / path->rise_s;
  path->braking = path->cruise * path->cruise / (2.0f * path->brake_mm);
  return path_heading_limits(path);
}

void Path_Sample(const PathTrajectory *path, float s, PathSample *sample)
{
  const PathSegment *seg;
  uint16_t i = 0U;
  float ds, angle, c, sn, u, q, d1, d2;
  float span = path->align_s - path->heading_begin_s;
  s = path_clamp(s, 0.0f, path->length);
  while (i + 1U < path->count && s >= path->segment[i + 1U].s) i++;
  seg = &path->segment[i]; ds = path_clamp(s - seg->s, 0.0f, seg->length);
  angle = ds * seg->curvature; c = cosf(angle); sn = sinf(angle);
  sample->tx = seg->tx * c - seg->ty * sn;
  sample->ty = seg->tx * sn + seg->ty * c;
  sample->curvature = seg->curvature;
  if (seg->curvature == 0.0f)
  {
    sample->x = seg->x + ds * seg->tx; sample->y = seg->y + ds * seg->ty;
  }
  else
  {
    sample->x = seg->cx + (seg->x - seg->cx) * c - (seg->y - seg->cy) * sn;
    sample->y = seg->cy + (seg->x - seg->cx) * sn + (seg->y - seg->cy) * c;
  }
  u = span > 0.0f ? path_clamp((s - path->heading_begin_s) / span, 0.0f, 1.0f) : 1.0f;
  q = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
  d1 = 30.0f * u * u * (1.0f - u) * (1.0f - u);
  d2 = 60.0f * u * (1.0f - u) * (1.0f - 2.0f * u);
  sample->heading = path->heading_start + path->heading_delta * q;
  sample->heading_d1 = span > 0.0f ? path->heading_delta * d1 / span : 0.0f;
  sample->heading_d2 = span > 0.0f ? path->heading_delta * d2 / (span * span) : 0.0f;
}

/* 分段积分2a·ds：窗口外用原减速度，窗口内同时服从转向加速度预算。 */
static float path_brake_energy(const PathTrajectory *path, float start, float end)
{
  float distance = fmaxf(0.0f, end - start), overlap = 0.0f;
  if (fabsf(path->heading_delta) >= 0.0001f)
    overlap = fmaxf(0.0f, fminf(end, path->align_s) - fmaxf(start, path->heading_begin_s));
  return 2.0f * (path->braking * (distance - overlap) +
                 fminf(path->braking, path->turn_acceleration) * overlap);
}

float Path_SpeedLimit(const PathTrajectory *path, float s)
{
  uint16_t i;
  float limit = path->cruise;
  if (fabsf(path->heading_delta) >= 0.0001f && s < path->align_s)
    limit = fminf(limit, sqrtf(path->road_limit * path->road_limit +
        path_brake_energy(path, s, fmaxf(s, path->heading_begin_s))));
  /* 从当前位置前瞻全部后段；到弯前就开始减速，只有整段终点速度为零。 */
  for (i = 0U; i < path->count; i++)
  {
    const PathSegment *seg = &path->segment[i];
    float cap, distance;
    if (seg->s + seg->length < s || seg->curvature == 0.0f) continue;
    cap = sqrtf(800.0f / fabsf(seg->curvature)); // 向心加速度初值800mm/s²，现场在此调参。
    distance = fmaxf(0.0f, seg->s - s);
    limit = fminf(limit, sqrtf(cap * cap + path_brake_energy(path, s, s + distance)));
  }
  /* 共用减速曲线只提供上限；参考轨迹仍由能量包络在整段终点降为零。 */
  limit = fminf(limit, path->end_speed + (path->cruise - path->end_speed) *
      Path_CurveValue(path->speed_curve, (path->length - s) / path->brake_mm));
  return fminf(limit, sqrtf(path_brake_energy(path, s, path->length)));
}

void Path_Advance(const PathTrajectory *path, PathProgress *progress, float dt)
{
  float old = progress->speed, cap, acceleration = path->acceleration;
  progress->elapsed += dt;
  cap = path->cruise * Path_CurveValue(path->speed_curve, progress->elapsed / path->rise_s);
  cap = fminf(cap, Path_SpeedLimit(path, progress->s));
  if (progress->s >= path->heading_begin_s && progress->s < path->align_s &&
      fabsf(path->heading_delta) >= 0.0001f)
    acceleration = fminf(acceleration, path->turn_acceleration);
  progress->speed = fminf(cap, old + acceleration * dt);
  progress->s = fminf(path->length, progress->s + (old + progress->speed) * 0.5f * dt);
  if (progress->s >= path->length) progress->speed = 0.0f;
}

int Path_ValidateClearance(const PathTrajectory *path, const PathClearance *clearance)
{
  PathSample sample;
  uint32_t i, steps = (uint32_t)ceilf(path->length / 5.0f);
  float span = path->align_s - path->heading_begin_s;
  float step, padding, d1 = span > 0.0f ? 1.875f * fabsf(path->heading_delta) / span : 0.0f;
  /* 无场地约束时，轨迹规划只检查几何和速度限制。 */
  if (clearance == 0 || clearance->pose_clear == 0) return 1;
  if (!steps) return 0;
  step = path->length / (float)steps;
  /* 任意两采样点之间，角点移动≤ds + 半对角线·|dθ/ds|max·ds。
   * 用半采样间距的移动上界再次膨胀，覆盖连续扫掠而非仅检查离散点。 */
  padding = step * 0.5f * (1.0f + clearance->body_radius_mm * d1 * 0.01745329252f);
  for (i = 0U; i <= steps; i++)
  {
    Path_Sample(path, step * (float)i, &sample);
    if (!clearance->pose_clear(sample.x, sample.y, sample.heading,
                               clearance->margin_mm + padding)) return 0;
  }
  return 1;
}

int Path_PlanClear(PathTrajectory *path, const PathPoint *points, uint16_t count,
                   float approach_mm, float start_heading, float final_heading,
                   const PathMotionConfig *motion, const PathClearance *clearance)
{
  const float deviations[5] = {50.0f, 40.0f, 30.0f, 20.0f, 10.0f};
  const float lead[4] = {50.0f, 100.0f, 0.0f, 200.0f};
  uint16_t d, k;
  int i;
  float entry, end;
  for (d = 0U; d < 5U; d++)
  {
    if (!Path_Build(path, points, count, deviations[d], approach_mm) ||
        !Path_SetMotion(path, start_heading, final_heading, motion)) continue;
    entry = path->align_s;
    if (Path_ValidateClearance(path, clearance)) return 1;
    if (fabsf(path->heading_delta) < 0.0001f) continue;
    /* 优先尝试末段圆角的转向窗口，再检查整条轨迹的扫掠。 */
    for (i = (int)path->count - 1; i >= 0; i--)
    {
      const PathSegment *seg = &path->segment[i];
      if (seg->curvature == 0.0f) continue;
      for (k = 0U; k < 4U; k++)
      {
        path->heading_begin_s = fmaxf(0.0f, seg->s - lead[k]);
        path->align_s = fminf(entry, seg->s + seg->length);
        if (path_heading_limits(path) && Path_ValidateClearance(path, clearance)) return 1;
      }
    }
    /* 直线段也可尝试缩短转向窗口。 */
    for (end = entry; end > 0.0f; end -= 100.0f)
    {
      path->align_s = end; path->heading_begin_s = fmaxf(0.0f, end - 300.0f);
      if (path_heading_limits(path) && Path_ValidateClearance(path, clearance)) return 1;
    }
  }
  return 0;
}
