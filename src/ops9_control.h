#ifndef _OPS9_CONTROL_H
#define _OPS9_CONTROL_H

#include "imu.h"
#include "encoder.h"

typedef struct
{
	float len;
	float angle;
	float rate;
	float speed;
} wheel_para_t;

typedef struct
{
	float x;
	float y;
	float z;
} chassis_geometry_t;

typedef struct
{
	const IMU_para_t *control_imu_point;
	const encoder_para_t *control_encoder_point[2];
	wheel_para_t wheel_para[2];
	chassis_geometry_t chassis_pos;
	chassis_geometry_t chassis_line;
} control_para_t;

void Control_init(uint16_t Task_time_out);
void Control_RequestZero(void);  // 其它任务只提交请求，由通信任务清积分和重采基线。
void Control_SetMapXY(float x_m, float y_m);  // 仅通信任务回写地图XY，单位米，航向保持不变。
void Control_ResetPosition(void);  // 仅通信任务调用。
void Control_task(uint8_t dirt);
uint8_t Control_StartupZeroDone(void);
const control_para_t *get_control_para_point(void);

#endif
