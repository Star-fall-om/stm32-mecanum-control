#include "ops9_control.h"
#include "app_comm.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>
#include <string.h>

uint8_t ops9_direction = 0;
control_para_t control_para;
static uint8_t startup_zero_done;
static volatile uint8_t zero_pending;
static float last_continuous_yaw_deg;  // 上次积分的连续航向，供安装偏移补偿取角差。

static void feed_back(control_para_t *feed_back_point);
static void control_loop(control_para_t *control_loop_point);

void Control_init(uint16_t Task_time_out)
{
	(void)Task_time_out;  // 保留 OPS9 接口，采样周期仍由现有通信任务决定。
	startup_zero_done = zero_pending = 0U;
	last_continuous_yaw_deg = 0.0f;
	ops9_direction = 2;
	memset(&control_para, 0, sizeof(control_para));
	for(int i=0; i<2; i++)
		control_para.control_encoder_point[i] = get_encoder_para_point(i);
	control_para.control_imu_point = get_imu_para_point();
}

void Control_RequestZero(void)
{
	taskENTER_CRITICAL();
	zero_pending = 1U;
	taskEXIT_CRITICAL();
}

void Control_SetMapXY(float x_m, float y_m)
{
	// 仅通信任务应用地图校准；积分位置与发布值同步，不改航向和定位轮计数。
	taskENTER_CRITICAL();
	control_para.chassis_pos.x = ops9_x_m = x_m;
	control_para.chassis_pos.y = ops9_y_m = y_m;
	taskEXIT_CRITICAL();
}

void Control_ResetPosition(void)
{
	// 同一任务重采编码器、清积分、丢弃惯导积压和半帧，旧字节不能建立新原点。
	Encoder_Reset();
	last_continuous_yaw_deg = 0.0f;  // 与本次 IMU 归零同步，旧航向不参与新原点的补偿。
	memset(control_para.wheel_para, 0, sizeof(control_para.wheel_para));
	memset(&control_para.chassis_line, 0, sizeof(control_para.chassis_line));
	memset(&control_para.chassis_pos, 0, sizeof(control_para.chassis_pos));
	Comm_ResetIMU();
	IMU_zero_angle();  // 下一合法航向帧建立新零点，角速度保持实测值。
	taskENTER_CRITICAL();
	ops9_x_m = control_para.chassis_pos.x;
	ops9_y_m = control_para.chassis_pos.y;
	ops9_yaw_deg = 0.0f;
	ops9_position_time_ms = 0U;
	ops9_position_ready = 0U;
	taskEXIT_CRITICAL();
}

void Control_task(uint8_t dirt)
{
	uint8_t reset, ready;
	(void)dirt;  // 沿用当前 OPS9 固定方向 2，不因调用参数改变轮序。
	taskENTER_CRITICAL();
	reset = zero_pending;
	zero_pending = 0U;
	taskEXIT_CRITICAL();
	if(reset)
	{
		Control_ResetPosition();
		return;
	}

	// 上电首次两类惯导数据齐全后归零一次，之后等待下一合法航向帧。
	ready = control_para.control_imu_point->get_data_flag &&
		control_para.control_imu_point->get_rate_flag;
	if(startup_zero_done == 0U)
	{
		if(ready)
		{
			Control_ResetPosition();
			startup_zero_done = 1U;
		}
		return;
	}
	if(!ready) return;
	feed_back(&control_para);
	control_loop(&control_para);

	// 发布OPS位置和航向，由同一通信任务换算为毫米供底盘位置闭环使用。
	taskENTER_CRITICAL();
	if(!zero_pending)
	{
		ops9_x_m = control_para.chassis_pos.x;
		ops9_y_m = control_para.chassis_pos.y;
		ops9_yaw_deg = control_para.chassis_pos.z;
		ops9_position_time_ms = HAL_GetTick();
		ops9_position_ready = 1U;
	}
	taskEXIT_CRITICAL();
}

static void feed_back(control_para_t *feed_back_point)
{
	float wheel_radius = 0.029f;  // 沿用 OPS9 的 29 mm 定位轮半径。
	for(int i=0; i<2; i++)
	{
		feed_back_point->wheel_para[i].angle = (float)feed_back_point->control_encoder_point[i]->total_count/encoder_counts_per_turn*2*PI;
		feed_back_point->wheel_para[i].len = feed_back_point->wheel_para[i].angle*wheel_radius;
		feed_back_point->wheel_para[i].rate = feed_back_point->control_encoder_point[i]->rate;
		feed_back_point->wheel_para[i].speed = feed_back_point->wheel_para[i].rate*wheel_radius;
	}
	// 直接读取共享地图航向，屏幕与底盘位置闭环不另设零点或改变符号。
	feed_back_point->chassis_pos.z = imu_raw_yaw;
}

static void control_loop(control_para_t *control_loop_point)
{
	float wheel_radius = 0.029f;  // 和原 OPS9 位移换算一致，单位米。
	float wheel_len[2];
	for(int i=0; i<2; i++)
	{
		// 位移直接用本次脉冲数计算，不依赖任务是否恰好为 1 ms。
		wheel_len[i] = (float)control_loop_point->control_encoder_point[i]->count *
			2.0f * PI * wheel_radius / encoder_counts_per_turn;
	}

	switch(ops9_direction)
	{
		case 0:
			control_loop_point->chassis_line.y = wheel_len[0];
			control_loop_point->chassis_line.x = -wheel_len[1];
			break;
		case 1:
			control_loop_point->chassis_line.x = wheel_len[0];
			control_loop_point->chassis_line.y = wheel_len[1];
			break;
		case 2:
			control_loop_point->chassis_line.y = -wheel_len[0];
			control_loop_point->chassis_line.x = wheel_len[1];
			break;
		case 3:
			control_loop_point->chassis_line.x = -wheel_len[0];
			control_loop_point->chassis_line.y = -wheel_len[1];
			break;
	}
	// 保留实测轮序和安装补偿；所有轴向标定都在车身坐标内完成。
	float mount_angle_deg = 180.0f;
	float mount_rad = mount_angle_deg * PI / 180.0f;
	float mount_sin = sin(mount_rad);
	float mount_cos = cos(mount_rad);
	float dx_ops = control_loop_point->chassis_line.x;
	float dy_ops = control_loop_point->chassis_line.y;
	control_loop_point->chassis_line.x = dx_ops * mount_cos - dy_ops * mount_sin;
	// 沿用原来依据推车结果确定的 Y 反号，不改变右轮/左轮和 A/B 极性。
	control_loop_point->chassis_line.y = -(dx_ops * mount_sin + dy_ops * mount_cos);

	const float cos_45 = 0.7071067811865475f;
	// 先按0度推车标定得到车身X右、Y前，镜像不能放到航向投影之后。
	float dx_body = -(control_loop_point->chassis_line.x + control_loop_point->chassis_line.y) * cos_45;
	float dy_body = (control_loop_point->chassis_line.y - control_loop_point->chassis_line.x) * cos_45;
	// 左轮(-37.86240685, 3.96280946)mm/135度，右轮(37.86240685, 3.96280946)mm/45度。
	// 沿用已实测正确的轮序和极性；几何解算的等效测量点在车中心后方33.89959739mm。
	const float offset_m = (37.86240685f - 3.96280946f) * 0.001f;
	float continuous_yaw_deg = control_loop_point->control_imu_point->yaw;
	float delta_yaw_rad = (continuous_yaw_deg - last_continuous_yaw_deg) * PI / 180.0f;
	last_continuous_yaw_deg = continuous_yaw_deg;
	// 逆时针为正；扣除旋转造成的车体X附加位移，再投影到地图，车体Y无需补偿。
	dx_body -= offset_m * delta_yaw_rad;
	float yaw_rad = control_loop_point->chassis_pos.z * PI / 180.0f;
	float sin_angle = sinf(yaw_rad);
	float cos_angle = cosf(yaw_rad);
	// 用当前IMU航向投影到地图；顺时针为负，不再对地图位移反号。
	control_loop_point->chassis_pos.x += dx_body * cos_angle - dy_body * sin_angle;
	control_loop_point->chassis_pos.y += dx_body * sin_angle + dy_body * cos_angle;
}

const control_para_t *get_control_para_point(void)
{
	return &control_para;
}

uint8_t Control_StartupZeroDone(void)
{
	return startup_zero_done;
}
