#include "encoder.h"
#include "tim.h"
#include <string.h>

const float encoder_counts_per_turn = 4096.0f;
encoder_para_t encoder_para[2];
static uint16_t last_counter[2];
static int32_t pending_count[2];
static uint32_t last_sample_ms;

void Encoder_init(void)
{
	// 右轮 PE9/PE11 使用 TIM1，左轮 PD12/PD13 使用 TIM4；A/B 顺序不换。
	if(HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	if(HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	Encoder_Reset();
}

void Encoder_Reset(void)
{
	// 清除旧脉冲和计速缓存，重采基线，不重启编码器定时器。
	memset(encoder_para, 0, sizeof(encoder_para));
	pending_count[0] = pending_count[1] = 0;
	last_counter[0] = (uint16_t)__HAL_TIM_GetCounter(&htim1);
	last_counter[1] = (uint16_t)__HAL_TIM_GetCounter(&htim4);
	last_sample_ms = HAL_GetTick();
}

void Encoder_task(void)
{
	TIM_HandleTypeDef *timer[2] = {&htim1, &htim4};
	uint32_t now_ms = HAL_GetTick();
	uint32_t elapsed_ms = now_ms - last_sample_ms;

	for(int i=0; i<2; i++)
	{
		uint16_t current = (uint16_t)__HAL_TIM_GetCounter(timer[i]);
		int32_t delta = (int32_t)current - (int32_t)last_counter[i];
		last_counter[i] = current;

		// 16 位计数器回绕后，仍按本次采样的正反转方向取差值。
		if(delta > 32767)
			delta -= 65536;
		else if(delta < -32768)
			delta += 65536;

		encoder_para[i].count = delta;
		encoder_para[i].dirt = (delta > 0) - (delta < 0);
		encoder_para[i].total_count += delta;
		pending_count[i] += delta;

		// 同一毫秒内的脉冲留到下次计速，避免时间差为零。
		if(elapsed_ms > 0)
		{
			encoder_para[i].rate = (float)pending_count[i] * 2.0f * PI * 1000.0f /
				(encoder_counts_per_turn * (float)elapsed_ms);
			pending_count[i] = 0;
		}
	}

	if(elapsed_ms > 0)
		last_sample_ms = now_ms;
}

const encoder_para_t *get_encoder_para_point(uint8_t num)
{
	if(num >= 2U) return NULL;  // 防止调试时传入非法轮号。
	return &encoder_para[num];
}
