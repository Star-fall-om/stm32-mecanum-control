#ifndef _ENCODER_H
#define _ENCODER_H

#include "main.h"

#ifndef PI
#define PI 3.1415926535897932384626433832795f
#endif

/* 沿用 OPS9 的四倍频计数换算，右轮为 0、左轮为 1。 */
typedef struct
{
	float rate;
	int8_t dirt;
	int32_t count;
	int32_t total_count;
} encoder_para_t;

extern const float encoder_counts_per_turn;
void Encoder_init(void);
void Encoder_Reset(void);  // 通信任务重采计数基线，不改硬件计数器。
void Encoder_task(void);
const encoder_para_t *get_encoder_para_point(uint8_t num);

#endif
