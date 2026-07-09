#ifndef __FILTER_H__
#define __FILTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ================================================================
 * 一阶低通滤波器
 * ================================================================ */
typedef struct
{
    float alpha;            /* 滤波系数 (0~1)，越大响应越快 */
    float prev_out;         /* 上一次输出 */
} LowPassFilter_TypeDef;

void LPF_Init(LowPassFilter_TypeDef *lpf, float alpha);
float LPF_Update(LowPassFilter_TypeDef *lpf, float input);
void LPF_Reset(LowPassFilter_TypeDef *lpf, float init_val);
void LPF_SetAlpha(LowPassFilter_TypeDef *lpf, float alpha);

/* ================================================================
 * 滑动平均滤波器
 * ================================================================ */
#define MAF_BUFFER_SIZE 16   /* 缓冲区大小，需为 2 的幂次 */

typedef struct
{
    float buffer[MAF_BUFFER_SIZE];
    float sum;              /* 当前累加和 */
    uint16_t index;         /* 当前写入位置 */
    uint16_t count;         /* 已填数据个数 */
} MovingAvgFilter_TypeDef;

void MAF_Init(MovingAvgFilter_TypeDef *maf);
float MAF_Update(MovingAvgFilter_TypeDef *maf, float input);
void MAF_Reset(MovingAvgFilter_TypeDef *maf);

/* ================================================================
 * 限幅滤波器 (直接限制输出范围)
 * ================================================================ */
float LimitFilter(float input, float min_val, float max_val);

/* ================================================================
 * 死区滤波器
 * ================================================================ */
float DeadbandFilter(float input, float deadband);

/* ================================================================
 * 变化率限制滤波器
 * ================================================================ */
typedef struct
{
    float rate_limit;       /* 变化率上限 (>0) */
    float prev_out;         /* 上一次输出 */
} RateLimitFilter_TypeDef;

void RLF_Init(RateLimitFilter_TypeDef *rlf, float rate_limit);
float RLF_Update(RateLimitFilter_TypeDef *rlf, float input, float Ts);

#ifdef __cplusplus
}
#endif

#endif /* __FILTER_H__ */
