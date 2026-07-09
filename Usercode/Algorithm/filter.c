#include "filter.h"
#include <string.h>

/* ================================================================
 * 一阶低通滤波器
 *   y(k) = alpha * x(k) + (1-alpha) * y(k-1)
 * ================================================================ */

void LPF_Init(LowPassFilter_TypeDef *lpf, float alpha)
{
    lpf->alpha = alpha;
    lpf->prev_out = 0.0f;
}

float LPF_Update(LowPassFilter_TypeDef *lpf, float input)
{
    lpf->prev_out = lpf->alpha * input + (1.0f - lpf->alpha) * lpf->prev_out;
    return lpf->prev_out;
}

void LPF_Reset(LowPassFilter_TypeDef *lpf, float init_val)
{
    lpf->prev_out = init_val;
}

void LPF_SetAlpha(LowPassFilter_TypeDef *lpf, float alpha)
{
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    lpf->alpha = alpha;
}

/* ================================================================
 * 滑动平均滤波器
 * ================================================================ */

void MAF_Init(MovingAvgFilter_TypeDef *maf)
{
    memset(maf->buffer, 0, sizeof(maf->buffer));
    maf->sum = 0.0f;
    maf->index = 0;
    maf->count = 0;
}

float MAF_Update(MovingAvgFilter_TypeDef *maf, float input)
{
    /* 减去将被覆盖的旧值 */
    maf->sum -= maf->buffer[maf->index];
    /* 写入新值 */
    maf->buffer[maf->index] = input;
    maf->sum += input;
    /* 推进索引 */
    maf->index = (maf->index + 1) & (MAF_BUFFER_SIZE - 1);
    /* 更新计数 */
    if (maf->count < MAF_BUFFER_SIZE)
        maf->count++;

    return maf->sum / (float)maf->count;
}

void MAF_Reset(MovingAvgFilter_TypeDef *maf)
{
    MAF_Init(maf);
}

/* ================================================================
 * 限幅滤波器
 * ================================================================ */

float LimitFilter(float input, float min_val, float max_val)
{
    if (input > max_val) return max_val;
    if (input < min_val) return min_val;
    return input;
}

/* ================================================================
 * 死区滤波器
 * ================================================================ */

float DeadbandFilter(float input, float deadband)
{
    if (input > -deadband && input < deadband)
        return 0.0f;
    return input;
}

/* ================================================================
 * 变化率限制滤波器
 * ================================================================ */

void RLF_Init(RateLimitFilter_TypeDef *rlf, float rate_limit)
{
    rlf->rate_limit = rate_limit;
    rlf->prev_out = 0.0f;
}

float RLF_Update(RateLimitFilter_TypeDef *rlf, float input, float Ts)
{
    float max_step = rlf->rate_limit * Ts;
    float delta = input - rlf->prev_out;

    if (delta > max_step)
        rlf->prev_out += max_step;
    else if (delta < -max_step)
        rlf->prev_out -= max_step;
    else
        rlf->prev_out = input;

    return rlf->prev_out;
}
