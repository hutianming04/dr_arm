#include "feedforward.h"
#include "filter.h"
#include <string.h>

/**
 * @brief  限幅函数
 */
static inline float ff_clamp(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
 * @brief  前馈控制器初始化
 */
void FeedForward_Init(FeedForward_TypeDef *ff, float kv, float ka,
                      float Ts, float tau, float out_max, float out_min)
{
    memset(ff, 0, sizeof(FeedForward_TypeDef));

    ff->kv = kv;
    ff->ka = ka;
    ff->Ts = Ts;
    ff->tau = tau;
    ff->out_max = out_max;
    ff->out_min = out_min;

    /* 初始化滤波器 */
    if (tau > 1e-9f)
        LPF_Init(&ff->lpf, Ts / (Ts + tau));
    else
        LPF_Init(&ff->lpf, 1.0f);  /* 不过滤时 alpha=1 */
}

/**
 * @brief  前馈计算
 *
 *  流程:
 *    1. 对目标值做一阶低通滤波
 *    2. 计算滤波后目标值的一阶导 (速度)
 *    3. 计算一阶导的一阶导 (加速度)
 *    4. 速度前馈 = kv * velocity
 *    5. 加速度前馈 = ka * acceleration
 *    6. 输出 = 速度前馈 + 加速度前馈，并限幅
 */
float FeedForward_Update(FeedForward_TypeDef *ff, float target)
{
    /* === 一阶低通滤波 === */
    float filt_target = LPF_Update(&ff->lpf, target);

    /* === 一阶导 (速度) === */
    float velocity = (filt_target - ff->lpf.prev_out) / ff->Ts;

    /* === 二阶导 (加速度) === */
    float acceleration = (velocity - ff->prev_deriv) / ff->Ts;

    /* === 前馈计算 === */
    ff->vff_out = ff->kv * velocity;
    ff->aff_out = ff->ka * acceleration;

    ff->out = ff->vff_out + ff->aff_out;
    ff->out = ff_clamp(ff->out, ff->out_min, ff->out_max);

    /* === 更新状态 === */
    ff->prev_target = target;
    ff->prev_deriv = velocity;

    return ff->out;
}

/**
 * @brief  重置前馈状态
 */
void FeedForward_Reset(FeedForward_TypeDef *ff)
{
    ff->prev_target = 0.0f;
    LPF_Reset(&ff->lpf, 0.0f);
    ff->prev_deriv = 0.0f;
    ff->out = 0.0f;
    ff->vff_out = 0.0f;
    ff->aff_out = 0.0f;
}

/**
 * @brief  在线修改前馈系数
 */
void FeedForward_SetParam(FeedForward_TypeDef *ff, float kv, float ka)
{
    ff->kv = kv;
    ff->ka = ka;

    /* 参数变化时更新滤波器 alpha */
    if (ff->tau > 1e-9f)
        LPF_SetAlpha(&ff->lpf, ff->Ts / (ff->Ts + ff->tau));
    else
        LPF_SetAlpha(&ff->lpf, 1.0f);
}
