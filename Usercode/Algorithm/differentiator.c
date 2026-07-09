#include "differentiator.h"
#include <string.h>

/**
 * @brief  微分器初始化
 *
 *  滤波器 alpha = Ts / (Ts + tau)
 *  当 tau=0 时 alpha=1，滤波输出 = 原始值（不过滤）
 */
void Diff_Init(Differentiator_TypeDef *diff, float Ts, float tau)
{
    memset(diff, 0, sizeof(Differentiator_TypeDef));

    diff->Ts  = Ts;
    diff->tau = tau;

    if (tau > 1e-9f)
        LPF_Init(&diff->lpf, Ts / (Ts + tau));
    else
        LPF_Init(&diff->lpf, 1.0f);  /* 不过滤时 alpha=1 */
}

/**
 * @brief  微分器计算
 *
 *   先滤波:  x_f(k) = LPF(x(k))
 *   再微分:  y(k)  = (x_f(k) - x_f(k-1)) / Ts
 *
 *   先滤后微，避免高频噪声被差分放大。
 *
 * @param  input  当前输入值 x(k)
 * @return 微分值 dx/dt (滤波后求导)
 */
float Diff_Update(Differentiator_TypeDef *diff, float input)
{
    /* 先对输入低通滤波 */
    float filtered = LPF_Update(&diff->lpf, input);

    /* 对滤波后的信号求微分 */
    diff->raw_out = filtered;                              /* 滤波后输入, 调试用 */
    diff->out     = (filtered - diff->prev_input) / diff->Ts;
    diff->prev_input = filtered;

    return diff->out;
}

/**
 * @brief  重置微分器状态
 */
void Diff_Reset(Differentiator_TypeDef *diff)
{
    diff->prev_input = 0.0f;
    diff->out        = 0.0f;
    diff->raw_out    = 0.0f;
    LPF_Reset(&diff->lpf, 0.0f);
}

/**
 * @brief  在线修改采样周期
 */
void Diff_SetTs(Differentiator_TypeDef *diff, float Ts)
{
    diff->Ts = Ts;

    /* 采样周期变化时更新滤波器 alpha */
    if (diff->tau > 1e-9f)
        LPF_SetAlpha(&diff->lpf, Ts / (Ts + diff->tau));
    else
        LPF_SetAlpha(&diff->lpf, 1.0f);
}
