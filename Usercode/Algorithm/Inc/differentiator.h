#ifndef __DIFFERENTIATOR_H__
#define __DIFFERENTIATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "filter.h"

/* 微分器结构体
 *
 *   输出: y(k) = (x(k) - x(k-1)) / Ts
 *   可选一阶低通滤波以抑制高频噪声
 */
typedef struct
{
    /* ---- 参数 ---- */
    float Ts;               /* 采样周期 (s) */
    float tau;              /* 低通滤波时间常数 (s)，0 = 不过滤 */

    /* ---- 状态 ---- */
    float prev_input;       /* 上一次输入 x(k-1) */

    /* ---- 滤波器 ---- */
    LowPassFilter_TypeDef lpf;  /* 微分输出一阶低通滤波器 */

    /* ---- 输出 ---- */
    float out;              /* 当前微分输出 */
    float raw_out;          /* 滤波前原始微分值 */
} Differentiator_TypeDef;

/**
 * @brief  微分器初始化
 * @param  diff  微分器结构体指针
 * @param  Ts    采样周期 (s)
 * @param  tau   低通滤波时间常数 (s)，0 = 不过滤
 */
void Diff_Init(Differentiator_TypeDef *diff, float Ts, float tau);

/**
 * @brief  微分器计算
 * @param  diff   微分器结构体指针
 * @param  input  当前输入值
 * @return 微分值 dx/dt
 */
float Diff_Update(Differentiator_TypeDef *diff, float input);

/**
 * @brief  重置微分器状态
 */
void Diff_Reset(Differentiator_TypeDef *diff);

/**
 * @brief  在线修改采样周期
 */
void Diff_SetTs(Differentiator_TypeDef *diff, float Ts);

#ifdef __cplusplus
}
#endif

#endif /* __DIFFERENTIATOR_H__ */
