#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "filter.h"

/* 前馈控制结构体 */
typedef struct
{
    /* ---- 参数 ---- */
    float kv;               /* 速度前馈系数 */
    float ka;               /* 加速度前馈系数 */
    float Ts;               /* 采样周期 (s) */
    float tau;              /* 一阶低通滤波时间常数 (s) */

    /* ---- 限幅 ---- */
    float out_max;          /* 输出上限 */
    float out_min;          /* 输出下限 */

    /* ---- 滤波器 ---- */
    LowPassFilter_TypeDef lpf;  /* 目标值一阶低通滤波器 */

    /* ---- 状态 ---- */
    float prev_target;      /* 上一次目标值 (滤波前) */
    float prev_deriv;       /* 上一次一阶导 */

    /* ---- 输出 ---- */
    float out;              /* 前馈输出 */
    float vff_out;          /* 速度前馈项 */
    float aff_out;          /* 加速度前馈项 */
} FeedForward_TypeDef;

/**
 * @brief  前馈控制器初始化
 * @param  ff      前馈结构体指针
 * @param  kv      速度前馈系数
 * @param  ka      加速度前馈系数
 * @param  Ts      采样周期 (s)
 * @param  tau     低通滤波时间常数 (s)
 * @param  out_max 输出上限
 * @param  out_min 输出下限
 */
void FeedForward_Init(FeedForward_TypeDef *ff, float kv, float ka,
                      float Ts, float tau, float out_max, float out_min);

/**
 * @brief  前馈计算
 * @param  ff      前馈结构体指针
 * @param  target  目标值（位置/速度等）
 * @return 前馈输出
 */
float FeedForward_Update(FeedForward_TypeDef *ff, float target);

/**
 * @brief  重置前馈状态
 */
void FeedForward_Reset(FeedForward_TypeDef *ff);

/**
 * @brief  在线修改前馈系数
 */
void FeedForward_SetParam(FeedForward_TypeDef *ff, float kv, float ka);

#ifdef __cplusplus
}
#endif

#endif /* __FEEDFORWARD_H__ */
