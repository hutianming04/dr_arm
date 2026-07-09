#ifndef __PID_H__
#define __PID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "filter.h"

/* PID 控制结构体 */
typedef struct
{
    /* ---- 参数 ---- */
    float Kp;               /* 比例系数 */
    float Ki;               /* 积分系数 */
    float Kd;               /* 微分系数 */
    float Ts;               /* 采样周期 (s) */
    float tau_sp;           /* setpoint 滤波时间常数 (s)，0 = 不滤波 */
    float tau_d;            /* 微分误差滤波时间常数 (s)，0 = 不滤波 */

    /* ---- 限幅 ---- */
    float out_max;          /* 输出上限 */
    float out_min;          /* 输出下限 */
    float int_max;          /* 积分累加上限 */
    float int_min;          /* 积分累加下限 */

    /* ---- 滤波器 ---- */
    LowPassFilter_TypeDef sp_filter;  /* setpoint 一阶低通滤波器 */
    LowPassFilter_TypeDef d_filter;   /* 微分项误差一阶低通滤波器 */

    /* ---- 状态 ---- */
    float integral;         /* 积分累加值 */
    float prev_error;       /* 上一次误差 */

    /* ---- 输出 ---- */
    float out;              /* 当前 PID 输出 */
    float P_out;            /* 比例项 */
    float I_out;            /* 积分项 */
    float D_out;            /* 微分项 */
} PID_TypeDef;

/**
 * @brief  PID 初始化
 * @param  pid     PID 结构体指针
 * @param  Kp      比例系数
 * @param  Ki      积分系数
 * @param  Kd      微分系数
 * @param  Ts      采样周期 (s)
 * @param  tau_sp  setpoint 滤波时间常数 (s)，0 = 不滤波
 * @param  tau_d   微分误差滤波时间常数 (s)，0 = 不滤波
 * @param  out_max 输出上限
 * @param  out_min 输出下限
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float Ts, float tau_sp, float tau_d, float out_max, float out_min);

/**
 * @brief  PID 计算（位置式）
 * @note   微分项先对 error 做一阶低通滤波，再计算差分
 * @param  pid       PID 结构体指针
 * @param  setpoint  目标值（会经过 sp_filter 低通滤波）
 * @param  feedback  反馈值
 * @return 计算结果
 */
float PID_Update(PID_TypeDef *pid, float setpoint, float feedback);

/**
 * @brief  重置 PID 状态
 */
void PID_Reset(PID_TypeDef *pid);

/**
 * @brief  设置 PID 输出限幅
 */
void PID_SetOutLimit(PID_TypeDef *pid, float out_max, float out_min);

/**
 * @brief  设置 PID 积分限幅
 */
void PID_SetIntLimit(PID_TypeDef *pid, float int_max, float int_min);

/**
 * @brief  在线修改 PID 参数 (Kp, Ki, Kd)
 */
void PID_SetParam(PID_TypeDef *pid, float Kp, float Ki, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
