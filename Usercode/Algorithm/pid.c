#include "pid.h"
#include "filter.h"
#include <string.h>

/**
 * @brief  限幅函数
 */
static inline float clamp(float val, float min_val, float max_val)
{
    if (val > max_val) return max_val;
    if (val < min_val) return min_val;
    return val;
}

/**
 * @brief  PID 初始化
 */
void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
              float Ts, float tau_sp, float tau_d, float out_max, float out_min)
{
    memset(pid, 0, sizeof(PID_TypeDef));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->tau_sp = tau_sp;
    pid->tau_d = tau_d;
    pid->out_max = out_max;
    pid->out_min = out_min;

    /* 积分限幅默认与输出限幅一致 */
    pid->int_max = out_max;
    pid->int_min = out_min;

    /* setpoint 滤波器: tau_sp=0 则 alpha=1 (不滤波) */
    if (tau_sp > 1e-9f)
        LPF_Init(&pid->sp_filter, Ts / (Ts + tau_sp));
    else
        LPF_Init(&pid->sp_filter, 1.0f);

    /* 微分误差滤波器: tau_d=0 则 alpha=1 (不滤波) */
    if (tau_d > 1e-9f)
        LPF_Init(&pid->d_filter, Ts / (Ts + tau_d));
    else
        LPF_Init(&pid->d_filter, 1.0f);
}

/**
 * @brief  PID 计算（位置式）
 *
 *  结构:
 *    setpoint ─→ [sp_filter] ─→ sp_filt
 *    error = sp_filt - feedback
 *    P = Kp * error
 *    I = Ki * Ts * Σerror → clamp → anti-windup
 *    D = Kd / Ts * (err_filt - err_filt_prev), 其中 err_filt = LPF(error)
 *    out = P + I + D → clamp
 */
float PID_Update(PID_TypeDef *pid, float setpoint, float feedback)
{
    /* === setpoint 低通滤波 === */
    float sp_filt = LPF_Update(&pid->sp_filter, setpoint);
    float error = sp_filt - feedback;

    /* === 比例项 === */
    pid->P_out = pid->Kp * error;

    /* === 积分项 === */
    pid->integral += pid->Ki * pid->Ts * error;
    pid->integral = clamp(pid->integral, pid->int_min, pid->int_max);
    pid->I_out = pid->integral;

    /* === 微分项: 先对 error 低通滤波, 再计算差分 === */
    float err_filt_prev = pid->d_filter.prev_out;  /* 上一拍的滤波误差 */
    float err_filt       = LPF_Update(&pid->d_filter, error);
    float D_raw          = pid->Kd / pid->Ts * (err_filt - err_filt_prev);
    pid->D_out = D_raw;

    /* === 合成输出 === */
    pid->out = pid->P_out + pid->I_out + pid->D_out;
    pid->out = clamp(pid->out, pid->out_min, pid->out_max);

    /* 抗积分饱和: 输出被限幅时，回退本轮积分增量 */
    if (pid->out == pid->out_max || pid->out == pid->out_min)
    {
        pid->integral -= pid->Ki * pid->Ts * error;
        pid->integral = clamp(pid->integral, pid->int_min, pid->int_max);
        pid->I_out = pid->integral;
    }

    pid->prev_error = error;
    return pid->out;
}

/**
 * @brief  重置 PID 状态
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->integral    = 0.0f;
    pid->prev_error  = 0.0f;
    LPF_Reset(&pid->sp_filter, 0.0f);
    LPF_Reset(&pid->d_filter, 0.0f);
    pid->out  = 0.0f;
    pid->P_out = 0.0f;
    pid->I_out = 0.0f;
    pid->D_out = 0.0f;
}

/**
 * @brief  设置 PID 输出限幅
 */
void PID_SetOutLimit(PID_TypeDef *pid, float out_max, float out_min)
{
    pid->out_max = out_max;
    pid->out_min = out_min;
}

/**
 * @brief  设置 PID 积分限幅
 */
void PID_SetIntLimit(PID_TypeDef *pid, float int_max, float int_min)
{
    pid->int_max = int_max;
    pid->int_min = int_min;
}

/**
 * @brief  在线修改 PID 参数 (Kp, Ki, Kd)
 */
void PID_SetParam(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    /* 参数变化时更新微分误差滤波器 alpha */
    if (pid->tau_d > 1e-9f)
        LPF_SetAlpha(&pid->d_filter, pid->Ts / (pid->Ts + pid->tau_d));
    else
        LPF_SetAlpha(&pid->d_filter, 1.0f);
}
