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
              float Ts, float tau, float out_max, float out_min)
{
    memset(pid, 0, sizeof(PID_TypeDef));

    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->Ts = Ts;
    pid->tau = tau;
    pid->out_max = out_max;
    pid->out_min = out_min;

    /* 积分限幅默认与输出限幅一致 */
    pid->int_max = out_max;
    pid->int_min = out_min;

    /* 初始化微分滤波器 */
    if (tau > 1e-9f)
        LPF_Init(&pid->d_filter, Ts / (Ts + tau));
    else
        LPF_Init(&pid->d_filter, 1.0f);  /* 不过滤时 alpha=1 */
}

/**
 * @brief  PID 计算（位置式，带微分滤波和抗积分饱和）
 *
 *  结构:  u = Kp*e(k) + Ki*Ts*sum(e) + Kd/Ts * (e(k)-e(k-1))
 *        微分项经一阶低通滤波: D_filtered = alpha*D_raw + (1-alpha)*D_prev
 *        抗积分饱和: clamping 方式
 */
float PID_Update(PID_TypeDef *pid, float setpoint, float feedback)
{
    float error = setpoint - feedback;

    /* === 比例项 === */
    pid->P_out = pid->Kp * error;

    /* === 积分项 (带限幅 = 抗积分饱和) === */
    pid->integral += pid->Ki * pid->Ts * error;
    pid->integral = clamp(pid->integral, pid->int_min, pid->int_max);
    pid->I_out = pid->integral;

    /* === 微分项 (带一阶低通滤波) === */
    float D_raw = pid->Kd / pid->Ts * (error - pid->prev_error);
    pid->D_out = LPF_Update(&pid->d_filter, D_raw);

    /* === 合成输出 === */
    pid->out = pid->P_out + pid->I_out + pid->D_out;
    pid->out = clamp(pid->out, pid->out_min, pid->out_max);

    /* 抗积分饱和: 输出被限幅时，冻结积分 */
    if (pid->out == pid->out_max || pid->out == pid->out_min)
    {
        /* 回退积分，防止 windup */
        pid->integral -= pid->Ki * pid->Ts * error;
        pid->integral = clamp(pid->integral, pid->int_min, pid->int_max);
        pid->I_out = pid->integral;
    }

    pid->prev_error = error;
    return pid->out*0.002197265625f;
}

/**
 * @brief  重置 PID 状态
 */
void PID_Reset(PID_TypeDef *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    LPF_Reset(&pid->d_filter, 0.0f);
    pid->out = 0.0f;
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
 * @brief  在线修改 PID 参数
 */
void PID_SetParam(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    /* 参数变化时更新微分滤波器 alpha */
    if (pid->tau > 1e-9f)
        LPF_SetAlpha(&pid->d_filter, pid->Ts / (pid->Ts + pid->tau));
    else
        LPF_SetAlpha(&pid->d_filter, 1.0f);
}
