#include "impedance.h"
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
 * @brief  阻抗控制器初始化
 */
void Impedance_Init(Impedance_TypeDef *imp, float Kp, float Kd,float kf,
                    float out_max, float out_min)
{
    memset(imp, 0, sizeof(Impedance_TypeDef));

    imp->Kp      = Kp;
    imp->Kd      = Kd;
    imp->kf     = kf;
    imp->out_max = out_max;
    imp->out_min = out_min;
}

/**
 * @brief  阻抗控制器计算
 *
 *  控制律: torque = Kp*(pos - pos_) + t_ff + Kd*(vel - vel_)
 *
 *  弹簧项: Kp*(pos - pos_)  — 虚拟弹簧，将实际位置拉向期望位置
 *  阻尼项: Kd*(vel - vel_)  — 虚拟阻尼，抑制速度差异
 *  前馈项: t_ff             — 直接前馈力矩
 *
 * @param  pos   实际位置反馈 (° 或 rad)
 * @param  pos_  虚拟期望位置 (° 或 rad)
 * @param  vel   实际速度反馈 (°/s 或 rad/s)
 * @param  vel_  虚拟期望速度 (°/s 或 rad/s)
 * @param  t_ff  前馈力矩 (Nm)
 * @return 输出扭矩 (Nm)
 */
float Impedance_Update(Impedance_TypeDef *imp,
                       float pos, float pos_,
                       float vel, float vel_,
                       float t_ff)
{
    /* === 弹簧项 === */
    imp->spring_out = imp->Kp * (pos - pos_);

    /* === 阻尼项 === */
    imp->damp_out   = imp->Kd * (vel - vel_);
    /* === 前馈项 === */
    imp->kf_out     = imp->kf * t_ff;
    /* === 合成输出 === */
    imp->out = imp->spring_out + imp->kf_out + imp->damp_out;
    imp->out = clamp(imp->out, imp->out_min, imp->out_max);

    return imp->out;
}

/**
 * @brief  重置阻抗控制器状态
 */
void Impedance_Reset(Impedance_TypeDef *imp)
{
    imp->out        = 0.0f;
    imp->spring_out = 0.0f;
    imp->damp_out   = 0.0f;
}

/**
 * @brief  设置输出限幅
 */
void Impedance_SetOutLimit(Impedance_TypeDef *imp, float out_max, float out_min)
{
    imp->out_max = out_max;
    imp->out_min = out_min;
}

/**
 * @brief  在线修改阻抗参数
 */
void Impedance_SetParam(Impedance_TypeDef *imp, float Kp, float Kd)
{
    imp->Kp = Kp;
    imp->Kd = Kd;
}
