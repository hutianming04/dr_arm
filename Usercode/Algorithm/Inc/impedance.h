#ifndef __IMPEDANCE_H__
#define __IMPEDANCE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* 阻抗控制结构体
 *
 *  控制律: torque = Kp*(pos - pos_) + t_ff + Kd*(vel - vel_)
 *
 *    pos   : 实际位置反馈
 *    pos_  : 虚拟期望位置
 *    vel   : 实际速度反馈
 *    vel_  : 虚拟期望速度
 *    t_ff  : 前馈力矩
 */
typedef struct
{
    /* ---- 参数 ---- */
    float Kp;               /* 刚度系数 (stiffness)，N·m/° 或 N·m/rad */
    float Kd;               /* 阻尼系数 (damping)，N·m/(°/s) 或 N·m/(rad/s) */

    /* ---- 限幅 ---- */
    float out_max;          /* 输出上限 */
    float out_min;          /* 输出下限 */

    /* ---- 输出 ---- */
    float out;              /* 当前输出扭矩 */
    float spring_out;       /* 弹簧项 = Kp*(pos - pos_) */
    float damp_out;         /* 阻尼项 = Kd*(vel - vel_) */
} Impedance_TypeDef;

/**
 * @brief  阻抗控制器初始化
 * @param  imp      阻抗控制结构体指针
 * @param  Kp       刚度系数
 * @param  Kd       阻尼系数
 * @param  out_max  输出上限
 * @param  out_min  输出下限
 */
void Impedance_Init(Impedance_TypeDef *imp, float Kp, float Kd,
                    float out_max, float out_min);

/**
 * @brief  阻抗控制器计算
 * @param  imp   阻抗控制结构体指针
 * @param  pos   实际位置反馈
 * @param  pos_  虚拟期望位置
 * @param  vel   实际速度反馈
 * @param  vel_  虚拟期望速度
 * @param  t_ff  前馈力矩
 * @return 输出扭矩
 */
float Impedance_Update(Impedance_TypeDef *imp,
                       float pos, float pos_,
                       float vel, float vel_,
                       float t_ff);

/**
 * @brief  重置阻抗控制器状态
 */
void Impedance_Reset(Impedance_TypeDef *imp);

/**
 * @brief  设置输出限幅
 */
void Impedance_SetOutLimit(Impedance_TypeDef *imp, float out_max, float out_min);

/**
 * @brief  在线修改阻抗参数
 */
void Impedance_SetParam(Impedance_TypeDef *imp, float Kp, float Kd);

#ifdef __cplusplus
}
#endif

#endif /* __IMPEDANCE_H__ */
