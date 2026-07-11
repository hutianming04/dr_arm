/**
 * @file    ref_traj.c
 * @brief   4-DOF 参考轨迹实现 — 高斯函数叠加, 直接输出目标角度
 *
 * 参数来源: 参考轨迹.docx (MATLAB 脚本)
 *
 * 轨迹公式:
 *   y(t) = Σ a_i * exp(-((t - b_i) / c_i)^2)
 *   关节1: 1 个高斯分量
 *   关节2: 8 个高斯分量
 *   关节3: 6 个高斯分量
 *   关节4: 2 个高斯分量
 */

#include "ref_traj.h"
#include <math.h>

/* ================================================================
 * 高斯参数结构体
 * ================================================================ */
typedef struct {
    float a;   /* 幅值 (度)   */
    float b;   /* 中心 (秒)   */
    float c;   /* 宽度 (秒)   */
} GaussParam;

/* ================================================================
 * 各关节高斯参数表
 * ================================================================ */

/* 关节 1: 1 个高斯分量 */
static const GaussParam j1[] = {
    {53.94f, 9.043f, 0.885f},
};
#define J1_N (sizeof(j1) / sizeof(j1[0]))

/* 关节 2: 8 个高斯分量 */
static const GaussParam j2[] = {
    {28.21f,  12.41f,  0.4207f},
    {50.88f,  13.02f,  0.5569f},
    {-5.252f, 12.44f,  0.2859f},
    {42.17f,  11.88f,  0.6581f},
    { 9.433f, 13.74f,  0.3785f},
    {10.95f,   4.288f, 1.826f},
    {10.97f,   2.188f, 0.7499f},
    {13.45f,   8.603f, 2.866f},
};
#define J2_N (sizeof(j2) / sizeof(j2[0]))

/* 关节 3: 6 个高斯分量 */
static const GaussParam j3[] = {
    {55.06f, 11.82f,  0.8624f},
    {64.46f, 10.67f,  1.22f},
    {54.87f,  2.578f, 0.9299f},
    {38.98f,  6.085f, 1.199f},
    {71.06f,  8.229f, 1.989f},
    {66.43f,  4.244f, 1.417f},
};
#define J3_N (sizeof(j3) / sizeof(j3[0]))

/* 关节 4: 2 个高斯分量 */
static const GaussParam j4[] = {
    {42.83f, 4.695f, 0.3673f},
    {39.80f, 2.86f,  0.3082f},
};
#define J4_N (sizeof(j4) / sizeof(j4[0]))

/* ================================================================
 * 内部: 高斯叠加计算
 * ================================================================ */

/**
 * @brief  对一组高斯分量求和
 * @param  p      高斯参数数组
 * @param  n      分量个数
 * @param  t      时间 (秒)
 * @return        Σ a_i * exp(-((t-b_i)/c_i)^2)
 */
static float gauss_sum(const GaussParam *p, int n, float t)
{
    float y = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = (t - p[i].b) / p[i].c;
        y += p[i].a * expf(-x * x);
    }
    return y;
}

/* ================================================================
 * 公开接口
 * ================================================================ */

/**
 * @brief  计算单个关节在时刻 t 的参考角度
 */
float ref_traj_get_angle(int joint, float t)
{
    /* 越界保护 */
    if (t < 0.0f || t > 15.0f) return 0.0f;

    switch (joint) {
    case 1:  return gauss_sum(j1, J1_N, t);
    case 2:  return gauss_sum(j2, J2_N, t);
    case 3:  return gauss_sum(j3, J3_N, t);
    case 4:  return gauss_sum(j4, J4_N, t);
    default: return 0.0f;
    }
}

/**
 * @brief  一次性计算全部 4 个关节的参考角度
 */
void ref_traj_compute_all(float t, float angles[4])
{
    angles[0] = gauss_sum(j1, J1_N, t);
    angles[1] = gauss_sum(j2, J2_N, t);
    angles[2] = gauss_sum(j3, J3_N, t);
    angles[3] = gauss_sum(j4, J4_N, t);
}
