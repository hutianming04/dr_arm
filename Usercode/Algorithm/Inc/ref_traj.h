/**
 * @file    ref_traj.h
 * @brief   4-DOF 机械臂参考轨迹 — 高斯函数叠加
 *
 * 轨迹: y(t) = Σ a_i * exp(-((t - b_i) / c_i)^2)
 * t ∈ [0, 15]s
 *
 * 用法:
 *   target[0] = ref_traj_get_angle(1, t);  // 关节1 → 电机2
 *   target[1] = ref_traj_get_angle(2, t);  // 关节2 → 电机3
 *
 *   或一次性取全部:
 *   float angles[4];
 *   ref_traj_compute_all(t, angles);
 */

#ifndef REF_TRAJ_H
#define REF_TRAJ_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  计算单个关节在时刻 t 的参考角度
 * @param  joint  关节编号 (1~4)
 * @param  t      时间 (秒), 范围 0~15
 * @return        角度值 (度), t 越界返回 0
 */
float ref_traj_get_angle(int joint, float t);

/**
 * @brief  一次性计算全部 4 个关节的参考角度
 * @param  t       时间 (秒), 范围 0~15
 * @param  angles  输出数组, angles[0..3] 对应关节 1..4
 */
void ref_traj_compute_all(float t, float angles[4]);

#ifdef __cplusplus
}
#endif

#endif /* REF_TRAJ_H */
