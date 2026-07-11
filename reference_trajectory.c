/**
 * reference_trajectory.c
 * 4-DOF 机械臂参考轨迹 — C 语言可执行版本
 *
 * 原作: MATLAB 脚本 (参考轨迹.docx)
 * 轨迹: y(t) = Σ a_i * exp(-((t - b_i) / c_i)^2)
 * t ∈ [0, 15]s, 原始步长 0.001s
 *
 * 编译: gcc -o ref_traj reference_trajectory.c -lm
 * 运行: ./ref_traj [模式]
 *   模式:
 *     csv      - 输出 CSV 到 stdout (默认)
 *     header   - 生成 C 头文件 (可直接嵌入 STM32 代码)
 *     lookup   - 输出稀疏查找表 (100Hz 控制频率, 即 dt=0.01s)
 *     value T  - 计算 t=T 时各关节的角度值
 *
 * 示例:
 *   ./ref_traj csv   > traj.csv
 *   ./ref_traj header > traj_table.h
 *   ./ref_traj value 3.5
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ================================================================
 * 高斯参数表 — 直接来自 MATLAB 脚本
 * ================================================================ */

typedef struct {
    double a, b, c;
} GaussParam;

/* 关节 1: 1 个高斯分量 */
static const GaussParam joint1_params[] = {
    {53.94, 9.043, 0.885},
};
#define JOINT1_COUNT (sizeof(joint1_params) / sizeof(joint1_params[0]))

/* 关节 2: 8 个高斯分量 */
static const GaussParam joint2_params[] = {
    {28.21,  12.41,  0.4207},
    {50.88,  13.02,  0.5569},
    {-5.252, 12.44,  0.2859},
    {42.17,  11.88,  0.6581},
    { 9.433, 13.74,  0.3785},
    {10.95,   4.288, 1.826},
    {10.97,   2.188, 0.7499},
    {13.45,   8.603, 2.866},
};
#define JOINT2_COUNT (sizeof(joint2_params) / sizeof(joint2_params[0]))

/* 关节 3: 6 个高斯分量 */
static const GaussParam joint3_params[] = {
    {55.06, 11.82,  0.8624},
    {64.46, 10.67,  1.22},
    {54.87,  2.578, 0.9299},
    {38.98,  6.085, 1.199},
    {71.06,  8.229, 1.989},
    {66.43,  4.244, 1.417},
};
#define JOINT3_COUNT (sizeof(joint3_params) / sizeof(joint3_params[0]))

/* 关节 4: 2 个高斯分量 */
static const GaussParam joint4_params[] = {
    {42.83, 4.695, 0.3673},
    {39.80, 2.86,  0.3082},
};
#define JOINT4_COUNT (sizeof(joint4_params) / sizeof(joint4_params[0]))

/* ================================================================
 * 轨迹计算
 * ================================================================ */

/**
 * 计算单个关节在时刻 t 的角度值
 * @param params  高斯参数数组
 * @param count   高斯分量个数
 * @param t       时间 (秒)
 * @return        角度值 (度)
 */
static double joint_angle(const GaussParam *params, int count, double t) {
    double y = 0.0;
    for (int i = 0; i < count; i++) {
        double x = (t - params[i].b) / params[i].c;
        y += params[i].a * exp(-x * x);
    }
    return y;
}

/**
 * 计算全部 4 个关节在时刻 t 的角度
 */
static void compute_all_joints(double t, double angle[4]) {
    angle[0] = joint_angle(joint1_params, JOINT1_COUNT, t);
    angle[1] = joint_angle(joint2_params, JOINT2_COUNT, t);
    angle[2] = joint_angle(joint3_params, JOINT3_COUNT, t);
    angle[3] = joint_angle(joint4_params, JOINT4_COUNT, t);
}

/* ================================================================
 * 输出模式
 * ================================================================ */

static void print_usage(const char *prog) {
    fprintf(stderr,
        "用法: %s [模式]\n"
        "  csv       输出 CSV (t, j1, j2, j3, j4), dt=0.001s (默认)\n"
        "  header    生成 C 头文件 (dt=0.01s, 100Hz 控制频率)\n"
        "  lookup    输出稀疏查找表 (dt=0.01s)\n"
        "  value T   计算 t=T 时各关节角度\n"
        "示例:\n"
        "  %s csv > traj.csv\n"
        "  %s header > traj_table.h\n"
        "  %s value 3.5\n",
        prog, prog, prog, prog);
}

static void mode_csv(void) {
    const double T_END = 15.0;
    const double DT    = 0.001;

    printf("t, joint1, joint2, joint3, joint4\n");
    for (double t = 0.0; t <= T_END + DT / 2; t += DT) {
        double angle[4];
        compute_all_joints(t, angle);
        printf("%.3f, %.6f, %.6f, %.6f, %.6f\n",
               t, angle[0], angle[1], angle[2], angle[3]);
    }
}

/**
 * 生成可直接嵌入 STM32 代码的 C 头文件
 * 100Hz 控制频率 → dt = 0.01s → 1501 个采样点
 * 4 关节 × 1501 × 4 字节 ≈ 24 KB，适合 STM32F103C8 的 64KB Flash
 */
static void mode_header(void) {
    const double T_END = 15.0;
    const double DT    = 0.01;  /* 100Hz */
    const int    N     = (int)(T_END / DT) + 1;  /* 1501 点 */

    printf("/* 自动生成 — 4-DOF 参考轨迹查找表 */\n");
    printf("/* dt = %.2fs, N = %d, 总时长 = %.1fs */\n", DT, N, T_END);
    printf("#ifndef REF_TRAJ_TABLE_H\n");
    printf("#define REF_TRAJ_TABLE_H\n\n");
    printf("#include <stdint.h>\n\n");
    printf("#define TRAJ_TABLE_SIZE %d\n", N);
    printf("#define TRAJ_DT %.4ff\n\n", DT);
    printf("/* 关节参考角度 (度), 按 [joint][step] 索引 */\n");
    printf("static const float traj_table[4][TRAJ_TABLE_SIZE] = {\n");

    for (int joint = 0; joint < 4; joint++) {
        printf("    {  /* Joint %d */\n        ", joint + 1);
        int col = 0;
        for (int i = 0; i < N; i++) {
            double t = i * DT;
            double angle[4];
            compute_all_joints(t, angle);
            printf("%.4ff", angle[joint]);
            if (i < N - 1) {
                printf(", ");
                if (++col >= 8) {
                    printf("\n        ");
                    col = 0;
                }
            }
        }
        printf("\n    }");
        if (joint < 3) printf(",");
        printf("\n");
    }
    printf("};\n\n");
    printf("/* 查找函数: 线性插值获取任意时刻的角度 */\n");
    printf("static inline float traj_lookup(int joint, float t) {\n");
    printf("    if (t <= 0.0f) return traj_table[joint][0];\n");
    printf("    float idx_f = t / TRAJ_DT;\n");
    printf("    int i = (int)idx_f;\n");
    printf("    if (i >= TRAJ_TABLE_SIZE - 1) return traj_table[joint][TRAJ_TABLE_SIZE - 1];\n");
    printf("    float frac = idx_f - (float)i;\n");
    printf("    return traj_table[joint][i] * (1.0f - frac) + traj_table[joint][i + 1] * frac;\n");
    printf("}\n\n");
    printf("#endif /* REF_TRAJ_TABLE_H */\n");
}

static void mode_lookup(void) {
    const double T_END = 15.0;
    const double DT    = 0.01;  /* 100Hz */

    printf("# t(s)  j1(deg)  j2(deg)  j3(deg)  j4(deg)\n");
    for (double t = 0.0; t <= T_END + DT / 2; t += DT) {
        double angle[4];
        compute_all_joints(t, angle);
        printf("%6.2f  %8.4f  %8.4f  %8.4f  %8.4f\n",
               t, angle[0], angle[1], angle[2], angle[3]);
    }
}

static void mode_value(double t) {
    double angle[4];
    compute_all_joints(t, angle);
    printf("t = %.3f s\n", t);
    for (int i = 0; i < 4; i++) {
        printf("  Joint %d: %10.6f deg\n", i + 1, angle[i]);
    }
}

/* ================================================================
 * main
 * ================================================================ */

int main(int argc, char *argv[]) {
    const char *mode = (argc > 1) ? argv[1] : "csv";

    if (strcmp(mode, "csv") == 0) {
        mode_csv();
    } else if (strcmp(mode, "header") == 0) {
        mode_header();
    } else if (strcmp(mode, "lookup") == 0) {
        mode_lookup();
    } else if (strcmp(mode, "value") == 0) {
        if (argc < 3) {
            fprintf(stderr, "错误: value 模式需要时间参数\n");
            print_usage(argv[0]);
            return 1;
        }
        double t = atof(argv[2]);
        mode_value(t);
    } else if (strcmp(mode, "-h") == 0 || strcmp(mode, "--help") == 0) {
        print_usage(argv[0]);
    } else {
        fprintf(stderr, "未知模式: %s\n", mode);
        print_usage(argv[0]);
        return 1;
    }

    return 0;
}
