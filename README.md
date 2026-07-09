# dr_arm — 电机位置环 PID 控制

STM32F1 + 大然盘式无刷伺服驱动器 (CAN 总线)。

---

## 控制架构

```
 target(°)                           torque(Nm)         CAN       ┌──────────────┐
   │                                     │              ────→     │  电机驱动器    │
   ▼                                     │               ←───     │  内置力矩环    │
┌──────────┐    feedback     ┌──────────┐│                        │  电流闭环      │
│ 正弦发生器├──→  setpoint ──→│ 位置环PID ├──→ set_torque_cmd ────→│               │
└──────────┘    (全局target)  └──────────┘                        └──────────────┘
                                   ▲
                                   │ angle_infinite
                              ┌────┴────────┐
                              │ 多圈角度解算  │
                              │ (跨圈检测)    │
                              └────┬────────┘
                                   │ raw_angle
                              motor_state[0][0]
```

- **外层** (STM32, 1kHz)：位置环 PID，输出力矩指令 `set_torque_cmd`（非阻塞 CAN 发送）
- **内层** (驱动器, 硬件)：力矩/电流闭环
- **反馈**：PID 使用解绕后的 `angle_infinite`（无范围限制），避免跨圈跳变

---

## 文件结构

```
Core/Src/main.c                 ← 主循环 + PID 初始化 + 目标生成
Usercode/
  Algorithm/
    pid.c / pid.h               ← 位置式 PID（微分滤波 + 抗积分饱和）
    feedforward.c / feedforward.h ← 速度/加速度前馈（备用）
    filter.c / filter.h         ← 一阶低通、滑动平均、死区等
  Bsp/
    DrMotor.c / DrMotor.h       ← CAN 电机驱动库 + 电机数据结构体 + 多圈解算
    bspcan.c / bspcan.h         ← CAN 滤波器 + 发送封装
  Star/
    ALLinit.c / ALLinit.h       ← 全局变量定义 (globe_time_ms, motor_update_flag)
    star.c / star.h             ← ISR: CAN 接收回调 + 定时器回调
```

---

## 关键设计决策

### 1. 变量定义与初始化位置

| 位置 | 变量 | 说明 |
|------|------|------|
| `main.c` USER CODE PV | `target`, `motor_pid` | 应用程序私有全局变量，带初始化值 |
| `DrMotor.c` 文件作用域 | `motor`, `motor_state`, `READ_FLAG` 等 | 驱动库全局状态，由库管理 |
| `ALLinit.c` 文件作用域 | `globe_time_ms`, `motor_update_flag` | ISR 共享变量，必须 `volatile` |

### 2. volatile 规则

所有被 ISR 写入、main 循环读取的变量必须加 `volatile`：

```c
// ALLinit.c — ISR (TIM) 写入
volatile uint32_t globe_time_ms = 0;
volatile uint8_t  motor_update_flag = 0;

// DrMotor.c — ISR (CAN RX) 写入
volatile uint8_t  rx_buffer[8];
volatile uint16_t can_id;
volatile int8_t   READ_FLAG;
```

### 3. 非阻塞 CAN 发送

高频控制循环 (1kHz) **禁止**调用含 `get_state()` → `receive_data()` 的函数（阻塞等待 CAN 回复，超时 500ms）。

| 函数 | 阻塞？ | 用途 |
|------|--------|------|
| `set_torque_cmd()` | 否 | 1kHz 循环发送力矩指令 |
| `get_state()` | 是 | 100ms 独立查询更新 `motor_state` |
| `set_torque()` | 是 | 低频直接力矩控制（调试用） |

`get_state()` 内部已加 `NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn)` 保护临界区，防止与 CAN RX ISR 竞争。

### 4. PID 库

使用 `Usercode/Algorithm/pid.c`，不手写 PID：

```c
PID_TypeDef motor_pid;                          // 全局实例

PID_Init(&motor_pid,
    0.0f, 0.0f, 0.0f,     // Kp, Ki, Kd (初始为零)
    0.001f,                // Ts  = 1ms (1kHz)
    0.0f,                  // tau = 0 → 微分不滤波
    5.0f, -5.0f);          // 输出限幅 ±5 Nm

float torque_cmd = PID_Update(&motor_pid, target, feedback);
```

PID_Update 内部已包含：
- 比例项 `Kp * e(k)`
- 积分项 `Ki * Ts * Σe` + 限幅（抗积分饱和）
- 微分项 `Kd/Ts * (e(k)-e(k-1))` + 一阶低通滤波
- 输出限幅 + clamping 回退

### 5. 多圈角度解算

电机驱动器返回的角度在 `[0, 360)` 范围内环绕。`motor_update_infinite_angle()` 通过检测相邻采样的跳变方向来累积圈数：

```
delta = raw_angle - pos_last

if   delta > +180°  →  laps--   (从 360 绕到 0, 正向跨圈)
elif delta < -180°  →  laps++   (从 0 绕到 360, 反向跨圈)

angle_infinite = raw_angle + laps × 360°
```

要求采样频率 ≥ 2× 最高转速（1kHz 足够覆盖 30000 rpm）。

### 6. 电机数据结构体

```c
typedef struct {
    float pos;              // 当前角度 (°)
    float pos_last;         // 上一周期角度，跨圈检测用
    float vel;              // 速度 (r/min)
    float tor;              // 扭矩 (Nm)
    float current;          // q轴电流 (A)
    float angle_infinite;   // 解绕累积角度
    int16_t laps;           // 圈数计数
    float aim;              // 目标角度
    float error;            // 跟随误差
} Dr_MOTOR_DATA_Typdef;
```

已删除的冗余字段：`Angle_last`/`Angle_now`/`Speed_last`/`Speed_now`（与 float 版本重复）、`kp`/`kd`（属于 PID_TypeDef）、`Stuck_Time`/`Stuck_Flag`（堵转检测非核心）、`ONLINE_JUDGE_TIME`。

---

## 控制时序

```
TIM1 (1kHz) ─→ globe_time_ms++           (时间基准)
TIM2 (1kHz) ─→ motor_update_flag = 1     (控制触发)

main loop:
  if motor_update_flag:
    ├─ motor_update_infinite_angle()     ← 跨圈解算
    ├─ PID_Update()                       ← 位置环
    └─ set_torque_cmd()                   ← 非阻塞 CAN 发送

  if globe_time_ms - last >= 100ms:
    └─ get_state(1)                       ← 独立状态查询
```
