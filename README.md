# dr_arm — 电机串级 PID 控制

STM32F103C8T6 + 大然盘式无刷伺服驱动器 (CAN 总线)。

---

## 控制架构

```
target (°)                     pid_pos out (r/min)     pid_vel out (Nm)   CAN
  │ 正弦波                         │                       │              ───→
  ▼                               ▼                       ▼              ←───
┌──────────┐    feedback    ┌──────────────┐  feedback  ┌──────────────┐
│  pid_pos  ├──────────────→│   pid_vel     ├───────────→│ set_torque()  ├──→ 驱动器
│ 外环-位置  │ motor_state   │ 内环-速度     │ motor_state│              │
│ Kp=20     │   [0][0](°)   │ Kp=1, Kd=0.02│  [0][1]    │  +reply_state│
│ out±3600  │               │ out±3600(Nm)  │  (r/min)  │  (更新状态)   │
└──────────┘               └──────────────┘           └──────┬───────┘
                                                              │
                                                       motor_state[0]
                                                      [angle,speed,torque,traj_done,error]
```

- **外环 (位置)**: setpoint 不滤波, 微分误差滤波 α=0.8
- **内环 (速度)**: setpoint 滤波 α=0.8, 微分不滤波
- **目标**: ±180° 正弦波 0.2Hz

---

## 文件结构

```
Core/Src/main.c                 ← 主循环 + 串级 PID + 目标生成
Core/Src/can.c                  ← CAN1 硬件初始化 (HAL)
Core/Src/stm32f1xx_it.c         ← 中断向量表 → HAL 分发
Core/Src/tim.c                  ← TIM1/TIM2 1kHz 定时器

Usercode/
  Algorithm/
    pid.c / pid.h               ← 位置式 PID（setpoint 滤波 + 微分误差滤波 + 抗积分饱和）
    filter.c / filter.h         ← 一阶低通、滑动平均、死区、变化率限制
    feedforward.c/h             ← 前馈（备用）
    impedance.c/h               ← 阻抗控制（备用）
    differentiator.c/h          ← 微分器（备用）
  Bsp/
    DrMotor.c / DrMotor.h       ← 大然电机 CAN 驱动库 (v2.1)
    bspcan.c / bspcan.h         ← CAN 滤波器配置 + HAL 发送封装
    VOFA.c / VOFA.h             ← VOFA+ JustFloat 遥测 (USART1)
  Star/
    ALLinit.c / ALLinit.h       ← 用户初始化 + 全局 volatile 变量
    star.c / star.h             ← CAN RX ISR 回调 + TIM 回调
```

---

## 控制时序 (1kHz)

```
TIM1 ─→ globe_time_ms++            (毫秒计数)
TIM2 ─→ motor_update_flag = 1      (控制触发)

main loop (每 1ms):
  target = 180° * sin(2π × 0.2Hz × t)
  vel_cmd    = PID_Update(&pid_pos, target, motor_state[0][0])     ← 外环
  torque_cmd = PID_Update(&pid_vel, vel_cmd, motor_state[0][1])    ← 内环
  set_torque(1, torque_cmd, 0, 1)                                 ← CAN 发送 + 更新 motor_state
  VOFA_justfloat(...)                                              ← 遥测
```

---

## PID 库设计

### 结构

```
setpoint ─→ [sp_filter] ─→ error = sp_filt - feedback
               │
               ├── P = Kp × error
               ├── I = Ki × Ts × Σerror → clamp → anti-windup (back-calculation)
               └── D = Kd/Ts × Δ(err_filt), err_filt = LPF(error)
```

### 关键特征

| 特征 | 说明 |
|------|------|
| 双滤波器 | `sp_filter` (setpoint) + `d_filter` (微分误差), 各自独立 tau |
| 滤波顺序 | 微分项先对 error 低通, 再计算差分, 用 `d_filter.prev_out` 取上一拍值 |
| 抗积分饱和 | 输出被限幅时回退本轮积分增量 (back-calculation) |
| 默认不滤波 | tau = 0 → alpha = 1.0 (直通) |
| 在线调参 | `PID_SetParam(Kp, Ki, Kd)` 不重置积分状态 |

### 当前参数

| 参数 | pid_pos (外环) | pid_vel (内环) |
|------|---------------|---------------|
| Kp | 20 | 1 |
| Ki | 0 | 0 |
| Kd | 0 | 0.02 |
| tau_sp | 0 (不滤波) | 0.00025 (α=0.8) |
| tau_d | 0.00025 (α=0.8) | 0 (不滤波) |
| out 限幅 | ±3600 r/min | ±3600 Nm |

---

## CAN 通信

### 硬件

- CAN1: PA11 (RX) / PA12 (TX)
- 时钟: APB1 = 36MHz, Prescaler = 9, BS1 = 13TQ, BS2 = 2TQ → **250kbps**
- 滤波器: 掩码模式，接收所有标准帧 → FIFO0

### 发送

所有控制函数 (`set_torque`, `set_angle`, `set_speed`, `get_state` 等) 通过 `send_command()` → `Can_Send_Msg()` → `HAL_CAN_AddTxMessage()` 发送标准帧。

CAN ID 编码: `(id_num << 5) | cmd`

### 接收

```
CAN 总线消息 → CAN 控制器 FIFO0
  → USB_LP_CAN1_RX0_IRQHandler (stm32f1xx_it.c)
  → HAL_CAN_IRQHandler
  → HAL_CAN_RxFifo0MsgPendingCallback (star.c)
  → rx_buffer + can_id + READ_FLAG = 1
```

`reply_state()` / `get_state()` / `read_property()` 内部调用 `receive_data()` 轮询 `READ_FLAG` 等待回复。

### 电机状态

```c
float motor_state[MOTOR_NUM][5];
// [id-1][0] = angle (°)
// [id-1][1] = speed (r/min)
// [id-1][2] = torque (Nm)
// [id-1][3] = traj_done
// [id-1][4] = axis_error
```

由 `reply_state()` 或 `get_state()` 更新。`set_torque()` 在发送指令后都会调用 `reply_state()`。

---

## volatile 变量

| 变量 | 定义位置 | 写入者 | 读取者 |
|------|---------|--------|--------|
| `READ_FLAG` | DrMotor.c:38 | CAN RX ISR | receive_data() 轮询 |
| `globe_time_ms` | ALLinit.c | TIM1 ISR | main loop |
| `motor_update_flag` | ALLinit.c | TIM2 ISR | main loop (读取后清零) |

---

## 遥测

VOFA+ JustFloat 协议, USART1 中断发送 (921600bps), 不阻塞控制循环。

输出通道: `[target, angle, speed, torque_cmd, vel_cmd, 0, 0, 0, 0, 0]`

---

## 调试

- **调试器**: CMSIS-DAP (DAPLink) + Ozone
- **GDB Server**: OpenOCD → localhost:3333
- **任务**: VS Code → 运行任务 → "Ozone 调试 (全流程)"
- **烧录**: VS Code → 运行任务 → "烧录: OpenOCD"
