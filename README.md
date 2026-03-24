# YTM32 无传感器 FOC 电机驱动演示工程

基于 **YTM32** 微控制器的三相永磁同步电机（PMSM）无传感器磁场定向控制（FOC）演示。
采用 **Ortega 非线性磁通观测器 + PLL 锁相环**，实现无霍尔 / 无编码器的电机无传感器驱动。

---

## 目录

- [项目概述](#项目概述)
- [目录结构](#目录结构)
- [系统架构](#系统架构)
- [观测器算法原理](#观测器算法原理)
- [电机适配关键参数](#电机适配关键参数)
- [启动与运行状态机](#启动与运行状态机)
- [构建与调试](#构建与调试)
- [许可证](#许可证)

---

## 项目概述

本工程实现了完整的无传感器 FOC 控制栈，主要模块包括：

| 模块 | 文件 | 说明 |
|------|------|------|
| FOC 核心算法 | `app/motor_foc.c/h` | Clarke/Park 变换、SVPWM、电流环 PI、观测器、PLL |
| 电机控制状态机 | `app/motor_control.c/h` | 启动流程、状态切换、保护逻辑、速度环 |
| 硬件抽象层 | `app/motor_hw_ytm32.c/h` | eTMR(PWM)、ADC、TMU、pTMR 驱动配置 |
| 用户参数配置 | `app/motor_user_config.h` | 电机参数、采样参数、PI增益、保护阈值 |
| 应用入口 | `app/main.c` | 按键处理、运行时指令下发 |

---

## 目录结构

```
├── app/                     # 应用层代码
│   ├── main.c               # 主函数与按键处理
│   ├── motor_control.c/h    # 电机控制状态机
│   ├── motor_foc.c/h        # FOC 核心算法
│   ├── motor_hw_ytm32.c/h   # YTM32 硬件驱动层
│   └── motor_user_config.h  # 用户可调参数（核心配置文件）
├── board/                   # 开发板配置
├── CMSIS/                   # ARM CMSIS 头文件
├── platform/                # 芯片平台驱动
├── rtos/                    # RTOS 支持（可选）
├── docs/                    # 调试说明文档
├── CMakeLists.txt           # CMake 构建脚本
└── README.md                # 本文件
```

---

## 系统架构

```
                          ┌──────────────────────────────────┐
                          │          速度环 (1 kHz)            │
                          │   目标RPM → Speed PI → Iq 指令    │
                          └───────────────┬──────────────────┘
                                          │ Iq目标
                          ┌───────────────▼──────────────────┐
                          │       快速环 / ADC ISR (20 kHz)     │
                          │                                    │
  三相电流 ──> Clarke ──> Park ──> 电流环 PI(d/q) ──> 反Park ──> SVPWM ──> PWM占空比
                                    ▲                                        │
                                    │ 电角度                                  │
                          ┌─────────┴──────────┐                             │
                          │   Ortega 磁通观测器  │                             │
                          │     + PLL 锁相环     │                             │
                          └────────────────────┘                             │
                                                                 eTMR(互补PWM with 死区)
```

**双环结构：**
- **快速环（Fast Loop）**：运行在 PWM 频率（默认 20 kHz），由 `ADC0_IRQHandler` 驱动。包含 Clarke/Park 变换、电流环 PI、观测器更新、反 Park 变换、SVPWM。
- **慢速环（Slow Loop）**：运行在 1 kHz（pTMR 中断），负责速度环 PI、状态机管理、斜坡限速。

---

## 观测器算法原理

### Ortega 非线性磁通观测器

本工程采用 **Ortega 非线性磁通观测器**（Reduced-order Nonlinear Flux Observer），其核心思想是在 αβ 静止坐标系中构建磁通状态方程，通过非线性校正项使估计磁通收敛到实际值。

#### 数学模型

PMSM 在 αβ 坐标系下的电压方程：

```
v_α = Rs · i_α + L · di_α/dt + dψ_α/dt
v_β = Rs · i_β + L · di_β/dt + dψ_β/dt
```

其中 `ψ_α` / `ψ_β` 是转子永磁体在定子产生的磁通分量。

定义观测器状态变量 `x1` / `x2`：

```
x1 = ψ_α + L · i_α    （α 轴总磁链）
x2 = ψ_β + L · i_β    （β 轴总磁链）
```

则观测器微分方程为：

```
dx1/dt = v_α - Rs · i_α + γ · φ_α · e
dx2/dt = v_β - Rs · i_β + γ · φ_β · e
```

其中：
- `φ_α = x1 - L · i_α` （估计的永磁磁通 α 分量）
- `φ_β = x2 - L · i_β` （估计的永磁磁通 β 分量）
- `e = λ² - (φ_α² + φ_β²)` （磁通幅值误差，非线性校正项）
- `γ` = `MOTOR_CFG_OBSERVER_GAIN`（观测器增益）
- `λ` = 已知的永磁磁链幅值

**关键特性：**
- 当估计磁通幅值大于实际值时 `e < 0`，校正项将拉回磁通幅值
- 当估计磁通幅值小于实际值时 `e > 0`，但代码中将 `e > 0` 截断为 0，仅做单侧校正，这增强了低速抗噪能力
- 转子电角度通过 `atan2(φ_β, φ_α)` 从估计磁通中提取

#### 在线磁链自适应

观测器还包含一个磁链幅值在线自适应机制：

```
λ_est += K_comp · dt · (clamp(|φ|, λ_min, λ_max) - λ_est)
```

- `K_comp` = `MOTOR_CFG_LAMBDA_COMP_BW_RAD_S`（补偿带宽，默认 80 rad/s）
- 补偿范围限制在 `[LAMBDA_MIN, LAMBDA_MAX]` 之间
- 使得观测器可以适应温度变化、磁路饱和等引起的磁链偏移

### PLL 锁相环速度估计

从观测器得到瞬时电角度后，通过二阶 PLL 提取平滑的电角度和转速：

```
phase_err = angle_observed - angle_pll          （相位误差）
integral += KI · phase_err · dt                 （积分路径 → 估计转速）
speed_est = integral + KP · phase_err           （比例路径 → 快速跟踪）
angle_pll += speed_est · dt                     （积分更新相位）
```

PLL 参数：
- `MOTOR_CFG_PLL_KP = 400`（比例增益）
- `MOTOR_CFG_PLL_KI = 20000`（积分增益）

PLL 将观测器输出的带噪声角度转化为平滑连续的电角度和转速估计，供闭环控制使用。

---

## 电机适配关键参数

所有可调参数集中在 `app/motor_user_config.h`。适配新电机时需重点关注以下参数：

### 1. 电机电气参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_POLE_PAIRS` | 4 | 极对数。影响机械转速与电气转速换算 |
| `MOTOR_CFG_RS_OHM` | 0.050 Ω | 定子相电阻（线电阻的一半）。影响观测器和电流环模型 |
| `MOTOR_CFG_LS_H` | 0.00045158 H | 相电感。影响电流环 PI 增益自动计算和观测器 |
| `MOTOR_CFG_LD_H` / `LQ_H` | = LS_H | D/Q 轴电感。表贴电机令 Ld = Lq；凸极电机需分别设置 |
| `MOTOR_CFG_FLUX_LINKAGE_VS` | ≈ 0.00341 Vs | 永磁体磁链。由反电势实测推导，是观测器最关键参数 |

**磁链估算方法（反电势法）：**
```
1. 外拖电机测 AB 线电压峰峰值 Vab_pp 和电频率 f_e
2. 相电压峰值 = Vab_pp / (2 × √3)
3. 磁链 λ = 相电压峰值 / (2π × f_e)
```

### 2. 采样链路参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_ADC_VREF_V` | 5.0 V | ADC 参考电压 |
| `MOTOR_CFG_PHASE_SHUNT_OHM` | 0.005 Ω | 电流采样电阻 |
| `MOTOR_CFG_CURRENT_AMP_GAIN` | 10 | 运放放大倍数 |
| `MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM` | 4000 Ω | 母线分压上臂 |
| `MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM` | 1000 Ω | 母线分压下臂 |

> ⚠️ 当前分压配置下，母线电压最大可测 25V。若系统母线超过 24V，需修改硬件分压。

### 3. 观测器 / PLL 参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_OBSERVER_GAIN` | 2.0e6 | 观测器校正增益 γ。越大收敛越快，但噪声敏感度增加 |
| `MOTOR_CFG_LAMBDA_COMP_BW_RAD_S` | 80 | 磁链在线补偿带宽 (rad/s) |
| `MOTOR_CFG_LAMBDA_MIN_VS` | 0.0025 | 磁链估计下限 |
| `MOTOR_CFG_LAMBDA_MAX_VS` | 0.0048 | 磁链估计上限 |
| `MOTOR_CFG_PLL_KP` | 400 | PLL 比例增益 |
| `MOTOR_CFG_PLL_KI` | 20000 | PLL 积分增益 |

### 4. 电流环参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_CURRENT_LOOP_BW_HZ` | 1500 Hz | 电流环带宽。PI 增益由此自动推导 |
| `MOTOR_CFG_ID_KP` / `IQ_KP` | L × ω_bw | 比例增益 = 电感 × 电流环角频率 |
| `MOTOR_CFG_ID_KI` / `IQ_KI` | Rs × ω_bw | 积分增益 = 电阻 × 电流环角频率 |

### 5. 速度环参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_SPEED_KP` | 0.006 | 速度环比例增益（需根据负载调整） |
| `MOTOR_CFG_SPEED_KI` | 0.100 | 速度环积分增益 |
| `MOTOR_CFG_SPEED_RAMP_RPM_PER_S` | 600 | 转速斜坡限制 (RPM/s) |
| `MOTOR_CFG_MAX_IQ_A` | 6.0 A | 最大 q 轴电流限幅 |

### 6. 启动参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_ALIGN_CURRENT_A` | 0.5 A | 对齐阶段 d 轴电流 |
| `MOTOR_CFG_ALIGN_TIME_MS` | 100 ms | 对齐持续时间 |
| `MOTOR_CFG_OPEN_LOOP_IQ_A` | 3.0 A | 开环拉升 q 轴电流 |
| `MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S` | 800 rad/s | 开环阶段最终电角速度 |
| `MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS` | 200 ms | 速度/电流斜坡时间 |
| `MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD` | 0.45 rad | 观测器锁定判据（相位误差阈值） |
| `MOTOR_CFG_OBSERVER_LOCK_COUNT` | 80 | 连续满足锁定条件的次数要求 |

### 7. 保护参数

| 参数宏 | 默认值 | 说明 |
|--------|--------|------|
| `MOTOR_CFG_PHASE_OVERCURRENT_A` | 10.0 A | 软件过流阈值 |
| `MOTOR_CFG_VBUS_UNDERVOLTAGE_V` | 7.0 V | 欠压保护 |
| `MOTOR_CFG_VBUS_OVERVOLTAGE_V` | 24.0 V | 过压保护 |
| `MOTOR_CFG_STARTUP_TIMEOUT_MS` | 300 ms | 启动超时 |
| `MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD` | 0.90 rad | 观测器丢锁相位误差阈值 |
| `MOTOR_CFG_OBSERVER_LOSS_COUNT` | 200 | 丢锁判定计数 |

---

## 启动与运行状态机

```
STOP ──> OFFSET_CAL ──> ALIGN ──> OPEN_LOOP_RAMP ──> CLOSED_LOOP
  ▲                                      │                │
  │            启动超时 / 过流            │    观测器丢锁    │
  │              ┌───────────────────────┘                │
  │              ▼                                        ▼
  └─────────── FAULT ◄────────────────────────────────────┘
```

| 状态 | 说明 |
|------|------|
| **STOP** | 停机，PWM 输出屏蔽 |
| **OFFSET_CAL** | ADC 零偏校准（采集 512 次取均值） |
| **ALIGN** | 转子对齐，施加固定 d 轴电流锁定转子到已知角度 |
| **OPEN_LOOP_RAMP** | 强制开环拉升，电角度由内部积分产生 |
| **CLOSED_LOOP** | 观测器闭环运行，角度由观测器提供 |
| **FAULT** | 故障锁定，需手动清除后重启 |

---

## 构建与调试

### 前置要求

- ARM GCC 工具链或 IAR / Keil
- CMake ≥ 3.16
- Segger Ozone（数据采样与调试，工程文件已包含 `.jdebug`）

### 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=../cmake/<your_toolchain>.cmake
make -j$(nproc)
```

### 首次调试建议

1. 保持 `MOTOR_APP_AUTO_START = 0`，上电不自动运行
2. 确认 `bus_voltage_v` 读数与万用表一致
3. 确认三相电流零偏接近 0A
4. 手动使能观察对齐 → 开环 → 闭环切换过程
5. 详细调试步骤参见 `docs/ytm32_sensorless_foc_debug_cn.md`

---

## 许可证

```
Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
All rights reserved.
SPDX-License-Identifier: BSD-3-Clause
```
