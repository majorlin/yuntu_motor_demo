#ifndef MOTOR_USER_CONFIG_H
#define MOTOR_USER_CONFIG_H

#include <stdint.h>

/**
 * @file motor_user_config_fan.h
 * @brief User configuration for the YTM32 sensorless FOC stack (Fan variant).
 *
 * The values in this file are safe compile-time templates and are not final
 * production numbers. Replace the board analog front-end values and motor
 * parameters with real hardware data before tuning on hardware.
 */

/* ------------------------------- Constants -------------------------------- */

#define MOTOR_CFG_PI_F (3.14159265358979323846f)
#define MOTOR_CFG_TWO_PI_F (6.28318530717958647692f)
#define MOTOR_CFG_HALF_PI_F (1.57079632679489661923f)
#define MOTOR_CFG_SQRT3_F (1.73205080756887729353f)
#define MOTOR_CFG_INV_SQRT3_F (0.57735026918962576451f)
#define MOTOR_CFG_SQRT3_BY_2_F (0.86602540378443864676f)

/* ------------------------------ App behavior ------------------------------ */

/* 首次实机调试建议关闭自动起转，确认采样和PWM正常后再打开。 */
#define MOTOR_APP_AUTO_START (0U)

/* Default mechanical speed target exposed to the application interface. */
#define MOTOR_CFG_DEFAULT_TARGET_RPM (1000.0f)

/* Upper clamp for runtime speed commands coming from keys or debugger. */
#define MOTOR_CFG_MAX_TARGET_RPM (8500.0f)

/** @brief Default direction used at startup. Valid values are +1 or -1. */
#define MOTOR_CFG_DEFAULT_DIRECTION (1)

/* ------------------------------ MCU clocks -------------------------------- */

/**
 * @brief eTMR base clock in Hz.
 * eTMR uses FAST_BUS_CLK on this SoC. The current board clock configuration
 * runs FAST_BUS at the full PLL rate.
 */
#define MOTOR_CFG_ETMR_CLOCK_HZ (120000000UL)

/**
 * @brief pTMR base clock in Hz.
 * pTMR derives its time base from SLOW_BUS_CLK through the driver.
 * With the current board clock tree that resolves to 40 MHz.
 */
#define MOTOR_CFG_PTMR_CLOCK_HZ (40000000UL)

/**
 * @brief ADC module clock in Hz.
 * ADC protocol clock is configured at runtime from FIRC / 3 = 32 MHz to stay
 * within the 1 MHz .. 32 MHz runtime limits enforced by the SDK.
 */
#define MOTOR_CFG_ADC_CLOCK_HZ (32000000UL)

/* ---------------------------- PWM / fast loop ----------------------------- */

/* PWM switching frequency. Higher values reduce current ripple but increase CPU
 * load. */
#define MOTOR_CFG_PWM_FREQUENCY_HZ (20000UL)

/** @brief Speed loop frequency driven by pTMR0_CH0. */
#define MOTOR_CFG_SPEED_LOOP_HZ (1000UL)

/* Requested deadtime between complementary switches, in nanoseconds. */
#define MOTOR_CFG_DEADTIME_NS (500UL)

/** @brief Enable CM33 DWT-based timing statistics for the fast loop. */
#define MOTOR_CFG_ENABLE_DWT_PROFILE (0U)

/** @brief Enable deadtime compensation in the FOC duty output path. */
#ifndef MOTOR_CFG_ENABLE_DEADTIME_COMP
#define MOTOR_CFG_ENABLE_DEADTIME_COMP (0U)
#endif

/** @brief Gain applied to the base deadtime duty compensation. */
#ifndef MOTOR_CFG_DEADTIME_COMP_GAIN
#define MOTOR_CFG_DEADTIME_COMP_GAIN (1.0f)
#endif

/** @brief Minimum phase current magnitude required to refresh the compensation
 * sign. */
#ifndef MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A
#define MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A (0.4f)
#endif

/**
 * @brief Alpha/beta voltage modulation limit used by the SVM routine.
 * This is a conservative value expressed in V_alpha/V_beta per-unit of bus
 * voltage.
 */
#define MOTOR_CFG_SVM_MAX_MODULATION (0.55f)

/* --------------------------- ADC / analog front end ----------------------- */

/** @brief ADC reference voltage. The current hardware is 5V. */
#define MOTOR_CFG_ADC_VREF_V (5.0f)

/** @brief ADC maximum raw count in 12-bit mode. */
#define MOTOR_CFG_ADC_MAX_COUNTS (4095.0f)

/*
 * Phase current shunt resistor in ohms. Replace with the actual board value.
 * Typical low-voltage motor boards use 2 mOhm .. 10 mOhm.
 */
#define MOTOR_CFG_PHASE_SHUNT_OHM (0.005f)

/*
 * Current-sense amplifier gain. Replace with the real analog front-end gain.
 */
#define MOTOR_CFG_CURRENT_AMP_GAIN (10.0f)

/*
 * Bus voltage divider: top resistor from bus to ADC input, bottom resistor from
 * ADC input to ground. Replace with the real board divider values.
 */
#define MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM (4000.0f)
#define MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM (1000.0f)

/**
 * @brief Number of software-triggered ADC frames used to calibrate
 * phase-current offsets before enabling PWM.
 */
#define MOTOR_CFG_OFFSET_CAL_SAMPLES (512U)

/* ------------------------------- Protection ------------------------------- */

/* Absolute software current fault threshold in amps. */
#define MOTOR_CFG_PHASE_OVERCURRENT_A (20.0f)

/** @brief DC bus under-voltage threshold in volts. */
#define MOTOR_CFG_VBUS_UNDERVOLTAGE_V (7.0f)

/**
 * @brief DC bus over-voltage threshold in volts.
 *
 * Note: A 4k/1k divider combined with a 5V ADC full-scale range can only
 * measure up to 25V bus voltage. Therefore, the software overvoltage threshold
 * must be strictly less than 25V. If the system bus is higher, the hardware
 * divider must be adjusted first.
 */
#define MOTOR_CFG_VBUS_OVERVOLTAGE_V (18.0f)

/* Startup timeout before declaring a fault, in milliseconds.
 * 鼓风机叶轮惯量大，开环锁定需要更多时间，放宽到 1.5s 给观测器更多收敛机会。 */
#define MOTOR_CFG_STARTUP_TIMEOUT_MS (1500U)

/* Allowed observer phase error during acquisition, in electrical radians.
 * 数据显示相位误差在开环阶段振荡导致锁定失败，放宽到 1.00 rad。 */
#define MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD (1.00f)

/* Observer phase error above this value counts as lost lock. */
#define MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD (1.20f)

/* Consecutive 1 kHz speed-loop samples required to accept observer lock.
 * 鼓风机负载稳定，适当降低连续样本要求以加快锁定。 */
#define MOTOR_CFG_OBSERVER_LOCK_COUNT (15U)

/** @brief Consecutive 1 kHz speed-loop samples that trigger observer-loss
 * fault. */
#define MOTOR_CFG_OBSERVER_LOSS_COUNT (200U)

/* Minimum electrical speed magnitude required before observer lock is accepted.
 */
#define MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S (80.0f)

/* --------------------------- Motor electrical model ----------------------- */

/*
 * Mechanical pole-pair count. This only affects the interface layer and the
 * speed conversion between RPM and electrical radians/second.
 */
#define MOTOR_CFG_POLE_PAIRS (3U)

/* Stator phase resistance in ohms. */
#define MOTOR_CFG_RS_OHM (0.105595f)

/* D-axis inductance in henries. */
#define MOTOR_CFG_LS_H (0.000130f)

/** @brief During initial debug, assume surface-mount (SPM) motor without
 * salience: Ld = Lq = Ls. */
#define MOTOR_CFG_LD_H (MOTOR_CFG_LS_H)

/** @brief Q-axis inductance in henries. */
#define MOTOR_CFG_LQ_H (MOTOR_CFG_LS_H)

/*
 * Permanent-magnet flux linkage in volt-seconds/rad. This is a template value
 * used by the observer and should be replaced by motor-specific data.
 */
/*
 * 由外拖反电势估算磁链：
 * 默认按 AB 线电压峰峰值 1.96V、电频 17Hz 计算。
 * 对正弦波，phase_peak = Vab_pp / (2 * sqrt(3))。
 * lambda = phase_peak / omega_e。
 */
#define MOTOR_CFG_BEMF_AB_LINE_PP_V (1.96f)
#define MOTOR_CFG_BEMF_ELEC_FREQ_HZ (17.0f)
#define MOTOR_CFG_BEMF_PHASE_PEAK_V                                            \
  ((0.5f * MOTOR_CFG_BEMF_AB_LINE_PP_V) * MOTOR_CFG_INV_SQRT3_F)
#define MOTOR_CFG_FLUX_LINKAGE_VS                                              \
  (MOTOR_CFG_BEMF_PHASE_PEAK_V /                                               \
   (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_BEMF_ELEC_FREQ_HZ))

/* Maximum commanded q-axis current during closed-loop operation. */
#define MOTOR_CFG_MAX_IQ_A (16.0f)

/* Default q-axis current target used by the debugger-facing current mode. */
#define MOTOR_CFG_DEFAULT_TARGET_IQ_A (1.0f)

/* ------------------------------ Current loop ------------------------------ */

/*
 * Current-loop bandwidth in Hz. The PI gains below are derived from the motor
 * R/L model using a standard continuous-time approximation.
 */
#define MOTOR_CFG_CURRENT_LOOP_BW_HZ (1200.0f)

/* ------------------------------- Speed loop ------------------------------- */

/*
 * Speed PI gains are intentionally exposed directly because the mechanical
 * plant depends on inertia, friction and load, which are not known yet.
 *
 * 鼓风机电机特点：叶轮惯量大、低速负载力矩小。
 * 开环阶段注入电流远大于闭环稳态需求，进入闭环后积分器预装值
 * 过高导致转速严重超调、来回振荡。需降低 Ki 防止积分器饱和,
 * 同时降低 Kp 减小比例环节冲击。
 */
#define MOTOR_CFG_SPEED_KP (0.0030f)
#define MOTOR_CFG_SPEED_KI (0.0060f)

/* Runtime speed-command ramp to avoid large torque steps when changing rpm.
 * 鼓风机叶轮惯量大，过快斜坡容易产生过流，放缓到 600 RPM/s。 */
#define MOTOR_CFG_SPEED_RAMP_RPM_PER_S (600.0f)

/* --------------------------- Startup / transition ------------------------- */

/** @brief Fixed electrical angle used during rotor alignment. */
#define MOTOR_CFG_ALIGN_ANGLE_RAD (0.0f)

/* D-axis current applied during alignment. */
#define MOTOR_CFG_ALIGN_CURRENT_A (5.0f)

/* Alignment duration in milliseconds. */
#define MOTOR_CFG_ALIGN_TIME_MS (500U)

/* Open-loop q-axis current target. It ramps up from the alignment current
 * magnitude. 鼓风机负载力矩小，过大的开环电流导致闭环切换后积分器
 * 预装值过高引起转速超调，降低到 5A 即可保证可靠启动。 */
#define MOTOR_CFG_OPEN_LOOP_IQ_A (8.0f)

/** @brief Total time for the open-loop current/speed ramp. */
#define MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS (1000U)

/* Final open-loop electrical speed magnitude before handover to observer. */
#define MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S (120.0f)

/* Closed-loop phase blend duration in milliseconds. */
#define MOTOR_CFG_CLOSED_LOOP_BLEND_MS (300U)

/* ========================= Wind Detect / Catch Spin ========================
 */

/** @brief Enable wind detect before alignment (0 = disabled, traditional Align
 * flow). */
#define MOTOR_CFG_ENABLE_WIND_DETECT (1U)

/** @brief Wind detect observer settling timeout (ms).
 * 鼓风机叶轮惯量大，停机后仍可能高速旋转，给观察器更多收敛时间。 */
#define MOTOR_CFG_WIND_DETECT_TIMEOUT_MS (1000U)

/** @brief Electrical speed below which the rotor is considered standstill
 * (rad/s). 提高阈值防止观测器噪声产生的虚假速度触发 CatchSpin。 */
#define MOTOR_CFG_WIND_DETECT_STANDSTILL_RAD_S (120.0f)

/** @brief Number of consecutive speed-loop samples with stable speed to declare
 * convergence. 增加到 120 要求观测器速度持续稳定 120ms 才决策，过滤短暂噪声。
 */
#define MOTOR_CFG_WIND_DETECT_SETTLE_COUNT (120U)

/** @brief Maximum speed change per speed-loop tick for convergence check
 * (rad/s). */
#define MOTOR_CFG_WIND_DETECT_SPEED_TOL_RAD_S (20.0f)

/** @brief Maximum mechanical RPM for direct tailwind catch. Above this, brake
 * first. */
#define MOTOR_CFG_CATCH_MAX_SPEED_RPM (300.0f)

/** @brief Electrical speed threshold for coast-down to transition to Align
 * (rad/s). Set below the wind-detect standstill threshold so the motor is
 * nearly stopped before alignment current is applied. */
#define MOTOR_CFG_COAST_DOWN_SAFE_SPEED_RAD_S (80.0f)

/** @brief Braking q-axis current magnitude for coast-down regenerative
 * deceleration (A). 制动电流不宜过大，以防无刹车电阻时母线过压。 */
#define MOTOR_CFG_COAST_BRAKE_IQ_A (2.0f)

/** @brief Bus voltage threshold to start reducing braking Iq (V).
 * When Vbus exceeds this, braking Iq is linearly reduced to zero over the
 * hysteresis band to prevent overvoltage. Set below VBUS_OVERVOLTAGE_V. */
#define MOTOR_CFG_COAST_BRAKE_VBUS_LIMIT_V (16.0f)

/** @brief Bus voltage hysteresis band width (V).
 * Braking Iq is linearly reduced from full to zero across this band above
 * the limit, i.e. Iq=0 when Vbus >= LIMIT + HYST. */
#define MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V (1.0f)

/** @brief Minimum observer flux magnitude (Vs) to trust speed estimate for
 * CatchSpin. 当磁链幅值低于此阈值时，认为观测器速度为噪声而非真实转动。 */
#define MOTOR_CFG_WIND_DETECT_MIN_FLUX_VS (MOTOR_CFG_FLUX_LINKAGE_VS * 0.30f)

/* ============================== Field Weakening ============================
 */

/** @brief Enable field weakening control (0 = disabled, id_target stays 0). */
#define MOTOR_CFG_ENABLE_FIELD_WEAKENING (1U)

/** @brief Voltage modulation ratio threshold to start injecting negative Id
 * (0~1). */
#define MOTOR_CFG_FW_VOLTAGE_THRESHOLD (0.50f)

/** @brief Field weakening integral gain. Larger = faster response but may
 * oscillate. */
#define MOTOR_CFG_FW_KI (50.0f)

/** @brief Maximum negative Id current for field weakening (A). Must be
 * negative. */
#define MOTOR_CFG_FW_MAX_NEGATIVE_ID_A (-4.0f)

/** @brief Recovery rate (A/s) for Id returning to zero when voltage margin
 * restores. */
#define MOTOR_CFG_FW_RECOVERY_RATE (20.0f)

/* ------------------------- Observer / PLL parameters ---------------------- */

/* Ortega observer correction gain. Larger values converge faster but add noise.
 */
#define MOTOR_CFG_OBSERVER_GAIN (1.8e6f)

/** @brief Flux magnitude compensation bandwidth in rad/s. */
#define MOTOR_CFG_LAMBDA_COMP_BW_RAD_S (80.0f)

/* Lower clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MIN_VS (0.0040f)

/* Upper clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MAX_VS (0.0072f)

/* PLL proportional gain. 降低 Kp 减小速度估计的噪声振荡。 */
#define MOTOR_CFG_PLL_KP (120.0f)

/* PLL integral gain. 同步降低 Ki 保持阻尼比，改善收敛稳定性。 */
#define MOTOR_CFG_PLL_KI (6000.0f)

/* 开环切环阶段对固定相位偏置的跟踪带宽。 */
#define MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S (20.0f)

/* ------------------------------- Derived data ----------------------------- */

#define MOTOR_CFG_FAST_LOOP_DT_S (1.0f / (float)MOTOR_CFG_PWM_FREQUENCY_HZ)

#define MOTOR_CFG_SPEED_LOOP_DT_S (1.0f / (float)MOTOR_CFG_SPEED_LOOP_HZ)

#define MOTOR_CFG_PWM_PERIOD_TICKS                                             \
  ((uint32_t)(MOTOR_CFG_ETMR_CLOCK_HZ / MOTOR_CFG_PWM_FREQUENCY_HZ))

#define MOTOR_CFG_PWM_HALF_PERIOD_TICKS (MOTOR_CFG_PWM_PERIOD_TICKS / 2UL)

#define MOTOR_CFG_PWM_MID_TICKS (MOTOR_CFG_PWM_HALF_PERIOD_TICKS)

#define MOTOR_CFG_DEADTIME_TICKS                                               \
  ((uint16_t)(((uint64_t)MOTOR_CFG_ETMR_CLOCK_HZ *                             \
               (uint64_t)MOTOR_CFG_DEADTIME_NS) /                              \
              1000000000ULL))

#define MOTOR_CFG_DEADTIME_COMP_DUTY                                           \
  (((float)MOTOR_CFG_DEADTIME_TICKS / (float)MOTOR_CFG_PWM_PERIOD_TICKS) *     \
   MOTOR_CFG_DEADTIME_COMP_GAIN)

#define MOTOR_CFG_CURRENT_SENSE_V_PER_A                                        \
  (MOTOR_CFG_PHASE_SHUNT_OHM * MOTOR_CFG_CURRENT_AMP_GAIN)

#define MOTOR_CFG_ADC_COUNT_TO_VOLT                                            \
  (MOTOR_CFG_ADC_VREF_V / MOTOR_CFG_ADC_MAX_COUNTS)

#define MOTOR_CFG_ADC_COUNT_TO_CURRENT_A                                       \
  (MOTOR_CFG_ADC_COUNT_TO_VOLT / MOTOR_CFG_CURRENT_SENSE_V_PER_A)

#define MOTOR_CFG_VBUS_DIVIDER_GAIN                                            \
  ((MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM + MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM) /  \
   MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM)

#define MOTOR_CFG_ADC_COUNT_TO_VBUS_V                                          \
  (MOTOR_CFG_ADC_COUNT_TO_VOLT * MOTOR_CFG_VBUS_DIVIDER_GAIN)

#define MOTOR_CFG_CURRENT_LOOP_W_RAD_S                                         \
  (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_CURRENT_LOOP_BW_HZ)

#define MOTOR_CFG_ID_KP_V_PER_A                                                \
  (MOTOR_CFG_LD_H * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_ID_KI_V_PER_AS                                               \
  (MOTOR_CFG_RS_OHM * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_IQ_KP_V_PER_A                                                \
  (MOTOR_CFG_LQ_H * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_IQ_KI_V_PER_AS                                               \
  (MOTOR_CFG_RS_OHM * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_MECH_RPM_TO_ELEC_RAD_S(rpm_value)                            \
  ((rpm_value) * MOTOR_CFG_TWO_PI_F * (float)MOTOR_CFG_POLE_PAIRS / 60.0f)

#define MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(speed_value)                          \
  ((speed_value) * 60.0f / (MOTOR_CFG_TWO_PI_F * (float)MOTOR_CFG_POLE_PAIRS))

#define MOTOR_CFG_USED_PWM_CHANNEL_MASK (0xF3U)

#endif /* MOTOR_USER_CONFIG_H */
