#ifndef MOTOR_PROFILE_SEAT_H
#define MOTOR_PROFILE_SEAT_H

/**
 * @file motor_profile_seat.h
 * @brief Motor profile: 座椅电机 (Seat motor, no load)
 *
 * 特点：无负载、转动惯量极小、响应快。
 * 外拖反电势实测：AB 线电压峰峰值 28.6V，电频 385Hz。
 *
 * 参数推导：
 *   phase_peak = 28.6 / (2 * sqrt(3)) = 8.256 V
 *   omega_e    = 2 * pi * 385        = 2419.0 rad/s
 *   lambda     = 8.256 / 2419.0      = 0.003413 Vs
 *   mech_rpm   = 385 / 4 * 60        = 5775 RPM
 */

/* ======================== Motor Electrical Model ========================== */

#define MOTOR_CFG_POLE_PAIRS (4U)

#define MOTOR_CFG_RS_OHM (0.05f)
#define MOTOR_CFG_LS_H (0.00045158f)
#define MOTOR_CFG_LD_H (MOTOR_CFG_LS_H)
#define MOTOR_CFG_LQ_H (MOTOR_CFG_LS_H)

/* 由外拖反电势估算磁链：
 * AB 线电压峰峰值 28.6V、电频 385Hz。
 * phase_peak = Vab_pp / (2 * sqrt(3))
 * lambda = phase_peak / omega_e
 */
#define MOTOR_CFG_BEMF_AB_LINE_PP_V (28.6f)
#define MOTOR_CFG_BEMF_ELEC_FREQ_HZ (385.0f)
#define MOTOR_CFG_BEMF_PHASE_PEAK_V                                            \
  ((0.5f * MOTOR_CFG_BEMF_AB_LINE_PP_V) * MOTOR_CFG_INV_SQRT3_F)
#define MOTOR_CFG_FLUX_LINKAGE_VS                                              \
  (MOTOR_CFG_BEMF_PHASE_PEAK_V /                                               \
   (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_BEMF_ELEC_FREQ_HZ))

/* 无负载座椅电机，额定电流较小 */
#define MOTOR_CFG_MAX_IQ_A (10.0f)
#define MOTOR_CFG_DEFAULT_TARGET_IQ_A (0.5f)

/* ======================== Speed / Target Range ============================ */

/* 外拖空载 5775RPM，留裕量上限 5500RPM */
#define MOTOR_CFG_DEFAULT_TARGET_RPM (2000.0f)
#define MOTOR_CFG_MAX_TARGET_RPM (5500.0f)

/* ======================== Current Loop =====================================
 */

/* Ls/Rs 时间常数 = 0.00045158/0.05 = 9.03ms，电流环带宽可取 2000Hz。
 * 无负载电机电流响应需要更快。 */
#define MOTOR_CFG_CURRENT_LOOP_BW_HZ (2000.0f)

/* ======================== Speed Loop =======================================
 */

/* 无负载惯量极小，Kp/Ki 需要更小防止超调振荡。
 * 速度命令斜坡可以设得更陡，因为无负载加速很快。 */
#define MOTOR_CFG_SPEED_KP (0.005375)
#define MOTOR_CFG_SPEED_KI (0.05f)
#define MOTOR_CFG_SPEED_RAMP_RPM_PER_S (2000.0f)

/* ======================== Startup / Transition =============================
 */

#define MOTOR_CFG_ALIGN_ANGLE_RAD (0.0f)

/* 无负载不需要大电流对齐和开环拖动 */
#define MOTOR_CFG_ALIGN_CURRENT_A (0.5f)
#define MOTOR_CFG_ALIGN_CURRENT_RAMP_TIME_MS (30U)
#define MOTOR_CFG_ALIGN_TIME_MS (100U)

#define MOTOR_CFG_OPEN_LOOP_IQ_A (1.0f)
#define MOTOR_CFG_IF_CURRENT_RAMP_TIME_MS (80U)
#define MOTOR_CFG_IF_MIN_IQ_A (0.35f)
#define MOTOR_CFG_IF_CURRENT_FILTER_BW_HZ (40.0f)
#define MOTOR_CFG_IF_CURRENT_MARGIN_A (0.35f)
#define MOTOR_CFG_IF_ACCEL_MIN_SCALE (0.10f)
#define MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS (300U)
#define MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S (150.0f)
#define MOTOR_CFG_CLOSED_LOOP_BLEND_MS (100U)

/* ======================== HFI / Initial Position Detect ==================== */

#define MOTOR_CFG_ENABLE_HFI_IPD (1U)
#define MOTOR_CFG_HFI_IPD_INJECT_V (1.20f)
#define MOTOR_CFG_HFI_IPD_PULSE_PAIRS (8U)
#define MOTOR_CFG_HFI_IPD_CANDIDATE_COUNT (4U)
#define MOTOR_CFG_HFI_IPD_CONFIDENCE_MIN (0.12f)
#define MOTOR_CFG_HFI_IPD_POLARITY_V (1.80f)
#define MOTOR_CFG_HFI_IPD_POLARITY_PULSES (6U)
#define MOTOR_CFG_HFI_IPD_ALIGN_HOLD_MS (40U)

/* ======================== Startup Retry / Stall ============================
 */

/* 无负载启动非常容易，缩短超时 */
#define MOTOR_CFG_STARTUP_TIMEOUT_MS (1000U)
#define MOTOR_CFG_STARTUP_MAX_RETRIES (0U)
#define MOTOR_CFG_STARTUP_IQ_BOOST_STEP_A (0.5f)
#define MOTOR_CFG_STARTUP_IQ_BOOST_MAX_A (2.0f)
#define MOTOR_CFG_STALL_ANGLE_ERR_RAD (1.80f)
#define MOTOR_CFG_STALL_ANGLE_DIV_COUNT (60U)

/* ======================== Observer / PLL ===================================
 */

/* Rs 低 + Ls 较大 → 降低观测器增益避免噪声放大 */
#define MOTOR_CFG_OBSERVER_GAIN (0.8e6f)
#define MOTOR_CFG_LAMBDA_COMP_BW_RAD_S (100.0f)

/* 磁链范围基于推导值 0.003413 Vs，±50% */
#define MOTOR_CFG_LAMBDA_MIN_VS (0.0017f)
#define MOTOR_CFG_LAMBDA_MAX_VS (0.0052f)

/* 速度快 → PLL 带宽适当提高 */
#define MOTOR_CFG_PLL_KP (200.0f)
#define MOTOR_CFG_PLL_KI (10000.0f)
#define MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S (30.0f)

/* 无负载锁定更快，可以降低锁定样本计数 */
#define MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD (1.00f)
#define MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD (1.20f)
#define MOTOR_CFG_OBSERVER_LOCK_COUNT (10U)
#define MOTOR_CFG_OBSERVER_LOSS_COUNT (150U)
#define MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S (100.0f)

/* ======================== Wind Detect / Catch Spin =========================
 */

#define MOTOR_CFG_ENABLE_WIND_DETECT (0U)
#define MOTOR_CFG_WIND_DETECT_TIMEOUT_MS (200U)
#define MOTOR_CFG_WIND_DETECT_STANDSTILL_RPM (200.0f)
#define MOTOR_CFG_WIND_DETECT_SETTLE_COUNT (80U)
#define MOTOR_CFG_WIND_DETECT_SPEED_TOL_RAD_S (30.0f)
#define MOTOR_CFG_CATCH_MAX_SPEED_RPM (3000.0f)
#define MOTOR_CFG_COAST_DOWN_SAFE_SPEED_RAD_S (100.0f)
#define MOTOR_CFG_COAST_BRAKE_IQ_A (1.5f)
#define MOTOR_CFG_COAST_BRAKE_VBUS_LIMIT_V (16.0f)
#define MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V (1.0f)
#define MOTOR_CFG_WIND_DETECT_MIN_FLUX_VS (MOTOR_CFG_FLUX_LINKAGE_VS * 0.30f)
#define MOTOR_CFG_BEMF_VOLTAGE_DIVIDER (5.0f)
#define MOTOR_CFG_WIND_DETECT_MIN_CROSSINGS (6U)

/* ======================== Field Weakening ==================================
 */

/* 座椅电机低磁链高速，弱磁更重要 */
#define MOTOR_CFG_ENABLE_FIELD_WEAKENING (0U)
#define MOTOR_CFG_FW_VOLTAGE_THRESHOLD (0.48f)
#define MOTOR_CFG_FW_KI (80.0f)
#define MOTOR_CFG_FW_MAX_NEGATIVE_ID_A (-3.0f)
#define MOTOR_CFG_FW_RECOVERY_RATE (30.0f)

#endif /* MOTOR_PROFILE_SEAT_H */
