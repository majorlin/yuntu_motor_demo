#ifndef MOTOR_PROFILE_FAN_H
#define MOTOR_PROFILE_FAN_H

/**
 * @file motor_profile_fan.h
 * @brief Motor profile: 鼓风机电机 (Fan/Blower)
 *
 * 特点：叶轮惯量大、低速负载力矩小。
 * 外拖反电势实测：AB 线电压峰峰值 1.96V，电频 17Hz。
 */

/* ======================== Motor Electrical Model ========================== */

#define MOTOR_CFG_POLE_PAIRS (3U)

#define MOTOR_CFG_RS_OHM (0.105595f)
#define MOTOR_CFG_LS_H (0.000130f)
#define MOTOR_CFG_LD_H (MOTOR_CFG_LS_H)
#define MOTOR_CFG_LQ_H (MOTOR_CFG_LS_H)

/* 由外拖反电势估算磁链：
 * AB 线电压峰峰值 1.96V、电频 17Hz。
 * phase_peak = Vab_pp / (2 * sqrt(3))
 * lambda = phase_peak / omega_e
 */
#define MOTOR_CFG_BEMF_AB_LINE_PP_V (1.96f)
#define MOTOR_CFG_BEMF_ELEC_FREQ_HZ (17.0f)
#define MOTOR_CFG_BEMF_PHASE_PEAK_V                                            \
  ((0.5f * MOTOR_CFG_BEMF_AB_LINE_PP_V) * MOTOR_CFG_INV_SQRT3_F)
#define MOTOR_CFG_FLUX_LINKAGE_VS                                              \
  (MOTOR_CFG_BEMF_PHASE_PEAK_V /                                               \
   (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_BEMF_ELEC_FREQ_HZ))

#define MOTOR_CFG_MAX_IQ_A (16.0f)
#define MOTOR_CFG_DEFAULT_TARGET_IQ_A (1.0f)

/* ======================== Speed / Target Range ============================ */

#define MOTOR_CFG_DEFAULT_TARGET_RPM (1000.0f)
#define MOTOR_CFG_MAX_TARGET_RPM (8500.0f)

/* ======================== Current Loop ===================================== */

#define MOTOR_CFG_CURRENT_LOOP_BW_HZ (1200.0f)

/* ======================== Speed Loop ======================================= */

/* 叶轮惯量大、低速负载力矩小。降低 Ki 防止积分器饱和，
 * 降低 Kp 减小比例环节冲击。 */
#define MOTOR_CFG_SPEED_KP (0.010250f)
#define MOTOR_CFG_SPEED_KI (0.0010f)
#define MOTOR_CFG_SPEED_RAMP_RPM_PER_S (600.0f)

/* ======================== Startup / Transition ============================= */

#define MOTOR_CFG_ALIGN_ANGLE_RAD (0.0f)
#define MOTOR_CFG_ALIGN_CURRENT_A (5.0f)
#define MOTOR_CFG_ALIGN_CURRENT_RAMP_TIME_MS (80U)
#define MOTOR_CFG_ALIGN_TIME_MS (300U)

/* 鼓风机负载力矩小，开环注入电流不宜过大 */
#define MOTOR_CFG_OPEN_LOOP_IQ_A (8.0f)
#define MOTOR_CFG_IF_CURRENT_RAMP_TIME_MS (180U)
#define MOTOR_CFG_IF_MIN_IQ_A (1.0f)
#define MOTOR_CFG_IF_CURRENT_FILTER_BW_HZ (25.0f)
#define MOTOR_CFG_IF_CURRENT_MARGIN_A (1.0f)
#define MOTOR_CFG_IF_ACCEL_MIN_SCALE (0.10f)
#define MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS (1000U)
#define MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S (120.0f)
#define MOTOR_CFG_CLOSED_LOOP_BLEND_MS (300U)

/* ======================== HFI / Initial Position Detect ==================== */

#define MOTOR_CFG_ENABLE_HFI_IPD (0U)
#define MOTOR_CFG_HFI_IPD_INJECT_V (1.00f)
#define MOTOR_CFG_HFI_IPD_PULSE_PAIRS (6U)
#define MOTOR_CFG_HFI_IPD_CANDIDATE_COUNT (4U)
#define MOTOR_CFG_HFI_IPD_CONFIDENCE_MIN (0.12f)
#define MOTOR_CFG_HFI_IPD_POLARITY_V (1.50f)
#define MOTOR_CFG_HFI_IPD_POLARITY_PULSES (4U)
#define MOTOR_CFG_HFI_IPD_ALIGN_HOLD_MS (60U)

/* ======================== Startup Retry / Stall ============================ */

#define MOTOR_CFG_STARTUP_TIMEOUT_MS (2000U)
#define MOTOR_CFG_STARTUP_MAX_RETRIES (0U)
#define MOTOR_CFG_STARTUP_IQ_BOOST_STEP_A (1.0f)
#define MOTOR_CFG_STARTUP_IQ_BOOST_MAX_A (3.0f)
#define MOTOR_CFG_STALL_ANGLE_ERR_RAD (1.80f)
#define MOTOR_CFG_STALL_ANGLE_DIV_COUNT (80U)

/* ======================== Observer / PLL =================================== */

#define MOTOR_CFG_OBSERVER_GAIN (1.8e6f)
#define MOTOR_CFG_LAMBDA_COMP_BW_RAD_S (80.0f)
#define MOTOR_CFG_LAMBDA_MIN_VS (0.0040f)
#define MOTOR_CFG_LAMBDA_MAX_VS (0.0072f)
#define MOTOR_CFG_PLL_KP (120.0f)
#define MOTOR_CFG_PLL_KI (6000.0f)
#define MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S (20.0f)

#define MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD (1.20f)
#define MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD (1.20f)
#define MOTOR_CFG_OBSERVER_LOCK_COUNT (15U)
#define MOTOR_CFG_OBSERVER_LOSS_COUNT (200U)
#define MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S (80.0f)

/* ======================== Wind Detect / Catch Spin ========================= */

#define MOTOR_CFG_ENABLE_WIND_DETECT (1U)
#define MOTOR_CFG_WIND_DETECT_TIMEOUT_MS (300U)
#define MOTOR_CFG_WIND_DETECT_STANDSTILL_RPM (150.0f)
#define MOTOR_CFG_WIND_DETECT_SETTLE_COUNT (120U)
#define MOTOR_CFG_WIND_DETECT_SPEED_TOL_RAD_S (20.0f)
#define MOTOR_CFG_CATCH_MAX_SPEED_RPM (2000.0f)
#define MOTOR_CFG_COAST_DOWN_SAFE_SPEED_RAD_S (80.0f)
#define MOTOR_CFG_COAST_BRAKE_IQ_A (2.0f)
#define MOTOR_CFG_COAST_BRAKE_VBUS_LIMIT_V (16.0f)
#define MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V (1.0f)
#define MOTOR_CFG_WIND_DETECT_MIN_FLUX_VS (MOTOR_CFG_FLUX_LINKAGE_VS * 0.30f)
#define MOTOR_CFG_BEMF_VOLTAGE_DIVIDER (5.0f)
#define MOTOR_CFG_WIND_DETECT_MIN_CROSSINGS (6U)

/* ======================== Field Weakening ================================== */

#define MOTOR_CFG_ENABLE_FIELD_WEAKENING (1U)
#define MOTOR_CFG_FW_VOLTAGE_THRESHOLD (0.50f)
#define MOTOR_CFG_FW_KI (50.0f)
#define MOTOR_CFG_FW_MAX_NEGATIVE_ID_A (-4.0f)
#define MOTOR_CFG_FW_RECOVERY_RATE (20.0f)

#endif /* MOTOR_PROFILE_FAN_H */
