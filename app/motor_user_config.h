#ifndef MOTOR_USER_CONFIG_H
#define MOTOR_USER_CONFIG_H

#include <stdint.h>

/**
 * @file motor_user_config.h
 * @brief User configuration for the YTM32 sensorless FOC stack.
 *
 * This file contains:
 *   1. Board/hardware-level parameters (ADC, PWM, protection) — shared by all
 *      motors.
 *   2. Motor profile selection via MOTOR_PROFILE_SELECT — one #define to switch
 *      between different motors.
 *
 * Motor-specific parameters (electrical model, control gains, startup, observer
 * etc.) live in separate profile headers under motor_profiles/.
 */

/* ======================== Motor Profile Selection =========================
 *
 * 切换电机只需修改下面一行：
 *   MOTOR_PROFILE_FAN  — 鼓风机电机
 *   MOTOR_PROFILE_SEAT — 座椅电机 (无负载)
 *
 * 也可在编译命令行中指定：
 *   -DMOTOR_PROFILE_SELECT=MOTOR_PROFILE_SEAT
 */
#define MOTOR_PROFILE_FAN  1
#define MOTOR_PROFILE_SEAT 2

#ifndef MOTOR_PROFILE_SELECT
#define MOTOR_PROFILE_SELECT MOTOR_PROFILE_SEAT
#endif

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

/* ----------------------- Sensorless angle monitor ------------------------- */

/**
 * @brief Minimum BEMF vector magnitude, in ADC counts, required to trust
 * manual-rotation angle monitoring.
 */
#define MOTOR_CFG_ANGLE_MONITOR_MIN_BEMF_COUNTS (18.0f)

/**
 * @brief Low-pass bandwidth for the hand-rotation speed estimate.
 */
#define MOTOR_CFG_ANGLE_MONITOR_SPEED_FILTER_BW_HZ (25.0f)

/**
 * @brief Minimum electrical speed magnitude required before the monitored
 * angle is marked valid.
 */
#define MOTOR_CFG_ANGLE_MONITOR_VALID_SPEED_RAD_S (8.0f)

/* ======================== Include Motor Profile ============================ */

#if MOTOR_PROFILE_SELECT == MOTOR_PROFILE_FAN
#include "motor_profiles/motor_profile_fan.h"
#elif MOTOR_PROFILE_SELECT == MOTOR_PROFILE_SEAT
#include "motor_profiles/motor_profile_seat.h"
#else
#error "Unknown MOTOR_PROFILE_SELECT value. Define as MOTOR_PROFILE_FAN or MOTOR_PROFILE_SEAT."
#endif

/* ======================== Derived Data (shared) ============================ */

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
  ((MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM + MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM) / \
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

#define MOTOR_CFG_WIND_DETECT_STANDSTILL_RAD_S                                 \
  MOTOR_CFG_MECH_RPM_TO_ELEC_RAD_S(MOTOR_CFG_WIND_DETECT_STANDSTILL_RPM)

#define MOTOR_CFG_USED_PWM_CHANNEL_MASK (0xF3U)

#endif /* MOTOR_USER_CONFIG_H */
