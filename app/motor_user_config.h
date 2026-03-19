#ifndef MOTOR_USER_CONFIG_H
#define MOTOR_USER_CONFIG_H

#include <stdint.h>

/*
 * User configuration for the YTM32 sensorless FOC stack.
 *
 * The values in this file are safe compile-time templates and are not final
 * production numbers. Replace the board analog front-end values and motor
 * parameters with real hardware data before tuning on hardware.
 */

/* ------------------------------- Constants -------------------------------- */

#define MOTOR_CFG_PI_F                           (3.14159265358979323846f)
#define MOTOR_CFG_TWO_PI_F                       (6.28318530717958647692f)
#define MOTOR_CFG_HALF_PI_F                      (1.57079632679489661923f)
#define MOTOR_CFG_SQRT3_F                        (1.73205080756887729353f)
#define MOTOR_CFG_INV_SQRT3_F                    (0.57735026918962576451f)
#define MOTOR_CFG_SQRT3_BY_2_F                   (0.86602540378443864676f)

/* ------------------------------ App behavior ------------------------------ */

/* 首次实机调试建议关闭自动起转，确认采样和PWM正常后再打开。 */
#define MOTOR_APP_AUTO_START                     (0U)

/* Default mechanical speed target exposed to the application interface. */
#define MOTOR_CFG_DEFAULT_TARGET_RPM             (500.0f)

/* Default direction used at startup. Valid values are +1 or -1. */
#define MOTOR_CFG_DEFAULT_DIRECTION              (1)

/* ------------------------------ MCU clocks -------------------------------- */

/*
 * eTMR uses FAST_BUS_CLK on this SoC. The current board clock configuration
 * runs FAST_BUS at the full PLL rate.
 */
#define MOTOR_CFG_ETMR_CLOCK_HZ                  (120000000UL)

/*
 * pTMR derives its time base from SLOW_BUS_CLK through the driver.
 * With the current board clock tree that resolves to 40 MHz.
 */
#define MOTOR_CFG_PTMR_CLOCK_HZ                  (40000000UL)

/*
 * ADC protocol clock is configured at runtime from FIRC / 3 = 32 MHz to stay
 * within the 1 MHz .. 32 MHz runtime limits enforced by the SDK.
 */
#define MOTOR_CFG_ADC_CLOCK_HZ                   (32000000UL)

/* ---------------------------- PWM / fast loop ----------------------------- */

/* PWM switching frequency. Higher values reduce current ripple but increase CPU load. */
#define MOTOR_CFG_PWM_FREQUENCY_HZ               (20000UL)

/* Speed loop frequency driven by pTMR0_CH0. */
#define MOTOR_CFG_SPEED_LOOP_HZ                  (1000UL)

/* Requested deadtime between complementary switches, in nanoseconds. */
#define MOTOR_CFG_DEADTIME_NS                    (500UL)

/*
 * Alpha/beta voltage modulation limit used by the SVM routine. This is a
 * conservative value expressed in V_alpha/V_beta per-unit of bus voltage.
 */
#define MOTOR_CFG_SVM_MAX_MODULATION             (0.55f)

/* --------------------------- ADC / analog front end ----------------------- */

/* ADC参考电平，当前硬件为5V。 */
#define MOTOR_CFG_ADC_VREF_V                     (5.0f)

/* ADC maximum raw count in 12-bit mode. */
#define MOTOR_CFG_ADC_MAX_COUNTS                 (4095.0f)

/*
 * Phase current shunt resistor in ohms. Replace with the actual board value.
 * Typical low-voltage motor boards use 2 mOhm .. 10 mOhm.
 */
#define MOTOR_CFG_PHASE_SHUNT_OHM                (0.005f)

/*
 * Current-sense amplifier gain. Replace with the real analog front-end gain.
 */
#define MOTOR_CFG_CURRENT_AMP_GAIN               (10.0f)

/*
 * Bus voltage divider: top resistor from bus to ADC input, bottom resistor from
 * ADC input to ground. Replace with the real board divider values.
 */
#define MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM         (4000.0f)
#define MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM      (1000.0f)

/*
 * Number of software-triggered ADC frames used to calibrate phase-current
 * offsets before enabling PWM.
 */
#define MOTOR_CFG_OFFSET_CAL_SAMPLES             (512U)

/* ------------------------------- Protection ------------------------------- */

/* Absolute software current fault threshold in amps. */
#define MOTOR_CFG_PHASE_OVERCURRENT_A            (10.0f)

/* DC bus under-voltage threshold in volts. */
#define MOTOR_CFG_VBUS_UNDERVOLTAGE_V            (7.0f)

/* DC bus over-voltage threshold in volts. */
/*
 * 4k/1k 分压配合 5V ADC 满量程只能测到 25V 母线电压，因此软件过压阈值
 * 必须低于 25V。若系统母线高于该范围，需要先修改硬件分压。
 */
#define MOTOR_CFG_VBUS_OVERVOLTAGE_V             (24.0f)

/* Startup timeout before declaring a fault, in milliseconds. */
#define MOTOR_CFG_STARTUP_TIMEOUT_MS             (3000U)

/* Allowed observer phase error during acquisition, in electrical radians. */
#define MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD    (0.45f)

/* Observer phase error above this value counts as lost lock. */
#define MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD    (0.90f)

/* Consecutive 1 kHz speed-loop samples required to accept observer lock. */
#define MOTOR_CFG_OBSERVER_LOCK_COUNT            (80U)

/* Consecutive 1 kHz speed-loop samples that trigger observer-loss fault. */
#define MOTOR_CFG_OBSERVER_LOSS_COUNT            (200U)

/* Minimum electrical speed magnitude required before observer lock is accepted. */
#define MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S      (300.0f)

/* --------------------------- Motor electrical model ----------------------- */

/*
 * Mechanical pole-pair count. This only affects the interface layer and the
 * speed conversion between RPM and electrical radians/second.
 */
#define MOTOR_CFG_POLE_PAIRS                     (4U)

/* Stator phase resistance in ohms. */
#define MOTOR_CFG_RS_OHM                         (0.050f)

/* D-axis inductance in henries. */
#define MOTOR_CFG_LS_H                           (0.00045158f)

/* 初始调试阶段先假设表贴式电机无显著凸极特性，因此Ld=Lq=Ls。 */
#define MOTOR_CFG_LD_H                           (MOTOR_CFG_LS_H)

/* Q-axis inductance in henries. */
#define MOTOR_CFG_LQ_H                           (MOTOR_CFG_LS_H)

/*
 * Permanent-magnet flux linkage in volt-seconds/rad. This is a template value
 * used by the observer and should be replaced by motor-specific data.
 */
/*
 * 由外拖反电势估算磁链：
 * AB线电压峰峰值 28.6V，电频 385Hz。
 * 对正弦波，phase_peak = Vab_pp / (2 * sqrt(3))。
 * lambda = phase_peak / omega_e。
 */
#define MOTOR_CFG_BEMF_AB_LINE_PP_V              (28.6f)
#define MOTOR_CFG_BEMF_ELEC_FREQ_HZ              (385.0f)
#define MOTOR_CFG_BEMF_PHASE_PEAK_V \
    ((0.5f * MOTOR_CFG_BEMF_AB_LINE_PP_V) * MOTOR_CFG_INV_SQRT3_F)
#define MOTOR_CFG_FLUX_LINKAGE_VS \
    (MOTOR_CFG_BEMF_PHASE_PEAK_V / (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_BEMF_ELEC_FREQ_HZ))

/* Maximum commanded q-axis current during closed-loop operation. */
#define MOTOR_CFG_MAX_IQ_A                       (6.0f)

/* ------------------------------ Current loop ------------------------------ */

/*
 * Current-loop bandwidth in Hz. The PI gains below are derived from the motor
 * R/L model using a standard continuous-time approximation.
 */
#define MOTOR_CFG_CURRENT_LOOP_BW_HZ             (1500.0f)

/* ------------------------------- Speed loop ------------------------------- */

/*
 * Speed PI gains are intentionally exposed directly because the mechanical
 * plant depends on inertia, friction and load, which are not known yet.
 */
#define MOTOR_CFG_SPEED_KP                       (0.010f)
#define MOTOR_CFG_SPEED_KI                       (0.200f)

/* --------------------------- Startup / transition ------------------------- */

/* Fixed electrical angle used during rotor alignment. */
#define MOTOR_CFG_ALIGN_ANGLE_RAD                (0.0f)

/* D-axis current applied during alignment. */
#define MOTOR_CFG_ALIGN_CURRENT_A                (0.3f)

/* Alignment duration in milliseconds. */
#define MOTOR_CFG_ALIGN_TIME_MS                  (100U)

/* Open-loop q-axis current during the startup ramp. */
#define MOTOR_CFG_OPEN_LOOP_IQ_A                 (1.0f)

/* Initial open-loop electrical speed magnitude in rad/s. */
#define MOTOR_CFG_OPEN_LOOP_START_RAD_S          (40.0f)

/* Target open-loop electrical speed magnitude before handover to observer. */
#define MOTOR_CFG_OPEN_LOOP_HANDOVER_RAD_S       (800.0f)

/* Open-loop electrical acceleration magnitude in rad/s^2. */
#define MOTOR_CFG_OPEN_LOOP_ACCEL_RAD_S2         (3000.0f)

/* Closed-loop phase blend duration in milliseconds. */
#define MOTOR_CFG_CLOSED_LOOP_BLEND_MS           (100U)

/* ------------------------- Observer / PLL parameters ---------------------- */

/* Ortega observer correction gain. Larger values converge faster but add noise. */
#define MOTOR_CFG_OBSERVER_GAIN                  (2000.0f)

/* Flux magnitude compensation bandwidth in rad/s. */
#define MOTOR_CFG_LAMBDA_COMP_BW_RAD_S           (80.0f)

/* Lower clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MIN_VS                  (0.0025f)

/* Upper clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MAX_VS                  (0.0048f)

/* PLL proportional gain. */
#define MOTOR_CFG_PLL_KP                         (400.0f)

/* PLL integral gain. */
#define MOTOR_CFG_PLL_KI                         (20000.0f)

/* ------------------------------- Derived data ----------------------------- */

#define MOTOR_CFG_FAST_LOOP_DT_S \
    (1.0f / (float)MOTOR_CFG_PWM_FREQUENCY_HZ)

#define MOTOR_CFG_SPEED_LOOP_DT_S \
    (1.0f / (float)MOTOR_CFG_SPEED_LOOP_HZ)

#define MOTOR_CFG_PWM_PERIOD_TICKS \
    ((uint32_t)(MOTOR_CFG_ETMR_CLOCK_HZ / MOTOR_CFG_PWM_FREQUENCY_HZ))

#define MOTOR_CFG_PWM_HALF_PERIOD_TICKS \
    (MOTOR_CFG_PWM_PERIOD_TICKS / 2UL)

#define MOTOR_CFG_PWM_MID_TICKS \
    (MOTOR_CFG_PWM_HALF_PERIOD_TICKS)

#define MOTOR_CFG_DEADTIME_TICKS \
    ((uint16_t)(((uint64_t)MOTOR_CFG_ETMR_CLOCK_HZ * (uint64_t)MOTOR_CFG_DEADTIME_NS) / 1000000000ULL))

#define MOTOR_CFG_CURRENT_SENSE_V_PER_A \
    (MOTOR_CFG_PHASE_SHUNT_OHM * MOTOR_CFG_CURRENT_AMP_GAIN)

#define MOTOR_CFG_ADC_COUNT_TO_VOLT \
    (MOTOR_CFG_ADC_VREF_V / MOTOR_CFG_ADC_MAX_COUNTS)

#define MOTOR_CFG_ADC_COUNT_TO_CURRENT_A \
    (MOTOR_CFG_ADC_COUNT_TO_VOLT / MOTOR_CFG_CURRENT_SENSE_V_PER_A)

#define MOTOR_CFG_VBUS_DIVIDER_GAIN \
    ((MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM + MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM) / MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM)

#define MOTOR_CFG_ADC_COUNT_TO_VBUS_V \
    (MOTOR_CFG_ADC_COUNT_TO_VOLT * MOTOR_CFG_VBUS_DIVIDER_GAIN)

#define MOTOR_CFG_CURRENT_LOOP_W_RAD_S \
    (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_CURRENT_LOOP_BW_HZ)

#define MOTOR_CFG_ID_KP_V_PER_A \
    (MOTOR_CFG_LD_H * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_ID_KI_V_PER_AS \
    (MOTOR_CFG_RS_OHM * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_IQ_KP_V_PER_A \
    (MOTOR_CFG_LQ_H * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_IQ_KI_V_PER_AS \
    (MOTOR_CFG_RS_OHM * MOTOR_CFG_CURRENT_LOOP_W_RAD_S)

#define MOTOR_CFG_MECH_RPM_TO_ELEC_RAD_S(rpm_value) \
    ((rpm_value) * MOTOR_CFG_TWO_PI_F * (float)MOTOR_CFG_POLE_PAIRS / 60.0f)

#define MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(speed_value) \
    ((speed_value) * 60.0f / (MOTOR_CFG_TWO_PI_F * (float)MOTOR_CFG_POLE_PAIRS))

#define MOTOR_CFG_USED_PWM_CHANNEL_MASK          (0xF3U)

#endif /* MOTOR_USER_CONFIG_H */
