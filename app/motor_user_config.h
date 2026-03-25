#ifndef MOTOR_USER_CONFIG_H
#define MOTOR_USER_CONFIG_H

#include <stdint.h>

/**
 * @file motor_user_config.h
 * @brief User configuration for the YTM32 sensorless FOC stack.
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

/** 
 * @brief Auto-start motor on power up.
 * 
 * For initial hardware debugging, it is recommended to disable auto-start (set to 0)
 * to confirm sampling and PWM are functioning normally before enabling it. 
 */
#define MOTOR_APP_AUTO_START                     (0U)

/** @brief Default mechanical speed target exposed to the application interface. */
#define MOTOR_CFG_DEFAULT_TARGET_RPM             (2500.0f)

/** @brief Upper clamp for runtime speed commands coming from keys or debugger. */
#define MOTOR_CFG_MAX_TARGET_RPM                 (8500.0f)

/** @brief Default direction used at startup. Valid values are +1 or -1. */
#define MOTOR_CFG_DEFAULT_DIRECTION              (1)

/* ------------------------------ MCU clocks -------------------------------- */

/**
 * @brief eTMR base clock in Hz.
 * eTMR uses FAST_BUS_CLK on this SoC. The current board clock configuration
 * runs FAST_BUS at the full PLL rate.
 */
#define MOTOR_CFG_ETMR_CLOCK_HZ                  (120000000UL)

/**
 * @brief pTMR base clock in Hz.
 * pTMR derives its time base from SLOW_BUS_CLK through the driver.
 * With the current board clock tree that resolves to 40 MHz.
 */
#define MOTOR_CFG_PTMR_CLOCK_HZ                  (40000000UL)

/**
 * @brief ADC module clock in Hz.
 * ADC protocol clock is configured at runtime from FIRC / 3 = 32 MHz to stay
 * within the 1 MHz .. 32 MHz runtime limits enforced by the SDK.
 */
#define MOTOR_CFG_ADC_CLOCK_HZ                   (32000000UL)

/* ---------------------------- PWM / fast loop ----------------------------- */

/** @brief PWM switching frequency. Higher values reduce current ripple but increase CPU load. */
#define MOTOR_CFG_PWM_FREQUENCY_HZ               (20000UL)

/** @brief Speed loop frequency driven by pTMR0_CH0. */
#define MOTOR_CFG_SPEED_LOOP_HZ                  (1000UL)

/** @brief Requested deadtime between complementary switches, in nanoseconds. */
#define MOTOR_CFG_DEADTIME_NS                    (500UL)

/** @brief Enable CM33 DWT-based timing statistics for the fast loop. */
#define MOTOR_CFG_ENABLE_DWT_PROFILE             (0U)

/** @brief Enable deadtime compensation in the FOC duty output path. */
#ifndef MOTOR_CFG_ENABLE_DEADTIME_COMP
#define MOTOR_CFG_ENABLE_DEADTIME_COMP           (0U)
#endif

/** @brief Gain applied to the base deadtime duty compensation. */
#ifndef MOTOR_CFG_DEADTIME_COMP_GAIN
#define MOTOR_CFG_DEADTIME_COMP_GAIN             (1.0f)
#endif

/** @brief Minimum phase current magnitude required to refresh the compensation sign. */
#ifndef MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A
#define MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A    (0.4f)
#endif

/**
 * @brief Alpha/beta voltage modulation limit used by the SVM routine. 
 * This is a conservative value expressed in V_alpha/V_beta per-unit of bus voltage.
 */
#define MOTOR_CFG_SVM_MAX_MODULATION             (0.55f)

/* --------------------------- ADC / analog front end ----------------------- */

/** @brief ADC reference voltage. The current hardware is 5V. */
#define MOTOR_CFG_ADC_VREF_V                     (5.0f)

/** @brief ADC maximum raw count in 12-bit mode. */
#define MOTOR_CFG_ADC_MAX_COUNTS                 (4095.0f)

/**
 * @brief Phase current shunt resistor in ohms. 
 * Replace with the actual board value. Typical low-voltage motor boards use 2 mOhm .. 10 mOhm.
 */
#define MOTOR_CFG_PHASE_SHUNT_OHM                (0.005f)

/**
 * @brief Current-sense amplifier gain. 
 * Replace with the real analog front-end gain.
 */
#define MOTOR_CFG_CURRENT_AMP_GAIN               (10.0f)

/**
 * @brief Bus voltage divider top resistor (from bus to ADC input) in ohms.
 * Replace with the real board divider values.
 */
#define MOTOR_CFG_VBUS_DIVIDER_R_TOP_OHM         (4000.0f)
/** @brief Bus voltage divider bottom resistor (from ADC input to ground) in ohms. */
#define MOTOR_CFG_VBUS_DIVIDER_R_BOTTOM_OHM      (1000.0f)

/**
 * @brief Number of software-triggered ADC frames used to calibrate phase-current
 * offsets before enabling PWM.
 */
#define MOTOR_CFG_OFFSET_CAL_SAMPLES             (512U)

/* ------------------------------- Protection ------------------------------- */

/** @brief Absolute software current fault threshold in amps. */
#define MOTOR_CFG_PHASE_OVERCURRENT_A            (10.0f)

/** @brief DC bus under-voltage threshold in volts. */
#define MOTOR_CFG_VBUS_UNDERVOLTAGE_V            (7.0f)

/** 
 * @brief DC bus over-voltage threshold in volts.
 * 
 * Note: A 4k/1k divider combined with a 5V ADC full-scale range can only measure 
 * up to 25V bus voltage. Therefore, the software overvoltage threshold must be 
 * strictly less than 25V. If the system bus is higher, the hardware divider must 
 * be adjusted first.
 */
#define MOTOR_CFG_VBUS_OVERVOLTAGE_V             (24.0f)

/** @brief Startup timeout before declaring a fault, in milliseconds. */
#define MOTOR_CFG_STARTUP_TIMEOUT_MS             (300U)

/** @brief Allowed observer phase error during acquisition, in electrical radians. */
#define MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD    (0.45f)

/** @brief Observer phase error above this value counts as lost lock. */
#define MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD    (0.90f)

/** @brief Consecutive 1 kHz speed-loop samples required to accept observer lock. */
#define MOTOR_CFG_OBSERVER_LOCK_COUNT            (80U)

/** @brief Consecutive 1 kHz speed-loop samples that trigger observer-loss fault. */
#define MOTOR_CFG_OBSERVER_LOSS_COUNT            (200U)

/** @brief Minimum electrical speed magnitude required before observer lock is accepted. */
#define MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S      (300.0f)

/* --------------------------- Motor electrical model ----------------------- */

/**
 * @brief Mechanical pole-pair count. 
 * This only affects the interface layer and the speed conversion between RPM and 
 * electrical radians/second.
 */
#define MOTOR_CFG_POLE_PAIRS                     (4U)

/** @brief Stator phase resistance in ohms. */
#define MOTOR_CFG_RS_OHM                         (0.050f)

/** @brief D-axis inductance in henries. */
#define MOTOR_CFG_LS_H                           (0.00045158f)

/** @brief During initial debug, assume surface-mount (SPM) motor without salience: Ld = Lq = Ls. */
#define MOTOR_CFG_LD_H                           (MOTOR_CFG_LS_H)

/** @brief Q-axis inductance in henries. */
#define MOTOR_CFG_LQ_H                           (MOTOR_CFG_LS_H)

/**
 * @brief Permanent-magnet flux linkage in volt-seconds/rad. 
 * This is a template value used by the observer and should be replaced by motor-specific data.
 * 
 * Flux estimation from BEMF measurement via back-driving the motor:
 * AB line-to-line voltage peak-to-peak is 28.6V at an electrical frequency of 385Hz.
 * For sine waves, phase_peak = Vab_pp / (2 * sqrt(3)).
 * Expected lambda = phase_peak / omega_electrical.
 */
#define MOTOR_CFG_BEMF_AB_LINE_PP_V              (28.6f)
#define MOTOR_CFG_BEMF_ELEC_FREQ_HZ              (385.0f)
#define MOTOR_CFG_BEMF_PHASE_PEAK_V \
    ((0.5f * MOTOR_CFG_BEMF_AB_LINE_PP_V) * MOTOR_CFG_INV_SQRT3_F)
#define MOTOR_CFG_FLUX_LINKAGE_VS \
    (MOTOR_CFG_BEMF_PHASE_PEAK_V / (MOTOR_CFG_TWO_PI_F * MOTOR_CFG_BEMF_ELEC_FREQ_HZ))

/** @brief Maximum commanded q-axis current during closed-loop operation. */
#define MOTOR_CFG_MAX_IQ_A                       (6.0f)

/** @brief Default q-axis current target used by the debugger-facing current mode. */
#define MOTOR_CFG_DEFAULT_TARGET_IQ_A            (0.5f)

/* ------------------------------ Current loop ------------------------------ */

/**
 * @brief Current-loop bandwidth in Hz. 
 * The PI gains below are derived from the motor R/L model using a standard 
 * continuous-time approximation.
 */
#define MOTOR_CFG_CURRENT_LOOP_BW_HZ             (1500.0f)

/* ------------------------------- Speed loop ------------------------------- */

/**
 * @brief Speed PI proportional and integral gains.
 * These are intentionally exposed directly because the mechanical
 * plant depends on inertia, friction and load, which are not known yet.
 */
#define MOTOR_CFG_SPEED_KP                       (0.006f)
#define MOTOR_CFG_SPEED_KI                       (0.100f)

/** @brief Runtime speed-command ramp to avoid large torque steps when changing rpm. */
#define MOTOR_CFG_SPEED_RAMP_RPM_PER_S           (600.0f)

/* --------------------------- Startup / transition ------------------------- */

/** @brief Fixed electrical angle used during rotor alignment. */
#define MOTOR_CFG_ALIGN_ANGLE_RAD                (0.0f)

/** @brief D-axis current applied during alignment. */
#define MOTOR_CFG_ALIGN_CURRENT_A                (0.5f)

/** @brief Alignment duration in milliseconds. */
#define MOTOR_CFG_ALIGN_TIME_MS                  (100U)

/** @brief Open-loop q-axis current target. It ramps up from the alignment current magnitude. */
#define MOTOR_CFG_OPEN_LOOP_IQ_A                 (3.0f)

/** @brief Total time for the open-loop current/speed ramp. */
#define MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS         (200U)

/** @brief Final open-loop electrical speed magnitude before handover to observer. */
#define MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S          (800.0f)

/** @brief Closed-loop phase blend duration in milliseconds. */
#define MOTOR_CFG_CLOSED_LOOP_BLEND_MS           (100U)

/* ========================= Wind Detect / Catch Spin ======================== */

/** @brief Enable wind detect before alignment (0 = disabled, traditional Align flow). */
#define MOTOR_CFG_ENABLE_WIND_DETECT             (1U)

/** @brief Wind detect observer settling timeout (ms). */
#define MOTOR_CFG_WIND_DETECT_TIMEOUT_MS         (500U)

/** @brief Electrical speed below which the rotor is considered standstill (rad/s). */
#define MOTOR_CFG_WIND_DETECT_STANDSTILL_RAD_S   (30.0f)

/** @brief Number of consecutive speed-loop samples with stable speed to declare convergence. */
#define MOTOR_CFG_WIND_DETECT_SETTLE_COUNT       (50U)

/** @brief Maximum speed change per speed-loop tick for convergence check (rad/s). */
#define MOTOR_CFG_WIND_DETECT_SPEED_TOL_RAD_S    (20.0f)

/** @brief Maximum mechanical RPM for direct tailwind catch. Above this, brake first. */
#define MOTOR_CFG_CATCH_MAX_SPEED_RPM            (6000.0f)

/** @brief Braking q-axis current magnitude for headwind deceleration (A). */
#define MOTOR_CFG_HEADWIND_BRAKE_IQ_A            (2.0f)

/** @brief Electrical speed threshold to stop braking and enter Align (rad/s). */
#define MOTOR_CFG_HEADWIND_BRAKE_STOP_RAD_S      (50.0f)

/* ============================== Field Weakening ============================ */

/** @brief Enable field weakening control (0 = disabled, id_target stays 0). */
#define MOTOR_CFG_ENABLE_FIELD_WEAKENING         (1U)

/** @brief Voltage modulation ratio threshold to start injecting negative Id (0~1). */
#define MOTOR_CFG_FW_VOLTAGE_THRESHOLD           (0.50f)

/** @brief Field weakening integral gain. Larger = faster response but may oscillate. */
#define MOTOR_CFG_FW_KI                          (50.0f)

/** @brief Maximum negative Id current for field weakening (A). Must be negative. */
#define MOTOR_CFG_FW_MAX_NEGATIVE_ID_A           (-4.0f)

/** @brief Recovery rate (A/s) for Id returning to zero when voltage margin restores. */
#define MOTOR_CFG_FW_RECOVERY_RATE               (20.0f)

/* ------------------------- Observer / PLL parameters ---------------------- */

/** @brief Ortega observer correction gain. Larger values converge faster but add noise. */
#define MOTOR_CFG_OBSERVER_GAIN                  (2.0e6f)

/** @brief Flux magnitude compensation bandwidth in rad/s. */
#define MOTOR_CFG_LAMBDA_COMP_BW_RAD_S           (80.0f)

/** @brief Lower clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MIN_VS                  (0.0025f)

/** @brief Upper clamp for estimated flux linkage. */
#define MOTOR_CFG_LAMBDA_MAX_VS                  (0.0048f)

/** @brief PLL proportional gain. */
#define MOTOR_CFG_PLL_KP                         (400.0f)

/** @brief PLL integral gain. */
#define MOTOR_CFG_PLL_KI                         (20000.0f)

/** @brief Tracker bandwidth (rad/s) for holding fixed phase offset during open-loop to closed-loop blending. */
#define MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S  (40.0f)

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

#define MOTOR_CFG_DEADTIME_COMP_DUTY \
    (((float)MOTOR_CFG_DEADTIME_TICKS / (float)MOTOR_CFG_PWM_PERIOD_TICKS) * MOTOR_CFG_DEADTIME_COMP_GAIN)

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
