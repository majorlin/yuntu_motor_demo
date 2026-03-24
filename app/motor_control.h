/**
 * @file motor_control.h
 * @brief Top-level motor control state machine and public API.
 *
 * Manages the high-level states of the motor drive (Stop, Align, 
 * Open-Loop Ramp, Closed-Loop, Fault), exposes the control interfaces
 * for speed and current, and handles profiling statistics.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief High-level motor control states.
 */
typedef enum
{
    MOTOR_STATE_STOP = 0,        /**< Motor stopped, PWM disabled.                */
    MOTOR_STATE_OFFSET_CAL,      /**< Calibrating ADC phase-current offsets.      */
    MOTOR_STATE_ALIGN,           /**< Rotor alignment with DC current injected.   */
    MOTOR_STATE_OPEN_LOOP_RAMP,  /**< Ramping speed/current for startup tracking. */
    MOTOR_STATE_CLOSED_LOOP,     /**< Normal sensorless FOC closed-loop operation.*/
    MOTOR_STATE_FAULT            /**< Drive faulted. Requires disable to reset.   */
} motor_control_state_t;

/**
 * @brief Supported motor fault conditions.
 */
typedef enum
{
    MOTOR_FAULT_NONE = 0,              /**< No fault present.                             */
    MOTOR_FAULT_ADC_OVERRUN = 1,       /**< Hardware ADC FIFO overrun detected.           */
    MOTOR_FAULT_OVERCURRENT = 2,       /**< Absolute phase current exceeded soft limit.   */
    MOTOR_FAULT_VBUS_UNDERVOLTAGE = 3, /**< DC bus voltage dropped below minimum bound.   */
    MOTOR_FAULT_VBUS_OVERVOLTAGE = 4,  /**< DC bus voltage exceeded maximum bound.        */
    MOTOR_FAULT_OBSERVER_LOSS = 5,     /**< Observer phase error exceeded acceptable limit.*/
    MOTOR_FAULT_STARTUP_TIMEOUT = 6,   /**< Open-loop transitions failed to achieve lock. */
    MOTOR_FAULT_BAD_DIRECTION = 7      /**< Rotation requested in unsupported direction.  */
} motor_fault_t;

/**
 * @brief Operating modes for the top-level FOC loop.
 */
typedef enum
{
    MOTOR_CONTROL_MODE_SPEED = 0,      /**< Outer loop tracks a target RPM.          */
    MOTOR_CONTROL_MODE_CURRENT         /**< Direct q-axis current reference control. */
} motor_control_mode_t;

/**
 * @brief Run-time status observability structure.
 */
typedef struct
{
    motor_control_state_t state;     /**< Current running state machine stage.        */
    motor_fault_t fault;             /**< Active hardware or software fault code.     */
    motor_control_mode_t control_mode;/**< Active (Speed or Current) control mode.    */
    bool observer_locked;            /**< Flag indicating if observer PLL is trusted. */
    bool enabled;                    /**< Flag reflecting the user run enable signal. */
    int8_t direction;                /**< Requested rotation direction (+1 or -1).    */
    float electrical_angle_rad;      /**< Current electrical angle used (rad) [0-2PI).*/
    float electrical_speed_rad_s;    /**< Observed electrical speed (rad/s).          */
    float mechanical_rpm;            /**< Estimated mechanical speed in RPM.          */
    float phase_current_a;           /**< Filtered/measured Phase A current (A).      */
    float phase_current_b;           /**< Filtered/measured Phase B current (A).      */
    float phase_current_c;           /**< Filtered/measured Phase C current (A).      */
    float bus_voltage_v;             /**< Measured DC bus voltage (V).                */
    float id_a;                      /**< Measured D-axis current (A).                */
    float iq_a;                      /**< Measured Q-axis current (A).                */
    float id_target_a;               /**< Target commanded D-axis current (A).        */
    float iq_target_a;               /**< Target commanded Q-axis current (A).        */
    float target_rpm;                /**< Currently ramped/commanded target speed.    */
} motor_status_t;

/**
 * @brief Accumulator structure for timing statistics.
 */
typedef struct
{
    uint32_t last_cycles;            /**< CPU tick count for the last iteration.      */
    uint32_t min_cycles;             /**< Minimum recorded CPU tick count.            */
    uint32_t max_cycles;             /**< Maximum recorded CPU tick count.            */
    uint32_t avg_cycles;             /**< Exponentially smoothed average tick count.  */
    uint64_t total_cycles;           /**< Cumulative total block time across samples. */
    uint32_t sample_count;           /**< Measurement count since last reset.         */
} motor_cycle_stat_t;

/**
 * @brief Root structure for high-resolution CPU timing breakdowns.
 */
typedef struct
{
    bool dwt_enabled;                    /**< Flag if the DWT hardware is enabled.   */
    uint32_t core_clock_hz;              /**< Core clock speed reference.            */
    uint32_t fast_loop_hz;               /**< Expected fast loop trigger frequency.  */
    motor_cycle_stat_t adc_irq_total;    /**< Total duration of the ADC interrupt.   */
    motor_cycle_stat_t foc_total;        /**< Entire FOC math pipeline duration.     */
    motor_cycle_stat_t foc_observer_pll; /**< Observer and PLL section execution.    */
    motor_cycle_stat_t foc_current_loop; /**< Park to SVM Current-Loop execution.    */
    motor_cycle_stat_t foc_svm;          /**< Space-Vector modulation calculation.   */
    motor_cycle_stat_t pwm_update;       /**< Hardware PWM register update transfer. */
} motor_fast_loop_profile_t;

/**
 * @brief Global pointer for debugging / profiling statistics.
 */
extern volatile motor_fast_loop_profile_t g_motorFastLoopProfile;

/**
 * @brief Initialize motor control structures and hardware logic.
 *
 * Should be called once at startup. Resets control states and configures 
 * necessary hardware timers and ADC interfaces via the HAL abstraction.
 */
void MotorControl_Init(void);

/**
 * @brief Enable or disable the motor drive.
 *
 * Transitioning to enable begins the startup sequence (calibration, align,
 * open-loop ramp, observer handover). Transitioning to disable halts PWM
 * and resets all internal integrators and timers.
 *
 * @param enable True to run motor, False to stop and mask PWM.
 */
void MotorControl_Enable(bool enable);

/**
 * @brief Change motor control tracking reference mode.
 *
 * Swaps PI structure between outer speed loop and torque (current) targets.
 *
 * @param mode Target mode enum (Speed or Current).
 * @return True if mode was applied, False otherwise.
 */
bool MotorControl_SetControlMode(motor_control_mode_t mode);

/**
 * @brief Issue a new target speed command for the speed loop.
 *
 * Updates internal ramper. Command is bounded by hardware capabilities.
 *
 * @param targetRpm Absolute target mechanical speed.
 * @return True if accepted.
 */
bool MotorControl_SetTargetRpm(float targetRpm);

/**
 * @brief Issue a new target torque step command.
 *
 * @param targetIqA Target Q-axis current in amperes.
 * @return True if accepted.
 */
bool MotorControl_SetTargetIqA(float targetIqA);

/**
 * @brief Set the desired rotation direction.
 *
 * @param direction Direction polarity (+1 for forward, -1 for reverse).
 * @return True if direction accepted (must be stopped), False otherwise.
 */
bool MotorControl_SetDirection(int8_t direction);

/**
 * @brief Read the real-time observability structural state of the motor.
 *
 * @return Pointer to read-only motor status block.
 */
const motor_status_t *MotorControl_GetStatus(void);

/**
 * @brief Read FOC fast-loop CPU profiling statistics.
 *
 * @return Pointer to read-only cycle stat tracing data.
 */
const volatile motor_fast_loop_profile_t *MotorControl_GetFastLoopProfile(void);

/**
 * @brief Clear accumulated statistical cycles trace structures.
 */
void MotorControl_ResetFastLoopProfile(void);

/**
 * @brief Get the millisecond system tick count since initialization.
 * @return System tick counter.
 */
uint32_t MotorControl_GetTickMs(void);

#endif /* MOTOR_CONTROL_H */
