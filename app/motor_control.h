/**
 * @file motor_control.h
 * @brief Top-level motor control state machine and public API.
 *
 * @defgroup motor_control  Motor Control State Machine
 * @{
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
    MOTOR_STATE_WIND_DETECT,     /**< Detecting rotor pre-rotation via observer.  */
    MOTOR_STATE_COAST_DOWN,      /**< Coasting with PWM off, waiting for safe speed. */
    MOTOR_STATE_ALIGN,           /**< Rotor alignment with DC current injected.   */
    MOTOR_STATE_OPEN_LOOP_RAMP,  /**< Ramping speed/current for startup tracking. */
    MOTOR_STATE_CLOSED_LOOP,     /**< Normal sensorless FOC closed-loop operation.*/
    MOTOR_STATE_FAULT,           /**< Drive faulted. Requires disable to reset.   */
    MOTOR_STATE_HFI_IPD = 10     /**< Static high-frequency initial position detect. */
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
    MOTOR_FAULT_BAD_DIRECTION = 7,     /**< Rotation requested in unsupported direction.  */
    MOTOR_FAULT_CATCH_FAIL = 8,        /**< Tailwind catch-spin failed after attempts.    */
    MOTOR_FAULT_STARTUP_FAIL = 9       /**< Startup failed after all retry attempts.      */
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
 * @brief HFI/IPD fallback cause exposed for telemetry.
 */
typedef enum
{
    MOTOR_HFI_FALLBACK_NONE = 0,             /**< HFI succeeded or was not attempted yet. */
    MOTOR_HFI_FALLBACK_DISABLED = 1,         /**< HFI feature disabled by configuration.  */
    MOTOR_HFI_FALLBACK_INVALID_VBUS = 2,     /**< Bus voltage unsuitable for HFI burst.   */
    MOTOR_HFI_FALLBACK_LOW_CONFIDENCE = 3,   /**< Candidate scan separation too small.    */
    MOTOR_HFI_FALLBACK_LOW_REPEATABILITY = 4,/**< Fine scan could not repeat the best bin. */
    MOTOR_HFI_FALLBACK_POLARITY_AMBIGUOUS = 5/**< 180-degree polarity pulse unresolved.   */
} motor_hfi_fallback_reason_t;

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
    uint8_t startup_retry_count;     /**< Number of startup retries attempted so far. */
    bool hfi_active;                 /**< True while HFI/IPD burst is running.        */
    bool hfi_angle_valid;            /**< True when HFI estimated an initial angle.   */
    bool hfi_used;                   /**< True when startup used the HFI angle.       */
    uint8_t hfi_fallback_reason;     /**< motor_hfi_fallback_reason_t raw value.      */
    float hfi_angle_rad;             /**< HFI estimated initial electrical angle.     */
    float hfi_confidence;            /**< HFI confidence / separation metric [0..1].  */
    float hfi_ripple_metric;         /**< Best synchronous current ripple metric (A). */
} motor_status_t;


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
 * @brief Get the millisecond system tick count since initialization.
 * @return System tick counter.
 */
uint32_t MotorControl_GetTickMs(void);

/**
 * @brief Get the rotor speed detected during the Wind Detect phase.
 * @return Mechanical RPM measured before startup (positive = forward).
 */
float MotorControl_GetWindDetectSpeedRpm(void);


/**
 * @brief Snapshot of FOC internal diagnostics for CAN telemetry.
 *
 * Exposes observer, PLL, and PI integrator state for real-time logging
 * and online calibration verification via the CAN FD Status2 message.
 */
typedef struct {
    float observer_angle_rad;        /**< Observer estimated electrical angle.   */
    float observer_flux_vs;          /**< Adaptive flux-linkage estimate (V·s).  */
    float pll_speed_rad_s;           /**< PLL estimated electrical speed.        */
    float phase_error_rad;           /**< Control-to-observer phase error.       */
    float speed_pi_integrator_a;     /**< Speed PI integrator state (A).         */
    float id_pi_integrator_v;        /**< D-axis current PI integrator (V).      */
    float iq_pi_integrator_v;        /**< Q-axis current PI integrator (V).      */
    float voltage_modulation_ratio;  /**< |Vab|/Vbus modulation ratio.           */
    float fw_id_target_a;            /**< Field weakening d-axis current.        */
    float open_loop_angle_rad;       /**< Forced open-loop angle.               */
    float open_loop_speed_rad_s;     /**< Open-loop forced speed.               */
    float closed_loop_blend;         /**< Closed-loop blend factor [0..1].       */
    uint32_t state_time_ms;          /**< Time in current state (ms).            */
    float obs_lock_residual_rad;     /**< Observer lock residual angle.          */
    uint16_t stall_div_count;        /**< Consecutive stall divergence count.    */
    float duty_u;                    /**< Phase U PWM duty [0..1].               */
    float duty_v;                    /**< Phase V PWM duty [0..1].               */
    float duty_w;                    /**< Phase W PWM duty [0..1].               */
    /* BEMF wind detection diagnostics */
    uint16_t bemf_u_raw;             /**< BEMF phase U raw ADC count.            */
    uint16_t bemf_v_raw;             /**< BEMF phase V raw ADC count.            */
    uint16_t bemf_w_raw;             /**< BEMF phase W raw ADC count.            */
    uint16_t bemf_com_raw;           /**< BEMF neutral raw ADC count.            */
    uint16_t bemf_crossing_count;    /**< Total zero crossings detected.         */
    int16_t  bemf_detected_rpm;      /**< Estimated mechanical RPM from BEMF.    */
    int8_t   bemf_phase_sequence;    /**< Phase sequence: +1=fwd, -1=rev, 0=unk. */
} motor_foc_diagnostics_t;

/**
 * @brief Populate a FOC diagnostics snapshot from internal state.
 *
 * Thread-safe: copies volatile data into the caller-provided struct.
 * Intended for periodic CAN telemetry (50ms cycle), not ISR context.
 *
 * @param[out] diag  Pointer to diagnostics structure to fill.
 */
void MotorControl_GetFocDiagnostics(motor_foc_diagnostics_t *diag);

/** @} */
#endif /* MOTOR_CONTROL_H */
