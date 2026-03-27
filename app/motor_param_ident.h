/**
 * @file motor_param_ident.h
 * @brief Motor parameter auto-identification module (standalone).
 *
 * Performs offline static identification of electrical parameters:
 *   Phase 1: Rs  — Bidirectional DC injection
 *   Phase 2: Ls  — Voltage step response
 *   Phase 3: λ   — Low-speed open-loop rotation
 *
 * Design principles:
 *   - Self-contained: does NOT depend on motor_control.h or motor_foc.h.
 *   - Only requires the hardware abstraction layer for duty output.
 *   - All FOC math (Clarke, Park, SinCos, SVM) is internal.
 *   - The host state machine calls the public API; this module never
 *     calls back into the host.
 */

#ifndef MOTOR_PARAM_IDENT_H
#define MOTOR_PARAM_IDENT_H

#include <stdbool.h>
#include <stdint.h>

/* ── Identification phase enumeration ── */

typedef enum
{
    MOTOR_IDENT_PHASE_IDLE = 0,   /**< Not running.                        */
    MOTOR_IDENT_PHASE_RS,         /**< Measuring stator resistance.        */
    MOTOR_IDENT_PHASE_LS,         /**< Measuring stator inductance.        */
    MOTOR_IDENT_PHASE_LAMBDA,     /**< Measuring PM flux linkage.          */
    MOTOR_IDENT_PHASE_COMPLETE,   /**< All measurements done successfully. */
    MOTOR_IDENT_PHASE_DRAG_FAIL,  /**< Drag failed, retrying with more Iq. */
    MOTOR_IDENT_PHASE_ERROR       /**< Identification failed / aborted.    */
} motor_ident_phase_t;

/* ── Identification results ── */

typedef struct
{
    float rs_ohm;       /**< Identified stator resistance (Ω).         */
    float ls_h;         /**< Identified stator inductance (H).         */
    float lambda_vs;    /**< Identified PM flux linkage (V·s).         */
    bool  rs_valid;     /**< Rs measurement succeeded.                 */
    bool  ls_valid;     /**< Ls measurement succeeded.                 */
    bool  lambda_valid; /**< Lambda measurement succeeded.             */
} motor_ident_result_t;

/* ── Identification configuration ── */

/**
 * @brief Configuration passed at start.
 *
 * The caller provides operating constants so that this module does not
 * depend on any project-specific config header.
 */
typedef struct
{
    /* Test excitation levels */
    float test_current_a;        /**< DC injection current for Rs (A).       */
    float voltage_step_v;        /**< Voltage step amplitude for Ls (V).     */

    /* Lambda identification — speed */
    uint8_t pole_pairs;          /**< Motor pole pairs (for RPM → rad/s).    */
    float target_mech_rpm;       /**< Desired mechanical test RPM for λ.     */

    /* Lambda identification — adaptive current */
    float lambda_iq_min_a;       /**< Initial Q-axis current for λ (A).      */
    float lambda_iq_max_a;       /**< Maximum Q-axis current for λ (A).      */
    float lambda_iq_step_a;      /**< Iq increment per drag retry (A).       */

    /* System parameters */
    float pwm_dt_s;              /**< Fast-loop period (s), e.g. 1/20000.    */
    float slow_dt_s;             /**< Slow-loop period (s), e.g. 1/1000.     */
    float max_duty_modulation;   /**< SVM max modulation (0-1), e.g. 0.55.   */
    float overcurrent_a;         /**< Software overcurrent limit (A).        */
} motor_ident_config_t;

/* ── Fast-loop I/O (called from ADC ISR) ── */

/**
 * @brief Input data for one fast-loop iteration.
 */
typedef struct
{
    float phase_current_a;  /**< Calibrated Phase-A current (A). */
    float phase_current_b;  /**< Calibrated Phase-B current (A). */
    float phase_current_c;  /**< Calibrated Phase-C current (A). */
    float bus_voltage_v;    /**< DC bus voltage (V).             */
} motor_ident_fast_input_t;

/**
 * @brief Output data from one fast-loop iteration.
 */
typedef struct
{
    float duty_u;  /**< Phase-U PWM duty [0, 1]. */
    float duty_v;  /**< Phase-V PWM duty [0, 1]. */
    float duty_w;  /**< Phase-W PWM duty [0, 1]. */
} motor_ident_fast_output_t;

/* ── Telemetry snapshot (for CAN / debug) ── */

typedef struct
{
    motor_ident_phase_t phase;      /**< Current identification phase.     */
    uint8_t             progress;   /**< Phase progress 0-100%.            */
    float               rs_ohm;     /**< Current Rs estimate.              */
    float               ls_h;       /**< Current Ls estimate.              */
    float               lambda_vs;  /**< Current λ estimate.               */
    float               id_measured; /**< Instantaneous d-axis current (A). */
    float               iq_measured; /**< Instantaneous q-axis current (A). */
    float               vd_command;  /**< Commanded d-axis voltage (V).     */
    float               vq_command;  /**< Commanded q-axis voltage (V).     */
    bool                drag_ok;     /**< True if forced drag is verified.  */
    uint8_t             drag_retry;  /**< Number of drag retries attempted. */
    float               lambda_iq_now; /**< Current λ Iq level (A).         */
} motor_ident_telemetry_t;

/* ══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialise the identification module to idle state.
 */
void MotorParamIdent_Init(void);

/**
 * @brief Start parameter identification with the given configuration.
 *
 * The module transitions from IDLE to RS phase.  The caller must ensure
 * that PWM outputs are unmasked and the motor is stationary.
 *
 * @param[in] config  Pointer to configuration.  Contents are copied.
 * @return true if started, false if already running or config is NULL.
 */
bool MotorParamIdent_Start(const motor_ident_config_t *config);

/**
 * @brief Abort a running identification and return to IDLE.
 *
 * Outputs 50% duty (safe state) on the next fast-loop call.
 */
void MotorParamIdent_Abort(void);

/**
 * @brief Execute one fast-loop iteration (call from ADC ISR).
 *
 * Reads calibrated phase currents and bus voltage, outputs PWM duties.
 * Must be called at the PWM rate while identification is active.
 *
 * @param[in]  input   Sampled currents and bus voltage.
 * @param[out] output  Computed PWM duty cycles.
 */
void MotorParamIdent_RunFastLoop(const motor_ident_fast_input_t *input,
                                 motor_ident_fast_output_t *output);

/**
 * @brief Execute one slow-loop iteration (call from 1 kHz timer).
 *
 * Manages phase transitions, timing, and accumulates results.
 */
void MotorParamIdent_RunSlowLoop(void);

/**
 * @brief Get the current identification phase.
 * @return Current phase enum value.
 */
motor_ident_phase_t MotorParamIdent_GetPhase(void);

/**
 * @brief Get the identification results.
 *
 * Results are valid only after the phase reaches COMPLETE.
 *
 * @return Pointer to read-only results structure.
 */
const motor_ident_result_t *MotorParamIdent_GetResults(void);

/**
 * @brief Fill a telemetry snapshot for CAN / debug display.
 * @param[out] telem  Pointer to telemetry structure to fill.
 */
void MotorParamIdent_GetTelemetry(motor_ident_telemetry_t *telem);

#endif /* MOTOR_PARAM_IDENT_H */
