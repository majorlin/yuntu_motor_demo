/**
 * @file motor_foc.h
 * @brief Field-Oriented Control (FOC) engine interface.
 *
 * Provides the sensorless FOC pipeline including Clarke/Park transforms,
 * Ortega nonlinear flux observer, PLL-based angle/speed estimation,
 * d-q current PI regulators, and space-vector modulation.
 */

#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Observer back-end algorithm selector.
 */
typedef enum
{
    MOTOR_OBSERVER_BACKEND_ORTEGA = 0,   /**< Ortega nonlinear flux observer. */
    MOTOR_OBSERVER_BACKEND_MXLEMMING,    /**< MxLemming observer (reserved).  */
    MOTOR_OBSERVER_BACKEND_MXV           /**< MxV observer (reserved).        */
} motor_observer_backend_t;

/**
 * @brief Alpha-beta (stationary) reference frame vector.
 */
typedef struct
{
    float alpha;  /**< Alpha-axis component. */
    float beta;   /**< Beta-axis component.  */
} motor_ab_frame_t;

/**
 * @brief D-Q (rotating) reference frame vector.
 */
typedef struct
{
    float d;  /**< Direct-axis component (flux axis).  */
    float q;  /**< Quadrature-axis component (torque axis). */
} motor_dq_frame_t;

/**
 * @brief Input structure for a single fast-loop FOC iteration.
 */
typedef struct
{
    float phase_current_a;      /**< Phase-A current in amperes.          */
    float phase_current_b;      /**< Phase-B current in amperes.          */
    float phase_current_c;      /**< Phase-C current in amperes.          */
    float bus_voltage_v;        /**< DC bus voltage in volts.             */
    float control_angle_rad;    /**< Electrical control angle in radians. */
    float id_target_a;          /**< D-axis current target in amperes.    */
    float iq_target_a;          /**< Q-axis current target in amperes.    */
    bool deadtime_comp_enable;  /**< Enable dead-time compensation flag.  */
} motor_foc_fast_input_t;

/**
 * @brief Output structure produced by a single fast-loop FOC iteration.
 */
typedef struct
{
    float duty_u;                    /**< Phase-U PWM duty cycle [0, 1].            */
    float duty_v;                    /**< Phase-V PWM duty cycle [0, 1].            */
    float duty_w;                    /**< Phase-W PWM duty cycle [0, 1].            */
    float id_a;                      /**< Measured d-axis current in amperes.        */
    float iq_a;                      /**< Measured q-axis current in amperes.        */
    float observer_angle_rad;        /**< Observer estimated electrical angle (rad). */
    float observer_speed_rad_s;      /**< Observer estimated electrical speed (rad/s). */
    float phase_error_rad;           /**< Control-angle vs observer-angle error (rad). */
    float bus_voltage_used_v;        /**< Clamped bus voltage actually used (V).     */
    motor_ab_frame_t commanded_vab_v;/**< Output alpha-beta voltage command (V).    */
    float voltage_modulation_ratio;  /**< |Vab| / Vbus ratio for field weakening.   */
} motor_foc_fast_output_t;

/**
 * @brief Persistent state for the FOC engine (observer, PLL, PI integrators).
 */
typedef struct
{
    motor_observer_backend_t backend; /**< Selected observer algorithm.                    */
    float current_pi_d_integrator_v;  /**< D-axis current PI integrator state (V).         */
    float current_pi_q_integrator_v;  /**< Q-axis current PI integrator state (V).         */
    float speed_pi_integrator_a;      /**< Speed PI integrator state (A).                  */
    float pll_phase_rad;              /**< PLL estimated electrical angle (rad).            */
    float pll_speed_rad_s;            /**< PLL estimated electrical speed (rad/s).          */
    float observer_x1;                /**< Observer state x1 (alpha-axis flux + L*i).       */
    float observer_x2;                /**< Observer state x2 (beta-axis flux + L*i).        */
    float observer_lambda_vs;         /**< Adaptive flux-linkage estimate (V·s).            */
    float phase_error_rad;            /**< Latest phase error between control and observer. */
    float last_i_alpha_a;             /**< Previous alpha current for observer (A).         */
    float last_i_beta_a;              /**< Previous beta current for observer (A).          */
    float last_v_alpha_v;             /**< Previous alpha voltage for observer (V).         */
    float last_v_beta_v;              /**< Previous beta voltage for observer (V).          */
    int8_t deadtime_comp_sign_a;      /**< Dead-time compensation sign, phase A.            */
    int8_t deadtime_comp_sign_b;      /**< Dead-time compensation sign, phase B.            */
    int8_t deadtime_comp_sign_c;      /**< Dead-time compensation sign, phase C.            */
} motor_foc_state_t;

/**
 * @brief Runtime-overridable current-loop PI gains.
 *
 * Defaults to the compile-time values from motor_user_config.h.
 * Can be updated at runtime via CAN calibration for current loop tuning.
 */
typedef struct {
    float id_kp;    /**< D-axis current PI proportional gain (V/A). */
    float id_ki;    /**< D-axis current PI integral gain (V/A/s).   */
    float iq_kp;    /**< Q-axis current PI proportional gain (V/A). */
    float iq_ki;    /**< Q-axis current PI integral gain (V/A/s).   */
} motor_foc_current_gains_t;

/**
 * @brief Initialise the FOC state to default values.
 * @param[in,out] state  Pointer to FOC state structure.
 */
void MotorFoc_Init(motor_foc_state_t *state);

/**
 * @brief Reset all FOC integrators and observer states to zero.
 * @param[in,out] state  Pointer to FOC state structure.
 */
void MotorFoc_Reset(motor_foc_state_t *state);

/**
 * @brief Override the current-loop PI gains at runtime.
 *
 * The new gains take effect from the next fast-loop iteration.
 * Pass NULL to revert to compile-time defaults.
 *
 * @param[in] gains  New gains, or NULL to restore defaults.
 */
void MotorFoc_SetCurrentGains(const motor_foc_current_gains_t *gains);

/**
 * @brief Get the currently active current-loop PI gains.
 * @param[out] gains  Pointer to store the active gains.
 */
void MotorFoc_GetCurrentGains(motor_foc_current_gains_t *gains);

/**
 * @brief Execute one fast-loop FOC iteration (observer + current PI + SVM).
 *
 * This function runs the full sensorless FOC pipeline at the PWM rate:
 * Clarke transform, flux observer, PLL, Park transform, d-q current PI,
 * inverse Park, and space-vector modulation.
 *
 * @param[in,out] state   Persistent FOC state.
 * @param[in]     input   Sampled currents, bus voltage, and targets.
 * @param[out]    output  Computed PWM duties, measured dq currents, and
 *                        observer estimates.
 */
void MotorFoc_RunFast(motor_foc_state_t *state,
                      const motor_foc_fast_input_t *input,
                      motor_foc_fast_output_t *output);

/**
 * @brief Execute one speed-loop PI iteration.
 *
 * Computes the q-axis current reference from the speed error.
 * Intended to be called at the slower speed-loop rate.
 *
 * @param[in,out] state                        Persistent FOC state.
 * @param[in]     targetElectricalSpeedRadS    Target electrical speed (rad/s).
 * @param[in]     measuredElectricalSpeedRadS  Measured electrical speed (rad/s).
 * @param[in]     resetIntegrator              If true, zero the speed integrator.
 * @return Q-axis current reference in amperes.
 */
float MotorFoc_RunSpeedPi(motor_foc_state_t *state,
                          float targetElectricalSpeedRadS,
                          float measuredElectricalSpeedRadS,
                          bool resetIntegrator);

/**
 * @brief Wrap an angle to the range [0, 2*pi).
 * @param[in] angleRad  Input angle in radians.
 * @return Wrapped angle in [0, 2*pi).
 */
float MotorFoc_WrapAngle0ToTwoPi(float angleRad);

/**
 * @brief Compute the shortest signed angle difference (target − measured).
 *
 * Both inputs are first wrapped to [0, 2*pi), and the result lies in
 * the range (−pi, pi].
 *
 * @param[in] targetAngleRad    Target angle in radians.
 * @param[in] measuredAngleRad  Measured angle in radians.
 * @return Signed angle difference in radians.
 */
float MotorFoc_AngleDiff(float targetAngleRad, float measuredAngleRad);

#endif /* MOTOR_FOC_H */
