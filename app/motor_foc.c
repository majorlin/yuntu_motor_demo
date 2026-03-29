/**
 * @file motor_foc.c
 * @brief Field-Oriented Control (FOC) engine implementation.
 *
 * Implements the sensorless FOC pipeline:
 *   - Clarke / Park transforms
 *   - Ortega nonlinear flux observer with adaptive flux tracking
 *   - PLL-based angle and speed estimation
 *   - D/Q-axis current PI regulators with anti-windup
 *   - Inverse Park + Space-Vector Modulation (SVM)
 *   - Speed-loop PI controller
 *
 * @defgroup motor_foc  FOC Engine
 * @{
 */

#include "motor_foc.h"

#include <stddef.h>

#include "motor_math.h"
#include "motor_user_config.h"

/** @brief PLL speed integrator saturation limit (rad/s). */
#define MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S  (120000.0f)


/* ────────────────────────────────────────────────────────────────────────── */
/*  Clarke transform                                                         */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Clarke transform (3-phase -> alpha-beta stationary frame).
 *
 * Amplitude-invariant form:
 *   i_alpha = ia
 *   i_beta  = (ia + 2*ib) / sqrt(3)
 *
 * @param[in]  ia   Phase-A current (A).
 * @param[in]  ib   Phase-B current (A).
 * @param[in]  ic   Phase-C current (A) — unused, balanced assumption.
 * @param[out] iab  Alpha-beta output.
 */
static inline void MotorFoc_Clarke(float ia, float ib, float ic,
                                   motor_ab_frame_t *iab)
{
    (void)ic;
    iab->alpha = ia;
    iab->beta  = (ia + ib + ib) * MOTOR_CFG_INV_SQRT3_F;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Park / Inverse Park transforms                                           */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Park transform (alpha-beta -> d-q rotating frame).
 *
 * @param[in]  iab       Stationary-frame current.
 * @param[in]  sinAngle  sin(theta_e).
 * @param[in]  cosAngle  cos(theta_e).
 * @param[out] idq       Rotating-frame current.
 */
static inline void MotorFoc_Park(const motor_ab_frame_t *iab,
                                 float sinAngle, float cosAngle,
                                 motor_dq_frame_t *idq)
{
    idq->d =  (iab->alpha * cosAngle) + (iab->beta * sinAngle);
    idq->q = -(iab->alpha * sinAngle) + (iab->beta * cosAngle);
}

/**
 * @brief Inverse Park transform (d-q -> alpha-beta stationary frame).
 *
 * @param[in]  vdq       Rotating-frame voltage command.
 * @param[in]  sinAngle  sin(theta_e).
 * @param[in]  cosAngle  cos(theta_e).
 * @param[out] vab       Stationary-frame voltage output.
 */
static inline void MotorFoc_InvPark(const motor_dq_frame_t *vdq,
                                    float sinAngle, float cosAngle,
                                    motor_ab_frame_t *vab)
{
    vab->alpha = (vdq->d * cosAngle) - (vdq->q * sinAngle);
    vab->beta  = (vdq->d * sinAngle) + (vdq->q * cosAngle);
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Space-Vector Modulation                                                  */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Mid-clamp (centred) space-vector modulation.
 *
 * Converts alpha-beta voltage to three-phase PWM duty cycles [0, 1].
 * Zero-sequence injection: offset = 0.5 - 0.5*(max + min).
 *
 * @param[in]  vab          Commanded alpha-beta voltage (V).
 * @param[in]  busVoltageV  DC bus voltage (V).
 * @param[out] dutyU        Phase-U duty [0, 1].
 * @param[out] dutyV        Phase-V duty [0, 1].
 * @param[out] dutyW        Phase-W duty [0, 1].
 */
static inline void MotorFoc_SpaceVector(const motor_ab_frame_t *vab,
                                        float busVoltageV,
                                        float *dutyU, float *dutyV,
                                        float *dutyW)
{
    const float invBus = 1.0f / busVoltageV;
    const float pu = vab->alpha * invBus;
    const float pv = ((-0.5f * vab->alpha) + (MOTOR_CFG_SQRT3_BY_2_F * vab->beta)) * invBus;
    const float pw = ((-0.5f * vab->alpha) - (MOTOR_CFG_SQRT3_BY_2_F * vab->beta)) * invBus;
    const float vMax   = MotorMath_Max3(pu, pv, pw);
    const float vMin   = MotorMath_Min3(pu, pv, pw);
    const float offset = 0.5f - (0.5f * (vMax + vMin));

    *dutyU = MotorMath_Clamp(pu + offset, 0.0f, 1.0f);
    *dutyV = MotorMath_Clamp(pv + offset, 0.0f, 1.0f);
    *dutyW = MotorMath_Clamp(pw + offset, 0.0f, 1.0f);
}


/* ════════════════════════════════════════════════════════════════════════════
 *  Public API
 * ════════════════════════════════════════════════════════════════════════════ */

void MotorFoc_Init(motor_foc_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->backend = MOTOR_OBSERVER_BACKEND_ORTEGA;
    MotorFoc_Reset(state);
}

void MotorFoc_Reset(motor_foc_state_t *state)
{
    if (state == NULL)
    {
        return;
    }

    state->current_pi_d_integrator_v = 0.0f;
    state->current_pi_q_integrator_v = 0.0f;
    state->speed_pi_integrator_a     = 0.0f;
    state->pll_phase_rad             = 0.0f;
    state->pll_speed_rad_s           = 0.0f;
    state->observer_x1               = 0.0f;
    state->observer_x2               = 0.0f;
    state->observer_lambda_vs        = MOTOR_CFG_FLUX_LINKAGE_VS;
    state->phase_error_rad           = 0.0f;
    state->last_i_alpha_a            = 0.0f;
    state->last_i_beta_a             = 0.0f;
    state->last_v_alpha_v            = 0.0f;
    state->last_v_beta_v             = 0.0f;
}

void MotorFoc_RunFast(motor_foc_state_t *state,
                      const motor_foc_fast_input_t *input,
                      motor_foc_fast_output_t *output)
{
    const float dt = MOTOR_CFG_FAST_LOOP_DT_S;
    const float busV = MotorMath_Clamp(input->bus_voltage_v, 1.0f, 1000.0f);
    const float limitV   = busV * MOTOR_CFG_SVM_MAX_MODULATION;
    const float limitSq  = limitV * limitV;
    const float obsL     = 0.5f * (MOTOR_CFG_LD_H + MOTOR_CFG_LQ_H);
    const float lambdaNow = MotorMath_Clamp(state->observer_lambda_vs,
                                            MOTOR_CFG_LAMBDA_MIN_VS,
                                            MOTOR_CFG_LAMBDA_MAX_VS);

    motor_ab_frame_t iab;
    motor_dq_frame_t idq;
    motor_dq_frame_t vdq;
    motor_ab_frame_t vab;
    float fluxA, fluxB, fluxSq, obsErr, fluxMag;
    float measAngle, pllErr, pllInt, obsSpeed;
    float idErr, iqErr;
    float sinCtrl, cosCtrl;
    float vMagSq, vScale;


    /* ── Clarke: 3-phase -> alpha-beta ── */
    MotorFoc_Clarke(input->phase_current_a, input->phase_current_b,
                    input->phase_current_c, &iab);

    /* ── Ortega nonlinear flux observer ── */
    fluxA  = state->observer_x1 - (obsL * iab.alpha);
    fluxB  = state->observer_x2 - (obsL * iab.beta);
    fluxSq = (fluxA * fluxA) + (fluxB * fluxB);
    obsErr = (lambdaNow * lambdaNow) - fluxSq;
    if (obsErr > 0.0f) { obsErr = 0.0f; }

    state->observer_x1 += dt * (state->last_v_alpha_v
                                - (MOTOR_CFG_RS_OHM * iab.alpha)
                                + (MOTOR_CFG_OBSERVER_GAIN * fluxA * obsErr));
    state->observer_x2 += dt * (state->last_v_beta_v
                                - (MOTOR_CFG_RS_OHM * iab.beta)
                                + (MOTOR_CFG_OBSERVER_GAIN * fluxB * obsErr));

    fluxA   = state->observer_x1 - (obsL * iab.alpha);
    fluxB   = state->observer_x2 - (obsL * iab.beta);
    fluxMag = MotorMath_Sqrt((fluxA * fluxA) + (fluxB * fluxB));

    /* Adaptive flux-linkage tracking */
    state->observer_lambda_vs +=
        MotorMath_Clamp(MOTOR_CFG_LAMBDA_COMP_BW_RAD_S * dt, 0.0f, 1.0f) *
        (MotorMath_Clamp(fluxMag, MOTOR_CFG_LAMBDA_MIN_VS, MOTOR_CFG_LAMBDA_MAX_VS)
         - state->observer_lambda_vs);

    /* ── PLL angle / speed estimation ── */
    measAngle = MotorMath_WrapAngle0To2Pi(MotorMath_Atan2(fluxB, fluxA));
    pllErr    = MotorMath_AngleDiff(measAngle, state->pll_phase_rad);
    pllInt    = state->pll_speed_rad_s + (MOTOR_CFG_PLL_KI * pllErr * dt);
    pllInt    = MotorMath_Clamp(pllInt,
                                -MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S,
                                 MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S);
    obsSpeed  = pllInt + (MOTOR_CFG_PLL_KP * pllErr);

    state->pll_speed_rad_s = pllInt;
    state->pll_phase_rad   = MotorMath_WrapAngle0To2Pi(
                                 state->pll_phase_rad + (obsSpeed * dt));
    state->phase_error_rad = pllErr;


    /* ── Park transform ── */
    MotorMath_SinCos(input->control_angle_rad, &sinCtrl, &cosCtrl);
    MotorFoc_Park(&iab, sinCtrl, cosCtrl, &idq);

    /* ── Current-loop PI (d-axis and q-axis) ── */
    idErr = input->id_target_a - idq.d;
    iqErr = input->iq_target_a - idq.q;

    state->current_pi_d_integrator_v += MOTOR_CFG_ID_KI_V_PER_AS * dt * idErr;
    state->current_pi_q_integrator_v += MOTOR_CFG_IQ_KI_V_PER_AS * dt * iqErr;

    vdq.d = state->current_pi_d_integrator_v + (MOTOR_CFG_ID_KP_V_PER_A * idErr);
    vdq.q = state->current_pi_q_integrator_v + (MOTOR_CFG_IQ_KP_V_PER_A * iqErr);

    /* Voltage vector saturation with integrator anti-windup */
    vMagSq = (vdq.d * vdq.d) + (vdq.q * vdq.q);
    if ((limitV > MOTOR_MATH_EPSILON_F) && (vMagSq > limitSq))
    {
        vScale = limitV / MotorMath_Sqrt(vMagSq);
        vdq.d *= vScale;
        vdq.q *= vScale;
        state->current_pi_d_integrator_v *= vScale;
        state->current_pi_q_integrator_v *= vScale;
    }


    /* ── Inverse Park + SVM ── */
    MotorFoc_InvPark(&vdq, sinCtrl, cosCtrl, &vab);
    MotorFoc_SpaceVector(&vab, busV,
                         &output->duty_u, &output->duty_v, &output->duty_w);

    /* ── Populate output structure ── */
    output->id_a                    = idq.d;
    output->iq_a                    = idq.q;
    output->observer_angle_rad      = state->pll_phase_rad;
    output->observer_speed_rad_s    = obsSpeed;
    output->phase_error_rad         = MotorMath_AngleDiff(input->control_angle_rad,
                                                          state->pll_phase_rad);
    output->bus_voltage_used_v      = busV;
    output->commanded_vab_v         = vab;
    output->voltage_modulation_ratio =
        MotorMath_Sqrt((vab.alpha * vab.alpha) + (vab.beta * vab.beta)) / busV;

    /* Store previous-cycle values for the observer */
    state->last_i_alpha_a = iab.alpha;
    state->last_i_beta_a  = iab.beta;
    state->last_v_alpha_v = vab.alpha;
    state->last_v_beta_v  = vab.beta;

}

float MotorFoc_RunSpeedPi(motor_foc_state_t *state,
                          float targetSpeedRadS,
                          float measuredSpeedRadS,
                          bool resetIntegrator)
{
    float err;
    float iqRef;

    if (state == NULL)
    {
        return 0.0f;
    }

    if (resetIntegrator)
    {
        state->speed_pi_integrator_a = 0.0f;
    }

    err = targetSpeedRadS - measuredSpeedRadS;
    state->speed_pi_integrator_a += MOTOR_CFG_SPEED_KI * MOTOR_CFG_SPEED_LOOP_DT_S * err;
    state->speed_pi_integrator_a  = MotorMath_Clamp(state->speed_pi_integrator_a,
                                                    -MOTOR_CFG_MAX_IQ_A,
                                                     MOTOR_CFG_MAX_IQ_A);

    iqRef = state->speed_pi_integrator_a + (MOTOR_CFG_SPEED_KP * err);
    iqRef = MotorMath_Clamp(iqRef, -MOTOR_CFG_MAX_IQ_A, MOTOR_CFG_MAX_IQ_A);

    return iqRef;
}

float MotorFoc_WrapAngle0ToTwoPi(float angleRad)
{
    return MotorMath_WrapAngle0To2Pi(angleRad);
}

float MotorFoc_AngleDiff(float targetAngleRad, float measuredAngleRad)
{
    return MotorMath_AngleDiff(MotorMath_WrapAngle0To2Pi(targetAngleRad),
                               MotorMath_WrapAngle0To2Pi(measuredAngleRad));
}

/** @} */ /* end of motor_foc group */
