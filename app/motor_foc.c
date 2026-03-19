#include "motor_foc.h"

#include <stddef.h>

#include "motor_user_config.h"

#define MOTOR_FOC_EPSILON_F                  (1.0e-6f)
#define MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S      (120000.0f)

static float MotorFoc_Clamp(float value, float minValue, float maxValue)
{
    float result = value;

    if (result < minValue)
    {
        result = minValue;
    }
    else if (result > maxValue)
    {
        result = maxValue;
    }

    return result;
}

static float MotorFoc_Max3(float a, float b, float c)
{
    float result = a;

    if (b > result)
    {
        result = b;
    }

    if (c > result)
    {
        result = c;
    }

    return result;
}

static float MotorFoc_Min3(float a, float b, float c)
{
    float result = a;

    if (b < result)
    {
        result = b;
    }

    if (c < result)
    {
        result = c;
    }

    return result;
}

static float MotorFoc_FastAbs(float value)
{
    return __builtin_fabsf(value);
}

static float MotorFoc_FastSqrt(float value)
{
    float safeValue = value;

    if (safeValue < 0.0f)
    {
        safeValue = 0.0f;
    }

    return __builtin_sqrtf(safeValue);
}

static float MotorFoc_FastSin(float angleRad)
{
    const float b = 4.0f / MOTOR_CFG_PI_F;
    const float c = -4.0f / (MOTOR_CFG_PI_F * MOTOR_CFG_PI_F);
    const float p = 0.225f;
    float wrappedAngle = MotorFoc_WrapAngle0ToTwoPi(angleRad);
    float y;

    if (wrappedAngle > MOTOR_CFG_PI_F)
    {
        wrappedAngle -= MOTOR_CFG_TWO_PI_F;
    }

    y = (b * wrappedAngle) + (c * wrappedAngle * MotorFoc_FastAbs(wrappedAngle));
    y = (p * ((y * MotorFoc_FastAbs(y)) - y)) + y;

    return y;
}

static float MotorFoc_FastCos(float angleRad)
{
    return MotorFoc_FastSin(angleRad + MOTOR_CFG_HALF_PI_F);
}

static float MotorFoc_FastAtan2(float y, float x)
{
    const float quarterPi = 0.78539816339744830962f;
    const float threeQuarterPi = 2.35619449019234492885f;
    const float absY = MotorFoc_FastAbs(y) + MOTOR_FOC_EPSILON_F;
    float angle;
    float ratio;

    if (x >= 0.0f)
    {
        ratio = (x - absY) / (x + absY);
        angle = quarterPi - (quarterPi * ratio);
    }
    else
    {
        ratio = (x + absY) / (absY - x);
        angle = threeQuarterPi - (quarterPi * ratio);
    }

    if (y < 0.0f)
    {
        angle = -angle;
    }

    return angle;
}

static void MotorFoc_Clarke(float ia, float ib, float ic, motor_ab_frame_t *iab)
{
    iab->alpha = ((2.0f * ia) - ib - ic) * (1.0f / 3.0f);
    iab->beta = (ib - ic) * MOTOR_CFG_INV_SQRT3_F;
}

static void MotorFoc_Park(const motor_ab_frame_t *iab, float angleRad, motor_dq_frame_t *idq)
{
    const float sinAngle = MotorFoc_FastSin(angleRad);
    const float cosAngle = MotorFoc_FastCos(angleRad);

    idq->d = (iab->alpha * cosAngle) + (iab->beta * sinAngle);
    idq->q = (-iab->alpha * sinAngle) + (iab->beta * cosAngle);
}

static void MotorFoc_InvPark(const motor_dq_frame_t *vdq, float angleRad, motor_ab_frame_t *vab)
{
    const float sinAngle = MotorFoc_FastSin(angleRad);
    const float cosAngle = MotorFoc_FastCos(angleRad);

    vab->alpha = (vdq->d * cosAngle) - (vdq->q * sinAngle);
    vab->beta = (vdq->d * sinAngle) + (vdq->q * cosAngle);
}

static void MotorFoc_SpaceVector(const motor_ab_frame_t *vab,
                                 float busVoltageV,
                                 float *dutyU,
                                 float *dutyV,
                                 float *dutyW)
{
    const float inverseBus = 1.0f / MotorFoc_Clamp(busVoltageV, 1.0f, 1000.0f);
    const float phaseU = vab->alpha * inverseBus;
    const float phaseV = ((-0.5f * vab->alpha) + (MOTOR_CFG_SQRT3_BY_2_F * vab->beta)) * inverseBus;
    const float phaseW = ((-0.5f * vab->alpha) - (MOTOR_CFG_SQRT3_BY_2_F * vab->beta)) * inverseBus;
    const float vMax = MotorFoc_Max3(phaseU, phaseV, phaseW);
    const float vMin = MotorFoc_Min3(phaseU, phaseV, phaseW);
    const float offset = 0.5f - (0.5f * (vMax + vMin));

    *dutyU = MotorFoc_Clamp(phaseU + offset, 0.0f, 1.0f);
    *dutyV = MotorFoc_Clamp(phaseV + offset, 0.0f, 1.0f);
    *dutyW = MotorFoc_Clamp(phaseW + offset, 0.0f, 1.0f);
}

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
    state->speed_pi_integrator_a = 0.0f;
    state->pll_phase_rad = 0.0f;
    state->pll_speed_rad_s = 0.0f;
    state->observer_x1 = 0.0f;
    state->observer_x2 = 0.0f;
    state->observer_lambda_vs = MOTOR_CFG_FLUX_LINKAGE_VS;
    state->phase_error_rad = 0.0f;
    state->last_i_alpha_a = 0.0f;
    state->last_i_beta_a = 0.0f;
    state->last_v_alpha_v = 0.0f;
    state->last_v_beta_v = 0.0f;
}

void MotorFoc_RunFast(motor_foc_state_t *state,
                      const motor_foc_fast_input_t *input,
                      motor_foc_fast_output_t *output)
{
    const float dt = MOTOR_CFG_FAST_LOOP_DT_S;
    const float currentLoopLimitV = input->bus_voltage_v * MOTOR_CFG_SVM_MAX_MODULATION;
    const float observerInductanceH = 0.5f * (MOTOR_CFG_LD_H + MOTOR_CFG_LQ_H);
    const float lambdaNowVs = MotorFoc_Clamp(state->observer_lambda_vs,
                                             MOTOR_CFG_LAMBDA_MIN_VS,
                                             MOTOR_CFG_LAMBDA_MAX_VS);
    motor_ab_frame_t iab;
    motor_dq_frame_t idq;
    motor_dq_frame_t vdq;
    motor_ab_frame_t vab;
    float fluxAlpha;
    float fluxBeta;
    float fluxSq;
    float observerErr;
    float fluxMagnitude;
    float measuredObserverAngleRad;
    float pllPhaseErrRad;
    float pllIntegralRadS;
    float observerSpeedRadS;
    float idErr;
    float iqErr;
    float voltageMagnitude;
    float voltageScale;

    MotorFoc_Clarke(input->phase_current_a,
                    input->phase_current_b,
                    input->phase_current_c,
                    &iab);

    fluxAlpha = state->observer_x1 - (observerInductanceH * iab.alpha);
    fluxBeta = state->observer_x2 - (observerInductanceH * iab.beta);
    fluxSq = (fluxAlpha * fluxAlpha) + (fluxBeta * fluxBeta);
    observerErr = (lambdaNowVs * lambdaNowVs) - fluxSq;
    if (observerErr > 0.0f)
    {
        observerErr = 0.0f;
    }

    state->observer_x1 += dt * (state->last_v_alpha_v -
                                (MOTOR_CFG_RS_OHM * iab.alpha) +
                                (MOTOR_CFG_OBSERVER_GAIN * fluxAlpha * observerErr));
    state->observer_x2 += dt * (state->last_v_beta_v -
                                (MOTOR_CFG_RS_OHM * iab.beta) +
                                (MOTOR_CFG_OBSERVER_GAIN * fluxBeta * observerErr));

    fluxAlpha = state->observer_x1 - (observerInductanceH * iab.alpha);
    fluxBeta = state->observer_x2 - (observerInductanceH * iab.beta);
    fluxMagnitude = MotorFoc_FastSqrt((fluxAlpha * fluxAlpha) + (fluxBeta * fluxBeta));
    state->observer_lambda_vs += MotorFoc_Clamp(MOTOR_CFG_LAMBDA_COMP_BW_RAD_S * dt, 0.0f, 1.0f) *
                                 (MotorFoc_Clamp(fluxMagnitude,
                                                 MOTOR_CFG_LAMBDA_MIN_VS,
                                                 MOTOR_CFG_LAMBDA_MAX_VS) -
                                  state->observer_lambda_vs);

    measuredObserverAngleRad = MotorFoc_WrapAngle0ToTwoPi(MotorFoc_FastAtan2(fluxBeta, fluxAlpha));
    pllPhaseErrRad = MotorFoc_AngleDiff(measuredObserverAngleRad, state->pll_phase_rad);
    pllIntegralRadS = state->pll_speed_rad_s + (MOTOR_CFG_PLL_KI * pllPhaseErrRad * dt);
    pllIntegralRadS = MotorFoc_Clamp(pllIntegralRadS,
                                     -MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S,
                                     MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S);
    observerSpeedRadS = pllIntegralRadS + (MOTOR_CFG_PLL_KP * pllPhaseErrRad);
    state->pll_speed_rad_s = pllIntegralRadS;
    state->pll_phase_rad = MotorFoc_WrapAngle0ToTwoPi(state->pll_phase_rad + (observerSpeedRadS * dt));
    state->phase_error_rad = pllPhaseErrRad;

    MotorFoc_Park(&iab, input->control_angle_rad, &idq);

    idErr = input->id_target_a - idq.d;
    iqErr = input->iq_target_a - idq.q;

    state->current_pi_d_integrator_v += MOTOR_CFG_ID_KI_V_PER_AS * dt * idErr;
    state->current_pi_q_integrator_v += MOTOR_CFG_IQ_KI_V_PER_AS * dt * iqErr;

    vdq.d = state->current_pi_d_integrator_v + (MOTOR_CFG_ID_KP_V_PER_A * idErr);
    vdq.q = state->current_pi_q_integrator_v + (MOTOR_CFG_IQ_KP_V_PER_A * iqErr);

    voltageMagnitude = MotorFoc_FastSqrt((vdq.d * vdq.d) + (vdq.q * vdq.q));
    if ((currentLoopLimitV > MOTOR_FOC_EPSILON_F) && (voltageMagnitude > currentLoopLimitV))
    {
        voltageScale = currentLoopLimitV / voltageMagnitude;
        vdq.d *= voltageScale;
        vdq.q *= voltageScale;
        state->current_pi_d_integrator_v *= voltageScale;
        state->current_pi_q_integrator_v *= voltageScale;
    }

    MotorFoc_InvPark(&vdq, input->control_angle_rad, &vab);
    MotorFoc_SpaceVector(&vab,
                         MotorFoc_Clamp(input->bus_voltage_v, 1.0f, 1000.0f),
                         &output->duty_u,
                         &output->duty_v,
                         &output->duty_w);

    output->id_a = idq.d;
    output->iq_a = idq.q;
    output->observer_angle_rad = state->pll_phase_rad;
    output->observer_speed_rad_s = observerSpeedRadS;
    output->phase_error_rad = MotorFoc_AngleDiff(input->control_angle_rad, state->pll_phase_rad);
    output->bus_voltage_used_v = MotorFoc_Clamp(input->bus_voltage_v, 1.0f, 1000.0f);
    output->commanded_vab_v = vab;

    state->last_i_alpha_a = iab.alpha;
    state->last_i_beta_a = iab.beta;
    state->last_v_alpha_v = vab.alpha;
    state->last_v_beta_v = vab.beta;
}

float MotorFoc_RunSpeedPi(motor_foc_state_t *state,
                          float targetElectricalSpeedRadS,
                          float measuredElectricalSpeedRadS,
                          bool resetIntegrator)
{
    float speedErrRadS;
    float iqRefA;

    if (state == NULL)
    {
        return 0.0f;
    }

    if (resetIntegrator)
    {
        state->speed_pi_integrator_a = 0.0f;
    }

    speedErrRadS = targetElectricalSpeedRadS - measuredElectricalSpeedRadS;
    state->speed_pi_integrator_a += MOTOR_CFG_SPEED_KI * MOTOR_CFG_SPEED_LOOP_DT_S * speedErrRadS;
    state->speed_pi_integrator_a = MotorFoc_Clamp(state->speed_pi_integrator_a,
                                                  -MOTOR_CFG_MAX_IQ_A,
                                                  MOTOR_CFG_MAX_IQ_A);

    iqRefA = state->speed_pi_integrator_a + (MOTOR_CFG_SPEED_KP * speedErrRadS);
    iqRefA = MotorFoc_Clamp(iqRefA, -MOTOR_CFG_MAX_IQ_A, MOTOR_CFG_MAX_IQ_A);

    return iqRefA;
}

float MotorFoc_WrapAngle0ToTwoPi(float angleRad)
{
    float wrappedAngle = angleRad;

    while (wrappedAngle >= MOTOR_CFG_TWO_PI_F)
    {
        wrappedAngle -= MOTOR_CFG_TWO_PI_F;
    }

    while (wrappedAngle < 0.0f)
    {
        wrappedAngle += MOTOR_CFG_TWO_PI_F;
    }

    return wrappedAngle;
}

float MotorFoc_AngleDiff(float targetAngleRad, float measuredAngleRad)
{
    float diffRad = MotorFoc_WrapAngle0ToTwoPi(targetAngleRad) -
                    MotorFoc_WrapAngle0ToTwoPi(measuredAngleRad);

    if (diffRad > MOTOR_CFG_PI_F)
    {
        diffRad -= MOTOR_CFG_TWO_PI_F;
    }
    else if (diffRad < -MOTOR_CFG_PI_F)
    {
        diffRad += MOTOR_CFG_TWO_PI_F;
    }

    return diffRad;
}
