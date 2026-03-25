#include "motor_foc.h"

#include <stddef.h>

#include "sdk_project_config.h"
#include "motor_user_config.h"

#define MOTOR_FOC_EPSILON_F                  (1.0e-6f)
#define MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S      (120000.0f)

/**
 * @brief Hook for profiling fast-loop timing (defined elsewhere).
 * @param focTotalCycles     Total CPU cycles spent in FOC fast loop.
 * @param observerCycles     CPU cycles spent in the observer.
 * @param currentLoopCycles  CPU cycles spent in the current PI loop.
 * @param svmCycles          CPU cycles spent in Space-Vector Modulation.
 */
void MotorControl_ProfileRecordFocTiming(uint32_t focTotalCycles,
                                         uint32_t observerCycles,
                                         uint32_t currentLoopCycles,
                                         uint32_t svmCycles);

/**
 * @brief Clamp a float value between a minimum and maximum.
 * @param value     Input value.
 * @param minValue  Lower bound.
 * @param maxValue  Upper bound.
 * @return Clamped value.
 */
static inline float MotorFoc_Clamp(float value, float minValue, float maxValue)
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

/**
 * @brief Calculate the maximum of three float values.
 * @param a First value.
 * @param b Second value.
 * @param c Third value.
 * @return Maximum of a, b, and c.
 */
static inline float MotorFoc_Max3(float a, float b, float c)
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

/**
 * @brief Calculate the minimum of three float values.
 * @param a First value.
 * @param b Second value.
 * @param c Third value.
 * @return Minimum of a, b, and c.
 */
static inline float MotorFoc_Min3(float a, float b, float c)
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

/**
 * @brief Fast floating-point absolute value using compiler intrinsics.
 * @param value Input value.
 * @return Absolute value of input.
 */
static inline float MotorFoc_FastAbs(float value)
{
    return __builtin_fabsf(value);
}

/**
 * @brief Fast floating-point square root using compiler intrinsics.
 *
 * Ensures the input is non-negative before applying the square root.
 *
 * @param value Input value (should be >= 0).
 * @return Square root of the input.
 */
static inline float MotorFoc_FastSqrt(float value)
{
    float safeValue = value;

    if (safeValue < 0.0f)
    {
        safeValue = 0.0f;
    }

    return __builtin_sqrtf(safeValue);
}

/**
 * @brief Fast sine/cosine approximation using a second-order polynomial with
 * extra precision refinement (Bhaskara-style).
 *
 * The input angle is first wrapped to [0, 2*pi), then shifted to [-pi, pi) for
 * the parabolic approximation: y = B*x + C*x*|x|, refined by y += P*(y*|y| - y).
 * Cosine is computed by phase-shifting the input by pi/2.
 *
 * @param[in]  angleRad  Angle in radians.
 * @param[out] sinOut    Pointer to store the calculated sine value.
 * @param[out] cosOut    Pointer to store the calculated cosine value.
 */
static void MotorFoc_FastSinCos(float angleRad, float *sinOut, float *cosOut)
{
    const float b = 4.0f / MOTOR_CFG_PI_F;
    const float c = -4.0f / (MOTOR_CFG_PI_F * MOTOR_CFG_PI_F);
    const float p = 0.225f;
    float wrappedAngle = MotorFoc_WrapAngle0ToTwoPi(angleRad);
    float sinInput = wrappedAngle;
    float cosInput = wrappedAngle + MOTOR_CFG_HALF_PI_F;
    float sinY;
    float cosY;

    if (sinInput > MOTOR_CFG_PI_F)
    {
        sinInput -= MOTOR_CFG_TWO_PI_F;
    }

    if (cosInput >= MOTOR_CFG_TWO_PI_F)
    {
        cosInput -= MOTOR_CFG_TWO_PI_F;
    }
    if (cosInput > MOTOR_CFG_PI_F)
    {
        cosInput -= MOTOR_CFG_TWO_PI_F;
    }

    sinY = (b * sinInput) + (c * sinInput * MotorFoc_FastAbs(sinInput));
    sinY = (p * ((sinY * MotorFoc_FastAbs(sinY)) - sinY)) + sinY;
    cosY = (b * cosInput) + (c * cosInput * MotorFoc_FastAbs(cosInput));
    cosY = (p * ((cosY * MotorFoc_FastAbs(cosY)) - cosY)) + cosY;

    *sinOut = sinY;
    *cosOut = cosY;
}

/**
 * @brief Fast two-argument arctangent approximation.
 *
 * Uses a first-order rational approximation with separate formulas for
 * |x| >= |y| and |x| < |y| regions, providing ~0.07 rad maximum error.
 *
 * @param y Y coordinate.
 * @param x X coordinate.
 * @return Angle in radians [-pi, pi].
 */
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

/**
 * @brief Clarke transform (3-phase -> alpha-beta stationary frame).
 *
 * Converts three-phase currents (ia, ib, ic) into the two-axis stationary
 * alpha-beta frame. Uses the amplitude-invariant form:
 *   i_alpha = ia
 *   i_beta  = (ia + 2*ib) / sqrt(3)
 * Phase-c current is unused because ia + ib + ic = 0.
 *
 * @param[in]  ia   Phase A current.
 * @param[in]  ib   Phase B current.
 * @param[in]  ic   Phase C current (unused, assumed balanced).
 * @param[out] iab  Pointer to the resulting alpha-beta frame structure.
 */
static void MotorFoc_Clarke(float ia, float ib, float ic, motor_ab_frame_t *iab)
{
    (void)ic;
    iab->alpha = ia;
    iab->beta = (ia + ib + ib) * MOTOR_CFG_INV_SQRT3_F;
}

/**
 * @brief Compute the shortest signed angle difference (target - measured).
 *
 * Wraps the result to the range (-pi, pi].
 * Assumes inputs are in a reasonable range (already mostly wrapped).
 *
 * @param targetAngleRad   The target angle in radians.
 * @param measuredAngleRad The measured angle in radians.
 * @return Shortest difference in radians.
 */
static inline float MotorFoc_AngleDiffRaw(float targetAngleRad, float measuredAngleRad)
{
    float diffRad = targetAngleRad - measuredAngleRad;

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

/**
 * @brief Park transform (alpha-beta stationary -> d-q rotating frame).
 *
 * Projects the stationary-frame currents onto the rotating d-q axes
 * aligned with the rotor flux:
 *   id =  i_alpha * cos(theta) + i_beta * sin(theta)
 *   iq = -i_alpha * sin(theta) + i_beta * cos(theta)
 * where theta is the electrical angle of the rotor.
 *
 * @param[in]  iab       Current in alpha-beta frame.
 * @param[in]  sinAngle  Sine of the electrical rotor angle.
 * @param[in]  cosAngle  Cosine of the electrical rotor angle.
 * @param[out] idq       Pointer to output d-q frame vector.
 */
static inline void MotorFoc_Park(const motor_ab_frame_t *iab,
                                 float sinAngle,
                                 float cosAngle,
                                 motor_dq_frame_t *idq)
{
    idq->d = (iab->alpha * cosAngle) + (iab->beta * sinAngle);
    idq->q = (-iab->alpha * sinAngle) + (iab->beta * cosAngle);
}

/**
 * @brief Inverse Park transform (d-q rotating -> alpha-beta stationary frame).
 *
 * Converts the d-q voltage commands back into the stationary frame for
 * space-vector modulation:
 *   v_alpha = vd * cos(theta) - vq * sin(theta)
 *   v_beta  = vd * sin(theta) + vq * cos(theta)
 *
 * @param[in]  vdq       Voltage in d-q frame.
 * @param[in]  sinAngle  Sine of the electrical rotor angle.
 * @param[in]  cosAngle  Cosine of the electrical rotor angle.
 * @param[out] vab       Pointer to output alpha-beta frame vector.
 */
static inline void MotorFoc_InvPark(const motor_dq_frame_t *vdq,
                                    float sinAngle,
                                    float cosAngle,
                                    motor_ab_frame_t *vab)
{
    vab->alpha = (vdq->d * cosAngle) - (vdq->q * sinAngle);
    vab->beta = (vdq->d * sinAngle) + (vdq->q * cosAngle);
}

/**
 * @brief Space-vector modulation with mid-clamp (centred) zero-sequence injection.
 *
 * 1. Convert alpha-beta voltages to three normalised phase voltages using
 *    the inverse Clarke relationship and divide by bus voltage.
 * 2. Apply a zero-sequence offset = 0.5 - 0.5*(max + min) to centre the
 *    waveform in the [0, 1] duty range, equivalent to SVPWM and giving
 *    ~15% more linear range than sinusoidal PWM.
 * 3. Clamp the resulting duty cycles to [0, 1].
 *
 * @param[in]  vab          Commanded voltage in alpha-beta frame.
 * @param[in]  busVoltageV  Available DC bus voltage in volts.
 * @param[out] dutyU        Calculated phase U duty cycle [0.0, 1.0].
 * @param[out] dutyV        Calculated phase V duty cycle [0.0, 1.0].
 * @param[out] dutyW        Calculated phase W duty cycle [0.0, 1.0].
 */
static inline void MotorFoc_SpaceVector(const motor_ab_frame_t *vab,
                                        float busVoltageV,
                                        float *dutyU,
                                        float *dutyV,
                                        float *dutyW)
{
    const float inverseBus = 1.0f / busVoltageV;
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

#if (MOTOR_CFG_ENABLE_DEADTIME_COMP != 0U)
static int8_t MotorFoc_UpdateDeadtimeCompSign(float phaseCurrentA,
                                              float minCurrentA,
                                              int8_t cachedSign)
{
    if (phaseCurrentA >= minCurrentA)
    {
        return 1;
    }

    if (phaseCurrentA <= -minCurrentA)
    {
        return -1;
    }

    return cachedSign;
}
#endif

/**
 * @brief Apply dynamic deadtime compensation to the calculated duty cycles.
 *
 * Adjusts the phase duty cycles based on the polarity of the currents
 * to compensate for the inverter dead-time voltage drops.
 *
 * @param[in,out] state   Persistent FOC state holding the current polarity signs.
 * @param[in]     input   Fast-loop inputs (currents).
 * @param[in,out] output  Fast-loop outputs (modifies duty cycles in place).
 */
static void MotorFoc_ApplyDeadtimeComp(motor_foc_state_t *state,
                                       const motor_foc_fast_input_t *input,
                                       motor_foc_fast_output_t *output)
{
#if (MOTOR_CFG_ENABLE_DEADTIME_COMP != 0U)
    const float compDuty = MOTOR_CFG_DEADTIME_COMP_DUTY;

    if ((!input->deadtime_comp_enable) || (compDuty <= 0.0f))
    {
        return;
    }

    state->deadtime_comp_sign_a = MotorFoc_UpdateDeadtimeCompSign(input->phase_current_a,
                                                                  MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A,
                                                                  state->deadtime_comp_sign_a);
    state->deadtime_comp_sign_b = MotorFoc_UpdateDeadtimeCompSign(input->phase_current_b,
                                                                  MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A,
                                                                  state->deadtime_comp_sign_b);
    state->deadtime_comp_sign_c = MotorFoc_UpdateDeadtimeCompSign(input->phase_current_c,
                                                                  MOTOR_CFG_DEADTIME_COMP_MIN_CURRENT_A,
                                                                  state->deadtime_comp_sign_c);

    output->duty_u = MotorFoc_Clamp(output->duty_u + (compDuty * (float)state->deadtime_comp_sign_a), 0.0f, 1.0f);
    output->duty_v = MotorFoc_Clamp(output->duty_v + (compDuty * (float)state->deadtime_comp_sign_b), 0.0f, 1.0f);
    output->duty_w = MotorFoc_Clamp(output->duty_w + (compDuty * (float)state->deadtime_comp_sign_c), 0.0f, 1.0f);
#else
    (void)state;
    (void)input;
    (void)output;
#endif
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
    state->deadtime_comp_sign_a = 0;
    state->deadtime_comp_sign_b = 0;
    state->deadtime_comp_sign_c = 0;
}

void MotorFoc_RunFast(motor_foc_state_t *state,
                      const motor_foc_fast_input_t *input,
                      motor_foc_fast_output_t *output)
{
    const float dt = MOTOR_CFG_FAST_LOOP_DT_S;
    const float busVoltageClampedV = MotorFoc_Clamp(input->bus_voltage_v, 1.0f, 1000.0f);
    const float currentLoopLimitV = busVoltageClampedV * MOTOR_CFG_SVM_MAX_MODULATION;
    const float currentLoopLimitSq = currentLoopLimitV * currentLoopLimitV;
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
    float sinControlAngle;
    float cosControlAngle;
    float voltageMagnitude;
    float voltageMagnitudeSq;
    float voltageScale;
#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
    const bool dwtEnabled = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
    uint32_t focStartCycles = 0U;
    uint32_t sectionStartCycles = 0U;
    uint32_t observerCycles = 0U;
    uint32_t currentLoopCycles = 0U;
    uint32_t svmCycles = 0U;
#endif

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
    if (dwtEnabled)
    {
        focStartCycles = DWT->CYCCNT;
    }
#endif

    /* ---- Clarke transform: 3-phase currents -> alpha-beta frame ---- */
    MotorFoc_Clarke(input->phase_current_a,
                    input->phase_current_b,
                    input->phase_current_c,
                    &iab);

    /** 
     * ---- Ortega nonlinear flux observer ----
     * State variables x1, x2 track (flux + L*i) in the alpha-beta frame.
     * The estimated rotor flux is:  psi = x - L_avg * i
     */
    /* Step 1: Extract the estimated flux from the observer states. */
    fluxAlpha = state->observer_x1 - (observerInductanceH * iab.alpha);
    fluxBeta = state->observer_x2 - (observerInductanceH * iab.beta);

    /* Step 2: Compute the nonlinear correction error.
     * err = lambda^2 - |psi|^2.  Only negative errors are used (i.e. the
     * observer only corrects when estimated flux exceeds the expected
     * magnitude), which makes the observer robust against noise. */
    fluxSq = (fluxAlpha * fluxAlpha) + (fluxBeta * fluxBeta);
    observerErr = (lambdaNowVs * lambdaNowVs) - fluxSq;
    if (observerErr > 0.0f)
    {
        observerErr = 0.0f;
    }

    /* Step 3: Integrate the observer differential equations.
     * dx/dt = v - Rs*i + gamma * psi * err
     * The first two terms form the motor voltage model; the last term
     * is the nonlinear correction that drives flux magnitude toward
     * the expected permanent-magnet flux linkage (lambda). */
    state->observer_x1 += dt * (state->last_v_alpha_v -
                                (MOTOR_CFG_RS_OHM * iab.alpha) +
                                (MOTOR_CFG_OBSERVER_GAIN * fluxAlpha * observerErr));
    state->observer_x2 += dt * (state->last_v_beta_v -
                                (MOTOR_CFG_RS_OHM * iab.beta) +
                                (MOTOR_CFG_OBSERVER_GAIN * fluxBeta * observerErr));

    /* Step 4: Re-extract the updated flux and compute its magnitude. */
    fluxAlpha = state->observer_x1 - (observerInductanceH * iab.alpha);
    fluxBeta = state->observer_x2 - (observerInductanceH * iab.beta);
    fluxMagnitude = MotorFoc_FastSqrt((fluxAlpha * fluxAlpha) + (fluxBeta * fluxBeta));

    /* Step 5: Adaptive flux-linkage tracking.
     * Slowly adjust the expected lambda toward the measured flux magnitude
     * using a first-order low-pass filter.  This compensates for
     * temperature-related changes in permanent-magnet strength. */
    state->observer_lambda_vs += MotorFoc_Clamp(MOTOR_CFG_LAMBDA_COMP_BW_RAD_S * dt, 0.0f, 1.0f) *
                                 (MotorFoc_Clamp(fluxMagnitude,
                                                 MOTOR_CFG_LAMBDA_MIN_VS,
                                                 MOTOR_CFG_LAMBDA_MAX_VS) -
                                  state->observer_lambda_vs);

    /** 
     * ---- Phase-locked loop (PLL) for rotor angle and speed estimation ----
     * The PLL tracks the observer flux angle to produce a smooth,
     * continuous electrical angle and speed estimate.
     *
     * 1. Compute the instantaneous flux angle from atan2(psi_beta, psi_alpha).
     * 2. Find the phase error between the measured flux angle and the
     *    PLL's internal phase.
     * 3. PI controller:
     *      integral += Ki * error * dt        (speed estimate)
     *      speed    = integral + Kp * error   (proportional boost)
     * 4. Integrate speed to update the PLL phase. 
     */
    measuredObserverAngleRad = MotorFoc_WrapAngle0ToTwoPi(MotorFoc_FastAtan2(fluxBeta, fluxAlpha));
    pllPhaseErrRad = MotorFoc_AngleDiffRaw(measuredObserverAngleRad, state->pll_phase_rad);
    pllIntegralRadS = state->pll_speed_rad_s + (MOTOR_CFG_PLL_KI * pllPhaseErrRad * dt);
    pllIntegralRadS = MotorFoc_Clamp(pllIntegralRadS,
                                     -MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S,
                                     MOTOR_FOC_PLL_SPEED_LIMIT_RAD_S);
    observerSpeedRadS = pllIntegralRadS + (MOTOR_CFG_PLL_KP * pllPhaseErrRad);
    state->pll_speed_rad_s = pllIntegralRadS;
    state->pll_phase_rad = MotorFoc_WrapAngle0ToTwoPi(state->pll_phase_rad + (observerSpeedRadS * dt));
    state->phase_error_rad = pllPhaseErrRad;

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
    if (dwtEnabled)
    {
        observerCycles = DWT->CYCCNT - focStartCycles;
        sectionStartCycles = DWT->CYCCNT;
    }
#endif

    /* ---- Park transform: alpha-beta -> d-q rotating frame ---- */
    MotorFoc_FastSinCos(input->control_angle_rad, &sinControlAngle, &cosControlAngle);
    MotorFoc_Park(&iab, sinControlAngle, cosControlAngle, &idq);

    /* ---- Current-loop PI controllers (d-axis and q-axis) ----
     * Compute the current errors and run independent PI regulators
     * for the d-axis (flux) and q-axis (torque) currents. */
    idErr = input->id_target_a - idq.d;
    iqErr = input->iq_target_a - idq.q;

    state->current_pi_d_integrator_v += MOTOR_CFG_ID_KI_V_PER_AS * dt * idErr;
    state->current_pi_q_integrator_v += MOTOR_CFG_IQ_KI_V_PER_AS * dt * iqErr;

    vdq.d = state->current_pi_d_integrator_v + (MOTOR_CFG_ID_KP_V_PER_A * idErr);
    vdq.q = state->current_pi_q_integrator_v + (MOTOR_CFG_IQ_KP_V_PER_A * iqErr);

    /* Voltage vector magnitude limiting with integrator anti-windup.
     * If the combined dq voltage exceeds the available bus voltage
     * (scaled by max modulation index), scale both axes proportionally
     * and apply the same scale to the integrators to prevent windup. */
    voltageMagnitudeSq = (vdq.d * vdq.d) + (vdq.q * vdq.q);
    if ((currentLoopLimitV > MOTOR_FOC_EPSILON_F) && (voltageMagnitudeSq > currentLoopLimitSq))
    {
        voltageMagnitude = MotorFoc_FastSqrt(voltageMagnitudeSq);
        voltageScale = currentLoopLimitV / voltageMagnitude;
        vdq.d *= voltageScale;
        vdq.q *= voltageScale;
        state->current_pi_d_integrator_v *= voltageScale;
        state->current_pi_q_integrator_v *= voltageScale;
    }

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
    if (dwtEnabled)
    {
        currentLoopCycles = DWT->CYCCNT - sectionStartCycles;
        sectionStartCycles = DWT->CYCCNT;
    }
#endif

    /* ---- Inverse Park + SVM: generate 3-phase PWM duty cycles ---- */
    MotorFoc_InvPark(&vdq, sinControlAngle, cosControlAngle, &vab);
    MotorFoc_SpaceVector(&vab,
                         busVoltageClampedV,
                         &output->duty_u,
                         &output->duty_v,
                         &output->duty_w);
    MotorFoc_ApplyDeadtimeComp(state, input, output);

    output->id_a = idq.d;
    output->iq_a = idq.q;
    output->observer_angle_rad = state->pll_phase_rad;
    output->observer_speed_rad_s = observerSpeedRadS;
    output->phase_error_rad = MotorFoc_AngleDiffRaw(input->control_angle_rad, state->pll_phase_rad);
    output->bus_voltage_used_v = busVoltageClampedV;
    output->commanded_vab_v = vab;
    output->voltage_modulation_ratio =
        MotorFoc_FastSqrt((vab.alpha * vab.alpha) + (vab.beta * vab.beta)) /
        busVoltageClampedV;

    state->last_i_alpha_a = iab.alpha;
    state->last_i_beta_a = iab.beta;
    state->last_v_alpha_v = vab.alpha;
    state->last_v_beta_v = vab.beta;

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
    if (dwtEnabled)
    {
        svmCycles = DWT->CYCCNT - sectionStartCycles;
        MotorControl_ProfileRecordFocTiming(DWT->CYCCNT - focStartCycles,
                                            observerCycles,
                                            currentLoopCycles,
                                            svmCycles);
    }
#endif
}

/**
 * @brief Speed-loop PI controller.
 *
 * Computes the q-axis current reference (iq) from the speed error.
 * Runs at the slower speed-loop rate (MOTOR_CFG_SPEED_LOOP_DT_S).
 *
 * @param[in,out] state                        FOC state holding the speed PI integrator.
 * @param[in]     targetElectricalSpeedRadS    The requested electrical speed.
 * @param[in]     measuredElectricalSpeedRadS  The actual electrical speed.
 * @param[in]     resetIntegrator              Flag to reset the integrator if true.
 * @return Commanded Q-axis current in amperes.
 */
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

    if (wrappedAngle >= MOTOR_CFG_TWO_PI_F)
    {
        wrappedAngle -= MOTOR_CFG_TWO_PI_F;
    }
    else if (wrappedAngle < 0.0f)
    {
        wrappedAngle += MOTOR_CFG_TWO_PI_F;
    }

    return wrappedAngle;
}

float MotorFoc_AngleDiff(float targetAngleRad, float measuredAngleRad)
{
    return MotorFoc_AngleDiffRaw(MotorFoc_WrapAngle0ToTwoPi(targetAngleRad),
                                 MotorFoc_WrapAngle0ToTwoPi(measuredAngleRad));
}
