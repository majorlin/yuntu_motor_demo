/**
 * @file motor_param_ident.c
 * @brief Motor parameter auto-identification engine.
 *
 * Self-contained implementation with internal FOC math primitives.
 * No dependency on motor_control.h or motor_foc.h.
 *
 * Algorithm overview:
 *   Phase 1 (Rs): Bidirectional DC injection on d-axis, measure Vd/Id.
 *   Phase 2 (Ls): Voltage step on d-axis, measure current slope dI/dt.
 *   Phase 3 (λ):  Open-loop rotation, extract BEMF from Vq - Rs·Iq.
 */

#include "motor_param_ident.h"

#include <stddef.h>
#include <string.h>

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal math constants
 * ══════════════════════════════════════════════════════════════════════════ */

#define IDENT_PI_F          (3.14159265358979323846f)
#define IDENT_TWO_PI_F      (6.28318530717958647692f)
#define IDENT_SQRT3_BY_2_F  (0.86602540378443864676f)
#define IDENT_INV_SQRT3_F   (0.57735026918962576451f)
#define IDENT_EPSILON_F     (1.0e-6f)

/* ── Timing constants (in slow-loop ticks, i.e. milliseconds at 1 kHz) ── */

#define IDENT_RS_SETTLE_MS       (300U)   /* Settling time per DC step        */
#define IDENT_RS_MEASURE_MS      (100U)   /* Averaging window per step        */
#define IDENT_LS_PRE_SETTLE_MS   (100U)   /* Quiescent period before step     */
#define IDENT_LS_STEP_MS         (2U)     /* 2ms step — short to stay in dI/dt region */
#define IDENT_LS_REPEAT_COUNT    (8U)     /* Number of step repetitions        */
#define IDENT_LS_PAUSE_MS        (100U)   /* Pause between step repetitions   */
#define IDENT_LAMBDA_RAMP_MS     (1000U)  /* Speed ramp-up time               */
#define IDENT_LAMBDA_SETTLE_MS   (1200U)  /* Settling at target speed         */
#define IDENT_LAMBDA_MEASURE_MS  (800U)   /* Averaging window                 */

/* ── Drag detection thresholds ── */
#define IDENT_DRAG_IQ_ERR_LIMIT  (0.5f)   /* Max |Iq_target - Iq_meas| (A)    */
#define IDENT_DRAG_BEMF_MARGIN   (1.5f)   /* Vq must exceed Rs*Iq * this      */
#define IDENT_DRAG_MAX_RETRIES   (5U)     /* Max adaptive current retries     */
#define IDENT_DRAG_RETRY_PAUSE_MS (300U)  /* Pause before retry               */
#define IDENT_DRAG_CHECK_WINDOW  (100U)   /* Samples for drag quality check   */

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal sub-state machines
 * ══════════════════════════════════════════════════════════════════════════ */

typedef enum
{
    RS_SUB_POS_SETTLE = 0,   /* Ramping / settling +Id */
    RS_SUB_POS_MEASURE,      /* Measuring +Vd, +Id     */
    RS_SUB_NEG_SETTLE,       /* Settling -Id            */
    RS_SUB_NEG_MEASURE,      /* Measuring -Vd, -Id      */
    RS_SUB_DONE
} ident_rs_sub_t;

typedef enum
{
    LS_SUB_PRE_SETTLE = 0,   /* Hold Id=0 before step */
    LS_SUB_STEP_ACTIVE,      /* Voltage step applied  */
    LS_SUB_PAUSE,            /* Pause before next rep */
    LS_SUB_DONE
} ident_ls_sub_t;

typedef enum
{
    LAMBDA_SUB_RAMP = 0,     /* Ramping open-loop speed */
    LAMBDA_SUB_SETTLE,       /* Speed settling          */
    LAMBDA_SUB_MEASURE,      /* Measuring Vq/Iq         */
    LAMBDA_SUB_DONE
} ident_lambda_sub_t;

/* ══════════════════════════════════════════════════════════════════════════
 *  Module state
 * ══════════════════════════════════════════════════════════════════════════ */

typedef struct
{
    /* Configuration (copied at start) */
    motor_ident_config_t cfg;

    /* Overall phase */
    motor_ident_phase_t phase;
    motor_ident_result_t result;

    /* Sub-state index and slow-loop tick counter */
    uint8_t  subState;
    uint32_t tickCounter;

    /* Current-loop PI integrators (simple internal PI for d/q axes) */
    float piDIntegV;
    float piQIntegV;

    /* Current targets */
    float idTargetA;
    float iqTargetA;

    /* Voltage command outputs (set by fast loop PI) */
    float vdCommandV;
    float vqCommandV;

    /* Open-loop angle (for lambda ident and alignment) */
    float angleRad;
    float speedRadS;

    /* Measured d-q currents (from fast loop) */
    float idMeasuredA;
    float iqMeasuredA;

    /* ── Rs sub-state accumulators ── */
    float rsVdPosSum;
    float rsIdPosSum;
    float rsVdNegSum;
    float rsIdNegSum;
    uint32_t rsSampleCount;

    /* ── Ls sub-state accumulators ── */
    float lsIdBeforeStep;       /* Id just before voltage step   */
    float lsIdAtEnd;            /* Id at end of step             */
    float lsVdStepApplied;      /* Actual Vd during step         */
    float lsSumLH;              /* Sum of L estimates across reps */
    float lsSamples[8];         /* Individual L estimates for median */
    uint8_t lsValidCount;       /* How many valid estimates stored */
    uint8_t lsRepCount;         /* Completed step repetitions    */
    bool  lsStepVoltageActive;  /* Flag: voltage step is being applied */

    /* ── Lambda sub-state accumulators ── */
    float lambdaVqSum;
    float lambdaIqSum;
    uint32_t lambdaSampleCount;

    /* ── Computed lambda parameters ── */
    float lambdaSpeedRadS;       /* Computed target elec speed (rad/s) */
    float lambdaIqNowA;          /* Current Iq level for lambda        */
    uint8_t dragRetryCount;      /* Number of drag retries attempted   */
    bool  dragVerified;          /* Drag quality verified              */

    /* ── Drag quality check accumulators ── */
    float dragIqErrSum;          /* Sum of |Iq_target - Iq_meas|       */
    uint32_t dragCheckCount;     /* Number of drag check samples       */

    /* PI gain constants (computed once from config) */
    float piKp;    /* Proportional gain for current loop */
    float piKi;    /* Integral gain for current loop     */
} ident_state_t;

static ident_state_t s_ident;

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal FOC math primitives
 *  (self-contained — no dependency on motor_foc.h)
 * ══════════════════════════════════════════════════════════════════════════ */

static inline float Ident_Clamp(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static inline float Ident_Fabsf(float v)
{
    return __builtin_fabsf(v);
}

static inline float Ident_Sqrtf(float v)
{
    return __builtin_sqrtf(v > 0.0f ? v : 0.0f);
}

static float Ident_WrapAngle(float a)
{
    if (a >= IDENT_TWO_PI_F) a -= IDENT_TWO_PI_F;
    else if (a < 0.0f)      a += IDENT_TWO_PI_F;
    return a;
}

/**
 * @brief Fast sin/cos (parabolic approximation, ~0.1% error).
 */
static void Ident_SinCos(float angleRad, float *sinOut, float *cosOut)
{
    const float b = 4.0f / IDENT_PI_F;
    const float c = -4.0f / (IDENT_PI_F * IDENT_PI_F);
    const float p = 0.225f;

    float wrapped = Ident_WrapAngle(angleRad);
    float sIn = wrapped;
    float cIn = wrapped + (IDENT_PI_F * 0.5f);

    if (sIn > IDENT_PI_F) sIn -= IDENT_TWO_PI_F;
    if (cIn >= IDENT_TWO_PI_F) cIn -= IDENT_TWO_PI_F;
    if (cIn > IDENT_PI_F) cIn -= IDENT_TWO_PI_F;

    float sy = (b * sIn) + (c * sIn * Ident_Fabsf(sIn));
    sy = (p * ((sy * Ident_Fabsf(sy)) - sy)) + sy;
    float cy = (b * cIn) + (c * cIn * Ident_Fabsf(cIn));
    cy = (p * ((cy * Ident_Fabsf(cy)) - cy)) + cy;

    *sinOut = sy;
    *cosOut = cy;
}

/**
 * @brief Clarke transform: 3-phase → α-β.
 */
static inline void Ident_Clarke(float ia, float ib, float *alpha, float *beta)
{
    *alpha = ia;
    *beta  = (ia + ib + ib) * IDENT_INV_SQRT3_F;
}

/**
 * @brief Park transform: α-β → d-q.
 */
static inline void Ident_Park(float alpha, float beta,
                               float sinA, float cosA,
                               float *id, float *iq)
{
    *id =  (alpha * cosA) + (beta * sinA);
    *iq = -(alpha * sinA) + (beta * cosA);
}

/**
 * @brief Inverse Park: d-q → α-β.
 */
static inline void Ident_InvPark(float vd, float vq,
                                  float sinA, float cosA,
                                  float *vAlpha, float *vBeta)
{
    *vAlpha = (vd * cosA) - (vq * sinA);
    *vBeta  = (vd * sinA) + (vq * cosA);
}

/**
 * @brief Space-vector modulation (mid-clamp): α-β voltage → 3-phase duty.
 */
static void Ident_Svm(float vAlpha, float vBeta, float vbus,
                       float *dutyU, float *dutyV, float *dutyW)
{
    const float inv = 1.0f / (vbus > 1.0f ? vbus : 1.0f);
    const float pU = vAlpha * inv;
    const float pV = ((-0.5f * vAlpha) + (IDENT_SQRT3_BY_2_F * vBeta)) * inv;
    const float pW = ((-0.5f * vAlpha) - (IDENT_SQRT3_BY_2_F * vBeta)) * inv;

    float vMax = pU; if (pV > vMax) vMax = pV; if (pW > vMax) vMax = pW;
    float vMin = pU; if (pV < vMin) vMin = pV; if (pW < vMin) vMin = pW;
    const float off = 0.5f - 0.5f * (vMax + vMin);

    *dutyU = Ident_Clamp(pU + off, 0.0f, 1.0f);
    *dutyV = Ident_Clamp(pV + off, 0.0f, 1.0f);
    *dutyW = Ident_Clamp(pW + off, 0.0f, 1.0f);
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Internal current-loop PI controller
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Run a simple PI current regulator on both d-q axes.
 *
 * During Ls identification the d-axis PI is bypassed and a direct voltage
 * command is used instead.
 */
static void Ident_RunCurrentPi(float vbusV)
{
    const float dt  = s_ident.cfg.pwm_dt_s;
    const float kp  = s_ident.piKp;
    const float ki  = s_ident.piKi;
    const float lim = vbusV * s_ident.cfg.max_duty_modulation;

    /* d-axis */
    if (!s_ident.lsStepVoltageActive)
    {
        float errD = s_ident.idTargetA - s_ident.idMeasuredA;
        s_ident.piDIntegV += ki * dt * errD;
        s_ident.vdCommandV = s_ident.piDIntegV + (kp * errD);
    }
    /* else: vdCommandV is set directly by the Ls step logic */

    /* q-axis */
    float errQ = s_ident.iqTargetA - s_ident.iqMeasuredA;
    s_ident.piQIntegV += ki * dt * errQ;
    s_ident.vqCommandV = s_ident.piQIntegV + (kp * errQ);

    /* Voltage magnitude limiting with anti-windup */
    float magSq = (s_ident.vdCommandV * s_ident.vdCommandV) +
                  (s_ident.vqCommandV * s_ident.vqCommandV);
    float limSq = lim * lim;
    if ((lim > IDENT_EPSILON_F) && (magSq > limSq))
    {
        float scale = lim / Ident_Sqrtf(magSq);
        s_ident.vdCommandV *= scale;
        s_ident.vqCommandV *= scale;
        if (!s_ident.lsStepVoltageActive)
        {
            s_ident.piDIntegV *= scale;
        }
        s_ident.piQIntegV *= scale;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Phase transition helpers
 * ══════════════════════════════════════════════════════════════════════════ */

static void Ident_EnterRs(void)
{
    s_ident.phase = MOTOR_IDENT_PHASE_RS;
    s_ident.subState = (uint8_t)RS_SUB_POS_SETTLE;
    s_ident.tickCounter = 0U;
    s_ident.idTargetA = s_ident.cfg.test_current_a;
    s_ident.iqTargetA = 0.0f;
    s_ident.angleRad  = 0.0f;
    s_ident.speedRadS = 0.0f;
    s_ident.piDIntegV = 0.0f;
    s_ident.piQIntegV = 0.0f;
    s_ident.rsVdPosSum = 0.0f;
    s_ident.rsIdPosSum = 0.0f;
    s_ident.rsVdNegSum = 0.0f;
    s_ident.rsIdNegSum = 0.0f;
    s_ident.rsSampleCount = 0U;
    s_ident.lsStepVoltageActive = false;
}

static void Ident_EnterLs(void)
{
    s_ident.phase = MOTOR_IDENT_PHASE_LS;
    s_ident.subState = (uint8_t)LS_SUB_PRE_SETTLE;
    s_ident.tickCounter = 0U;
    s_ident.idTargetA = 0.0f;
    s_ident.iqTargetA = 0.0f;
    s_ident.piDIntegV = 0.0f;
    s_ident.piQIntegV = 0.0f;
    s_ident.lsSumLH = 0.0f;
    s_ident.lsValidCount = 0U;
    s_ident.lsRepCount = 0U;
    s_ident.lsStepVoltageActive = false;
    s_ident.lsIdBeforeStep = 0.0f;
    s_ident.lsIdAtEnd = 0.0f;
    s_ident.lsVdStepApplied = 0.0f;
}

/**
 * @brief Compute target electrical speed from user-configured pole pairs and RPM.
 */
static float Ident_ComputeLambdaSpeed(const motor_ident_config_t *cfg)
{
    /* ω_e = RPM × 2π × pole_pairs / 60 */
    uint8_t pp = cfg->pole_pairs;
    if (pp == 0U) pp = 1U;  /* safety */
    return cfg->target_mech_rpm * IDENT_TWO_PI_F * (float)pp / 60.0f;
}

static void Ident_EnterLambda(void)
{
    s_ident.phase = MOTOR_IDENT_PHASE_LAMBDA;
    s_ident.subState = (uint8_t)LAMBDA_SUB_RAMP;
    s_ident.tickCounter = 0U;
    s_ident.idTargetA = 0.0f;
    s_ident.iqTargetA = s_ident.lambdaIqNowA;
    s_ident.angleRad  = 0.0f;
    s_ident.speedRadS = 0.0f;
    s_ident.piDIntegV = 0.0f;
    s_ident.piQIntegV = 0.0f;
    s_ident.lsStepVoltageActive = false;
    s_ident.lambdaVqSum = 0.0f;
    s_ident.lambdaIqSum = 0.0f;
    s_ident.lambdaSampleCount = 0U;
    s_ident.dragVerified = false;
    s_ident.dragIqErrSum = 0.0f;
    s_ident.dragCheckCount = 0U;
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Slow-loop phase handlers
 * ══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Rs slow-loop: manage settle / measure sub-states.
 */
static void Ident_SlowLoopRs(void)
{
    s_ident.tickCounter++;
    ident_rs_sub_t sub = (ident_rs_sub_t)s_ident.subState;

    switch (sub)
    {
    case RS_SUB_POS_SETTLE:
        s_ident.idTargetA = s_ident.cfg.test_current_a;
        if (s_ident.tickCounter >= IDENT_RS_SETTLE_MS)
        {
            s_ident.subState = (uint8_t)RS_SUB_POS_MEASURE;
            s_ident.tickCounter = 0U;
            s_ident.rsSampleCount = 0U;
            s_ident.rsVdPosSum = 0.0f;
            s_ident.rsIdPosSum = 0.0f;
        }
        break;

    case RS_SUB_POS_MEASURE:
        /* Accumulate from fast loop (sampled every PWM cycle) */
        s_ident.rsVdPosSum += s_ident.vdCommandV;
        s_ident.rsIdPosSum += s_ident.idMeasuredA;
        s_ident.rsSampleCount++;
        if (s_ident.tickCounter >= IDENT_RS_MEASURE_MS)
        {
            s_ident.subState = (uint8_t)RS_SUB_NEG_SETTLE;
            s_ident.tickCounter = 0U;
        }
        break;

    case RS_SUB_NEG_SETTLE:
        s_ident.idTargetA = -s_ident.cfg.test_current_a;
        if (s_ident.tickCounter >= IDENT_RS_SETTLE_MS)
        {
            s_ident.subState = (uint8_t)RS_SUB_NEG_MEASURE;
            s_ident.tickCounter = 0U;
            s_ident.rsVdNegSum = 0.0f;
            s_ident.rsIdNegSum = 0.0f;
            s_ident.rsSampleCount = 0U;
        }
        break;

    case RS_SUB_NEG_MEASURE:
        s_ident.rsVdNegSum += s_ident.vdCommandV;
        s_ident.rsIdNegSum += s_ident.idMeasuredA;
        s_ident.rsSampleCount++;
        if (s_ident.tickCounter >= IDENT_RS_MEASURE_MS)
        {
            s_ident.subState = (uint8_t)RS_SUB_DONE;
        }
        break;

    case RS_SUB_DONE:
    default:
        break;
    }

    /* Compute Rs when done */
    if (sub != RS_SUB_DONE && (ident_rs_sub_t)s_ident.subState == RS_SUB_DONE)
    {
        float n = (float)s_ident.rsSampleCount;
        if (n > 0.0f)
        {
            float vdPos = s_ident.rsVdPosSum / n;
            float idPos = s_ident.rsIdPosSum / n;
            float vdNeg = s_ident.rsVdNegSum / n;
            float idNeg = s_ident.rsIdNegSum / n;
            float rsPos = (Ident_Fabsf(idPos) > IDENT_EPSILON_F)
                              ? (vdPos / idPos) : 0.0f;
            float rsNeg = (Ident_Fabsf(idNeg) > IDENT_EPSILON_F)
                              ? (vdNeg / idNeg) : 0.0f;
            s_ident.result.rs_ohm = 0.5f * (rsPos + Ident_Fabsf(rsNeg));

            if (s_ident.result.rs_ohm > 0.001f && s_ident.result.rs_ohm < 100.0f)
            {
                s_ident.result.rs_valid = true;
                /* Transition to Ls */
                Ident_EnterLs();
            }
            else
            {
                s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
            }
        }
        else
        {
            s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
        }
    }
}

/**
 * @brief Ls slow-loop: voltage step response method.
 *
 * Apply a known Vd step and measure the current slope.
 *   L ≈ Vd_step × Δt / ΔId
 * Repeated multiple times for averaging.
 */
static void Ident_SlowLoopLs(void)
{
    s_ident.tickCounter++;
    ident_ls_sub_t sub = (ident_ls_sub_t)s_ident.subState;

    switch (sub)
    {
    case LS_SUB_PRE_SETTLE:
        /* Hold zero current to let transients die out */
        s_ident.idTargetA = 0.0f;
        s_ident.lsStepVoltageActive = false;
        if (s_ident.tickCounter >= IDENT_LS_PRE_SETTLE_MS)
        {
            /* Record baseline current and start step */
            s_ident.lsIdBeforeStep = s_ident.idMeasuredA;
            s_ident.lsStepVoltageActive = true;
            s_ident.vdCommandV = s_ident.cfg.voltage_step_v;
            s_ident.lsVdStepApplied = s_ident.cfg.voltage_step_v;
            s_ident.piDIntegV = 0.0f; /* Not used during step */
            s_ident.subState = (uint8_t)LS_SUB_STEP_ACTIVE;
            s_ident.tickCounter = 0U;
        }
        break;

    case LS_SUB_STEP_ACTIVE:
        /* Keep step voltage applied via direct Vd override */
        if (s_ident.tickCounter >= IDENT_LS_STEP_MS)
        {
            /* Record final current */
            s_ident.lsIdAtEnd = s_ident.idMeasuredA;
            s_ident.lsStepVoltageActive = false;
            s_ident.vdCommandV = 0.0f;

            /* Compute L for this repetition:
             * During the step: V = L·dI/dt + R·I
             * For short steps where I is still small:
             *   L ≈ (V_step - Rs·I_avg) × Δt / ΔI
             * We use Rs from Phase 1 for correction. */
            float deltaI = s_ident.lsIdAtEnd - s_ident.lsIdBeforeStep;
            float deltaT = (float)IDENT_LS_STEP_MS * s_ident.cfg.slow_dt_s;
            float iAvg   = 0.5f * (s_ident.lsIdBeforeStep + s_ident.lsIdAtEnd);
            float vNet   = s_ident.lsVdStepApplied -
                           (s_ident.result.rs_ohm * iAvg);

            if (Ident_Fabsf(deltaI) > 0.05f)
            {
                float lEst = (vNet * deltaT) / deltaI;
                if (lEst > 1.0e-7f && lEst < 1.0f)
                {
                    s_ident.lsSumLH += lEst;
                    if (s_ident.lsValidCount < 8U)
                    {
                        s_ident.lsSamples[s_ident.lsValidCount] = lEst;
                        s_ident.lsValidCount++;
                    }
                }
            }
            s_ident.lsRepCount++;

            if (s_ident.lsRepCount >= IDENT_LS_REPEAT_COUNT)
            {
                s_ident.subState = (uint8_t)LS_SUB_DONE;
            }
            else
            {
                /* Pause then repeat */
                s_ident.subState = (uint8_t)LS_SUB_PAUSE;
                s_ident.tickCounter = 0U;
            }
        }
        break;

    case LS_SUB_PAUSE:
        s_ident.idTargetA = 0.0f;
        if (s_ident.tickCounter >= IDENT_LS_PAUSE_MS)
        {
            s_ident.subState = (uint8_t)LS_SUB_PRE_SETTLE;
            s_ident.tickCounter = 0U;
        }
        break;

    case LS_SUB_DONE:
    default:
        break;
    }

    /* Compute final Ls when done — use median for robustness */
    if (sub != LS_SUB_DONE && (ident_ls_sub_t)s_ident.subState == LS_SUB_DONE)
    {
        if (s_ident.lsValidCount >= 3U)
        {
            /* Simple insertion sort on valid samples */
            uint8_t n = s_ident.lsValidCount;
            for (uint8_t i = 1U; i < n; i++)
            {
                float key = s_ident.lsSamples[i];
                int8_t j = (int8_t)i - 1;
                while (j >= 0 && s_ident.lsSamples[j] > key)
                {
                    s_ident.lsSamples[j + 1] = s_ident.lsSamples[j];
                    j--;
                }
                s_ident.lsSamples[j + 1] = key;
            }
            /* Median: middle element (or average of two middle) */
            if (n % 2U == 1U)
            {
                s_ident.result.ls_h = s_ident.lsSamples[n / 2U];
            }
            else
            {
                s_ident.result.ls_h = 0.5f * (s_ident.lsSamples[n / 2U - 1U]
                                             + s_ident.lsSamples[n / 2U]);
            }

            if (s_ident.result.ls_h > 1.0e-7f && s_ident.result.ls_h < 1.0f)
            {
                s_ident.result.ls_valid = true;
                /* Update PI gains for lambda phase using measured Rs/Ls */
                float bw = 500.0f * IDENT_TWO_PI_F; /* ~500 Hz bandwidth */
                s_ident.piKp = s_ident.result.ls_h * bw;
                s_ident.piKi = s_ident.result.rs_ohm * bw;
                Ident_EnterLambda();
            }
            else
            {
                s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
            }
        }
        else
        {
            s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
        }
    }
}

/**
 * @brief Check drag quality during lambda settle/measure phase.
 *
 * Accumulates |Iq_target - Iq_measured| and checks both current tracking
 * and Vq BEMF presence after enough samples.
 *
 * @return true if drag is verified OK, false if still collecting or failed.
 */
static bool Ident_CheckDragQuality(void)
{
    float iqErr = Ident_Fabsf(s_ident.iqTargetA - s_ident.iqMeasuredA);
    s_ident.dragIqErrSum += iqErr;
    s_ident.dragCheckCount++;

    if (s_ident.dragCheckCount >= IDENT_DRAG_CHECK_WINDOW)
    {
        float avgIqErr = s_ident.dragIqErrSum / (float)s_ident.dragCheckCount;

        /* Check 1: Iq tracking error within acceptable limits */
        if (avgIqErr > IDENT_DRAG_IQ_ERR_LIMIT)
        {
            return false;  /* PI can't track → motor likely stalled */
        }

        /* Check 2: Vq should exceed Rs*Iq by a margin (BEMF present) */
        float rsIq = s_ident.result.rs_ohm * Ident_Fabsf(s_ident.iqMeasuredA);
        if (Ident_Fabsf(s_ident.vqCommandV) < rsIq * IDENT_DRAG_BEMF_MARGIN)
        {
            return false;  /* No BEMF → rotor not spinning */
        }

        s_ident.dragVerified = true;
    }

    return true;
}

/**
 * @brief Attempt to retry lambda identification with increased Iq.
 * @return true if retry was initiated, false if max retries exhausted.
 */
static bool Ident_RetryLambdaWithMoreCurrent(void)
{
    s_ident.dragRetryCount++;

    if (s_ident.dragRetryCount >= IDENT_DRAG_MAX_RETRIES)
    {
        return false;  /* Exhausted retries */
    }

    /* Increment Iq by the configured step */
    s_ident.lambdaIqNowA += s_ident.cfg.lambda_iq_step_a;
    if (s_ident.lambdaIqNowA > s_ident.cfg.lambda_iq_max_a)
    {
        s_ident.lambdaIqNowA = s_ident.cfg.lambda_iq_max_a;
    }

    /* Signal brief DRAG_FAIL phase for telemetry visibility */
    s_ident.phase = MOTOR_IDENT_PHASE_DRAG_FAIL;

    /* Re-enter lambda with increased current */
    Ident_EnterLambda();
    return true;
}

/**
 * @brief Lambda slow-loop: open-loop rotation method.
 *
 * At steady-state: Vq = Rs·Iq + ωe·λ  →  λ = (Vq - Rs·Iq) / ωe
 *
 * Includes forced-drag verification:
 *   - During settle: checks Iq tracking error and Vq BEMF presence.
 *   - On drag failure: retries with increased Iq (adaptive current).
 */
static void Ident_SlowLoopLambda(void)
{
    s_ident.tickCounter++;
    ident_lambda_sub_t sub = (ident_lambda_sub_t)s_ident.subState;
    const float targetSpeed = s_ident.lambdaSpeedRadS;

    switch (sub)
    {
    case LAMBDA_SUB_RAMP:
    {
        /* Linearly ramp speed from 0 to target */
        float rampFrac = (float)s_ident.tickCounter / (float)IDENT_LAMBDA_RAMP_MS;
        if (rampFrac > 1.0f) rampFrac = 1.0f;
        s_ident.speedRadS = rampFrac * targetSpeed;

        if (s_ident.tickCounter >= IDENT_LAMBDA_RAMP_MS)
        {
            s_ident.subState = (uint8_t)LAMBDA_SUB_SETTLE;
            s_ident.tickCounter = 0U;
            /* Reset drag quality check for settle phase */
            s_ident.dragIqErrSum = 0.0f;
            s_ident.dragCheckCount = 0U;
        }
        break;
    }

    case LAMBDA_SUB_SETTLE:
        s_ident.speedRadS = targetSpeed;

        /* Run drag quality check during settle */
        if (s_ident.tickCounter >= (IDENT_LAMBDA_SETTLE_MS / 2U))
        {
            if (!Ident_CheckDragQuality())
            {
                /* Drag failed — try with more current */
                s_ident.speedRadS = 0.0f;
                s_ident.idTargetA = 0.0f;
                s_ident.iqTargetA = 0.0f;

                if (!Ident_RetryLambdaWithMoreCurrent())
                {
                    /* All retries exhausted */
                    s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
                }
                return;
            }
        }

        if (s_ident.tickCounter >= IDENT_LAMBDA_SETTLE_MS)
        {
            s_ident.subState = (uint8_t)LAMBDA_SUB_MEASURE;
            s_ident.tickCounter = 0U;
            s_ident.lambdaVqSum = 0.0f;
            s_ident.lambdaIqSum = 0.0f;
            s_ident.lambdaSampleCount = 0U;
        }
        break;

    case LAMBDA_SUB_MEASURE:
        s_ident.lambdaVqSum += s_ident.vqCommandV;
        s_ident.lambdaIqSum += s_ident.iqMeasuredA;
        s_ident.lambdaSampleCount++;
        if (s_ident.tickCounter >= IDENT_LAMBDA_MEASURE_MS)
        {
            s_ident.subState = (uint8_t)LAMBDA_SUB_DONE;
        }
        break;

    case LAMBDA_SUB_DONE:
    default:
        break;
    }

    /* Compute lambda when done */
    if (sub != LAMBDA_SUB_DONE &&
        (ident_lambda_sub_t)s_ident.subState == LAMBDA_SUB_DONE)
    {
        float n = (float)s_ident.lambdaSampleCount;
        if (n > 0.0f && Ident_Fabsf(targetSpeed) > IDENT_EPSILON_F)
        {
            float vqAvg = s_ident.lambdaVqSum / n;
            float iqAvg = s_ident.lambdaIqSum / n;
            float lambda = (vqAvg - s_ident.result.rs_ohm * iqAvg) / targetSpeed;

            if (lambda > 1.0e-5f && lambda < 1.0f)
            {
                s_ident.result.lambda_vs = lambda;
                s_ident.result.lambda_valid = true;
                s_ident.phase = MOTOR_IDENT_PHASE_COMPLETE;
            }
            else
            {
                /* Lambda out of range — may need more current to spin */
                if (!Ident_RetryLambdaWithMoreCurrent())
                {
                    s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
                }
            }
        }
        else
        {
            s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
        }

        /* Stop rotation (regardless of outcome) */
        s_ident.speedRadS = 0.0f;
        s_ident.idTargetA = 0.0f;
        s_ident.iqTargetA = 0.0f;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 *  Public API implementation
 * ══════════════════════════════════════════════════════════════════════════ */

void MotorParamIdent_Init(void)
{
    (void)memset(&s_ident, 0, sizeof(s_ident));
    s_ident.phase = MOTOR_IDENT_PHASE_IDLE;
}

bool MotorParamIdent_Start(const motor_ident_config_t *config)
{
    if (config == NULL)
    {
        return false;
    }

    if (s_ident.phase != MOTOR_IDENT_PHASE_IDLE &&
        s_ident.phase != MOTOR_IDENT_PHASE_COMPLETE &&
        s_ident.phase != MOTOR_IDENT_PHASE_DRAG_FAIL &&
        s_ident.phase != MOTOR_IDENT_PHASE_ERROR)
    {
        return false;  /* Already running */
    }

    (void)memset(&s_ident, 0, sizeof(s_ident));
    s_ident.cfg = *config;

    /* Compute target electrical speed from pole pairs and RPM */
    s_ident.lambdaSpeedRadS = Ident_ComputeLambdaSpeed(config);

    /* Initialize adaptive Iq at minimum level */
    s_ident.lambdaIqNowA = config->lambda_iq_min_a;
    s_ident.dragRetryCount = 0U;

    /* Initial PI gains: use conservative estimate assuming typical motor.
     * These will be refined after Rs/Ls are measured. */
    float initBw = 300.0f * IDENT_TWO_PI_F; /* ~300 Hz initial bandwidth */
    s_ident.piKp = 0.0002f * initBw;   /* ~0.4 V/A  (for ~200µH typical) */
    s_ident.piKi = 0.2f * initBw;      /* ~377 V/A/s (for ~0.2Ω typical)  */

    Ident_EnterRs();
    return true;
}

void MotorParamIdent_Abort(void)
{
    s_ident.phase = MOTOR_IDENT_PHASE_IDLE;
    s_ident.speedRadS = 0.0f;
    s_ident.idTargetA = 0.0f;
    s_ident.iqTargetA = 0.0f;
    s_ident.vdCommandV = 0.0f;
    s_ident.vqCommandV = 0.0f;
    s_ident.piDIntegV = 0.0f;
    s_ident.piQIntegV = 0.0f;
    s_ident.lsStepVoltageActive = false;
}

void MotorParamIdent_RunFastLoop(const motor_ident_fast_input_t *input,
                                 motor_ident_fast_output_t *output)
{
    if (input == NULL || output == NULL)
    {
        return;
    }

    /* Safe default: 50% duty (zero voltage) */
    output->duty_u = 0.5f;
    output->duty_v = 0.5f;
    output->duty_w = 0.5f;

    if (s_ident.phase == MOTOR_IDENT_PHASE_IDLE ||
        s_ident.phase == MOTOR_IDENT_PHASE_COMPLETE ||
        s_ident.phase == MOTOR_IDENT_PHASE_ERROR)
    {
        return;
    }

    /* Overcurrent protection */
    if (Ident_Fabsf(input->phase_current_a) > s_ident.cfg.overcurrent_a ||
        Ident_Fabsf(input->phase_current_b) > s_ident.cfg.overcurrent_a ||
        Ident_Fabsf(input->phase_current_c) > s_ident.cfg.overcurrent_a)
    {
        s_ident.phase = MOTOR_IDENT_PHASE_ERROR;
        return;
    }

    const float vbus = Ident_Clamp(input->bus_voltage_v, 1.0f, 1000.0f);

    /* Update open-loop angle */
    s_ident.angleRad = Ident_WrapAngle(
        s_ident.angleRad + (s_ident.speedRadS * s_ident.cfg.pwm_dt_s));

    /* Clarke + Park: measure Id/Iq */
    float iAlpha, iBeta;
    float sinA, cosA;
    Ident_Clarke(input->phase_current_a, input->phase_current_b,
                 &iAlpha, &iBeta);
    Ident_SinCos(s_ident.angleRad, &sinA, &cosA);
    Ident_Park(iAlpha, iBeta, sinA, cosA,
               &s_ident.idMeasuredA, &s_ident.iqMeasuredA);

    /* Current PI → Vd/Vq */
    Ident_RunCurrentPi(vbus);

    /* Inverse Park + SVM → duties */
    float vAlpha, vBeta;
    Ident_InvPark(s_ident.vdCommandV, s_ident.vqCommandV,
                  sinA, cosA, &vAlpha, &vBeta);
    Ident_Svm(vAlpha, vBeta, vbus,
              &output->duty_u, &output->duty_v, &output->duty_w);
}

void MotorParamIdent_RunSlowLoop(void)
{
    switch (s_ident.phase)
    {
    case MOTOR_IDENT_PHASE_RS:
        Ident_SlowLoopRs();
        break;

    case MOTOR_IDENT_PHASE_LS:
        Ident_SlowLoopLs();
        break;

    case MOTOR_IDENT_PHASE_LAMBDA:
    case MOTOR_IDENT_PHASE_DRAG_FAIL:  /* Retry re-enters LAMBDA internally */
        Ident_SlowLoopLambda();
        break;

    case MOTOR_IDENT_PHASE_IDLE:
    case MOTOR_IDENT_PHASE_COMPLETE:
    case MOTOR_IDENT_PHASE_ERROR:
    default:
        break;
    }
}

motor_ident_phase_t MotorParamIdent_GetPhase(void)
{
    return s_ident.phase;
}

const motor_ident_result_t *MotorParamIdent_GetResults(void)
{
    return &s_ident.result;
}

void MotorParamIdent_GetTelemetry(motor_ident_telemetry_t *telem)
{
    if (telem == NULL) return;

    telem->phase       = s_ident.phase;
    telem->rs_ohm      = s_ident.result.rs_ohm;
    telem->ls_h        = s_ident.result.ls_h;
    telem->lambda_vs   = s_ident.result.lambda_vs;
    telem->id_measured = s_ident.idMeasuredA;
    telem->iq_measured = s_ident.iqMeasuredA;
    telem->vd_command  = s_ident.vdCommandV;
    telem->vq_command  = s_ident.vqCommandV;
    telem->drag_ok     = s_ident.dragVerified;
    telem->drag_retry  = s_ident.dragRetryCount;
    telem->lambda_iq_now = s_ident.lambdaIqNowA;

    /* Compute progress percentage */
    uint8_t prog = 0U;
    switch (s_ident.phase)
    {
    case MOTOR_IDENT_PHASE_RS:
    {
        uint32_t totalMs = 2U * (IDENT_RS_SETTLE_MS + IDENT_RS_MEASURE_MS);
        uint32_t elapsed = s_ident.tickCounter;
        ident_rs_sub_t sub = (ident_rs_sub_t)s_ident.subState;
        if (sub >= RS_SUB_NEG_SETTLE)
        {
            elapsed += IDENT_RS_SETTLE_MS + IDENT_RS_MEASURE_MS;
        }
        prog = (uint8_t)((elapsed * 100U) / (totalMs > 0U ? totalMs : 1U));
        break;
    }
    case MOTOR_IDENT_PHASE_LS:
        if (IDENT_LS_REPEAT_COUNT > 0U)
        {
            prog = (uint8_t)((s_ident.lsRepCount * 100U) /
                             IDENT_LS_REPEAT_COUNT);
        }
        break;
    case MOTOR_IDENT_PHASE_LAMBDA:
    case MOTOR_IDENT_PHASE_DRAG_FAIL:
    {
        uint32_t totalMs = IDENT_LAMBDA_RAMP_MS + IDENT_LAMBDA_SETTLE_MS +
                           IDENT_LAMBDA_MEASURE_MS;
        uint32_t elapsed = s_ident.tickCounter;
        ident_lambda_sub_t sub = (ident_lambda_sub_t)s_ident.subState;
        if (sub >= LAMBDA_SUB_SETTLE)
        {
            elapsed += IDENT_LAMBDA_RAMP_MS;
        }
        if (sub >= LAMBDA_SUB_MEASURE)
        {
            elapsed += IDENT_LAMBDA_SETTLE_MS;
        }
        prog = (uint8_t)((elapsed * 100U) / (totalMs > 0U ? totalMs : 1U));
        break;
    }
    case MOTOR_IDENT_PHASE_COMPLETE:
        prog = 100U;
        break;
    default:
        break;
    }
    telem->progress = (prog > 100U) ? 100U : prog;
}
