/**
 * @file motor_control.c
 * @brief High-level motor state machine and profiling implementation.
 */

#include "motor_control.h"

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "can_config.h"
#include "device_registers.h"
#include "motor_foc.h"
#include "motor_hw_ytm32.h"
#include "motor_param_ident.h"
#include "motor_user_config.h"
#include "sdk_project_config.h"

#define MOTOR_CTRL_DWT_LAR_KEY (0xC5ACCE55UL)
#define MOTOR_CTRL_DWT_LAR (*((volatile uint32_t *)(DWT_BASE + 0xFB0UL)))
#define MOTOR_CTRL_EPSILON_F (1.0e-6f)
#define MOTOR_CTRL_HFI_MAX_CANDIDATES (8U)

typedef enum {
  MOTOR_HFI_PHASE_IDLE = 0,
  MOTOR_HFI_PHASE_COARSE_SCAN,
  MOTOR_HFI_PHASE_FINE_SCAN,
  MOTOR_HFI_PHASE_POLARITY_POS,
  MOTOR_HFI_PHASE_POLARITY_NEG,
} motor_hfi_phase_t;

typedef struct {
  motor_hfi_phase_t phase;
  bool command_valid;
  int8_t command_sign;
  uint8_t candidate_count;
  uint8_t active_index;
  uint16_t sample_count;
  uint16_t sample_target;
  float command_angle_rad;
  float command_voltage_v;
  float accum_metric;
  float accum_abs_current;
  float candidate_angles[MOTOR_CTRL_HFI_MAX_CANDIDATES];
  float candidate_metrics[MOTOR_CTRL_HFI_MAX_CANDIDATES];
  float coarse_step_rad;
  float best_metric;
  float best_angle_rad;
  float scan_confidence;
  float polarity_positive_metric;
  float polarity_negative_metric;
} motor_hfi_ipd_ctx_t;

typedef struct {
  motor_status_t status;
  motor_foc_state_t foc;
  bool enableRequest;
  bool angleMonitorRequest;
  bool angleMonitorSeeded;
  bool fastLoopRunning;
  bool outputsUnmasked;
  uint16_t offsetSampleCount;
  float currentOffsetASum;
  float currentOffsetBSum;
  float currentOffsetCSum;
  float currentOffsetARaw;
  float currentOffsetBRaw;
  float currentOffsetCRaw;
  uint32_t stateTimeMs;
  uint32_t startupTimeMs;
  uint32_t observerLockCount;
  uint32_t observerLossCount;
  float openLoopAngleRad;
  float openLoopSpeedRadS;
  float openLoopFilteredCurrentA;
  float angleMonitorBemfAngleRad;
  float closedLoopBlend;
  float latestPhaseErrorRad;
  float latestObserverAngleRad;
  float observerPhaseOffsetRad;
  float observerLockResidualRad;
  motor_control_mode_t controlModeRequest;
  float targetRpmCommand;
  float targetIqCommandA;
  uint32_t tickMs;

  /* Wind detect / catch spin */
  float windDetectSpeedRadS;     /* Detected initial electrical speed */
  float windDetectPrevSpeedRadS; /* Previous speed for convergence check */
  int8_t windDetectDirection; /* Detected rotation direction +1/-1, 0=still */
  uint32_t windDetectSettleCount; /* Observer convergence counter */
  bool windCatchActive;           /* Tailwind catch in progress */

  /* BEMF voltage zero-crossing detection (for passive wind detect) */
  int8_t bemfSignPrev[3]; /* Previous sign of each BEMF phase (-1/0/+1) */
  uint32_t bemfZeroCrossingCount; /* Total zero crossings detected */
  uint32_t bemfFirstCrossingTick; /* ISR tick of the first zero crossing */
  uint32_t bemfLastCrossingTick;  /* ISR tick of the most recent crossing */
  uint32_t bemfIsrTickCounter; /* Running ISR tick counter during wind detect */
  int8_t bemfPhaseSequence;    /* +1 = forward, -1 = reverse, 0 = unknown */

  /* Last BEMF raw values (cached from ISR for telemetry) */
  uint16_t bemfLastU;
  uint16_t bemfLastV;
  uint16_t bemfLastW;
  uint16_t bemfLastCom;

  /* Field weakening */
  float fwIdTargetA;            /* Field weakening d-axis current command */
  float voltageModulationRatio; /* Latest |Vab|/Vbus from FOC output */

  /* Startup retry / stall detection */
  uint8_t startupRetryCount;   /* Current retry attempt number */
  float startupIqBoostA;       /* Accumulated Iq boost for retries (A) */
  uint32_t stallAngleDivCount; /* Consecutive angle-divergence counter */

  /* Parameter identification */
  bool paramIdentRequested; /* Trigger flag for param ident after offset cal */

  /* HFI / initial position detection */
  motor_hfi_ipd_ctx_t hfi;
  float hfiInitialAngleRad;
} motor_control_ctx_t;

static motor_control_ctx_t s_motorCtrl;
volatile motor_fast_loop_profile_t g_motorFastLoopProfile;

/**
 * @brief Clamp a float value between a minimum and maximum.
 * @param value     Input value.
 * @param minValue  Lower bound.
 * @param maxValue  Upper bound.
 * @return Clamped float value.
 */
static float MotorControl_Clamp(float value, float minValue, float maxValue) {
  float result = value;

  if (result < minValue) {
    result = minValue;
  } else if (result > maxValue) {
    result = maxValue;
  }

  return result;
}

/**
 * @brief Fast floating-point absolute value helper.
 * @param value Input value.
 * @return Absolute value.
 */
static inline float MotorControl_FastAbs(float value) {
  return __builtin_fabsf(value);
}

/**
 * @brief Fast non-negative square root helper.
 * @param value Input value.
 * @return Square root with negative inputs clamped to zero.
 */
static inline float MotorControl_FastSqrt(float value) {
  float safeValue = value;

  if (safeValue < 0.0f) {
    safeValue = 0.0f;
  }

  return __builtin_sqrtf(safeValue);
}

/**
 * @brief Lightweight atan2 approximation for ISR-side BEMF angle monitoring.
 * @param y Beta-axis component.
 * @param x Alpha-axis component.
 * @return Angle in radians [-pi, pi].
 */
static float MotorControl_FastAtan2(float y, float x) {
  const float quarterPi = 0.78539816339744830962f;
  const float threeQuarterPi = 2.35619449019234492885f;
  const float absY = MotorControl_FastAbs(y) + MOTOR_CTRL_EPSILON_F;
  float angle;
  float ratio;

  if (x >= 0.0f) {
    ratio = (x - absY) / (x + absY);
    angle = quarterPi - (quarterPi * ratio);
  } else {
    ratio = (x + absY) / (absY - x);
    angle = threeQuarterPi - (quarterPi * ratio);
  }

  if (y < 0.0f) {
    angle = -angle;
  }

  return angle;
}

/**
 * @brief Check if the requested control mode is valid.
 * @param mode The control mode enum.
 * @return True if supported.
 */
static bool MotorControl_IsValidControlMode(motor_control_mode_t mode) {
  return (mode == MOTOR_CONTROL_MODE_SPEED) ||
         (mode == MOTOR_CONTROL_MODE_CURRENT);
}

/**
 * @brief Clamp the target Q-axis current to the configured motor limits.
 * @param targetIqA Requested IQ in amperes.
 * @return Clamped IQ in amperes.
 */
static float MotorControl_ClampIqTarget(float targetIqA) {
  return MotorControl_Clamp(targetIqA, -MOTOR_CFG_MAX_IQ_A, MOTOR_CFG_MAX_IQ_A);
}

/**
 * @brief Record execution cycle counts into a statistics accumulator.
 * @param stat   Pointer to the statistics structure.
 * @param cycles Value of the cycles measured.
 */
static inline void
MotorControl_ProfileRecordStat(volatile motor_cycle_stat_t *stat,
                               uint32_t cycles) {
  stat->last_cycles = cycles;
  if ((stat->sample_count == 0U) || (cycles < stat->min_cycles)) {
    stat->min_cycles = cycles;
  }
  if (cycles > stat->max_cycles) {
    stat->max_cycles = cycles;
  }

  stat->total_cycles += cycles;
  stat->sample_count++;

  if (stat->sample_count <= 1U) {
    stat->avg_cycles = cycles;
  } else {
    stat->avg_cycles += ((int32_t)cycles - (int32_t)stat->avg_cycles) >> 4;
  }
}

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
static bool MotorControl_EnableDwtCycleCounter(void) {
  bool enabled = false;

  SystemCoreClockUpdate();

  if ((DWT->CTRL & DWT_CTRL_NOCYCCNT_Msk) == 0U) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    if (DWT->LSR != 0U) {
      MOTOR_CTRL_DWT_LAR = MOTOR_CTRL_DWT_LAR_KEY;
    }
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    enabled = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
  }

  return enabled;
}
#endif

/**
 * @brief Safely step a value toward a target by a limited slew amount.
 * @param currentValue The current value.
 * @param targetValue  The desired limit.
 * @param step         The maximum allowable step size (must be >= 0).
 * @return The updated value.
 */
static float MotorControl_SlewTowards(float currentValue, float targetValue,
                                      float step) {
  if (currentValue < targetValue) {
    currentValue += step;
    if (currentValue > targetValue) {
      currentValue = targetValue;
    }
  } else if (currentValue > targetValue) {
    currentValue -= step;
    if (currentValue < targetValue) {
      currentValue = targetValue;
    }
  }

  return currentValue;
}

/**
 * @brief Wrap an angle to the range (-pi, pi].
 * @param angleRad Input angle in radians.
 * @return Wrapped angle.
 */
static float MotorControl_WrapAngleSigned(float angleRad) {
  float wrappedAngle = MotorFoc_WrapAngle0ToTwoPi(angleRad);

  if (wrappedAngle > MOTOR_CFG_PI_F) {
    wrappedAngle -= MOTOR_CFG_TWO_PI_F;
  }

  return wrappedAngle;
}

/**
 * @brief Wrap an angle into the saliency-symmetric range [0, pi).
 * @param angleRad Input angle.
 * @return Wrapped angle in [0, pi).
 */
static float MotorControl_WrapAngle0ToPi(float angleRad) {
  float wrappedAngle = MotorFoc_WrapAngle0ToTwoPi(angleRad);

  if (wrappedAngle >= MOTOR_CFG_PI_F) {
    wrappedAngle -= MOTOR_CFG_PI_F;
  }

  return wrappedAngle;
}

/**
 * @brief Max helper for three floats.
 * @return Maximum of a, b and c.
 */
static float MotorControl_Max3(float a, float b, float c) {
  float result = a;

  if (b > result) {
    result = b;
  }
  if (c > result) {
    result = c;
  }

  return result;
}

/**
 * @brief Min helper for three floats.
 * @return Minimum of a, b and c.
 */
static float MotorControl_Min3(float a, float b, float c) {
  float result = a;

  if (b < result) {
    result = b;
  }
  if (c < result) {
    result = c;
  }

  return result;
}

/**
 * @brief Clarke transform for HFI / startup helper paths.
 * @param ia Phase-A current.
 * @param ib Phase-B current.
 * @param ic Phase-C current.
 * @param iab Output alpha-beta current vector.
 */
static void MotorControl_Clarke(float ia, float ib, float ic,
                                motor_ab_frame_t *iab) {
  (void)ic;
  iab->alpha = ia;
  iab->beta = (ia + ib + ib) * MOTOR_CFG_INV_SQRT3_F;
}

/**
 * @brief Project an alpha-beta current vector onto a test angle.
 * @param iab Alpha-beta current vector.
 * @param angleRad Projection angle.
 * @return Current component along the projection angle.
 */
static float MotorControl_ProjectAlongAngle(const motor_ab_frame_t *iab,
                                            float angleRad) {
  return (iab->alpha * __builtin_cosf(angleRad)) +
         (iab->beta * __builtin_sinf(angleRad));
}

/**
 * @brief Convert a stationary-frame voltage vector to duty cycles.
 * @param angleRad Target vector angle in radians.
 * @param magnitudeV Target voltage magnitude in volts.
 * @param busVoltageV Available DC bus voltage in volts.
 */
static void MotorControl_ApplyVoltageVector(float angleRad, float magnitudeV,
                                            float busVoltageV) {
  const float busClampedV = MotorControl_Clamp(busVoltageV, 1.0f, 1000.0f);
  const float maxVoltageV = busClampedV * MOTOR_CFG_SVM_MAX_MODULATION;
  const float limitedMagnitudeV =
      MotorControl_Clamp(magnitudeV, -maxVoltageV, maxVoltageV);
  const float vabAlpha = limitedMagnitudeV * __builtin_cosf(angleRad);
  const float vabBeta = limitedMagnitudeV * __builtin_sinf(angleRad);
  const float inverseBus = 1.0f / busClampedV;
  const float phaseU = vabAlpha * inverseBus;
  const float phaseV =
      ((-0.5f * vabAlpha) + (MOTOR_CFG_SQRT3_BY_2_F * vabBeta)) * inverseBus;
  const float phaseW =
      ((-0.5f * vabAlpha) - (MOTOR_CFG_SQRT3_BY_2_F * vabBeta)) * inverseBus;
  const float phaseMax = MotorControl_Max3(phaseU, phaseV, phaseW);
  const float phaseMin = MotorControl_Min3(phaseU, phaseV, phaseW);
  const float offset = 0.5f - (0.5f * (phaseMax + phaseMin));
  const float dutyU = MotorControl_Clamp(phaseU + offset, 0.0f, 1.0f);
  const float dutyV = MotorControl_Clamp(phaseV + offset, 0.0f, 1.0f);
  const float dutyW = MotorControl_Clamp(phaseW + offset, 0.0f, 1.0f);

  MotorHwYtm32_ApplyPhaseDuty(dutyU, dutyV, dutyW);
}

/**
 * @brief Transition to a new motor control state.
 * @param nextState The state to transition to.
 */
static void MotorControl_SetState(motor_control_state_t nextState) {
  s_motorCtrl.status.state = nextState;
  s_motorCtrl.stateTimeMs = 0U;
  s_motorCtrl.status.hfi_active = (nextState == MOTOR_STATE_HFI_IPD);
}



/**
 * @brief Return sign multiplier for mechanical directions (-1.0f or 1.0f).
 * @return Multiplier float.
 */
static float MotorControl_GetSignedDirection(void) {
  return (s_motorCtrl.status.direction >= 0) ? 1.0f : -1.0f;
}

/**
 * @brief Get the configured alignment angle adjusted for direction.
 * @return Target electrical angle for alignment in radians.
 */
static inline float MotorControl_GetAlignAngle(void) {
  float angleRad = MOTOR_CFG_ALIGN_ANGLE_RAD;

  if (s_motorCtrl.status.hfi_used && s_motorCtrl.status.hfi_angle_valid) {
    angleRad = s_motorCtrl.hfiInitialAngleRad;
  }

  if (s_motorCtrl.status.direction < 0) {
    angleRad += MOTOR_CFG_PI_F;
  }

  return MotorFoc_WrapAngle0ToTwoPi(angleRad);
}

/**
 * @brief Read the active alignment current, honoring CAN calibration overrides.
 * @return Alignment d-axis current in amperes.
 */
static float MotorControl_GetConfiguredAlignCurrentA(void) {
  float alignCurrentA = MOTOR_CFG_ALIGN_CURRENT_A;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->align_current_a > 0.0f)) {
    alignCurrentA = calib->align_current_a;
  }

  return MotorControl_Clamp(alignCurrentA, 0.0f, MOTOR_CFG_MAX_IQ_A);
}

/**
 * @brief Read the active open-loop startup current limit.
 * @return Open-loop q-axis current limit in amperes.
 */
static float MotorControl_GetConfiguredOpenLoopIqA(void) {
  float openLoopIqA = MOTOR_CFG_OPEN_LOOP_IQ_A;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->open_loop_iq_a > 0.0f)) {
    openLoopIqA = calib->open_loop_iq_a;
  }

  return MotorControl_Clamp(openLoopIqA, 0.0f, MOTOR_CFG_MAX_IQ_A);
}

/**
 * @brief Alignment current target including retry boost.
 * @return Target d-axis current in amperes.
 */
static float MotorControl_GetAlignCurrentTargetA(void) {
  return MotorControl_Clamp(
      MotorControl_GetConfiguredAlignCurrentA() +
          (s_motorCtrl.startupIqBoostA * 0.5f),
      0.0f, MOTOR_CFG_MAX_IQ_A);
}

/**
 * @brief Startup q-axis current limit including retry boost.
 * @return Target startup q-axis current magnitude in amperes.
 */
static float MotorControl_GetStartupIqLimitA(void) {
  return MotorControl_Clamp(
      MotorControl_GetConfiguredOpenLoopIqA() + s_motorCtrl.startupIqBoostA,
      0.0f, MOTOR_CFG_MAX_IQ_A);
}

/**
 * @brief Magnitude of the measured dq startup current vector.
 * @return Current magnitude in amperes.
 */
static float MotorControl_GetStartupCurrentMagnitudeA(void) {
  return __builtin_sqrtf((s_motorCtrl.status.id_a * s_motorCtrl.status.id_a) +
                         (s_motorCtrl.status.iq_a * s_motorCtrl.status.iq_a));
}

/**
 * @brief Read whether the HFI/IPD pre-positioning path is enabled.
 * @return True if static HFI/IPD should be attempted.
 */
static bool MotorControl_IsHfiEnabled(void) {
  bool enabled = (MOTOR_CFG_ENABLE_HFI_IPD != 0U);
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if (calib != NULL) {
    enabled = calib->hfi_enable;
  }

  return enabled;
}

/**
 * @brief Read the configured bipolar HFI burst amplitude.
 * @param busVoltageV Present DC bus voltage in volts.
 * @return Clamped injection voltage magnitude in volts.
 */
static float MotorControl_GetConfiguredHfiInjectVoltageV(float busVoltageV) {
  float injectVoltageV = MOTOR_CFG_HFI_IPD_INJECT_V;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_inject_voltage_v > 0.0f)) {
    injectVoltageV = calib->hfi_inject_voltage_v;
  }

  return MotorControl_Clamp(
      injectVoltageV, 0.1f,
      MotorControl_Clamp(busVoltageV, 1.0f, 1000.0f) *
          MOTOR_CFG_SVM_MAX_MODULATION * 0.8f);
}

/**
 * @brief Read the configured polarity pulse amplitude.
 * @param busVoltageV Present DC bus voltage in volts.
 * @return Clamped polarity pulse voltage in volts.
 */
static float MotorControl_GetConfiguredHfiPolarityVoltageV(float busVoltageV) {
  float polarityVoltageV = MOTOR_CFG_HFI_IPD_POLARITY_V;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_polarity_voltage_v > 0.0f)) {
    polarityVoltageV = calib->hfi_polarity_voltage_v;
  }

  return MotorControl_Clamp(
      polarityVoltageV, 0.1f,
      MotorControl_Clamp(busVoltageV, 1.0f, 1000.0f) *
          MOTOR_CFG_SVM_MAX_MODULATION * 0.8f);
}

/**
 * @brief Read the configured HFI pulse pair count.
 * @return Pulse pair count per candidate.
 */
static uint8_t MotorControl_GetConfiguredHfiPulsePairs(void) {
  uint8_t pulsePairs = MOTOR_CFG_HFI_IPD_PULSE_PAIRS;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_pulse_pairs > 0U)) {
    pulsePairs = calib->hfi_pulse_pairs;
  }

  if (pulsePairs > 32U) {
    pulsePairs = 32U;
  }

  return (pulsePairs == 0U) ? 1U : pulsePairs;
}

/**
 * @brief Read the configured number of coarse HFI candidates.
 * @return Candidate count over [0, pi).
 */
static uint8_t MotorControl_GetConfiguredHfiCandidateCount(void) {
  uint8_t candidateCount = MOTOR_CFG_HFI_IPD_CANDIDATE_COUNT;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_candidate_count > 0U)) {
    candidateCount = calib->hfi_candidate_count;
  }

  if (candidateCount < 3U) {
    candidateCount = 3U;
  } else if (candidateCount > MOTOR_CTRL_HFI_MAX_CANDIDATES) {
    candidateCount = MOTOR_CTRL_HFI_MAX_CANDIDATES;
  }

  return candidateCount;
}

/**
 * @brief Read the configured HFI confidence threshold.
 * @return Confidence threshold in [0, 1].
 */
static float MotorControl_GetConfiguredHfiConfidenceMin(void) {
  float confidenceMin = MOTOR_CFG_HFI_IPD_CONFIDENCE_MIN;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_confidence_threshold > 0.0f)) {
    confidenceMin = calib->hfi_confidence_threshold;
  }

  return MotorControl_Clamp(confidenceMin, 0.01f, 1.0f);
}

/**
 * @brief Read the configured polarity pulse count.
 * @return Number of positive polarity pulse samples.
 */
static uint8_t MotorControl_GetConfiguredHfiPolarityPulseCount(void) {
  uint8_t pulseCount = MOTOR_CFG_HFI_IPD_POLARITY_PULSES;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_polarity_pulse_count > 0U)) {
    pulseCount = calib->hfi_polarity_pulse_count;
  }

  if (pulseCount > 32U) {
    pulseCount = 32U;
  }

  return (pulseCount == 0U) ? 1U : pulseCount;
}

/**
 * @brief Read the short align hold duration after HFI success.
 * @return Align hold in milliseconds.
 */
static uint16_t MotorControl_GetConfiguredHfiAlignHoldMs(void) {
  uint16_t alignHoldMs = MOTOR_CFG_HFI_IPD_ALIGN_HOLD_MS;
  const can_calib_params_t *calib = CanConfig_GetCalibParams();

  if ((calib != NULL) && (calib->hfi_align_hold_ms > 0U)) {
    alignHoldMs = calib->hfi_align_hold_ms;
  }

  return alignHoldMs;
}

/**
 * @brief Return the active align-state dwell time.
 * @return Align dwell time in milliseconds.
 */
static uint16_t MotorControl_GetAlignHoldTimeMs(void) {
  if (s_motorCtrl.status.hfi_used && s_motorCtrl.status.hfi_angle_valid) {
    return MotorControl_GetConfiguredHfiAlignHoldMs();
  }

  return MOTOR_CFG_ALIGN_TIME_MS;
}

/**
 * @brief Clear HFI/IPD runtime status and context.
 */
static void MotorControl_ResetHfiStatus(void) {
  (void)memset(&s_motorCtrl.hfi, 0, sizeof(s_motorCtrl.hfi));
  s_motorCtrl.hfiInitialAngleRad = 0.0f;
  s_motorCtrl.status.hfi_active = false;
  s_motorCtrl.status.hfi_angle_valid = false;
  s_motorCtrl.status.hfi_used = false;
  s_motorCtrl.status.hfi_fallback_reason = MOTOR_HFI_FALLBACK_NONE;
  s_motorCtrl.status.hfi_angle_rad = 0.0f;
  s_motorCtrl.status.hfi_confidence = 0.0f;
  s_motorCtrl.status.hfi_ripple_metric = 0.0f;
}

/**
 * @brief Reset passive BEMF angle-monitor telemetry fields.
 */
static void MotorControl_ResetAngleMonitorStatus(void) {
  s_motorCtrl.angleMonitorSeeded = false;
  s_motorCtrl.angleMonitorBemfAngleRad = 0.0f;
  s_motorCtrl.status.angle_monitor_active = false;
  s_motorCtrl.status.angle_monitor_valid = false;
  s_motorCtrl.status.angle_monitor_angle_rad = 0.0f;
  s_motorCtrl.status.angle_monitor_speed_rad_s = 0.0f;
  s_motorCtrl.status.angle_monitor_bemf_mag = 0U;
}

/**
 * @brief Convert target mechanical speed to electrical rad/s.
 * @return Target electrical speed with direction sign.
 */
static float MotorControl_GetTargetElectricalSpeedRadS(void) {
  return MotorControl_GetSignedDirection() *
         MOTOR_CFG_MECH_RPM_TO_ELEC_RAD_S(s_motorCtrl.status.target_rpm);
}

/**
 * @brief Linearly blend between two wrapping angles.
 * @param fromAngleRad Source angle.
 * @param toAngleRad   Destination angle.
 * @param blend        Blend fraction [0..1].
 * @return Interpolated angle in [0, 2*pi).
 */
static inline float MotorControl_BlendAngle(float fromAngleRad,
                                            float toAngleRad, float blend) {
  return MotorFoc_WrapAngle0ToTwoPi(
      fromAngleRad + (blend * MotorFoc_AngleDiff(toAngleRad, fromAngleRad)));
}

/**
 * @brief Linearly blend two signed angles (phases).
 * @param fromPhaseRad Source angle.
 * @param toPhaseRad   Destination angle.
 * @param blend        Blend fraction [0..1].
 * @return Interpolated signed angle (-pi, pi].
 */
static float MotorControl_BlendPhase(float fromPhaseRad, float toPhaseRad,
                                     float blend) {
  return MotorControl_WrapAngleSigned(
      fromPhaseRad + (blend * MotorFoc_AngleDiff(toPhaseRad, fromPhaseRad)));
}

/**
 * @brief Reset internal control vars related to active motor operation.
 */
static void MotorControl_ResetRuntime(void) {
  s_motorCtrl.fastLoopRunning = false;
  s_motorCtrl.outputsUnmasked = false;
  s_motorCtrl.stateTimeMs = 0U;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.observerLockCount = 0U;
  s_motorCtrl.observerLossCount = 0U;
  s_motorCtrl.closedLoopBlend = 0.0f;
  s_motorCtrl.latestPhaseErrorRad = 0.0f;
  s_motorCtrl.latestObserverAngleRad = 0.0f;
  s_motorCtrl.observerPhaseOffsetRad = 0.0f;
  s_motorCtrl.observerLockResidualRad = 0.0f;
  s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
  s_motorCtrl.openLoopSpeedRadS = 0.0f;
  s_motorCtrl.openLoopFilteredCurrentA = 0.0f;
  s_motorCtrl.angleMonitorBemfAngleRad = 0.0f;
  s_motorCtrl.windDetectSpeedRadS = 0.0f;
  s_motorCtrl.windDetectPrevSpeedRadS = 0.0f;
  s_motorCtrl.windDetectDirection = 0;
  s_motorCtrl.windDetectSettleCount = 0U;
  s_motorCtrl.windCatchActive = false;
  s_motorCtrl.fwIdTargetA = 0.0f;
  s_motorCtrl.voltageModulationRatio = 0.0f;
  s_motorCtrl.stallAngleDivCount = 0U;
  MotorControl_ResetHfiStatus();
  MotorControl_ResetAngleMonitorStatus();
  /* NOTE: startupRetryCount and startupIqBoostA are NOT reset here —
   * they must persist across retry attempts within a single enable cycle.
   * They are only cleared in MotorControl_EnterStop(). */
  MotorFoc_Reset(&s_motorCtrl.foc);
}

/**
 * @brief Sub-routine to cleanly enter the Stopped state.
 * Halts PWM and masks HW timer channels.
 */
static void MotorControl_EnterStop(void) {
  MotorHwYtm32_DisableFastLoopSampling();
  if ((s_motorCtrl.status.state == MOTOR_STATE_WIND_DETECT) ||
      (s_motorCtrl.status.state == MOTOR_STATE_ANGLE_MONITOR)) {
    MotorHwYtm32_SwitchAdcToCurrentSensing();
  }
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_ResetRuntime();
  /* Full stop: clear retry state — next enable starts fresh */
  s_motorCtrl.startupRetryCount = 0U;
  s_motorCtrl.startupIqBoostA = 0.0f;
  s_motorCtrl.status.startup_retry_count = 0U;
  s_motorCtrl.status.fault = MOTOR_FAULT_NONE;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
  s_motorCtrl.status.electrical_speed_rad_s = 0.0f;
  s_motorCtrl.status.mechanical_rpm = 0.0f;
  s_motorCtrl.status.id_a = 0.0f;
  s_motorCtrl.status.iq_a = 0.0f;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.phase_current_a = 0.0f;
  s_motorCtrl.status.phase_current_b = 0.0f;
  s_motorCtrl.status.phase_current_c = 0.0f;
  s_motorCtrl.status.bus_voltage_v = 0.0f;
  MotorControl_SetState(MOTOR_STATE_STOP);
}

/**
 * @brief Trigger a hard fault, immediately disabling output drives.
 * @param fault Type of fault detected.
 */
static void MotorControl_LatchFault(motor_fault_t fault) {
  MotorHwYtm32_DisableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  s_motorCtrl.fastLoopRunning = false;
  s_motorCtrl.outputsUnmasked = false;
  s_motorCtrl.status.fault = fault;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.status.id_a = 0.0f;
  s_motorCtrl.status.iq_a = 0.0f;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  MotorControl_SetState(MOTOR_STATE_FAULT);
}

/**
 * @brief Trigger initial phase current reading offsets calibration sequence.
 */
static void MotorControl_BeginOffsetCalibration(void) {
  MotorHwYtm32_DisableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  MotorHwYtm32_ApplyPhaseDuty(0.5f, 0.5f, 0.5f);
  MotorControl_ResetRuntime();
  s_motorCtrl.currentOffsetASum = 0.0f;
  s_motorCtrl.currentOffsetBSum = 0.0f;
  s_motorCtrl.currentOffsetCSum = 0.0f;
  s_motorCtrl.offsetSampleCount = 0U;
  s_motorCtrl.status.fault = MOTOR_FAULT_NONE;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.fastLoopRunning = true;
  MotorHwYtm32_EnableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_SetState(MOTOR_STATE_OFFSET_CAL);
}

/**
 * @brief Enter passive BEMF angle-monitor mode with outputs masked.
 */
static void MotorControl_StartAngleMonitor(void) {
  MotorHwYtm32_DisableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_ResetRuntime();
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.id_a = 0.0f;
  s_motorCtrl.status.iq_a = 0.0f;
  s_motorCtrl.status.electrical_speed_rad_s = 0.0f;
  s_motorCtrl.status.mechanical_rpm = 0.0f;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.status.angle_monitor_active = true;
  s_motorCtrl.fastLoopRunning = true;
  MotorHwYtm32_SwitchAdcToBemfSensing();
  MotorHwYtm32_EnableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_SetState(MOTOR_STATE_ANGLE_MONITOR);
}

/**
 * @brief Transition step to Align mode injecting pure DC current.
 */
static void MotorControl_StartAlign(void) {
  /* Reset observer & PI integrators to prevent stale state from WindDetect
   * or a previous failed startup corrupting the next Align -> OpenLoop
   * transition. */
  MotorFoc_Reset(&s_motorCtrl.foc);
  s_motorCtrl.latestPhaseErrorRad = 0.0f;
  s_motorCtrl.latestObserverAngleRad = 0.0f;
  s_motorCtrl.observerPhaseOffsetRad = 0.0f;
  s_motorCtrl.observerLockResidualRad = 0.0f;
  s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
  s_motorCtrl.openLoopSpeedRadS = 0.0f;
  s_motorCtrl.closedLoopBlend = 0.0f;
  s_motorCtrl.observerLockCount = 0U;
  s_motorCtrl.observerLossCount = 0U;
  s_motorCtrl.stallAngleDivCount = 0U;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.openLoopFilteredCurrentA = 0.0f;
  /* ALIGN now ramps the d-axis current up over a short interval instead of
   * stepping straight to the full locking current. This reduces current shock
   * while keeping the rotor clamped before I/F startup begins. */
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
  s_motorCtrl.status.electrical_speed_rad_s = 0.0f;
  s_motorCtrl.status.mechanical_rpm = 0.0f;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.fastLoopRunning = true;
  MotorControl_SetState(MOTOR_STATE_ALIGN);
}

/**
 * @brief Load one HFI candidate and reset the synchronous demod accumulators.
 * @param candidateIndex Candidate slot to activate.
 * @param voltageV Voltage amplitude for the active HFI phase.
 * @param sampleTarget Number of measured pulse responses required.
 */
static void MotorControl_HfiLoadCandidate(uint8_t candidateIndex, float voltageV,
                                          uint16_t sampleTarget) {
  s_motorCtrl.hfi.active_index = candidateIndex;
  s_motorCtrl.hfi.sample_count = 0U;
  s_motorCtrl.hfi.sample_target = sampleTarget;
  s_motorCtrl.hfi.accum_metric = 0.0f;
  s_motorCtrl.hfi.accum_abs_current = 0.0f;
  s_motorCtrl.hfi.command_valid = false;
  s_motorCtrl.hfi.command_sign = 1;
  s_motorCtrl.hfi.command_voltage_v = voltageV;
  s_motorCtrl.hfi.command_angle_rad =
      s_motorCtrl.hfi.candidate_angles[candidateIndex];
  s_motorCtrl.status.hfi_angle_rad = s_motorCtrl.hfi.command_angle_rad;
}

/**
 * @brief Begin the fine HFI scan around the best coarse candidate.
 * @param centerAngleRad Best coarse candidate angle.
 * @param injectVoltageV Bipolar HFI burst amplitude.
 */
static void MotorControl_HfiStartFineScan(float centerAngleRad,
                                          float injectVoltageV) {
  const float fineStep = s_motorCtrl.hfi.coarse_step_rad * 0.5f;

  s_motorCtrl.hfi.phase = MOTOR_HFI_PHASE_FINE_SCAN;
  s_motorCtrl.hfi.candidate_count = 3U;
  s_motorCtrl.hfi.candidate_angles[0] =
      MotorControl_WrapAngle0ToPi(centerAngleRad - fineStep);
  s_motorCtrl.hfi.candidate_angles[1] =
      MotorControl_WrapAngle0ToPi(centerAngleRad);
  s_motorCtrl.hfi.candidate_angles[2] =
      MotorControl_WrapAngle0ToPi(centerAngleRad + fineStep);
  s_motorCtrl.hfi.candidate_metrics[0] = 0.0f;
  s_motorCtrl.hfi.candidate_metrics[1] = 0.0f;
  s_motorCtrl.hfi.candidate_metrics[2] = 0.0f;
  MotorControl_HfiLoadCandidate(
      0U, injectVoltageV,
      (uint16_t)(MotorControl_GetConfiguredHfiPulsePairs() * 2U));
}

/**
 * @brief Begin one polarity comparison burst.
 * @param positiveHalfTurn True to test the saliency angle, false for +pi.
 * @param polarityVoltageV Positive polarity pulse amplitude.
 */
static void MotorControl_HfiStartPolarityBurst(bool positiveHalfTurn,
                                               float polarityVoltageV) {
  s_motorCtrl.hfi.phase = positiveHalfTurn ? MOTOR_HFI_PHASE_POLARITY_POS
                                           : MOTOR_HFI_PHASE_POLARITY_NEG;
  s_motorCtrl.hfi.candidate_count = 1U;
  s_motorCtrl.hfi.candidate_angles[0] = positiveHalfTurn
                                            ? s_motorCtrl.hfi.best_angle_rad
                                            : MotorFoc_WrapAngle0ToTwoPi(
                                                  s_motorCtrl.hfi.best_angle_rad +
                                                  MOTOR_CFG_PI_F);
  MotorControl_HfiLoadCandidate(
      0U, polarityVoltageV,
      MotorControl_GetConfiguredHfiPolarityPulseCount());
}

/**
 * @brief Finish HFI/IPD and hand off to the regular ALIGN state.
 * @param success True when a valid initial angle was found.
 * @param reason Fallback reason recorded on failure.
 * @param finalConfidence Final confidence metric to expose via telemetry.
 */
static void MotorControl_HfiFinishToAlign(bool success,
                                          motor_hfi_fallback_reason_t reason,
                                          float finalConfidence) {
  MotorHwYtm32_SetOutputsMasked(true);
  s_motorCtrl.outputsUnmasked = false;
  MotorHwYtm32_ApplyPhaseDuty(0.5f, 0.5f, 0.5f);

  s_motorCtrl.hfi.phase = MOTOR_HFI_PHASE_IDLE;
  s_motorCtrl.hfi.command_valid = false;
  s_motorCtrl.status.hfi_active = false;
  s_motorCtrl.status.hfi_confidence = finalConfidence;
  s_motorCtrl.status.hfi_ripple_metric = s_motorCtrl.hfi.best_metric;

  if (success) {
    s_motorCtrl.hfiInitialAngleRad =
        MotorFoc_WrapAngle0ToTwoPi(s_motorCtrl.hfi.best_angle_rad);
    s_motorCtrl.status.hfi_angle_valid = true;
    s_motorCtrl.status.hfi_used = true;
    s_motorCtrl.status.hfi_fallback_reason = MOTOR_HFI_FALLBACK_NONE;
    s_motorCtrl.status.hfi_angle_rad = s_motorCtrl.hfiInitialAngleRad;
    s_motorCtrl.status.electrical_angle_rad = s_motorCtrl.hfiInitialAngleRad;
  } else {
    s_motorCtrl.hfiInitialAngleRad = 0.0f;
    s_motorCtrl.status.hfi_angle_valid = false;
    s_motorCtrl.status.hfi_used = false;
    s_motorCtrl.status.hfi_fallback_reason = (uint8_t)reason;
  }

  MotorControl_StartAlign();
}

/**
 * @brief Evaluate the current HFI scan phase and advance to the next step.
 * @param injectVoltageV Bipolar scan voltage.
 * @param polarityVoltageV Polarity pulse voltage.
 */
static void MotorControl_HfiAdvancePhase(float injectVoltageV,
                                         float polarityVoltageV) {
  uint8_t bestIndex = 0U;
  float bestMetric = s_motorCtrl.hfi.candidate_metrics[0];
  float worstMetric = s_motorCtrl.hfi.candidate_metrics[0];
  float secondMetric = 0.0f;
  float confidence;
  float repeatability;
  float confidenceMin = MotorControl_GetConfiguredHfiConfidenceMin();
  uint8_t i;

  for (i = 1U; i < s_motorCtrl.hfi.candidate_count; i++) {
    const float metric = s_motorCtrl.hfi.candidate_metrics[i];

    if (metric > bestMetric) {
      secondMetric = bestMetric;
      bestMetric = metric;
      bestIndex = i;
    } else if (metric > secondMetric) {
      secondMetric = metric;
    }

    if (metric < worstMetric) {
      worstMetric = metric;
    }
  }

  confidence = (bestMetric - worstMetric) /
               ((bestMetric > MOTOR_CTRL_EPSILON_F) ? bestMetric
                                                    : MOTOR_CTRL_EPSILON_F);

  if (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_COARSE_SCAN) {
    s_motorCtrl.hfi.best_metric = bestMetric;
    s_motorCtrl.hfi.best_angle_rad =
        s_motorCtrl.hfi.candidate_angles[bestIndex];
    s_motorCtrl.hfi.scan_confidence = confidence;
    s_motorCtrl.status.hfi_confidence = confidence;
    s_motorCtrl.status.hfi_ripple_metric = bestMetric;
    s_motorCtrl.status.hfi_angle_rad = s_motorCtrl.hfi.best_angle_rad;

    if (confidence < confidenceMin) {
      MotorControl_HfiFinishToAlign(false, MOTOR_HFI_FALLBACK_LOW_CONFIDENCE,
                                    confidence);
      return;
    }

    MotorControl_HfiStartFineScan(s_motorCtrl.hfi.best_angle_rad,
                                  injectVoltageV);
    return;
  }

  if (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_FINE_SCAN) {
    s_motorCtrl.hfi.best_metric = bestMetric;
    s_motorCtrl.hfi.best_angle_rad =
        s_motorCtrl.hfi.candidate_angles[bestIndex];
    s_motorCtrl.hfi.scan_confidence = confidence;
    s_motorCtrl.status.hfi_confidence = confidence;
    s_motorCtrl.status.hfi_ripple_metric = bestMetric;
    s_motorCtrl.status.hfi_angle_rad = s_motorCtrl.hfi.best_angle_rad;

    if (confidence < confidenceMin) {
      MotorControl_HfiFinishToAlign(false, MOTOR_HFI_FALLBACK_LOW_CONFIDENCE,
                                    confidence);
      return;
    }

    repeatability =
        (bestMetric - secondMetric) /
        ((bestMetric > MOTOR_CTRL_EPSILON_F) ? bestMetric
                                             : MOTOR_CTRL_EPSILON_F);
    if (repeatability < MotorControl_Clamp(confidenceMin * 0.5f, 0.05f, 1.0f)) {
      MotorControl_HfiFinishToAlign(false, MOTOR_HFI_FALLBACK_LOW_REPEATABILITY,
                                    confidence);
      return;
    }

    MotorControl_HfiStartPolarityBurst(true, polarityVoltageV);
    return;
  }

  if (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_POLARITY_POS) {
    s_motorCtrl.hfi.polarity_positive_metric = bestMetric;
    MotorControl_HfiStartPolarityBurst(false, polarityVoltageV);
    return;
  }

  s_motorCtrl.hfi.polarity_negative_metric = bestMetric;
  confidence =
      MotorControl_FastAbs(s_motorCtrl.hfi.polarity_positive_metric -
                           s_motorCtrl.hfi.polarity_negative_metric) /
      MotorControl_Clamp(
          (s_motorCtrl.hfi.polarity_positive_metric >
                   s_motorCtrl.hfi.polarity_negative_metric
               ? s_motorCtrl.hfi.polarity_positive_metric
               : s_motorCtrl.hfi.polarity_negative_metric),
          MOTOR_CTRL_EPSILON_F, 1.0e6f);

  if (confidence < MotorControl_Clamp(confidenceMin * 0.5f, 0.05f, 1.0f)) {
    MotorControl_HfiFinishToAlign(false,
                                  MOTOR_HFI_FALLBACK_POLARITY_AMBIGUOUS,
                                  confidence);
    return;
  }

  if (s_motorCtrl.hfi.polarity_negative_metric >
      s_motorCtrl.hfi.polarity_positive_metric) {
    s_motorCtrl.hfi.best_angle_rad = MotorFoc_WrapAngle0ToTwoPi(
        s_motorCtrl.hfi.best_angle_rad + MOTOR_CFG_PI_F);
  } else {
    s_motorCtrl.hfi.best_angle_rad =
        MotorFoc_WrapAngle0ToTwoPi(s_motorCtrl.hfi.best_angle_rad);
  }

  if (confidence < s_motorCtrl.hfi.scan_confidence) {
    s_motorCtrl.hfi.scan_confidence = confidence;
  }

  MotorControl_HfiFinishToAlign(true, MOTOR_HFI_FALLBACK_NONE,
                                s_motorCtrl.hfi.scan_confidence);
}

/**
 * @brief Execute one fast-loop HFI/IPD iteration.
 * @param iab Measured phase currents in alpha-beta frame.
 */
static void MotorControl_RunHfiIpdFastLoop(const motor_ab_frame_t *iab) {
  float projectedCurrentA = 0.0f;
  float orthogonalCurrentA = 0.0f;
  const float injectVoltageV =
      MotorControl_GetConfiguredHfiInjectVoltageV(s_motorCtrl.status.bus_voltage_v);
  const float polarityVoltageV = MotorControl_GetConfiguredHfiPolarityVoltageV(
      s_motorCtrl.status.bus_voltage_v);

  if (s_motorCtrl.hfi.command_valid) {
    projectedCurrentA =
        MotorControl_ProjectAlongAngle(iab, s_motorCtrl.hfi.command_angle_rad);
    orthogonalCurrentA = MotorControl_ProjectAlongAngle(
        iab, s_motorCtrl.hfi.command_angle_rad + MOTOR_CFG_HALF_PI_F);

    if ((s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_COARSE_SCAN) ||
        (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_FINE_SCAN)) {
      s_motorCtrl.hfi.accum_metric +=
          (float)s_motorCtrl.hfi.command_sign * projectedCurrentA;
    } else {
      s_motorCtrl.hfi.accum_metric += projectedCurrentA;
    }

    s_motorCtrl.hfi.accum_abs_current += MotorControl_FastAbs(projectedCurrentA);
    s_motorCtrl.hfi.sample_count++;
    s_motorCtrl.status.id_a = projectedCurrentA;
    s_motorCtrl.status.iq_a = orthogonalCurrentA;
    s_motorCtrl.status.electrical_speed_rad_s = 0.0f;
    s_motorCtrl.status.mechanical_rpm = 0.0f;
    s_motorCtrl.status.electrical_angle_rad = s_motorCtrl.hfi.command_angle_rad;
    s_motorCtrl.status.observer_locked = false;
    CanConfig_WaveformSample(
        s_motorCtrl.status.phase_current_a, s_motorCtrl.status.phase_current_b,
        s_motorCtrl.status.phase_current_c, s_motorCtrl.status.bus_voltage_v,
        projectedCurrentA, orthogonalCurrentA, s_motorCtrl.hfi.command_angle_rad);
  }

  if (s_motorCtrl.hfi.sample_count >= s_motorCtrl.hfi.sample_target) {
    const float metric = MotorControl_FastAbs(s_motorCtrl.hfi.accum_metric) /
                         (float)s_motorCtrl.hfi.sample_count;

    s_motorCtrl.hfi.candidate_metrics[s_motorCtrl.hfi.active_index] = metric;

    if ((s_motorCtrl.hfi.active_index + 1U) < s_motorCtrl.hfi.candidate_count) {
      MotorControl_HfiLoadCandidate(
          s_motorCtrl.hfi.active_index + 1U,
          ((s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_POLARITY_POS) ||
           (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_POLARITY_NEG))
              ? polarityVoltageV
              : injectVoltageV,
          s_motorCtrl.hfi.sample_target);
    } else {
      MotorControl_HfiAdvancePhase(injectVoltageV, polarityVoltageV);
    }
  }

  if (s_motorCtrl.status.state != MOTOR_STATE_HFI_IPD) {
    return;
  }

  if (!s_motorCtrl.outputsUnmasked) {
    s_motorCtrl.outputsUnmasked = true;
  }

  if ((s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_POLARITY_POS) ||
      (s_motorCtrl.hfi.phase == MOTOR_HFI_PHASE_POLARITY_NEG)) {
    s_motorCtrl.hfi.command_sign = 1;
    s_motorCtrl.hfi.command_valid = true;
    MotorControl_ApplyVoltageVector(s_motorCtrl.hfi.command_angle_rad,
                                    s_motorCtrl.hfi.command_voltage_v,
                                    s_motorCtrl.status.bus_voltage_v);
  } else {
    s_motorCtrl.hfi.command_sign = -s_motorCtrl.hfi.command_sign;
    if (!s_motorCtrl.hfi.command_valid) {
      s_motorCtrl.hfi.command_sign = 1;
    }
    s_motorCtrl.hfi.command_valid = true;
    MotorControl_ApplyVoltageVector(
        s_motorCtrl.hfi.command_angle_rad,
        (float)s_motorCtrl.hfi.command_sign * s_motorCtrl.hfi.command_voltage_v,
        s_motorCtrl.status.bus_voltage_v);
  }

  MotorHwYtm32_SetOutputsMasked(false);
}

/**
 * @brief Attempt HFI/IPD before the classic ALIGN -> OPEN_LOOP startup chain.
 */
static void MotorControl_StartHfiIpd(void) {
  uint8_t candidateCount;
  uint8_t i;

  MotorFoc_Reset(&s_motorCtrl.foc);
  s_motorCtrl.latestPhaseErrorRad = 0.0f;
  s_motorCtrl.latestObserverAngleRad = 0.0f;
  s_motorCtrl.observerPhaseOffsetRad = 0.0f;
  s_motorCtrl.observerLockResidualRad = 0.0f;
  s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
  s_motorCtrl.openLoopSpeedRadS = 0.0f;
  s_motorCtrl.closedLoopBlend = 0.0f;
  s_motorCtrl.observerLockCount = 0U;
  s_motorCtrl.observerLossCount = 0U;
  s_motorCtrl.stallAngleDivCount = 0U;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.openLoopFilteredCurrentA = 0.0f;
  MotorControl_ResetHfiStatus();

  candidateCount = MotorControl_GetConfiguredHfiCandidateCount();
  s_motorCtrl.hfi.phase = MOTOR_HFI_PHASE_COARSE_SCAN;
  s_motorCtrl.hfi.candidate_count = candidateCount;
  s_motorCtrl.hfi.coarse_step_rad = MOTOR_CFG_PI_F / (float)candidateCount;
  s_motorCtrl.hfi.polarity_positive_metric = 0.0f;
  s_motorCtrl.hfi.polarity_negative_metric = 0.0f;

  for (i = 0U; i < candidateCount; i++) {
    s_motorCtrl.hfi.candidate_angles[i] =
        (float)i * s_motorCtrl.hfi.coarse_step_rad;
    s_motorCtrl.hfi.candidate_metrics[i] = 0.0f;
  }

  MotorControl_HfiLoadCandidate(
      0U, MotorControl_GetConfiguredHfiInjectVoltageV(s_motorCtrl.status.bus_voltage_v),
      (uint16_t)(MotorControl_GetConfiguredHfiPulsePairs() * 2U));

  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.id_a = 0.0f;
  s_motorCtrl.status.iq_a = 0.0f;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.fastLoopRunning = true;
  s_motorCtrl.outputsUnmasked = false;
  MotorHwYtm32_SetOutputsMasked(true);
  MotorHwYtm32_ApplyPhaseDuty(0.5f, 0.5f, 0.5f);
  MotorControl_SetState(MOTOR_STATE_HFI_IPD);
}

/**
 * @brief Choose between HFI/IPD and the legacy ALIGN state.
 */
static void MotorControl_StartStaticStartupSequence(void) {
  MotorControl_ResetHfiStatus();

  if (!MotorControl_IsHfiEnabled()) {
    s_motorCtrl.status.hfi_fallback_reason = MOTOR_HFI_FALLBACK_DISABLED;
    MotorControl_StartAlign();
    return;
  }

  if ((s_motorCtrl.status.bus_voltage_v < MOTOR_CFG_VBUS_UNDERVOLTAGE_V) ||
      (s_motorCtrl.status.bus_voltage_v > MOTOR_CFG_VBUS_OVERVOLTAGE_V)) {
    s_motorCtrl.status.hfi_fallback_reason = MOTOR_HFI_FALLBACK_INVALID_VBUS;
    MotorControl_StartAlign();
    return;
  }

  MotorControl_StartHfiIpd();
}

/**
 * @brief Transition step from Align to Open-Loop spinning.
 */
static void MotorControl_StartOpenLoop(void) {
  s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
  s_motorCtrl.openLoopSpeedRadS = 0.0f;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.stallAngleDivCount = 0U;
  s_motorCtrl.openLoopFilteredCurrentA = 0.0f;
  s_motorCtrl.observerPhaseOffsetRad = s_motorCtrl.latestPhaseErrorRad;
  s_motorCtrl.observerLockResidualRad = 0.0f;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  MotorControl_SetState(MOTOR_STATE_OPEN_LOOP_RAMP);
}

/**
 * @brief Transition step from Open-Loop spinning to Sensorless Closed-Loop
 * control.
 */
static void MotorControl_StartClosedLoop(void) {
  const float observedMechanicalRpm =
      __builtin_fabsf(MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(
          s_motorCtrl.status.electrical_speed_rad_s));

  s_motorCtrl.closedLoopBlend = 0.0f;
  s_motorCtrl.observerLossCount = 0U;
  s_motorCtrl.status.observer_locked = true;
  s_motorCtrl.status.target_rpm = observedMechanicalRpm;
  /* Clamp integrator pre-load: open-loop injection current (5-8A) far exceeds
   * the steady-state torque demand of a blower fan.  A large pre-load causes
   * severe speed overshoot and oscillation on the first closed-loop cycles.
   * Limit to half the open-loop current so the PI can ramp smoothly. */
  {
    const float openLoopIqClampA = MotorControl_GetStartupIqLimitA();
    s_motorCtrl.foc.speed_pi_integrator_a = MotorControl_Clamp(
        s_motorCtrl.status.iq_target_a, -openLoopIqClampA * 0.5f,
        openLoopIqClampA * 0.5f);
  }
  s_motorCtrl.fwIdTargetA = 0.0f;
  MotorControl_SetState(MOTOR_STATE_CLOSED_LOOP);
}

/**
 * @brief Update passive BEMF angle monitor from one raw sample set.
 * @param bemfURaw   Phase-U BEMF ADC count.
 * @param bemfVRaw   Phase-V BEMF ADC count.
 * @param bemfWRaw   Phase-W BEMF ADC count.
 * @param bemfComRaw Virtual-neutral BEMF ADC count.
 */
static void MotorControl_UpdateAngleMonitorSample(uint16_t bemfURaw,
                                                  uint16_t bemfVRaw,
                                                  uint16_t bemfWRaw,
                                                  uint16_t bemfComRaw) {
  const float bemfU = (float)((int32_t)bemfURaw - (int32_t)bemfComRaw);
  const float bemfV = (float)((int32_t)bemfVRaw - (int32_t)bemfComRaw);
  const float bemfW = (float)((int32_t)bemfWRaw - (int32_t)bemfComRaw);
  const float bemfAlpha = bemfU;
  const float bemfBeta = (bemfU + (2.0f * bemfV)) * MOTOR_CFG_INV_SQRT3_F;
  const float bemfMagnitude =
      MotorControl_FastSqrt((bemfAlpha * bemfAlpha) + (bemfBeta * bemfBeta));
  const float speedFilterBlend = MotorControl_Clamp(
      MOTOR_CFG_TWO_PI_F * MOTOR_CFG_ANGLE_MONITOR_SPEED_FILTER_BW_HZ *
          MOTOR_CFG_FAST_LOOP_DT_S,
      0.0f, 1.0f);
  int8_t direction = 0;

  s_motorCtrl.bemfLastU = bemfURaw;
  s_motorCtrl.bemfLastV = bemfVRaw;
  s_motorCtrl.bemfLastW = bemfWRaw;
  s_motorCtrl.bemfLastCom = bemfComRaw;
  s_motorCtrl.status.angle_monitor_active = true;
  s_motorCtrl.status.angle_monitor_bemf_mag = (uint16_t)bemfMagnitude;

  if (bemfMagnitude < MOTOR_CFG_ANGLE_MONITOR_MIN_BEMF_COUNTS) {
    s_motorCtrl.angleMonitorSeeded = false;
    s_motorCtrl.status.angle_monitor_valid = false;
    s_motorCtrl.status.angle_monitor_speed_rad_s = 0.0f;
    s_motorCtrl.status.electrical_speed_rad_s = 0.0f;
    s_motorCtrl.status.mechanical_rpm = 0.0f;
    s_motorCtrl.status.observer_locked = false;
    return;
  }

  {
    const float bemfAngleRad = MotorFoc_WrapAngle0ToTwoPi(
        MotorControl_FastAtan2(bemfBeta, bemfAlpha));

    if (!s_motorCtrl.angleMonitorSeeded) {
      s_motorCtrl.angleMonitorSeeded = true;
      s_motorCtrl.angleMonitorBemfAngleRad = bemfAngleRad;
      s_motorCtrl.status.angle_monitor_speed_rad_s = 0.0f;
      return;
    }

    const float bemfDeltaRad = MotorFoc_AngleDiff(
        bemfAngleRad, s_motorCtrl.angleMonitorBemfAngleRad);
    const float instantaneousSpeedRadS =
        bemfDeltaRad / MOTOR_CFG_FAST_LOOP_DT_S;

    s_motorCtrl.angleMonitorBemfAngleRad = bemfAngleRad;
    s_motorCtrl.status.angle_monitor_speed_rad_s +=
        speedFilterBlend *
        (instantaneousSpeedRadS - s_motorCtrl.status.angle_monitor_speed_rad_s);
  }

  if (s_motorCtrl.status.angle_monitor_speed_rad_s >
      MOTOR_CFG_ANGLE_MONITOR_VALID_SPEED_RAD_S) {
    direction = 1;
  } else if (s_motorCtrl.status.angle_monitor_speed_rad_s <
             -MOTOR_CFG_ANGLE_MONITOR_VALID_SPEED_RAD_S) {
    direction = -1;
  }

  if (direction != 0) {
    s_motorCtrl.status.angle_monitor_valid = true;
    s_motorCtrl.status.angle_monitor_angle_rad = MotorFoc_WrapAngle0ToTwoPi(
        s_motorCtrl.angleMonitorBemfAngleRad -
        ((float)direction * MOTOR_CFG_HALF_PI_F));
    s_motorCtrl.status.electrical_angle_rad =
        s_motorCtrl.status.angle_monitor_angle_rad;
    s_motorCtrl.status.electrical_speed_rad_s =
        s_motorCtrl.status.angle_monitor_speed_rad_s;
    s_motorCtrl.status.mechanical_rpm = MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(
        s_motorCtrl.status.angle_monitor_speed_rad_s);
    s_motorCtrl.status.observer_locked = true;
  } else {
    s_motorCtrl.status.angle_monitor_valid = false;
    s_motorCtrl.status.observer_locked = false;
  }

  (void)bemfW;
}

/* ========================= Startup Failure Retry ========================== */

/**
 * @brief Handle a startup failure: either retry with increased current or
 * latch a hard fault if all retries are exhausted.
 *
 * Called when:
 *  (a) Angle divergence stall is detected during open-loop ramp, or
 *  (b) The startup timeout expires.
 *
 * On each retry the q-axis injection current is increased by a configured
 * step to overcome higher load torque.  After the maximum number of retries
 * the drive enters a permanent fault state requiring a disable/enable cycle.
 */
static void MotorControl_HandleStartupFailure(void) {
  s_motorCtrl.startupRetryCount++;
  s_motorCtrl.status.startup_retry_count = s_motorCtrl.startupRetryCount;

  if (s_motorCtrl.startupRetryCount > MOTOR_CFG_STARTUP_MAX_RETRIES) {
    /* All retries exhausted — latch a hard fault */
    MotorControl_LatchFault(MOTOR_FAULT_STARTUP_FAIL);
    return;
  }

  /* Compute progressive current boost for the next attempt */
  s_motorCtrl.startupIqBoostA = MotorControl_Clamp(
      (float)s_motorCtrl.startupRetryCount * MOTOR_CFG_STARTUP_IQ_BOOST_STEP_A,
      0.0f, MOTOR_CFG_STARTUP_IQ_BOOST_MAX_A);

  /* Mask outputs and re-enter alignment for a fresh startup attempt */
  MotorHwYtm32_SetOutputsMasked(true);
  s_motorCtrl.outputsUnmasked = false;
  MotorControl_StartAlign();
}

/* ========================= Wind Detect / Catch Spin ========================
 */
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)

/* Forward declaration — defined after UpdateWindDetectState which calls it. */
static void MotorControl_StartCoastDown(void);

/**
 * @brief Enter the Wind Detect state: observer runs on BEMF, PWM outputs
 * masked.
 */
static void MotorControl_StartWindDetect(void) {
  s_motorCtrl.windDetectSpeedRadS = 0.0f;
  s_motorCtrl.windDetectPrevSpeedRadS = 0.0f;
  s_motorCtrl.windDetectDirection = 0;
  s_motorCtrl.windDetectSettleCount = 0U;
  s_motorCtrl.windCatchActive = false;
  s_motorCtrl.stateTimeMs = 0U;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.fastLoopRunning = true;

  /* BEMF zero-crossing detection state */
  s_motorCtrl.bemfSignPrev[0] = 0;
  s_motorCtrl.bemfSignPrev[1] = 0;
  s_motorCtrl.bemfSignPrev[2] = 0;
  s_motorCtrl.bemfZeroCrossingCount = 0U;
  s_motorCtrl.bemfFirstCrossingTick = 0U;
  s_motorCtrl.bemfLastCrossingTick = 0U;
  s_motorCtrl.bemfIsrTickCounter = 0U;
  s_motorCtrl.bemfPhaseSequence = 0;

  /* Keep all MOSFETs OFF — zero braking torque.
   * Switch ADC sequence from current sensing to BEMF phase voltage channels. */
  MotorHwYtm32_SetOutputsMasked(true);
  MotorHwYtm32_SwitchAdcToBemfSensing();
  MotorControl_SetState(MOTOR_STATE_WIND_DETECT);
}

/**
 * @brief Tailwind catch: jump straight to closed-loop using observer angle.
 */
static void MotorControl_StartCatchSpin(void) {
  const float observedMechanicalRpm = __builtin_fabsf(
      MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(s_motorCtrl.windDetectSpeedRadS));

  s_motorCtrl.windCatchActive = true;
  s_motorCtrl.closedLoopBlend = 1.0f; /* fully trust observer immediately */
  s_motorCtrl.observerLossCount = 0U;
  s_motorCtrl.status.observer_locked = true;
  s_motorCtrl.status.target_rpm = observedMechanicalRpm;
  s_motorCtrl.targetRpmCommand = observedMechanicalRpm;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.foc.speed_pi_integrator_a = 0.0f;
  s_motorCtrl.fwIdTargetA = 0.0f;

  /* Un-mask outputs to start driving */
  MotorHwYtm32_SetOutputsMasked(false);
  s_motorCtrl.outputsUnmasked = true;
  MotorControl_SetState(MOTOR_STATE_CLOSED_LOOP);
}

/**
 * @brief Periodic slow-loop handler for the Wind Detect state.
 * Waits for observer PLL to converge, then decides the startup path.
 */
static void MotorControl_UpdateWindDetectState(void) {
  s_motorCtrl.stateTimeMs++;
  s_motorCtrl.startupTimeMs++;

  /* Timeout protection — assume standstill if no valid reading in time */
  if (s_motorCtrl.stateTimeMs >= MOTOR_CFG_WIND_DETECT_TIMEOUT_MS) {
    MotorHwYtm32_SwitchAdcToCurrentSensing();
    MotorControl_StartStaticStartupSequence();
    return;
  }

  /* Need enough zero crossings for a reliable speed estimate. */
  if (s_motorCtrl.bemfZeroCrossingCount < MOTOR_CFG_WIND_DETECT_MIN_CROSSINGS) {
    return;
  }

  /* Compute speed from BEMF zero-crossing rate.
   * Each phase has 2 crossings per electrical cycle (rising + falling).
   * Total crossings across 3 phases = 6 per electrical cycle.
   * electrical_freq = crossings / 6 / time_window_s
   * electrical_speed_rad_s = electrical_freq * 2π */
  const uint32_t tickSpan =
      s_motorCtrl.bemfLastCrossingTick - s_motorCtrl.bemfFirstCrossingTick;
  if (tickSpan == 0U) {
    return; /* Division by zero guard */
  }

  const float timeWindowS = (float)tickSpan * MOTOR_CFG_FAST_LOOP_DT_S;
  const float crossingsPerSec =
      (float)(s_motorCtrl.bemfZeroCrossingCount - 1U) / timeWindowS;
  /* 6 crossings per electrical cycle (3 phases × 2 edges) */
  const float elecFreqHz = crossingsPerSec / 6.0f;
  const float elecSpeedRadS = elecFreqHz * 6.2831853f; /* × 2π */
  const float mechRpm =
      __builtin_fabsf(MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(elecSpeedRadS));

  /* Store detected speed for telemetry */
  s_motorCtrl.windDetectSpeedRadS = elecSpeedRadS;
  s_motorCtrl.status.electrical_speed_rad_s = elecSpeedRadS;

  /* Below standstill threshold — normal startup */
  if (mechRpm < MOTOR_CFG_WIND_DETECT_STANDSTILL_RPM) {
    MotorHwYtm32_SwitchAdcToCurrentSensing();
    MotorControl_StartStaticStartupSequence();
    return;
  }

  /* Determine detected direction from BEMF phase sequence. */
  s_motorCtrl.windDetectDirection =
      (s_motorCtrl.bemfPhaseSequence >= 0) ? 1 : -1;

  /* Always restore ADC to current sensing before leaving wind detect */
  MotorHwYtm32_SwitchAdcToCurrentSensing();

  if (s_motorCtrl.windDetectDirection == s_motorCtrl.status.direction) {
    /* Tailwind: rotor spinning in the desired direction */
    if (mechRpm <= MOTOR_CFG_CATCH_MAX_SPEED_RPM) {
      MotorControl_StartCatchSpin();
    } else {
      MotorControl_StartCoastDown();
    }
  } else {
    /* Headwind — coast down with outputs off */
    MotorControl_StartCoastDown();
  }
}

/**
 * @brief Enter Coast-Down state with voltage-limited regenerative braking.
 * Observer tracks the rotor while reverse Iq decelerates the motor.
 * Bus voltage is monitored to prevent overvoltage on brakeless systems.
 */
static void MotorControl_StartCoastDown(void) {
  const float speedSign =
      (s_motorCtrl.status.electrical_speed_rad_s >= 0.0f) ? 1.0f : -1.0f;

  /* Inject reverse Iq for regenerative braking */
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a = -speedSign * MOTOR_CFG_COAST_BRAKE_IQ_A;

  /* Un-mask PWM so braking current can flow */
  MotorHwYtm32_SetOutputsMasked(false);
  s_motorCtrl.outputsUnmasked = true;
  MotorControl_SetState(MOTOR_STATE_COAST_DOWN);
}

/**
 * @brief Periodic slow-loop handler for Coast-Down state.
 * Dynamically adjusts braking Iq based on bus voltage to prevent overvoltage,
 * and transitions to Align when speed is low enough.
 */
static void MotorControl_UpdateCoastDownState(void) {
  s_motorCtrl.stateTimeMs++;
  s_motorCtrl.startupTimeMs++;

  const float absSpeedRadS =
      __builtin_fabsf(s_motorCtrl.status.electrical_speed_rad_s);
  const float speedSign =
      (s_motorCtrl.status.electrical_speed_rad_s >= 0.0f) ? 1.0f : -1.0f;
  const float vbus = s_motorCtrl.status.bus_voltage_v;

  /* ---- Voltage-limited braking gain ----
   * brakeGain = 1.0 when Vbus <= LIMIT          (full braking)
   * brakeGain = 0.0 when Vbus >= LIMIT + HYST   (no braking, coast)
   * Linear interpolation in between. */
  float brakeGain =
      MotorControl_Clamp((MOTOR_CFG_COAST_BRAKE_VBUS_LIMIT_V +
                          MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V - vbus) /
                             MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V,
                         0.0f, 1.0f);

  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a =
      -speedSign * MOTOR_CFG_COAST_BRAKE_IQ_A * brakeGain;

  /* When speed drops below safe threshold, enter normal Align startup */
  if (absSpeedRadS < MOTOR_CFG_COAST_DOWN_SAFE_SPEED_RAD_S) {
    MotorControl_StartStaticStartupSequence();
    return;
  }

  /* Global startup timeout protection */
  if (s_motorCtrl.startupTimeMs >= MOTOR_CFG_STARTUP_TIMEOUT_MS) {
    MotorControl_LatchFault(MOTOR_FAULT_STARTUP_TIMEOUT);
  }
}

#endif /* MOTOR_CFG_ENABLE_WIND_DETECT */

/**
 * @brief Periodic slow-loop handler managing Open-Loop speed sweeps.
 * Called at 1 kHz configured timing rate.
 */
static void MotorControl_UpdateOpenLoopState(void) {
  const float openLoopRampTimeS =
      (MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS > 0U)
          ? ((float)MOTOR_CFG_OPEN_LOOP_RAMP_TIME_MS * 0.001f)
          : MOTOR_CFG_SPEED_LOOP_DT_S;
  const float speedStepRadS =
      (__builtin_fabsf(MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S) *
       MOTOR_CFG_SPEED_LOOP_DT_S) /
      openLoopRampTimeS;
  const float openLoopIqFinalA = MotorControl_GetStartupIqLimitA();
  const float minStartupIqA =
      MotorControl_Clamp(MOTOR_CFG_IF_MIN_IQ_A, 0.0f, openLoopIqFinalA);
  const float iqRampTimeS =
      (MOTOR_CFG_IF_CURRENT_RAMP_TIME_MS > 0U)
          ? ((float)MOTOR_CFG_IF_CURRENT_RAMP_TIME_MS * 0.001f)
          : MOTOR_CFG_SPEED_LOOP_DT_S;
  const float iqStepA =
      (openLoopIqFinalA * MOTOR_CFG_SPEED_LOOP_DT_S) / iqRampTimeS;
  const float phaseTrackBlend = MotorControl_Clamp(
      MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S * MOTOR_CFG_SPEED_LOOP_DT_S, 0.0f,
      1.0f);
  const float currentFilterBlend = MotorControl_Clamp(
      MOTOR_CFG_TWO_PI_F * MOTOR_CFG_IF_CURRENT_FILTER_BW_HZ *
          MOTOR_CFG_SPEED_LOOP_DT_S,
      0.0f, 1.0f);
  float openLoopSpeedMagRadS;
  float currentAccelScale = 1.0f;
  float residualAccelScale = 1.0f;
  float accelScale;
  float targetIqMagA = openLoopIqFinalA;
  float iqTargetMagA;

  s_motorCtrl.stateTimeMs++;
  s_motorCtrl.startupTimeMs++;

  s_motorCtrl.openLoopFilteredCurrentA += currentFilterBlend *
                                          (MotorControl_GetStartupCurrentMagnitudeA() -
                                           s_motorCtrl.openLoopFilteredCurrentA);

  if (MOTOR_CFG_IF_CURRENT_MARGIN_A > 0.0f) {
    currentAccelScale = MotorControl_Clamp(
        (openLoopIqFinalA + MOTOR_CFG_IF_CURRENT_MARGIN_A -
         s_motorCtrl.openLoopFilteredCurrentA) /
            (2.0f * MOTOR_CFG_IF_CURRENT_MARGIN_A),
        MOTOR_CFG_IF_ACCEL_MIN_SCALE, 1.0f);

    if (s_motorCtrl.openLoopFilteredCurrentA >=
        (openLoopIqFinalA + MOTOR_CFG_IF_CURRENT_MARGIN_A)) {
      currentAccelScale = 0.0f;
    }
  }

  if ((__builtin_fabsf(s_motorCtrl.openLoopSpeedRadS) >=
       MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S) &&
      (MOTOR_CFG_STALL_ANGLE_ERR_RAD >
       MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD)) {
    residualAccelScale = MotorControl_Clamp(
        (MOTOR_CFG_STALL_ANGLE_ERR_RAD -
         __builtin_fabsf(s_motorCtrl.observerLockResidualRad)) /
            (MOTOR_CFG_STALL_ANGLE_ERR_RAD -
             MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD),
        MOTOR_CFG_IF_ACCEL_MIN_SCALE, 1.0f);

    if (__builtin_fabsf(s_motorCtrl.observerLockResidualRad) >=
        MOTOR_CFG_STALL_ANGLE_ERR_RAD) {
      residualAccelScale = 0.0f;
    }
  }

  accelScale = (currentAccelScale < residualAccelScale) ? currentAccelScale
                                                        : residualAccelScale;
  openLoopSpeedMagRadS = MotorControl_Clamp(
      (__builtin_fabsf(s_motorCtrl.openLoopSpeedRadS) +
       (speedStepRadS * accelScale)),
      0.0f,
      MOTOR_CFG_OPEN_LOOP_FINAL_RAD_S);
  s_motorCtrl.openLoopSpeedRadS =
      MotorControl_GetSignedDirection() * openLoopSpeedMagRadS;
  s_motorCtrl.observerPhaseOffsetRad =
      MotorControl_BlendPhase(s_motorCtrl.observerPhaseOffsetRad,
                              s_motorCtrl.latestPhaseErrorRad, phaseTrackBlend);
  s_motorCtrl.observerLockResidualRad =
      MotorControl_WrapAngleSigned(MotorFoc_AngleDiff(
          s_motorCtrl.latestPhaseErrorRad, s_motorCtrl.observerPhaseOffsetRad));
  s_motorCtrl.status.id_target_a = 0.0f;
  if (s_motorCtrl.openLoopFilteredCurrentA > openLoopIqFinalA) {
    targetIqMagA = MotorControl_Clamp(
        openLoopIqFinalA -
            (s_motorCtrl.openLoopFilteredCurrentA - openLoopIqFinalA),
        minStartupIqA, openLoopIqFinalA);
  }
  iqTargetMagA =
      MotorControl_SlewTowards(__builtin_fabsf(s_motorCtrl.status.iq_target_a),
                               targetIqMagA, iqStepA);
  s_motorCtrl.status.iq_target_a =
      MotorControl_GetSignedDirection() * iqTargetMagA;

  /* ---------- Angle Divergence Stall Detection ----------
   * When the open-loop forced speed has ramped above the minimum lock speed,
   * the observer should start converging.  If the angular residual between
   * observer angle and forced angle stays above STALL_ANGLE_ERR_RAD for
   * STALL_ANGLE_DIV_COUNT consecutive ticks, the rotor is not following the
   * magnetic field → declare stall and trigger retry. */
  if ((__builtin_fabsf(s_motorCtrl.openLoopSpeedRadS) >=
       MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S) &&
      (__builtin_fabsf(s_motorCtrl.observerLockResidualRad) >
       MOTOR_CFG_STALL_ANGLE_ERR_RAD)) {
    s_motorCtrl.stallAngleDivCount++;
  } else {
    s_motorCtrl.stallAngleDivCount = 0U;
  }

  if (s_motorCtrl.stallAngleDivCount >= MOTOR_CFG_STALL_ANGLE_DIV_COUNT) {
    /* Early stall detected — attempt retry with boosted current */
    MotorControl_HandleStartupFailure();
    return;
  }

  /* ---------- Observer Lock Check ---------- */
  if ((__builtin_fabsf(s_motorCtrl.status.electrical_speed_rad_s) >=
       MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S) &&
      (__builtin_fabsf(s_motorCtrl.observerLockResidualRad) <=
       MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD)) {
    s_motorCtrl.observerLockCount++;
  } else {
    s_motorCtrl.observerLockCount = 0U;
  }

  if (s_motorCtrl.observerLockCount >= MOTOR_CFG_OBSERVER_LOCK_COUNT) {
    /* Successful lock — clear retry state and enter closed-loop */
    s_motorCtrl.startupRetryCount = 0U;
    s_motorCtrl.startupIqBoostA = 0.0f;
    s_motorCtrl.status.startup_retry_count = 0U;
    MotorControl_StartClosedLoop();
    return;
  }

  /* ---------- Startup Timeout ---------- */
  if (s_motorCtrl.startupTimeMs >= MOTOR_CFG_STARTUP_TIMEOUT_MS) {
    MotorControl_HandleStartupFailure();
  }
}

/**
 * @brief Periodic slow-loop handler active during Closed-Loop control.
 * Calculates PI speed requests and field weakening. Called at 1 kHz.
 */
static void MotorControl_UpdateClosedLoopState(void) {
  const float blendStep = (MOTOR_CFG_CLOSED_LOOP_BLEND_MS > 0U)
                              ? (MOTOR_CFG_SPEED_LOOP_DT_S * 1000.0f /
                                 (float)MOTOR_CFG_CLOSED_LOOP_BLEND_MS)
                              : 1.0f;

  s_motorCtrl.stateTimeMs++;
  if (s_motorCtrl.closedLoopBlend < 1.0f) {
    s_motorCtrl.closedLoopBlend =
        MotorControl_Clamp(s_motorCtrl.closedLoopBlend + blendStep, 0.0f, 1.0f);
  }

  /* ---- Field Weakening Controller ---- */
#if (MOTOR_CFG_ENABLE_FIELD_WEAKENING != 0U)
  {
    const float fwError =
        s_motorCtrl.voltageModulationRatio - MOTOR_CFG_FW_VOLTAGE_THRESHOLD;

    if (fwError > 0.0f) {
      /* Voltage saturating — inject more negative Id */
      s_motorCtrl.fwIdTargetA -=
          MOTOR_CFG_FW_KI * MOTOR_CFG_SPEED_LOOP_DT_S * fwError;
      s_motorCtrl.fwIdTargetA = MotorControl_Clamp(
          s_motorCtrl.fwIdTargetA, MOTOR_CFG_FW_MAX_NEGATIVE_ID_A, 0.0f);
    } else {
      /* Voltage margin available — slowly recover Id back to zero */
      s_motorCtrl.fwIdTargetA = MotorControl_SlewTowards(
          s_motorCtrl.fwIdTargetA, 0.0f,
          MOTOR_CFG_FW_RECOVERY_RATE * MOTOR_CFG_SPEED_LOOP_DT_S);
    }
    s_motorCtrl.status.id_target_a = s_motorCtrl.fwIdTargetA;
  }
#else
  s_motorCtrl.status.id_target_a = 0.0f;
#endif

  if (s_motorCtrl.controlModeRequest == MOTOR_CONTROL_MODE_CURRENT) {
    s_motorCtrl.status.iq_target_a = s_motorCtrl.targetIqCommandA;
  } else {
    s_motorCtrl.status.iq_target_a = MotorFoc_RunSpeedPi(
        &s_motorCtrl.foc, MotorControl_GetTargetElectricalSpeedRadS(),
        s_motorCtrl.status.electrical_speed_rad_s, false);
  }

  if (__builtin_fabsf(s_motorCtrl.latestPhaseErrorRad) >
      MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD) {
    s_motorCtrl.observerLossCount++;
  } else {
    s_motorCtrl.observerLossCount = 0U;
  }

  if (s_motorCtrl.observerLossCount >= MOTOR_CFG_OBSERVER_LOSS_COUNT) {
    MotorControl_LatchFault(MOTOR_FAULT_OBSERVER_LOSS);
  }
}

/**
 * @brief Main periodic slow loop managing state machine dispatch and user
 * ramping.
 */
static void MotorControl_HandleSlowLoop(void) {
  const float speedRampStepRpm =
      MOTOR_CFG_SPEED_RAMP_RPM_PER_S * MOTOR_CFG_SPEED_LOOP_DT_S;

  s_motorCtrl.tickMs++;
  s_motorCtrl.status.target_rpm =
      MotorControl_SlewTowards(s_motorCtrl.status.target_rpm,
                               s_motorCtrl.targetRpmCommand, speedRampStepRpm);
  s_motorCtrl.status.enabled = s_motorCtrl.enableRequest;
  s_motorCtrl.status.mechanical_rpm = MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(
      s_motorCtrl.status.electrical_speed_rad_s);

  if (s_motorCtrl.angleMonitorRequest) {
    s_motorCtrl.status.enabled = false;
    if (s_motorCtrl.status.state != MOTOR_STATE_ANGLE_MONITOR) {
      MotorControl_StartAngleMonitor();
    }
    return;
  }

  if (!s_motorCtrl.enableRequest) {
    if (s_motorCtrl.status.state != MOTOR_STATE_STOP) {
      MotorControl_EnterStop();
    }
    return;
  }

  if (s_motorCtrl.status.state == MOTOR_STATE_ANGLE_MONITOR) {
    MotorControl_EnterStop();
  }

  switch (s_motorCtrl.status.state) {
  case MOTOR_STATE_STOP:
    MotorControl_BeginOffsetCalibration();
    break;

  case MOTOR_STATE_OFFSET_CAL:
    break;

#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
  case MOTOR_STATE_WIND_DETECT:
    MotorControl_UpdateWindDetectState();
    break;

  case MOTOR_STATE_COAST_DOWN:
    MotorControl_UpdateCoastDownState();
    break;
#endif

  case MOTOR_STATE_HFI_IPD:
    s_motorCtrl.stateTimeMs++;
    break;

  case MOTOR_STATE_ALIGN:
  {
    const float alignTargetA = MotorControl_GetAlignCurrentTargetA();
    const float alignRampTimeS =
        (MOTOR_CFG_ALIGN_CURRENT_RAMP_TIME_MS > 0U)
            ? ((float)MOTOR_CFG_ALIGN_CURRENT_RAMP_TIME_MS * 0.001f)
            : MOTOR_CFG_SPEED_LOOP_DT_S;
    const float alignStepA =
        (alignTargetA * MOTOR_CFG_SPEED_LOOP_DT_S) / alignRampTimeS;

    s_motorCtrl.stateTimeMs++;
    s_motorCtrl.startupTimeMs++;
    s_motorCtrl.status.id_target_a = MotorControl_SlewTowards(
        s_motorCtrl.status.id_target_a, alignTargetA, alignStepA);
    s_motorCtrl.status.iq_target_a = 0.0f;

    if (s_motorCtrl.stateTimeMs >= MotorControl_GetAlignHoldTimeMs()) {
      MotorControl_StartOpenLoop();
    }
    break;
  }

  case MOTOR_STATE_OPEN_LOOP_RAMP:
    MotorControl_UpdateOpenLoopState();
    break;

  case MOTOR_STATE_CLOSED_LOOP:
    MotorControl_UpdateClosedLoopState();
    break;

  case MOTOR_STATE_PARAM_IDENT:
    MotorParamIdent_RunSlowLoop();
    /* Check if identification finished or errored */
    {
      motor_ident_phase_t identPhase = MotorParamIdent_GetPhase();
      if (identPhase == MOTOR_IDENT_PHASE_COMPLETE ||
          identPhase == MOTOR_IDENT_PHASE_ERROR) {
        /* Disable motor so it stays in STOP (IDLE) after ident */
        s_motorCtrl.enableRequest = false;
        s_motorCtrl.status.enabled = false;
        MotorControl_EnterStop();
      }
      /* DRAG_FAIL is transient — module retries internally */
    }
    break;

  case MOTOR_STATE_FAULT:
  default:
    break;
  }
}

void MotorControl_Init(void) {
  (void)memset(&s_motorCtrl, 0, sizeof(s_motorCtrl));

  s_motorCtrl.status.state = MOTOR_STATE_STOP;
  s_motorCtrl.status.fault = MOTOR_FAULT_NONE;
  s_motorCtrl.status.control_mode = MOTOR_CONTROL_MODE_SPEED;
  s_motorCtrl.status.direction = MOTOR_CFG_DEFAULT_DIRECTION;
  s_motorCtrl.status.target_rpm = MOTOR_CFG_DEFAULT_TARGET_RPM;
  s_motorCtrl.controlModeRequest = MOTOR_CONTROL_MODE_SPEED;
  s_motorCtrl.targetRpmCommand = MOTOR_CFG_DEFAULT_TARGET_RPM;
  s_motorCtrl.targetIqCommandA =
      MotorControl_ClampIqTarget(MOTOR_CFG_DEFAULT_TARGET_IQ_A);
  s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
  s_motorCtrl.enableRequest = (MOTOR_APP_AUTO_START != 0U);
  s_motorCtrl.angleMonitorRequest = false;
  s_motorCtrl.status.enabled = s_motorCtrl.enableRequest;

  MotorFoc_Init(&s_motorCtrl.foc);
  MotorParamIdent_Init();
  MotorHwYtm32_Init();
  MotorHwYtm32_SetOutputsMasked(true);
#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
  MotorControl_ResetFastLoopProfile();
  g_motorFastLoopProfile.dwt_enabled = MotorControl_EnableDwtCycleCounter();
#endif
  MotorHwYtm32_StartSpeedLoopTimer();
}

void MotorControl_Enable(bool enable) {
  SDK_ENTER_CRITICAL();
  if (enable) {
    s_motorCtrl.angleMonitorRequest = false;
  }
  s_motorCtrl.enableRequest = enable;
  s_motorCtrl.status.enabled = enable;

  if (!enable) {
    MotorControl_EnterStop();
  }

  SDK_EXIT_CRITICAL();
}

void MotorControl_EnableAngleMonitor(bool enable) {
  SDK_ENTER_CRITICAL();

  s_motorCtrl.angleMonitorRequest = enable;
  if (enable) {
    s_motorCtrl.enableRequest = false;
    s_motorCtrl.status.enabled = false;
  } else if (s_motorCtrl.status.state == MOTOR_STATE_ANGLE_MONITOR) {
    MotorControl_EnterStop();
  }

  SDK_EXIT_CRITICAL();
}

bool MotorControl_SetControlMode(motor_control_mode_t mode) {
  bool accepted = false;

  SDK_ENTER_CRITICAL();
  if (MotorControl_IsValidControlMode(mode)) {
    if ((mode == MOTOR_CONTROL_MODE_SPEED) &&
        (s_motorCtrl.controlModeRequest != MOTOR_CONTROL_MODE_SPEED) &&
        (s_motorCtrl.status.state == MOTOR_STATE_CLOSED_LOOP)) {
      s_motorCtrl.foc.speed_pi_integrator_a = s_motorCtrl.status.iq_target_a;
    }

    s_motorCtrl.controlModeRequest = mode;
    s_motorCtrl.status.control_mode = mode;
    accepted = true;
  }
  SDK_EXIT_CRITICAL();

  return accepted;
}

bool MotorControl_SetTargetRpm(float targetRpm) {
  float clampedTargetRpm = targetRpm;

  if (clampedTargetRpm < 0.0f) {
    clampedTargetRpm = -clampedTargetRpm;
  }

  clampedTargetRpm =
      MotorControl_Clamp(clampedTargetRpm, 0.0f, MOTOR_CFG_MAX_TARGET_RPM);

  SDK_ENTER_CRITICAL();
  s_motorCtrl.targetRpmCommand = clampedTargetRpm;
  SDK_EXIT_CRITICAL();

  return true;
}

bool MotorControl_SetTargetIqA(float targetIqA) {
  SDK_ENTER_CRITICAL();
  s_motorCtrl.targetIqCommandA = MotorControl_ClampIqTarget(targetIqA);
  SDK_EXIT_CRITICAL();

  return true;
}

bool MotorControl_SetDirection(int8_t direction) {
  bool accepted = false;

  SDK_ENTER_CRITICAL();
  if (((direction == 1) || (direction == -1)) &&
      (s_motorCtrl.status.state == MOTOR_STATE_STOP)) {
    s_motorCtrl.status.direction = direction;
    s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
    accepted = true;
  }
  SDK_EXIT_CRITICAL();

  return accepted;
}

const motor_status_t *MotorControl_GetStatus(void) {
  return &s_motorCtrl.status;
}

const volatile motor_fast_loop_profile_t *
MotorControl_GetFastLoopProfile(void) {
  return &g_motorFastLoopProfile;
}

void MotorControl_ResetFastLoopProfile(void) {
  const bool dwtEnabled = ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);

  (void)memset((void *)&g_motorFastLoopProfile, 0,
               sizeof(g_motorFastLoopProfile));
  SystemCoreClockUpdate();
  if (dwtEnabled) {
    DWT->CYCCNT = 0U;
  }
  g_motorFastLoopProfile.dwt_enabled = dwtEnabled;
  g_motorFastLoopProfile.core_clock_hz = SystemCoreClock;
  g_motorFastLoopProfile.fast_loop_hz = MOTOR_CFG_PWM_FREQUENCY_HZ;
}

uint32_t MotorControl_GetTickMs(void) { return s_motorCtrl.tickMs; }

float MotorControl_GetWindDetectSpeedRpm(void) {
  return MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(s_motorCtrl.windDetectSpeedRadS);
}

bool MotorControl_StartParamIdent(void) {
  /* Use compile-time defaults for backward compatibility */
  motor_ident_config_t defaultCfg;
  defaultCfg.pole_pairs = MOTOR_CFG_POLE_PAIRS;
  defaultCfg.target_mech_rpm = 300.0f;
  defaultCfg.test_current_a = MOTOR_CFG_ALIGN_CURRENT_A * 0.4f;
  defaultCfg.lambda_iq_min_a = 0.3f;
  defaultCfg.lambda_iq_max_a = 2.0f;
  defaultCfg.lambda_iq_step_a = 0.2f;
  return MotorControl_StartParamIdentWithConfig(&defaultCfg);
}

/* Stored ident configuration from CAN or defaults */
static motor_ident_config_t s_pendingIdentCfg;

bool MotorControl_StartParamIdentWithConfig(const motor_ident_config_t *cfg) {
  bool accepted = false;

  SDK_ENTER_CRITICAL();
  if (s_motorCtrl.status.state == MOTOR_STATE_STOP) {
    s_pendingIdentCfg = *cfg;
    s_motorCtrl.paramIdentRequested = true;
    s_motorCtrl.enableRequest = true;
    s_motorCtrl.status.enabled = true;
    accepted = true;
  }
  SDK_EXIT_CRITICAL();

  return accepted;
}

void MotorControl_ProfileRecordFocTiming(uint32_t focTotalCycles,
                                         uint32_t observerCycles,
                                         uint32_t currentLoopCycles,
                                         uint32_t svmCycles) {
  MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.foc_total,
                                 focTotalCycles);
  MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.foc_observer_pll,
                                 observerCycles);
  MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.foc_current_loop,
                                 currentLoopCycles);
  MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.foc_svm, svmCycles);
}

/**
 * @brief Fast-loop ADC sequence completion handler.
 * Executed after ADC acquires phase currents and bus voltage.
 */
void ADC0_IRQHandler(void) {
  /* Channel-indexed scatter buffer: FIFO word = [ch_id:16 | adc_val:16].
   * Writing by channel index avoids dependence on ADC conversion order. */
  static uint16_t adcSampleBuffer[MOTOR_HW_ADC_BUFFER_SIZE];
  motor_foc_fast_input_t focInput;
  motor_foc_fast_output_t focOutput;
#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
  uint32_t isrStartCycles = 0U;
  uint32_t pwmStartCycles = 0U;
  uint32_t pwmCycles = 0U;
  uint32_t isrTotalCycles = 0U;
  const bool dwtEnabled = g_motorFastLoopProfile.dwt_enabled;

  MotorHwYtm32_SetAdcIrqDebugPinHigh();
  if (dwtEnabled) {
    isrStartCycles = DWT->CYCCNT;
  }
#else
  MotorHwYtm32_SetAdcIrqDebugPinHigh();
#endif
  /* Drain all FIFO entries: scatter each result into the buffer by its
   * channel index (upper 16 bits), keeping the 12-bit value (lower 16). */
  for (uint8_t i = 0; i < MotorHwYtm32_GetActiveAdcSequenceLength(); i++) {
    const uint32_t fifoWord = ADC0->FIFO;
    adcSampleBuffer[fifoWord >> 16] = (uint16_t)fifoWord;
  }
  ADC0->STS = ADC_STS_EOSEQ_MASK | ADC_STS_OVR_MASK;

#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
  /* ---- BEMF voltage wind detection (all MOSFET off, no FOC) ----------- */
  if (s_motorCtrl.status.state == MOTOR_STATE_WIND_DETECT) {
    s_motorCtrl.bemfIsrTickCounter++;

    /* Compute BEMF relative to the sampled virtual neutral node. */
    int16_t bemf_u_raw = adcSampleBuffer[MOTOR_HW_ADC_BEMF_U_CH];
    int16_t bemf_v_raw = adcSampleBuffer[MOTOR_HW_ADC_BEMF_V_CH];
    int16_t bemf_w_raw = adcSampleBuffer[MOTOR_HW_ADC_BEMF_W_CH];
    int16_t bemf_com_raw = adcSampleBuffer[MOTOR_HW_ADC_BEMF_COM_CH];
    const int16_t bemfU = (int16_t)bemf_u_raw - (int16_t)bemf_com_raw;
    const int16_t bemfV = (int16_t)bemf_v_raw - (int16_t)bemf_com_raw;
    const int16_t bemfW = (int16_t)bemf_w_raw - (int16_t)bemf_com_raw;

    /* Compute current sign for each phase (-1, 0, +1). */
    const int8_t signU = (bemfU > 30) ? 1 : ((bemfU < -30) ? -1 : 0);
    const int8_t signV = (bemfV > 30) ? 1 : ((bemfV < -30) ? -1 : 0);
    const int8_t signW = (bemfW > 30) ? 1 : ((bemfW < -30) ? -1 : 0);
    const int8_t signs[3] = {signU, signV, signW};

    /* Detect zero crossings on each phase. */
    for (uint8_t ph = 0U; ph < 3U; ph++) {
      if (signs[ph] != 0 && s_motorCtrl.bemfSignPrev[ph] != 0 &&
          signs[ph] != s_motorCtrl.bemfSignPrev[ph]) {
        /* Zero crossing detected on this phase. */
        if (s_motorCtrl.bemfZeroCrossingCount == 0U) {
          s_motorCtrl.bemfFirstCrossingTick = s_motorCtrl.bemfIsrTickCounter;
        }
        s_motorCtrl.bemfLastCrossingTick = s_motorCtrl.bemfIsrTickCounter;
        s_motorCtrl.bemfZeroCrossingCount++;

        /* Determine phase sequence from which phase crosses after which.
         * If U rises then V rises next → forward (+1).
         * If U rises then W rises next → reverse (-1). */
        if (signs[ph] > 0) { /* Rising edge */
          if (ph == 0U)
            s_motorCtrl.bemfPhaseSequence = 1; /* U rise → expect V next */
          else if (ph == 1U && s_motorCtrl.bemfPhaseSequence == 1)
            s_motorCtrl.bemfPhaseSequence = 1; /* Confirmed forward */
          else if (ph == 2U && s_motorCtrl.bemfPhaseSequence == 1)
            s_motorCtrl.bemfPhaseSequence = -1; /* W rose before V → reverse */
        }
      }
      if (signs[ph] != 0) {
        s_motorCtrl.bemfSignPrev[ph] = signs[ph];
      }
    }

    /* Cache raw BEMF values for CAN telemetry */
    s_motorCtrl.bemfLastU = bemf_u_raw;
    s_motorCtrl.bemfLastV = bemf_v_raw;
    s_motorCtrl.bemfLastW = bemf_w_raw;
    s_motorCtrl.bemfLastCom = bemf_com_raw;

    /* Store avg BEMF amplitude in observer_flux_vs for debugging. */
    s_motorCtrl.foc.observer_lambda_vs =
        (float)(__builtin_abs(bemfU) + __builtin_abs(bemfV) +
                __builtin_abs(bemfW)) /
        3.0f;

    goto irq_exit;
  }
#endif

  if (s_motorCtrl.status.state == MOTOR_STATE_ANGLE_MONITOR) {
    MotorControl_UpdateAngleMonitorSample(
        adcSampleBuffer[MOTOR_HW_ADC_BEMF_U_CH],
        adcSampleBuffer[MOTOR_HW_ADC_BEMF_V_CH],
        adcSampleBuffer[MOTOR_HW_ADC_BEMF_W_CH],
        adcSampleBuffer[MOTOR_HW_ADC_BEMF_COM_CH]);
    goto irq_exit;
  }

  /* Convert raw ADC values → physical units and store into status */
  s_motorCtrl.status.phase_current_a =
      ((float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_A_CH] -
       s_motorCtrl.currentOffsetARaw) *
      MOTOR_CFG_ADC_COUNT_TO_CURRENT_A;
  s_motorCtrl.status.phase_current_b =
      ((float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_B_CH] -
       s_motorCtrl.currentOffsetBRaw) *
      MOTOR_CFG_ADC_COUNT_TO_CURRENT_A;
  s_motorCtrl.status.phase_current_c =
      ((float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_C_CH] -
       s_motorCtrl.currentOffsetCRaw) *
      MOTOR_CFG_ADC_COUNT_TO_CURRENT_A;
  s_motorCtrl.status.bus_voltage_v =
      (float)adcSampleBuffer[MOTOR_HW_ADC_VBUS_CH] *
      MOTOR_CFG_ADC_COUNT_TO_VBUS_V;

  if (s_motorCtrl.status.state == MOTOR_STATE_OFFSET_CAL) {
    s_motorCtrl.currentOffsetASum +=
        (float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_A_CH];
    s_motorCtrl.currentOffsetBSum +=
        (float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_B_CH];
    s_motorCtrl.currentOffsetCSum +=
        (float)adcSampleBuffer[MOTOR_HW_ADC_CURRENT_C_CH];
    s_motorCtrl.offsetSampleCount++;

    if (s_motorCtrl.offsetSampleCount >= MOTOR_CFG_OFFSET_CAL_SAMPLES) {
      const float sampleCount = (float)s_motorCtrl.offsetSampleCount;

      s_motorCtrl.currentOffsetARaw =
          s_motorCtrl.currentOffsetASum / sampleCount;
      s_motorCtrl.currentOffsetBRaw =
          s_motorCtrl.currentOffsetBSum / sampleCount;
      s_motorCtrl.currentOffsetCRaw =
          s_motorCtrl.currentOffsetCSum / sampleCount;
      if (s_motorCtrl.paramIdentRequested) {
        /* Start param ident with user-provided config */
        s_motorCtrl.paramIdentRequested = false;
        motor_ident_config_t identCfg = s_pendingIdentCfg;
        /* Fill in system / timing fields the GUI can't know */
        identCfg.voltage_step_v = 2.0f;
        identCfg.pwm_dt_s = MOTOR_CFG_FAST_LOOP_DT_S;
        identCfg.slow_dt_s = MOTOR_CFG_SPEED_LOOP_DT_S;
        identCfg.max_duty_modulation = MOTOR_CFG_SVM_MAX_MODULATION;
        identCfg.overcurrent_a = MOTOR_CFG_PHASE_OVERCURRENT_A;
        MotorParamIdent_Start(&identCfg);
        MotorHwYtm32_SetOutputsMasked(false);
        s_motorCtrl.outputsUnmasked = true;
        MotorControl_SetState(MOTOR_STATE_PARAM_IDENT);
      } else {
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
        MotorControl_StartWindDetect();
#else
        MotorControl_StartStaticStartupSequence();
#endif
      }
    }
    goto irq_exit;
  }

  if ((s_motorCtrl.status.state == MOTOR_STATE_STOP) ||
      (s_motorCtrl.status.state == MOTOR_STATE_FAULT)) {
    goto irq_exit;
  }

  /* ---- Parameter identification fast loop (bypasses normal FOC) ---- */
  if (s_motorCtrl.status.state == MOTOR_STATE_PARAM_IDENT) {
    motor_ident_fast_input_t identIn;
    motor_ident_fast_output_t identOut;
    identIn.phase_current_a = s_motorCtrl.status.phase_current_a;
    identIn.phase_current_b = s_motorCtrl.status.phase_current_b;
    identIn.phase_current_c = s_motorCtrl.status.phase_current_c;
    identIn.bus_voltage_v = s_motorCtrl.status.bus_voltage_v;
    MotorParamIdent_RunFastLoop(&identIn, &identOut);
    MotorHwYtm32_ApplyPhaseDuty(identOut.duty_u, identOut.duty_v,
                                identOut.duty_w);
    goto irq_exit;
  }

  if (__builtin_fabsf(s_motorCtrl.status.phase_current_a) >
          MOTOR_CFG_PHASE_OVERCURRENT_A ||
      __builtin_fabsf(s_motorCtrl.status.phase_current_b) >
          MOTOR_CFG_PHASE_OVERCURRENT_A ||
      __builtin_fabsf(s_motorCtrl.status.phase_current_c) >
          MOTOR_CFG_PHASE_OVERCURRENT_A) {
    MotorControl_LatchFault(MOTOR_FAULT_OVERCURRENT);
    goto irq_exit;
  }

  if (s_motorCtrl.status.bus_voltage_v < MOTOR_CFG_VBUS_UNDERVOLTAGE_V) {
    MotorControl_LatchFault(MOTOR_FAULT_VBUS_UNDERVOLTAGE);
    goto irq_exit;
  }

  if (s_motorCtrl.status.bus_voltage_v > MOTOR_CFG_VBUS_OVERVOLTAGE_V) {
    MotorControl_LatchFault(MOTOR_FAULT_VBUS_OVERVOLTAGE);
    goto irq_exit;
  }

  if (s_motorCtrl.status.state == MOTOR_STATE_HFI_IPD) {
    motor_ab_frame_t hfiIab;

    MotorControl_Clarke(s_motorCtrl.status.phase_current_a,
                        s_motorCtrl.status.phase_current_b,
                        s_motorCtrl.status.phase_current_c, &hfiIab);
    MotorControl_RunHfiIpdFastLoop(&hfiIab);
    goto irq_exit;
  }

  /* Determine control angle based on motor state */
  if (s_motorCtrl.status.state == MOTOR_STATE_ALIGN) {
    focInput.control_angle_rad = MotorControl_GetAlignAngle();
  }
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
  /* WIND_DETECT is handled above (BEMF branch) and never reaches here. */
  else if (s_motorCtrl.status.state == MOTOR_STATE_COAST_DOWN) {
    /* Coast-down: observer angle drives Park transform for regen braking. */
    focInput.control_angle_rad = s_motorCtrl.latestObserverAngleRad;
  }
#endif
  else {
    s_motorCtrl.openLoopAngleRad = MotorFoc_WrapAngle0ToTwoPi(
        s_motorCtrl.openLoopAngleRad +
        (s_motorCtrl.openLoopSpeedRadS * MOTOR_CFG_FAST_LOOP_DT_S));

    if (s_motorCtrl.status.state == MOTOR_STATE_OPEN_LOOP_RAMP) {
      focInput.control_angle_rad = s_motorCtrl.openLoopAngleRad;
    } else if (s_motorCtrl.closedLoopBlend >= 1.0f) {
      focInput.control_angle_rad = s_motorCtrl.latestObserverAngleRad;
    } else {
      focInput.control_angle_rad = MotorControl_BlendAngle(
          s_motorCtrl.openLoopAngleRad, s_motorCtrl.latestObserverAngleRad,
          s_motorCtrl.closedLoopBlend);
    }
  }

  /* Populate FOC input directly from status fields */
  focInput.phase_current_a = s_motorCtrl.status.phase_current_a;
  focInput.phase_current_b = s_motorCtrl.status.phase_current_b;
  focInput.phase_current_c = s_motorCtrl.status.phase_current_c;
  focInput.bus_voltage_v = s_motorCtrl.status.bus_voltage_v;
  focInput.id_target_a = s_motorCtrl.status.id_target_a;
  focInput.iq_target_a = s_motorCtrl.status.iq_target_a;
  focInput.deadtime_comp_enable =
      (s_motorCtrl.status.state != MOTOR_STATE_ALIGN);

  /* D-axis step injection from waveform capture */
  {
    float stepInjectId = CanConfig_GetStepInjectId();
    if (stepInjectId != 0.0f) {
      focInput.id_target_a = stepInjectId;
    }
  }

  MotorFoc_RunFast(&s_motorCtrl.foc, &focInput, &focOutput);

  s_motorCtrl.status.electrical_speed_rad_s = focOutput.observer_speed_rad_s;
  s_motorCtrl.status.id_a = focOutput.id_a;
  s_motorCtrl.status.iq_a = focOutput.iq_a;
  s_motorCtrl.latestPhaseErrorRad = focOutput.phase_error_rad;
  s_motorCtrl.latestObserverAngleRad = focOutput.observer_angle_rad;
  s_motorCtrl.voltageModulationRatio = focOutput.voltage_modulation_ratio;
  s_motorCtrl.status.electrical_angle_rad =
      (s_motorCtrl.status.state == MOTOR_STATE_CLOSED_LOOP)
          ? focOutput.observer_angle_rad
          : focInput.control_angle_rad;

  /* --- Waveform capture hook (Status3 high-speed data) --- */
  CanConfig_WaveformSample(
      s_motorCtrl.status.phase_current_a, s_motorCtrl.status.phase_current_b,
      s_motorCtrl.status.phase_current_c, s_motorCtrl.status.bus_voltage_v,
      focOutput.id_a, focOutput.iq_a, s_motorCtrl.status.electrical_angle_rad);

  /* Un-mask outputs if not yet done.
   * Wind detect exits via BEMF branch (never reaches here).
   * Coast-Down already un-masks in StartCoastDown. */
  if (!s_motorCtrl.outputsUnmasked) {
    MotorHwYtm32_SetOutputsMasked(false);
    s_motorCtrl.outputsUnmasked = true;
  }

#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
  if (dwtEnabled) {
    pwmStartCycles = DWT->CYCCNT;
  }
#endif
  MotorHwYtm32_ApplyPhaseDuty(focOutput.duty_u, focOutput.duty_v,
                              focOutput.duty_w);
#if (MOTOR_CFG_ENABLE_DWT_PROFILE != 0U)
  if (dwtEnabled) {
    pwmCycles = DWT->CYCCNT - pwmStartCycles;
    isrTotalCycles = DWT->CYCCNT - isrStartCycles;
    MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.pwm_update,
                                   pwmCycles);
    MotorControl_ProfileRecordStat(&g_motorFastLoopProfile.adc_irq_total,
                                   isrTotalCycles);
  }
#endif

irq_exit:
  MotorHwYtm32_SetAdcIrqDebugPinLow();
}

/**
 * @brief Slow-loop (Speed Loop) Timer IRQ handler.
 */
void pTMR0_Ch0_IRQHandler(void) {
  MotorHwYtm32_ClearSpeedLoopIrq();
  MotorControl_HandleSlowLoop();
}

/**
 * @brief Populate a FOC diagnostics snapshot for CAN telemetry.
 */
void MotorControl_GetFocDiagnostics(motor_foc_diagnostics_t *diag) {
  if (diag == NULL) {
    return;
  }

  diag->observer_angle_rad = s_motorCtrl.foc.pll_phase_rad;
  diag->observer_flux_vs = s_motorCtrl.foc.observer_lambda_vs;
  diag->pll_speed_rad_s = s_motorCtrl.foc.pll_speed_rad_s;
  diag->phase_error_rad = s_motorCtrl.latestPhaseErrorRad;
  diag->speed_pi_integrator_a = s_motorCtrl.foc.speed_pi_integrator_a;
  diag->id_pi_integrator_v = s_motorCtrl.foc.current_pi_d_integrator_v;
  diag->iq_pi_integrator_v = s_motorCtrl.foc.current_pi_q_integrator_v;
  diag->voltage_modulation_ratio = s_motorCtrl.voltageModulationRatio;
  diag->fw_id_target_a = s_motorCtrl.fwIdTargetA;
  diag->open_loop_angle_rad = s_motorCtrl.openLoopAngleRad;
  diag->open_loop_speed_rad_s = s_motorCtrl.openLoopSpeedRadS;
  diag->closed_loop_blend = s_motorCtrl.closedLoopBlend;
  diag->state_time_ms = s_motorCtrl.stateTimeMs;
  diag->obs_lock_residual_rad = s_motorCtrl.observerLockResidualRad;
  diag->stall_div_count = (uint16_t)s_motorCtrl.stallAngleDivCount;

  /* Duty cycles: not cached in s_motorCtrl — report 0 unless extended.
   * The FOC output is local to the ADC ISR. For duty telemetry, the
   * caller can read directly from the eTMR hardware registers if needed,
   * or we can extend the IRQ to cache last duties in s_motorCtrl. */
  diag->duty_u = 0.0f;
  diag->duty_v = 0.0f;
  diag->duty_w = 0.0f;

  /* BEMF wind detection diagnostics — only meaningful in WIND_DETECT state */
  diag->bemf_crossing_count = (uint16_t)s_motorCtrl.bemfZeroCrossingCount;
  diag->bemf_detected_rpm = (int16_t)MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(
      s_motorCtrl.windDetectSpeedRadS);
  diag->bemf_phase_sequence = s_motorCtrl.bemfPhaseSequence;
  diag->bemf_u_raw = s_motorCtrl.bemfLastU;
  diag->bemf_v_raw = s_motorCtrl.bemfLastV;
  diag->bemf_w_raw = s_motorCtrl.bemfLastW;
  diag->bemf_com_raw = s_motorCtrl.bemfLastCom;
}
