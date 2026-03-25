/**
 * @file motor_control.c
 * @brief High-level motor state machine and profiling implementation.
 */

#include "motor_control.h"

#include <string.h>

#include "motor_foc.h"
#include "motor_hw_ytm32.h"
#include "motor_user_config.h"
#include "sdk_project_config.h"

#define MOTOR_CTRL_DWT_LAR_KEY (0xC5ACCE55UL)
#define MOTOR_CTRL_DWT_LAR (*((volatile uint32_t *)(DWT_BASE + 0xFB0UL)))

typedef struct {
  motor_status_t status;
  motor_foc_state_t foc;
  bool enableRequest;
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
  float windDetectSpeedRadS;        /* Detected initial electrical speed */
  float windDetectPrevSpeedRadS;    /* Previous speed for convergence check */
  int8_t windDetectDirection;       /* Detected rotation direction +1/-1, 0=still */
  uint32_t windDetectSettleCount;   /* Observer convergence counter */
  bool windCatchActive;             /* Tailwind catch in progress */

  /* Field weakening */
  float fwIdTargetA;                /* Field weakening d-axis current command */
  float voltageModulationRatio;     /* Latest |Vab|/Vbus from FOC output */
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
 * @brief Transition to a new motor control state.
 * @param nextState The state to transition to.
 */
static void MotorControl_SetState(motor_control_state_t nextState) {
  s_motorCtrl.status.state = nextState;
  s_motorCtrl.stateTimeMs = 0U;
}

/**
 * @brief Convert raw ADC count to current (A) based on board configuration.
 * @param rawValue   ADC reading (12-bit).
 * @param rawOffset  Average ADC zero-current offset.
 * @return Calibrated phase current in amperes.
 */
static inline float MotorControl_CurrentFromRaw(uint16_t rawValue,
                                                float rawOffset) {
  return ((float)rawValue - rawOffset) * MOTOR_CFG_ADC_COUNT_TO_CURRENT_A;
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

  if (s_motorCtrl.status.direction < 0) {
    angleRad += MOTOR_CFG_PI_F;
  }

  return MotorFoc_WrapAngle0ToTwoPi(angleRad);
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
  s_motorCtrl.windDetectSpeedRadS = 0.0f;
  s_motorCtrl.windDetectPrevSpeedRadS = 0.0f;
  s_motorCtrl.windDetectDirection = 0;
  s_motorCtrl.windDetectSettleCount = 0U;
  s_motorCtrl.windCatchActive = false;
  s_motorCtrl.fwIdTargetA = 0.0f;
  s_motorCtrl.voltageModulationRatio = 0.0f;
  MotorFoc_Reset(&s_motorCtrl.foc);
}

/**
 * @brief Sub-routine to cleanly enter the Stopped state.
 * Halts PWM and masks HW timer channels.
 */
static void MotorControl_EnterStop(void) {
  MotorHwYtm32_DisableFastLoopSampling();
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_ResetRuntime();
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
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.status.id_target_a = MOTOR_CFG_ALIGN_CURRENT_A;
  s_motorCtrl.status.iq_target_a = 0.0f;
  s_motorCtrl.status.observer_locked = false;
  s_motorCtrl.fastLoopRunning = true;
  MotorControl_SetState(MOTOR_STATE_ALIGN);
}

/**
 * @brief Transition step from Align to Open-Loop spinning.
 */
static void MotorControl_StartOpenLoop(void) {
  s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
  s_motorCtrl.openLoopSpeedRadS = 0.0f;
  s_motorCtrl.startupTimeMs = 0U;
  s_motorCtrl.observerPhaseOffsetRad = s_motorCtrl.latestPhaseErrorRad;
  s_motorCtrl.observerLockResidualRad = 0.0f;
  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a =
      MotorControl_GetSignedDirection() * MOTOR_CFG_ALIGN_CURRENT_A;
  MotorControl_SetState(MOTOR_STATE_OPEN_LOOP_RAMP);
}

/**
 * @brief Transition step from Open-Loop spinning to Sensorless Closed-Loop control.
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
  s_motorCtrl.foc.speed_pi_integrator_a = MotorControl_Clamp(
      s_motorCtrl.status.iq_target_a,
      -MOTOR_CFG_OPEN_LOOP_IQ_A * 0.5f,
       MOTOR_CFG_OPEN_LOOP_IQ_A * 0.5f);
  s_motorCtrl.fwIdTargetA = 0.0f;
  MotorControl_SetState(MOTOR_STATE_CLOSED_LOOP);
}

/* ========================= Wind Detect / Catch Spin ======================== */
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)

/* Forward declaration — defined after UpdateWindDetectState which calls it. */
static void MotorControl_StartCoastDown(void);

/**
 * @brief Enter the Wind Detect state: observer runs on BEMF, PWM outputs masked.
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

  /* Keep PWM outputs masked — only the observer is active (BEMF-driven). */
  MotorHwYtm32_SetOutputsMasked(true);
  MotorControl_SetState(MOTOR_STATE_WIND_DETECT);
}

/**
 * @brief Tailwind catch: jump straight to closed-loop using observer angle.
 */
static void MotorControl_StartCatchSpin(void) {
  const float observedMechanicalRpm =
      __builtin_fabsf(MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(
          s_motorCtrl.windDetectSpeedRadS));

  s_motorCtrl.windCatchActive = true;
  s_motorCtrl.closedLoopBlend = 1.0f;  /* fully trust observer immediately */
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
  const float currentSpeedRadS = s_motorCtrl.status.electrical_speed_rad_s;
  const float speedDelta =
      __builtin_fabsf(currentSpeedRadS - s_motorCtrl.windDetectPrevSpeedRadS);

  s_motorCtrl.stateTimeMs++;
  s_motorCtrl.startupTimeMs++;
  s_motorCtrl.windDetectPrevSpeedRadS = currentSpeedRadS;

  /* Check if observer speed has settled */
  if (speedDelta < MOTOR_CFG_WIND_DETECT_SPEED_TOL_RAD_S) {
    s_motorCtrl.windDetectSettleCount++;
  } else {
    s_motorCtrl.windDetectSettleCount = 0U;
  }

  /* Timeout protection */
  if (s_motorCtrl.stateTimeMs >= MOTOR_CFG_WIND_DETECT_TIMEOUT_MS) {
    /* Timed out waiting for convergence — assume standstill, go Align */
    MotorControl_StartAlign();
    return;
  }

  /* Wait until settled */
  if (s_motorCtrl.windDetectSettleCount < MOTOR_CFG_WIND_DETECT_SETTLE_COUNT) {
    return;
  }

  /* Observer has converged — record detected speed */
  s_motorCtrl.windDetectSpeedRadS = currentSpeedRadS;

  /* Flux magnitude validation: if the observer's estimated flux linkage is
   * below the minimum threshold, the BEMF is too weak to trust the speed
   * estimate. This filters out false CatchSpin triggers caused by ADC noise
   * when the motor is actually stationary with near-zero BEMF. */
  const float observerFluxVs = s_motorCtrl.foc.observer_lambda_vs;
  const bool fluxTooLow =
      (observerFluxVs < MOTOR_CFG_WIND_DETECT_MIN_FLUX_VS);

  if (fluxTooLow ||
      (__builtin_fabsf(currentSpeedRadS) <
       MOTOR_CFG_WIND_DETECT_STANDSTILL_RAD_S)) {
    /* Rotor is essentially standstill or flux too weak to trust — normal
     * startup flow */
    MotorControl_StartAlign();
    return;
  }

  /* Determine detected direction */
  s_motorCtrl.windDetectDirection = (currentSpeedRadS > 0.0f) ? 1 : -1;

  if (s_motorCtrl.windDetectDirection == s_motorCtrl.status.direction) {
    /* Tailwind: rotor spinning in the desired direction */
    const float detectedRpm = __builtin_fabsf(
        MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(currentSpeedRadS));

    if (detectedRpm <= MOTOR_CFG_CATCH_MAX_SPEED_RPM) {
      /* Speed within safe catch range — direct jump to closed-loop */
      MotorControl_StartCatchSpin();
    } else {
      /* Speed too high — coast down with outputs off, wait for safe speed */
      MotorControl_StartCoastDown();
    }
  } else {
    /* Headwind: rotor spinning opposite to desired direction.
     * No braking resistor — coast down with outputs off. */
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
  float brakeGain = MotorControl_Clamp(
      (MOTOR_CFG_COAST_BRAKE_VBUS_LIMIT_V + MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V
       - vbus) / MOTOR_CFG_COAST_BRAKE_VBUS_HYST_V,
      0.0f, 1.0f);

  s_motorCtrl.status.id_target_a = 0.0f;
  s_motorCtrl.status.iq_target_a =
      -speedSign * MOTOR_CFG_COAST_BRAKE_IQ_A * brakeGain;

  /* When speed drops below safe threshold, enter normal Align startup */
  if (absSpeedRadS < MOTOR_CFG_COAST_DOWN_SAFE_SPEED_RAD_S) {
    MotorControl_StartAlign();
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
  const float iqStepA =
      (__builtin_fabsf(MOTOR_CFG_OPEN_LOOP_IQ_A - MOTOR_CFG_ALIGN_CURRENT_A) *
       MOTOR_CFG_SPEED_LOOP_DT_S) /
      openLoopRampTimeS;
  const float phaseTrackBlend = MotorControl_Clamp(
      MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S * MOTOR_CFG_SPEED_LOOP_DT_S, 0.0f,
      1.0f);
  float openLoopSpeedMagRadS;

  s_motorCtrl.stateTimeMs++;
  s_motorCtrl.startupTimeMs++;

  openLoopSpeedMagRadS = MotorControl_Clamp(
      (__builtin_fabsf(s_motorCtrl.openLoopSpeedRadS) + speedStepRadS), 0.0f,
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
  s_motorCtrl.status.iq_target_a = MotorControl_SlewTowards(
      s_motorCtrl.status.iq_target_a,
      MotorControl_GetSignedDirection() * MOTOR_CFG_OPEN_LOOP_IQ_A, iqStepA);

  if ((__builtin_fabsf(s_motorCtrl.status.electrical_speed_rad_s) >=
       MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S) &&
      (__builtin_fabsf(s_motorCtrl.observerLockResidualRad) <=
       MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD)) {
    s_motorCtrl.observerLockCount++;
  } else {
    s_motorCtrl.observerLockCount = 0U;
  }

  if (s_motorCtrl.observerLockCount >= MOTOR_CFG_OBSERVER_LOCK_COUNT) {
    MotorControl_StartClosedLoop();
    return;
  }

  if (s_motorCtrl.startupTimeMs >= MOTOR_CFG_STARTUP_TIMEOUT_MS) {
    MotorControl_LatchFault(MOTOR_FAULT_STARTUP_TIMEOUT);
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
 * @brief Main periodic slow loop managing state machine dispatch and user ramping.
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

  if (!s_motorCtrl.enableRequest) {
    if (s_motorCtrl.status.state != MOTOR_STATE_STOP) {
      MotorControl_EnterStop();
    }
    return;
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

  case MOTOR_STATE_ALIGN:
    s_motorCtrl.stateTimeMs++;
    s_motorCtrl.startupTimeMs++;
    s_motorCtrl.status.id_target_a = MOTOR_CFG_ALIGN_CURRENT_A;
    s_motorCtrl.status.iq_target_a = 0.0f;

    if (s_motorCtrl.stateTimeMs >= MOTOR_CFG_ALIGN_TIME_MS) {
      MotorControl_StartOpenLoop();
    }
    break;

  case MOTOR_STATE_OPEN_LOOP_RAMP:
    MotorControl_UpdateOpenLoopState();
    break;

  case MOTOR_STATE_CLOSED_LOOP:
    MotorControl_UpdateClosedLoopState();
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
  s_motorCtrl.status.enabled = s_motorCtrl.enableRequest;

  MotorFoc_Init(&s_motorCtrl.foc);
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
  s_motorCtrl.enableRequest = enable;
  s_motorCtrl.status.enabled = enable;

  if (!enable) {
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
  motor_adc_raw_frame_t rawFrame;
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

  rawFrame.overrun = false;
  MotorHwYtm32_ReadTriggeredFrame(&rawFrame);
  if (rawFrame.overrun) {
    MotorControl_LatchFault(MOTOR_FAULT_ADC_OVERRUN);
    goto irq_exit;
  }

  /* Convert raw ADC values and store directly into status */
  s_motorCtrl.status.phase_current_a = MotorControl_CurrentFromRaw(
      rawFrame.current_a_raw, s_motorCtrl.currentOffsetARaw);
  s_motorCtrl.status.phase_current_b = MotorControl_CurrentFromRaw(
      rawFrame.current_b_raw, s_motorCtrl.currentOffsetBRaw);
  s_motorCtrl.status.phase_current_c = MotorControl_CurrentFromRaw(
      rawFrame.current_c_raw, s_motorCtrl.currentOffsetCRaw);
  s_motorCtrl.status.bus_voltage_v =
      (float)rawFrame.vbus_raw * MOTOR_CFG_ADC_COUNT_TO_VBUS_V;

  if (s_motorCtrl.status.state == MOTOR_STATE_OFFSET_CAL) {
    s_motorCtrl.currentOffsetASum += (float)rawFrame.current_a_raw;
    s_motorCtrl.currentOffsetBSum += (float)rawFrame.current_b_raw;
    s_motorCtrl.currentOffsetCSum += (float)rawFrame.current_c_raw;
    s_motorCtrl.offsetSampleCount++;

    if (s_motorCtrl.offsetSampleCount >= MOTOR_CFG_OFFSET_CAL_SAMPLES) {
      const float sampleCount = (float)s_motorCtrl.offsetSampleCount;

      s_motorCtrl.currentOffsetARaw =
          s_motorCtrl.currentOffsetASum / sampleCount;
      s_motorCtrl.currentOffsetBRaw =
          s_motorCtrl.currentOffsetBSum / sampleCount;
      s_motorCtrl.currentOffsetCRaw =
          s_motorCtrl.currentOffsetCSum / sampleCount;
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
      MotorControl_StartWindDetect();
#else
      MotorControl_StartAlign();
#endif
    }
    goto irq_exit;
  }

  if ((s_motorCtrl.status.state == MOTOR_STATE_STOP) ||
      (s_motorCtrl.status.state == MOTOR_STATE_FAULT)) {
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

  /* Determine control angle based on motor state */
  if (s_motorCtrl.status.state == MOTOR_STATE_ALIGN) {
    focInput.control_angle_rad = MotorControl_GetAlignAngle();
  }
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
  else if (s_motorCtrl.status.state == MOTOR_STATE_WIND_DETECT) {
    /* During wind detect, use observer angle as control angle.
     * PWM outputs are masked so no current flows — observer runs on BEMF. */
    focInput.control_angle_rad = s_motorCtrl.latestObserverAngleRad;
  } else if (s_motorCtrl.status.state == MOTOR_STATE_COAST_DOWN) {
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
      (s_motorCtrl.status.state != MOTOR_STATE_ALIGN)
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
      && (s_motorCtrl.status.state != MOTOR_STATE_WIND_DETECT)
      /* Coast-Down uses active braking — deadtime comp enabled */
#endif
      ;

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

  /* Un-mask outputs if not yet done (skip during wind detect — outputs stay masked) */
  if (!s_motorCtrl.outputsUnmasked
#if (MOTOR_CFG_ENABLE_WIND_DETECT != 0U)
      && (s_motorCtrl.status.state != MOTOR_STATE_WIND_DETECT)
      /* Coast-Down already un-masks in StartCoastDown for active braking */
#endif
  ) {
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
