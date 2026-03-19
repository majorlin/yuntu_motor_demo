#include "motor_control.h"

#include <string.h>

#include "sdk_project_config.h"
#include "motor_foc.h"
#include "motor_hw_ytm32.h"
#include "motor_user_config.h"

typedef struct
{
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
} motor_control_ctx_t;

static motor_control_ctx_t s_motorCtrl;
volatile motor_debug_t g_motorDebug;

static float MotorControl_Clamp(float value, float minValue, float maxValue)
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

static float MotorControl_WrapAngleSigned(float angleRad)
{
    float wrappedAngle = MotorFoc_WrapAngle0ToTwoPi(angleRad);

    if (wrappedAngle > MOTOR_CFG_PI_F)
    {
        wrappedAngle -= MOTOR_CFG_TWO_PI_F;
    }

    return wrappedAngle;
}

static void MotorControl_SetState(motor_control_state_t nextState)
{
    s_motorCtrl.status.state = nextState;
    s_motorCtrl.stateTimeMs = 0U;
}

static float MotorControl_CurrentFromRaw(uint16_t rawValue, float rawOffset)
{
    return ((float)rawValue - rawOffset) * MOTOR_CFG_ADC_COUNT_TO_CURRENT_A;
}

static float MotorControl_GetSignedDirection(void)
{
    return (s_motorCtrl.status.direction >= 0) ? 1.0f : -1.0f;
}

static float MotorControl_GetAlignAngle(void)
{
    float angleRad = MOTOR_CFG_ALIGN_ANGLE_RAD;

    if (s_motorCtrl.status.direction < 0)
    {
        angleRad += MOTOR_CFG_PI_F;
    }

    return MotorFoc_WrapAngle0ToTwoPi(angleRad);
}

static float MotorControl_GetTargetElectricalSpeedRadS(void)
{
    return MotorControl_GetSignedDirection() *
           MOTOR_CFG_MECH_RPM_TO_ELEC_RAD_S(s_motorCtrl.status.target_rpm);
}

static float MotorControl_BlendAngle(float fromAngleRad, float toAngleRad, float blend)
{
    return MotorFoc_WrapAngle0ToTwoPi(fromAngleRad +
                                      (blend * MotorFoc_AngleDiff(toAngleRad, fromAngleRad)));
}

static float MotorControl_BlendPhase(float fromPhaseRad, float toPhaseRad, float blend)
{
    return MotorControl_WrapAngleSigned(fromPhaseRad +
                                        (blend * MotorFoc_AngleDiff(toPhaseRad, fromPhaseRad)));
}

static void MotorControl_UpdateDebugState(float controlAngleRad)
{
    g_motorDebug.control_state = (uint32_t)s_motorCtrl.status.state;
    g_motorDebug.fault_code = (uint32_t)s_motorCtrl.status.fault;
    g_motorDebug.observer_locked = s_motorCtrl.status.observer_locked;
    g_motorDebug.outputs_unmasked = s_motorCtrl.outputsUnmasked;
    g_motorDebug.fast_loop_running = s_motorCtrl.fastLoopRunning;
    g_motorDebug.startup_time_ms = s_motorCtrl.startupTimeMs;
    g_motorDebug.state_time_ms = s_motorCtrl.stateTimeMs;
    g_motorDebug.observer_lock_count = s_motorCtrl.observerLockCount;
    g_motorDebug.observer_loss_count = s_motorCtrl.observerLossCount;
    g_motorDebug.closed_loop_blend = s_motorCtrl.closedLoopBlend;
    g_motorDebug.open_loop_angle_rad = s_motorCtrl.openLoopAngleRad;
    g_motorDebug.open_loop_speed_rad_s = s_motorCtrl.openLoopSpeedRadS;
    g_motorDebug.control_angle_rad = controlAngleRad;
    g_motorDebug.observer_angle_rad = s_motorCtrl.latestObserverAngleRad;
    g_motorDebug.observer_speed_rad_s = s_motorCtrl.status.electrical_speed_rad_s;
    g_motorDebug.observer_phase_offset_rad = s_motorCtrl.observerPhaseOffsetRad;
    g_motorDebug.observer_lock_residual_rad = s_motorCtrl.observerLockResidualRad;
    g_motorDebug.phase_error_rad = s_motorCtrl.latestPhaseErrorRad;
    g_motorDebug.electrical_angle_rad = s_motorCtrl.status.electrical_angle_rad;
    g_motorDebug.electrical_speed_rad_s = s_motorCtrl.status.electrical_speed_rad_s;
    g_motorDebug.target_electrical_speed_rad_s = MotorControl_GetTargetElectricalSpeedRadS();
    g_motorDebug.mechanical_rpm = s_motorCtrl.status.mechanical_rpm;
    g_motorDebug.bus_voltage_v = s_motorCtrl.status.bus_voltage_v;
    g_motorDebug.phase_current_a = s_motorCtrl.status.phase_current_a;
    g_motorDebug.phase_current_b = s_motorCtrl.status.phase_current_b;
    g_motorDebug.phase_current_c = s_motorCtrl.status.phase_current_c;
    g_motorDebug.id_a = s_motorCtrl.status.id_a;
    g_motorDebug.iq_a = s_motorCtrl.status.iq_a;
    g_motorDebug.id_target_a = s_motorCtrl.status.id_target_a;
    g_motorDebug.iq_target_a = s_motorCtrl.status.iq_target_a;
}

static void MotorControl_ResetRuntime(void)
{
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
    MotorFoc_Reset(&s_motorCtrl.foc);
    (void)memset((void *)&g_motorDebug, 0, sizeof(g_motorDebug));
    MotorControl_UpdateDebugState(s_motorCtrl.openLoopAngleRad);
}

static void MotorControl_EnterStop(void)
{
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
    MotorControl_SetState(MOTOR_STATE_STOP);
}

static void MotorControl_LatchFault(motor_fault_t fault)
{
    MotorHwYtm32_DisableFastLoopSampling();
    MotorHwYtm32_SetOutputsMasked(true);
    s_motorCtrl.fastLoopRunning = false;
    s_motorCtrl.outputsUnmasked = false;
    s_motorCtrl.status.fault = fault;
    s_motorCtrl.status.observer_locked = false;
    s_motorCtrl.status.id_target_a = 0.0f;
    s_motorCtrl.status.iq_target_a = 0.0f;
    MotorControl_SetState(MOTOR_STATE_FAULT);
}

static void MotorControl_BeginOffsetCalibration(void)
{
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

static void MotorControl_StartAlign(void)
{
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

static void MotorControl_StartOpenLoop(void)
{
    s_motorCtrl.openLoopAngleRad = MotorControl_GetAlignAngle();
    s_motorCtrl.openLoopSpeedRadS = MotorControl_GetSignedDirection() *
                                    MOTOR_CFG_OPEN_LOOP_START_RAD_S;
    s_motorCtrl.observerPhaseOffsetRad = s_motorCtrl.latestPhaseErrorRad;
    s_motorCtrl.observerLockResidualRad = 0.0f;
    s_motorCtrl.status.id_target_a = 0.0f;
    s_motorCtrl.status.iq_target_a = MotorControl_GetSignedDirection() *
                                     MOTOR_CFG_OPEN_LOOP_IQ_A;
    MotorControl_SetState(MOTOR_STATE_OPEN_LOOP_RAMP);
}

static void MotorControl_StartClosedLoop(void)
{
    s_motorCtrl.closedLoopBlend = 0.0f;
    s_motorCtrl.observerLossCount = 0U;
    s_motorCtrl.status.observer_locked = true;
    s_motorCtrl.foc.speed_pi_integrator_a = s_motorCtrl.status.iq_target_a;
    MotorControl_SetState(MOTOR_STATE_CLOSED_LOOP);
}

static void MotorControl_UpdateOpenLoopState(void)
{
    const float speedStepRadS = MOTOR_CFG_OPEN_LOOP_ACCEL_RAD_S2 * MOTOR_CFG_SPEED_LOOP_DT_S;
    const float phaseTrackBlend = MotorControl_Clamp(MOTOR_CFG_OBSERVER_PHASE_TRACK_BW_RAD_S *
                                                     MOTOR_CFG_SPEED_LOOP_DT_S,
                                                     0.0f,
                                                     1.0f);
    float openLoopSpeedMagRadS;

    s_motorCtrl.stateTimeMs++;
    s_motorCtrl.startupTimeMs++;

    openLoopSpeedMagRadS = MotorControl_Clamp((__builtin_fabsf(s_motorCtrl.openLoopSpeedRadS) + speedStepRadS),
                                              0.0f,
                                              MOTOR_CFG_OPEN_LOOP_HANDOVER_RAD_S);
    s_motorCtrl.openLoopSpeedRadS = MotorControl_GetSignedDirection() * openLoopSpeedMagRadS;
    s_motorCtrl.observerPhaseOffsetRad = MotorControl_BlendPhase(s_motorCtrl.observerPhaseOffsetRad,
                                                                 s_motorCtrl.latestPhaseErrorRad,
                                                                 phaseTrackBlend);
    s_motorCtrl.observerLockResidualRad = MotorControl_WrapAngleSigned(
        MotorFoc_AngleDiff(s_motorCtrl.latestPhaseErrorRad, s_motorCtrl.observerPhaseOffsetRad));
    s_motorCtrl.status.id_target_a = 0.0f;
    s_motorCtrl.status.iq_target_a = MotorControl_GetSignedDirection() * MOTOR_CFG_OPEN_LOOP_IQ_A;

    if ((__builtin_fabsf(s_motorCtrl.status.electrical_speed_rad_s) >= MOTOR_CFG_MIN_LOCK_ELEC_SPEED_RAD_S) &&
        (__builtin_fabsf(s_motorCtrl.observerLockResidualRad) <= MOTOR_CFG_OBSERVER_LOCK_PHASE_ERR_RAD))
    {
        s_motorCtrl.observerLockCount++;
    }
    else
    {
        s_motorCtrl.observerLockCount = 0U;
    }

    if (s_motorCtrl.observerLockCount >= MOTOR_CFG_OBSERVER_LOCK_COUNT)
    {
        MotorControl_StartClosedLoop();
        return;
    }

    if (s_motorCtrl.startupTimeMs >= MOTOR_CFG_STARTUP_TIMEOUT_MS)
    {
        MotorControl_LatchFault(MOTOR_FAULT_STARTUP_TIMEOUT);
    }
}

static void MotorControl_UpdateClosedLoopState(void)
{
    const float blendStep = (MOTOR_CFG_CLOSED_LOOP_BLEND_MS > 0U) ?
                            (MOTOR_CFG_SPEED_LOOP_DT_S * 1000.0f / (float)MOTOR_CFG_CLOSED_LOOP_BLEND_MS) :
                            1.0f;

    s_motorCtrl.stateTimeMs++;
    if (s_motorCtrl.closedLoopBlend < 1.0f)
    {
        s_motorCtrl.closedLoopBlend = MotorControl_Clamp(s_motorCtrl.closedLoopBlend + blendStep, 0.0f, 1.0f);
    }

    s_motorCtrl.status.id_target_a = 0.0f;
    s_motorCtrl.status.iq_target_a = MotorFoc_RunSpeedPi(&s_motorCtrl.foc,
                                                         MotorControl_GetTargetElectricalSpeedRadS(),
                                                         s_motorCtrl.status.electrical_speed_rad_s,
                                                         false);

    if (__builtin_fabsf(s_motorCtrl.latestPhaseErrorRad) > MOTOR_CFG_OBSERVER_LOSS_PHASE_ERR_RAD)
    {
        s_motorCtrl.observerLossCount++;
    }
    else
    {
        s_motorCtrl.observerLossCount = 0U;
    }

    if (s_motorCtrl.observerLossCount >= MOTOR_CFG_OBSERVER_LOSS_COUNT)
    {
        MotorControl_LatchFault(MOTOR_FAULT_OBSERVER_LOSS);
    }
}

static void MotorControl_HandleSlowLoop(void)
{
    s_motorCtrl.status.enabled = s_motorCtrl.enableRequest;
    s_motorCtrl.status.mechanical_rpm = MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(s_motorCtrl.status.electrical_speed_rad_s);

    if (!s_motorCtrl.enableRequest)
    {
        if (s_motorCtrl.status.state != MOTOR_STATE_STOP)
        {
            MotorControl_EnterStop();
        }
        return;
    }

    switch (s_motorCtrl.status.state)
    {
        case MOTOR_STATE_STOP:
            MotorControl_BeginOffsetCalibration();
            break;

        case MOTOR_STATE_OFFSET_CAL:
            break;

        case MOTOR_STATE_ALIGN:
            s_motorCtrl.stateTimeMs++;
            s_motorCtrl.startupTimeMs++;
            s_motorCtrl.status.id_target_a = MOTOR_CFG_ALIGN_CURRENT_A;
            s_motorCtrl.status.iq_target_a = 0.0f;

            if (s_motorCtrl.stateTimeMs >= MOTOR_CFG_ALIGN_TIME_MS)
            {
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

    MotorControl_UpdateDebugState(s_motorCtrl.status.electrical_angle_rad);
}

void MotorControl_Init(void)
{
    (void)memset(&s_motorCtrl, 0, sizeof(s_motorCtrl));
    (void)memset((void *)&g_motorDebug, 0, sizeof(g_motorDebug));

    s_motorCtrl.status.state = MOTOR_STATE_STOP;
    s_motorCtrl.status.fault = MOTOR_FAULT_NONE;
    s_motorCtrl.status.direction = MOTOR_CFG_DEFAULT_DIRECTION;
    s_motorCtrl.status.target_rpm = MOTOR_CFG_DEFAULT_TARGET_RPM;
    s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
    s_motorCtrl.enableRequest = (MOTOR_APP_AUTO_START != 0U);
    s_motorCtrl.status.enabled = s_motorCtrl.enableRequest;

    MotorFoc_Init(&s_motorCtrl.foc);
    MotorHwYtm32_Init();
    MotorHwYtm32_SetOutputsMasked(true);
    MotorHwYtm32_StartSpeedLoopTimer();
    MotorControl_UpdateDebugState(s_motorCtrl.status.electrical_angle_rad);
}

void MotorControl_Enable(bool enable)
{
    SDK_ENTER_CRITICAL();
    s_motorCtrl.enableRequest = enable;
    s_motorCtrl.status.enabled = enable;

    if (!enable)
    {
        MotorControl_EnterStop();
    }

    SDK_EXIT_CRITICAL();
}

bool MotorControl_SetTargetRpm(float targetRpm)
{
    SDK_ENTER_CRITICAL();
    s_motorCtrl.status.target_rpm = (targetRpm < 0.0f) ? -targetRpm : targetRpm;
    SDK_EXIT_CRITICAL();

    return true;
}

bool MotorControl_SetDirection(int8_t direction)
{
    bool accepted = false;

    SDK_ENTER_CRITICAL();
    if (((direction == 1) || (direction == -1)) &&
        (s_motorCtrl.status.state == MOTOR_STATE_STOP))
    {
        s_motorCtrl.status.direction = direction;
        s_motorCtrl.status.electrical_angle_rad = MotorControl_GetAlignAngle();
        accepted = true;
    }
    SDK_EXIT_CRITICAL();

    return accepted;
}

const motor_status_t *MotorControl_GetStatus(void)
{
    return &s_motorCtrl.status;
}

const volatile motor_debug_t *MotorControl_GetDebug(void)
{
    return &g_motorDebug;
}

void ADC0_IRQHandler(void)
{
    motor_adc_raw_frame_t rawFrame;
    motor_foc_fast_input_t focInput;
    motor_foc_fast_output_t focOutput;
    float phaseCurrentA;
    float phaseCurrentB;
    float phaseCurrentC;
    float busVoltageV;
    float controlAngleRad;

    if (!MotorHwYtm32_ReadTriggeredFrame(&rawFrame))
    {
        if (ADC_DRV_GetOvrRunOfConversionFlag(0U))
        {
            ADC_DRV_ClearOvrFlagCmd(0U);
            MotorControl_LatchFault(MOTOR_FAULT_ADC_OVERRUN);
        }
        return;
    }

    if (rawFrame.overrun)
    {
        MotorControl_LatchFault(MOTOR_FAULT_ADC_OVERRUN);
        return;
    }

    phaseCurrentA = MotorControl_CurrentFromRaw(rawFrame.current_a_raw, s_motorCtrl.currentOffsetARaw);
    phaseCurrentB = MotorControl_CurrentFromRaw(rawFrame.current_b_raw, s_motorCtrl.currentOffsetBRaw);
    phaseCurrentC = MotorControl_CurrentFromRaw(rawFrame.current_c_raw, s_motorCtrl.currentOffsetCRaw);
    busVoltageV = (float)rawFrame.vbus_raw * MOTOR_CFG_ADC_COUNT_TO_VBUS_V;

    s_motorCtrl.status.phase_current_a = phaseCurrentA;
    s_motorCtrl.status.phase_current_b = phaseCurrentB;
    s_motorCtrl.status.phase_current_c = phaseCurrentC;
    s_motorCtrl.status.bus_voltage_v = busVoltageV;

    if (s_motorCtrl.status.state == MOTOR_STATE_OFFSET_CAL)
    {
        s_motorCtrl.currentOffsetASum += (float)rawFrame.current_a_raw;
        s_motorCtrl.currentOffsetBSum += (float)rawFrame.current_b_raw;
        s_motorCtrl.currentOffsetCSum += (float)rawFrame.current_c_raw;
        s_motorCtrl.offsetSampleCount++;

        if (s_motorCtrl.offsetSampleCount >= MOTOR_CFG_OFFSET_CAL_SAMPLES)
        {
            const float sampleCount = (float)s_motorCtrl.offsetSampleCount;

            s_motorCtrl.currentOffsetARaw = s_motorCtrl.currentOffsetASum / sampleCount;
            s_motorCtrl.currentOffsetBRaw = s_motorCtrl.currentOffsetBSum / sampleCount;
            s_motorCtrl.currentOffsetCRaw = s_motorCtrl.currentOffsetCSum / sampleCount;
            MotorControl_StartAlign();
        }
        return;
    }

    if ((s_motorCtrl.status.state == MOTOR_STATE_STOP) ||
        (s_motorCtrl.status.state == MOTOR_STATE_FAULT))
    {
        return;
    }

    if (__builtin_fabsf(phaseCurrentA) > MOTOR_CFG_PHASE_OVERCURRENT_A ||
        __builtin_fabsf(phaseCurrentB) > MOTOR_CFG_PHASE_OVERCURRENT_A ||
        __builtin_fabsf(phaseCurrentC) > MOTOR_CFG_PHASE_OVERCURRENT_A)
    {
        MotorControl_LatchFault(MOTOR_FAULT_OVERCURRENT);
        return;
    }

    if (busVoltageV < MOTOR_CFG_VBUS_UNDERVOLTAGE_V)
    {
        MotorControl_LatchFault(MOTOR_FAULT_VBUS_UNDERVOLTAGE);
        return;
    }

    if (busVoltageV > MOTOR_CFG_VBUS_OVERVOLTAGE_V)
    {
        MotorControl_LatchFault(MOTOR_FAULT_VBUS_OVERVOLTAGE);
        return;
    }

    if (s_motorCtrl.status.state == MOTOR_STATE_ALIGN)
    {
        controlAngleRad = MotorControl_GetAlignAngle();
    }
    else
    {
        s_motorCtrl.openLoopAngleRad = MotorFoc_WrapAngle0ToTwoPi(s_motorCtrl.openLoopAngleRad +
                                                                  (s_motorCtrl.openLoopSpeedRadS * MOTOR_CFG_FAST_LOOP_DT_S));

        if (s_motorCtrl.status.state == MOTOR_STATE_OPEN_LOOP_RAMP)
        {
            controlAngleRad = s_motorCtrl.openLoopAngleRad;
        }
        else
        {
            controlAngleRad = MotorControl_BlendAngle(s_motorCtrl.openLoopAngleRad,
                                                      s_motorCtrl.latestObserverAngleRad,
                                                      s_motorCtrl.closedLoopBlend);
        }
    }

    focInput.phase_current_a = phaseCurrentA;
    focInput.phase_current_b = phaseCurrentB;
    focInput.phase_current_c = phaseCurrentC;
    focInput.bus_voltage_v = busVoltageV;
    focInput.control_angle_rad = controlAngleRad;
    focInput.id_target_a = s_motorCtrl.status.id_target_a;
    focInput.iq_target_a = s_motorCtrl.status.iq_target_a;

    MotorFoc_RunFast(&s_motorCtrl.foc, &focInput, &focOutput);

    s_motorCtrl.status.id_a = focOutput.id_a;
    s_motorCtrl.status.iq_a = focOutput.iq_a;
    s_motorCtrl.status.electrical_speed_rad_s = focOutput.observer_speed_rad_s;
    s_motorCtrl.status.mechanical_rpm = MOTOR_CFG_ELEC_RAD_S_TO_MECH_RPM(focOutput.observer_speed_rad_s);
    s_motorCtrl.latestPhaseErrorRad = focOutput.phase_error_rad;
    s_motorCtrl.latestObserverAngleRad = focOutput.observer_angle_rad;
    s_motorCtrl.status.electrical_angle_rad =
        (s_motorCtrl.status.state == MOTOR_STATE_CLOSED_LOOP) ?
        focOutput.observer_angle_rad : controlAngleRad;
    MotorControl_UpdateDebugState(controlAngleRad);

    if (!s_motorCtrl.outputsUnmasked)
    {
        MotorHwYtm32_SetOutputsMasked(false);
        s_motorCtrl.outputsUnmasked = true;
    }

    MotorHwYtm32_ApplyPhaseDuty(focOutput.duty_u, focOutput.duty_v, focOutput.duty_w);
}

void pTMR0_Ch0_IRQHandler(void)
{
    MotorHwYtm32_ClearSpeedLoopIrq();
    MotorControl_HandleSlowLoop();
}
