/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * @file can_config.c
 * @brief CAN FD motor control telemetry / command / calibration implementation.
 *
 * Implements FlexCAN FD initialization, periodic TX of motor status,
 * and RX processing of commands and calibration writes.
 *
 * DBC reference: docs/motor_control.dbc
 */

#include "can_config.h"

#include <string.h>

#include "motor_control.h"
#include "motor_foc.h"
#include "motor_user_config.h"
#include "sdk_project_config.h"

/* ═══════════════════════════════════════════════════════════════════════════════
 * Internal State
 * ═══════════════════════════════════════════════════════════════════════════════ */

/** @brief FlexCAN driver state. */
static flexcan_state_t s_canState;

/** @brief TX data buffer (64 bytes). */
static uint8_t s_txBuf[CAN_MSG_DLC];

/** @brief RX message buffer structures. */
static flexcan_msgbuff_t s_rxMbCommand;
static flexcan_msgbuff_t s_rxMbCalibWrite;

/** @brief TX scheduling counters. */
static uint32_t s_lastTxStatus1Ms;
static uint32_t s_lastTxStatus2Ms;
static uint32_t s_lastTxCalibMs;

/** @brief Temperature readings from external sensors. */
static can_temperature_t s_temperature;

/** @brief Runtime calibration parameters (RAM). */
static can_calib_params_t s_calibParams;

/** @brief Flag: RX command message received and pending processing. */
static volatile bool s_rxCommandPending;
static volatile bool s_rxCalibPending;

/* ═══════════════════════════════════════════════════════════════════════════════
 * Signal Packing Helpers (Little-Endian, match DBC)
 * ═══════════════════════════════════════════════════════════════════════════════ */

static inline void pack_u8(uint8_t *buf, uint32_t bytePos, uint8_t val)
{
    buf[bytePos] = val;
}

static inline void pack_s8(uint8_t *buf, uint32_t bytePos, int8_t val)
{
    buf[bytePos] = (uint8_t)val;
}

static inline void pack_u16(uint8_t *buf, uint32_t bytePos, uint16_t val)
{
    buf[bytePos]     = (uint8_t)(val & 0xFFU);
    buf[bytePos + 1] = (uint8_t)((val >> 8) & 0xFFU);
}

static inline void pack_s16(uint8_t *buf, uint32_t bytePos, int16_t val)
{
    pack_u16(buf, bytePos, (uint16_t)val);
}

static inline void pack_u32(uint8_t *buf, uint32_t bytePos, uint32_t val)
{
    buf[bytePos]     = (uint8_t)(val & 0xFFU);
    buf[bytePos + 1] = (uint8_t)((val >> 8) & 0xFFU);
    buf[bytePos + 2] = (uint8_t)((val >> 16) & 0xFFU);
    buf[bytePos + 3] = (uint8_t)((val >> 24) & 0xFFU);
}

static inline void pack_f32(uint8_t *buf, uint32_t bytePos, float val)
{
    uint32_t raw;
    memcpy(&raw, &val, sizeof(raw));
    pack_u32(buf, bytePos, raw);
}

static inline uint8_t unpack_u8(const uint8_t *buf, uint32_t bytePos)
{
    return buf[bytePos];
}

static inline int8_t unpack_s8(const uint8_t *buf, uint32_t bytePos)
{
    return (int8_t)buf[bytePos];
}

static inline uint16_t unpack_u16(const uint8_t *buf, uint32_t bytePos)
{
    return (uint16_t)buf[bytePos] | ((uint16_t)buf[bytePos + 1] << 8);
}

static inline int16_t unpack_s16(const uint8_t *buf, uint32_t bytePos)
{
    return (int16_t)unpack_u16(buf, bytePos);
}

static inline uint32_t unpack_u32(const uint8_t *buf, uint32_t bytePos)
{
    return (uint32_t)buf[bytePos]
         | ((uint32_t)buf[bytePos + 1] << 8)
         | ((uint32_t)buf[bytePos + 2] << 16)
         | ((uint32_t)buf[bytePos + 3] << 24);
}

static inline float unpack_f32(const uint8_t *buf, uint32_t bytePos)
{
    float val;
    uint32_t raw = unpack_u32(buf, bytePos);
    memcpy(&val, &raw, sizeof(val));
    return val;
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * FlexCAN Callback
 * ═══════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief FlexCAN IRQ callback for TX complete and RX events.
 */
static void CanConfig_Callback(uint8_t instance, flexcan_event_type_t eventType,
                               uint32_t buffIdx, flexcan_state_t *state)
{
    (void)instance;
    (void)state;

    if (eventType == FLEXCAN_EVENT_RX_COMPLETE)
    {
        if (buffIdx == CAN_MB_RX_COMMAND)
        {
            s_rxCommandPending = true;
            /* Re-arm RX MB for next message */
            FLEXCAN_DRV_Receive(CAN_CFG_INSTANCE, CAN_MB_RX_COMMAND,
                                &s_rxMbCommand);
        }
        else if (buffIdx == CAN_MB_RX_CALIB_WRITE)
        {
            s_rxCalibPending = true;
            FLEXCAN_DRV_Receive(CAN_CFG_INSTANCE, CAN_MB_RX_CALIB_WRITE,
                                &s_rxMbCalibWrite);
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * TX Frame Builders
 * ═══════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Pack and send MCU_MotorStatus1 (0x180).
 *
 * Signal layout matches DBC byte positions (little-endian, start bit / 8).
 */
static void CanConfig_SendStatus1(void)
{
    const motor_status_t *st = MotorControl_GetStatus();
    const volatile motor_fast_loop_profile_t *prof = MotorControl_GetFastLoopProfile();
    (void)prof;  /* Reserved for future use */

    memset(s_txBuf, 0, CAN_MSG_DLC);

    /* Byte 0: State */
    pack_u8(s_txBuf, 0, (uint8_t)st->state);
    /* Byte 1: FaultCode */
    pack_u8(s_txBuf, 1, (uint8_t)st->fault);
    /* Byte 2: ControlMode */
    pack_u8(s_txBuf, 2, (uint8_t)st->control_mode);
    /* Byte 3: Flags — Bit0=enabled, Bit1=observer_locked, Bit2=direction(1=fwd) */
    {
        uint8_t flags = 0U;
        if (st->enabled)         flags |= 0x01U;
        if (st->observer_locked) flags |= 0x02U;
        if (st->direction >= 0)  flags |= 0x04U;
        pack_u8(s_txBuf, 3, flags);
    }
    /* Bytes 4-5: MechanicalRpm, scale=0.5 */
    pack_s16(s_txBuf, 4, (int16_t)(st->mechanical_rpm / 0.5f));
    /* Bytes 6-7: BusVoltage, scale=0.01 */
    pack_u16(s_txBuf, 6, (uint16_t)(st->bus_voltage_v / 0.01f));
    /* Bytes 8-9: IdMeas, scale=0.01 */
    pack_s16(s_txBuf, 8, (int16_t)(st->id_a / 0.01f));
    /* Bytes 10-11: IqMeas, scale=0.01 */
    pack_s16(s_txBuf, 10, (int16_t)(st->iq_a / 0.01f));
    /* Bytes 12-13: IdTarget, scale=0.01 */
    pack_s16(s_txBuf, 12, (int16_t)(st->id_target_a / 0.01f));
    /* Bytes 14-15: IqTarget, scale=0.01 */
    pack_s16(s_txBuf, 14, (int16_t)(st->iq_target_a / 0.01f));
    /* Bytes 16-17: TargetRpm, scale=0.5 (signed: apply direction to match MechanicalRpm) */
    {
        float signedTargetRpm = st->target_rpm * (float)st->direction;
        pack_s16(s_txBuf, 16, (int16_t)(signedTargetRpm / 0.5f));
    }
    /* Bytes 18-19: ElecAngle, scale=0.0001 */
    pack_u16(s_txBuf, 18, (uint16_t)(st->electrical_angle_rad / 0.0001f));
    /* Bytes 20-21: ElecSpeedRadS, scale=0.1 */
    pack_s16(s_txBuf, 20, (int16_t)(st->electrical_speed_rad_s / 0.1f));
    /* Bytes 22-23: PhaseCurrentA, scale=0.01 */
    pack_s16(s_txBuf, 22, (int16_t)(st->phase_current_a / 0.01f));
    /* Bytes 24-25: PhaseCurrentB, scale=0.01 */
    pack_s16(s_txBuf, 24, (int16_t)(st->phase_current_b / 0.01f));
    /* Bytes 26-27: PhaseCurrentC, scale=0.01 */
    pack_s16(s_txBuf, 26, (int16_t)(st->phase_current_c / 0.01f));
    /* Byte 28: StartupRetryCount */
    pack_u8(s_txBuf, 28, st->startup_retry_count);

    /* Bytes 29-30: DutyU, scale=0.0001 — requires FOC output access */
    /* Bytes 31-32: DutyV, scale=0.0001 */
    /* Bytes 33-34: DutyW, scale=0.0001 */
    /* NOTE: PWM duty is available from the last FOC output; we read it via
     * external accessor if available, otherwise leave as 0 for now.
     * These will be populated when the FOC output accessor is added. */
    pack_u16(s_txBuf, 29, 0U);  /* DutyU placeholder */
    pack_u16(s_txBuf, 31, 0U);  /* DutyV placeholder */
    pack_u16(s_txBuf, 33, 0U);  /* DutyW placeholder */

    /* Bytes 35-38: Timestamp */
    pack_u32(s_txBuf, 35, MotorControl_GetTickMs());

    /* Send CAN FD frame */
    static const flexcan_data_info_t txInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_MSG_DLC,
        .fd_enable   = true,
        .fd_padding  = 0x00U,
        .enable_brs  = true,
        .is_remote   = false,
    };
    FLEXCAN_DRV_Send(CAN_CFG_INSTANCE, CAN_MB_TX_STATUS1,
                     &txInfo, CAN_MSG_ID_MOTOR_STATUS1, s_txBuf);
}

/**
 * @brief Pack and send MCU_MotorStatus2 (0x181).
 */
static void CanConfig_SendStatus2(void)
{
    motor_foc_diagnostics_t diag;
    MotorControl_GetFocDiagnostics(&diag);

    memset(s_txBuf, 0, CAN_MSG_DLC);

    /* Bytes 0-1: ObserverAngle, scale=0.0001 */
    pack_u16(s_txBuf, 0, (uint16_t)(diag.observer_angle_rad / 0.0001f));
    /* Bytes 2-3: ObserverFluxVs, scale=0.000001 */
    pack_u16(s_txBuf, 2, (uint16_t)(diag.observer_flux_vs / 0.000001f));
    /* Bytes 4-5: PLLSpeedRadS, scale=0.1 */
    pack_s16(s_txBuf, 4, (int16_t)(diag.pll_speed_rad_s / 0.1f));
    /* Bytes 6-7: PhaseErrorRad, scale=0.0001 */
    pack_s16(s_txBuf, 6, (int16_t)(diag.phase_error_rad / 0.0001f));
    /* Bytes 8-9: SpeedPiIntegrator, scale=0.01 */
    pack_s16(s_txBuf, 8, (int16_t)(diag.speed_pi_integrator_a / 0.01f));
    /* Bytes 10-11: IdPiIntegrator, scale=0.01 */
    pack_s16(s_txBuf, 10, (int16_t)(diag.id_pi_integrator_v / 0.01f));
    /* Bytes 12-13: IqPiIntegrator, scale=0.01 */
    pack_s16(s_txBuf, 12, (int16_t)(diag.iq_pi_integrator_v / 0.01f));
    /* Bytes 14-15: VoltModRatio, scale=0.0001 */
    pack_u16(s_txBuf, 14, (uint16_t)(diag.voltage_modulation_ratio / 0.0001f));
    /* Bytes 16-17: FwIdTarget, scale=0.01 */
    pack_s16(s_txBuf, 16, (int16_t)(diag.fw_id_target_a / 0.01f));
    /* Bytes 18-19: OpenLoopAngle, scale=0.0001 */
    pack_u16(s_txBuf, 18, (uint16_t)(diag.open_loop_angle_rad / 0.0001f));
    /* Bytes 20-21: OpenLoopSpeedRadS, scale=0.1 */
    pack_s16(s_txBuf, 20, (int16_t)(diag.open_loop_speed_rad_s / 0.1f));
    /* Bytes 22-23: ClosedLoopBlend, scale=0.0001 */
    pack_u16(s_txBuf, 22, (uint16_t)(diag.closed_loop_blend / 0.0001f));
    /* Bytes 24-27: StateTimeMs */
    pack_u32(s_txBuf, 24, diag.state_time_ms);
    /* Bytes 28-29: ObsLockResidualRad, scale=0.0001 */
    pack_s16(s_txBuf, 28, (int16_t)(diag.obs_lock_residual_rad / 0.0001f));
    /* Bytes 30-31: StallDivCount */
    pack_u16(s_txBuf, 30, diag.stall_div_count);
    /* Bytes 32-33: PcbTemperature, scale=0.1 */
    pack_s16(s_txBuf, 32, (int16_t)(s_temperature.pcb_temperature_degc / 0.1f));
    /* Bytes 34-35: ChipTemperature, scale=0.1 */
    pack_s16(s_txBuf, 34, (int16_t)(s_temperature.chip_temperature_degc / 0.1f));

    static const flexcan_data_info_t txInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_MSG_DLC,
        .fd_enable   = true,
        .fd_padding  = 0x00U,
        .enable_brs  = true,
        .is_remote   = false,
    };
    FLEXCAN_DRV_Send(CAN_CFG_INSTANCE, CAN_MB_TX_STATUS2,
                     &txInfo, CAN_MSG_ID_MOTOR_STATUS2, s_txBuf);
}

/**
 * @brief Pack and send MCU_CalibReadback (0x182).
 */
static void CanConfig_SendCalibReadback(void)
{
    memset(s_txBuf, 0, CAN_MSG_DLC);

    /* Bytes 0-3:   ActiveSpeedKp (float32) */
    pack_f32(s_txBuf,  0, s_calibParams.speed_kp);
    /* Bytes 4-7:   ActiveSpeedKi (float32) */
    pack_f32(s_txBuf,  4, s_calibParams.speed_ki);
    /* Bytes 8-11:  ActiveObsGain (float32) */
    pack_f32(s_txBuf,  8, s_calibParams.observer_gain);
    /* Bytes 12-15: ActivePllKp (float32) */
    pack_f32(s_txBuf, 12, s_calibParams.pll_kp);
    /* Bytes 16-19: ActivePllKi (float32) */
    pack_f32(s_txBuf, 16, s_calibParams.pll_ki);
    /* Bytes 20-21: ActiveMaxIqA, scale=0.01 */
    pack_u16(s_txBuf, 20, (uint16_t)(s_calibParams.max_iq_a / 0.01f));
    /* Bytes 22-23: ActiveOpenLoopIqA, scale=0.01 */
    pack_u16(s_txBuf, 22, (uint16_t)(s_calibParams.open_loop_iq_a / 0.01f));
    /* Bytes 24-25: ActiveAlignCurrentA, scale=0.01 */
    pack_u16(s_txBuf, 24, (uint16_t)(s_calibParams.align_current_a / 0.01f));
    /* Bytes 26-27: ActiveFwThreshold, scale=0.0001 */
    pack_u16(s_txBuf, 26, (uint16_t)(s_calibParams.fw_voltage_threshold / 0.0001f));
    /* Bytes 28-29: ActiveRpmRamp, scale=1 */
    pack_u16(s_txBuf, 28, (uint16_t)s_calibParams.speed_ramp_rpm_per_s);

    static const flexcan_data_info_t txInfo = {
        .msg_id_type = FLEXCAN_MSG_ID_STD,
        .data_length = CAN_MSG_DLC,
        .fd_enable   = true,
        .fd_padding  = 0x00U,
        .enable_brs  = true,
        .is_remote   = false,
    };
    FLEXCAN_DRV_Send(CAN_CFG_INSTANCE, CAN_MB_TX_CALIB_READBACK,
                     &txInfo, CAN_MSG_ID_CALIB_READBACK, s_txBuf);
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * RX Processing
 * ═══════════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Process a received PC_MotorCommand (0x200).
 */
static void CanConfig_ProcessCommand(void)
{
    const uint8_t *data = s_rxMbCommand.data;

    uint8_t cmdEnable      = unpack_u8(data, 0);
    uint8_t cmdControlMode = unpack_u8(data, 1);
    int8_t  cmdDirection   = unpack_s8(data, 2);
    int16_t cmdTargetRpm   = unpack_s16(data, 3);
    int16_t cmdTargetIqA   = unpack_s16(data, 5);
    uint16_t cmdRpmRamp    = unpack_u16(data, 7);

    /* Apply speed ramp override if non-zero */
    if (cmdRpmRamp > 0U)
    {
        s_calibParams.speed_ramp_rpm_per_s = (float)cmdRpmRamp;
    }

    /* Apply direction change (only accepted in STOP state) */
    if (cmdDirection == 1 || cmdDirection == -1)
    {
        MotorControl_SetDirection(cmdDirection);
    }

    /* Apply control mode */
    MotorControl_SetControlMode((motor_control_mode_t)cmdControlMode);

    /* Apply targets */
    float targetRpm = (float)cmdTargetRpm * 0.5f;
    float targetIqA = (float)cmdTargetIqA * 0.01f;
    MotorControl_SetTargetRpm(targetRpm);
    MotorControl_SetTargetIqA(targetIqA);

    /* Apply enable/disable — do this last so targets are set before run */
    MotorControl_Enable(cmdEnable != 0U);
}

/**
 * @brief Process a received PC_CalibWrite (0x201).
 */
static void CanConfig_ProcessCalibWrite(void)
{
    const uint8_t *data = s_rxMbCalibWrite.data;

    /* uint8_t selector = unpack_u8(data, 0); — reserved for future group select */

    s_calibParams.speed_kp            = unpack_f32(data,  1);
    s_calibParams.speed_ki            = unpack_f32(data,  5);
    s_calibParams.observer_gain       = unpack_f32(data,  9);
    s_calibParams.pll_kp              = unpack_f32(data, 13);
    s_calibParams.pll_ki              = unpack_f32(data, 17);
    s_calibParams.max_iq_a            = (float)unpack_u16(data, 21) * 0.01f;
    s_calibParams.open_loop_iq_a      = (float)unpack_u16(data, 23) * 0.01f;
    s_calibParams.align_current_a     = (float)unpack_u16(data, 25) * 0.01f;
    s_calibParams.fw_voltage_threshold = (float)unpack_u16(data, 27) * 0.0001f;

    uint8_t apply = unpack_u8(data, 29);

    s_calibParams.pending = true;

    if (apply == 0xFFU)
    {
        s_calibParams.committed = true;
    }
}

/* ═══════════════════════════════════════════════════════════════════════════════
 * Public API Implementation
 * ═══════════════════════════════════════════════════════════════════════════════ */

void CanConfig_Init(void)
{
    /* ── Initialize calibration parameters from compile-time defaults ── */
    s_calibParams.speed_kp             = MOTOR_CFG_SPEED_KP;
    s_calibParams.speed_ki             = MOTOR_CFG_SPEED_KI;
    s_calibParams.observer_gain        = MOTOR_CFG_OBSERVER_GAIN;
    s_calibParams.pll_kp               = MOTOR_CFG_PLL_KP;
    s_calibParams.pll_ki               = MOTOR_CFG_PLL_KI;
    s_calibParams.max_iq_a             = MOTOR_CFG_MAX_IQ_A;
    s_calibParams.open_loop_iq_a       = MOTOR_CFG_OPEN_LOOP_IQ_A;
    s_calibParams.align_current_a      = MOTOR_CFG_ALIGN_CURRENT_A;
    s_calibParams.fw_voltage_threshold = MOTOR_CFG_FW_VOLTAGE_THRESHOLD;
    s_calibParams.speed_ramp_rpm_per_s = MOTOR_CFG_SPEED_RAMP_RPM_PER_S;
    s_calibParams.pending              = false;
    s_calibParams.committed            = false;

    s_temperature.pcb_temperature_degc  = 0.0f;
    s_temperature.chip_temperature_degc = 0.0f;

    s_rxCommandPending = false;
    s_rxCalibPending   = false;

    /* ── Wake CAN transceiver (STB low = active) ── */
    PINS_DRV_WritePin(CAN_CFG_STB_PORT, CAN_CFG_STB_PIN, 0U);

    /* ── FlexCAN FD configuration ── */
    flexcan_user_config_t canConfig;
    (void)FLEXCAN_DRV_GetDefaultConfig(&canConfig);

    canConfig.max_num_mb      = 8U;
    canConfig.flexcanMode     = FLEXCAN_NORMAL_MODE;
    canConfig.fd_enable       = true;
    canConfig.payload         = FLEXCAN_PAYLOAD_SIZE_64;
    canConfig.pe_clock        = FLEXCAN_CLK_SOURCE_PERIPH;
    canConfig.is_rx_fifo_needed = false;
    canConfig.transfer_type   = FLEXCAN_RXFIFO_USING_INTERRUPTS;

    /* Arbitration phase: 500 kbit/s */
    canConfig.bitrate.preDivider = CAN_CFG_ARB_PREDIV;
    canConfig.bitrate.propSeg    = CAN_CFG_ARB_PROPSEG;
    canConfig.bitrate.phaseSeg1  = CAN_CFG_ARB_PSEG1;
    canConfig.bitrate.phaseSeg2  = CAN_CFG_ARB_PSEG2;
    canConfig.bitrate.rJumpwidth = CAN_CFG_ARB_RJUMP;

    /* Data phase: 2 Mbit/s */
    canConfig.bitrate_cbt.preDivider = CAN_CFG_DATA_PREDIV;
    canConfig.bitrate_cbt.propSeg    = CAN_CFG_DATA_PROPSEG;
    canConfig.bitrate_cbt.phaseSeg1  = CAN_CFG_DATA_PSEG1;
    canConfig.bitrate_cbt.phaseSeg2  = CAN_CFG_DATA_PSEG2;
    canConfig.bitrate_cbt.rJumpwidth = CAN_CFG_DATA_RJUMP;

    (void)FLEXCAN_DRV_Init(CAN_CFG_INSTANCE, &s_canState, &canConfig);

    /* ── Enable TDC for data phase ── */
    /* TDC_offset = propSeg + phaseSeg1 + pipeline ≈ 6+7+1 = 14 Tq (see can_config.h) */
    FLEXCAN_DRV_SetTDCOffset(CAN_CFG_INSTANCE, true, CAN_CFG_TDC_OFFSET);

    /* ── Install callback ── */
    FLEXCAN_DRV_InstallEventCallback(CAN_CFG_INSTANCE, CanConfig_Callback, NULL);

    /* ── Configure TX message buffers ── */
    {
        static const flexcan_data_info_t txInfo = {
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .data_length = CAN_MSG_DLC,
            .fd_enable   = true,
            .fd_padding  = 0x00U,
            .enable_brs  = true,
            .is_remote   = false,
        };
        FLEXCAN_DRV_ConfigTxMb(CAN_CFG_INSTANCE, CAN_MB_TX_STATUS1,
                               &txInfo, CAN_MSG_ID_MOTOR_STATUS1);
        FLEXCAN_DRV_ConfigTxMb(CAN_CFG_INSTANCE, CAN_MB_TX_STATUS2,
                               &txInfo, CAN_MSG_ID_MOTOR_STATUS2);
        FLEXCAN_DRV_ConfigTxMb(CAN_CFG_INSTANCE, CAN_MB_TX_CALIB_READBACK,
                               &txInfo, CAN_MSG_ID_CALIB_READBACK);
    }

    /* ── Configure RX message buffers ── */
    {
        static const flexcan_data_info_t rxInfo = {
            .msg_id_type = FLEXCAN_MSG_ID_STD,
            .data_length = CAN_MSG_DLC,
            .fd_enable   = true,
            .fd_padding  = 0x00U,
            .enable_brs  = true,
            .is_remote   = false,
        };
        FLEXCAN_DRV_ConfigRxMb(CAN_CFG_INSTANCE, CAN_MB_RX_COMMAND,
                               &rxInfo, CAN_MSG_ID_MOTOR_COMMAND);
        FLEXCAN_DRV_ConfigRxMb(CAN_CFG_INSTANCE, CAN_MB_RX_CALIB_WRITE,
                               &rxInfo, CAN_MSG_ID_CALIB_WRITE);
    }

    /* ── Set individual masks for exact ID match ── */
    FLEXCAN_DRV_SetRxMaskType(CAN_CFG_INSTANCE, FLEXCAN_RX_MASK_INDIVIDUAL);
    FLEXCAN_DRV_SetRxIndividualMask(CAN_CFG_INSTANCE, FLEXCAN_MSG_ID_STD,
                                    CAN_MB_RX_COMMAND, 0x7FFU);
    FLEXCAN_DRV_SetRxIndividualMask(CAN_CFG_INSTANCE, FLEXCAN_MSG_ID_STD,
                                    CAN_MB_RX_CALIB_WRITE, 0x7FFU);

    /* ── Arm RX message buffers ── */
    FLEXCAN_DRV_Receive(CAN_CFG_INSTANCE, CAN_MB_RX_COMMAND, &s_rxMbCommand);
    FLEXCAN_DRV_Receive(CAN_CFG_INSTANCE, CAN_MB_RX_CALIB_WRITE, &s_rxMbCalibWrite);

    /* ── Initialize TX timers ── */
    s_lastTxStatus1Ms = 0U;
    s_lastTxStatus2Ms = 0U;
    s_lastTxCalibMs   = 0U;
}

void CanConfig_Task(uint32_t tickMs)
{
    /* ── Process received commands ── */
    if (s_rxCommandPending)
    {
        s_rxCommandPending = false;
        CanConfig_ProcessCommand();
    }

    if (s_rxCalibPending)
    {
        s_rxCalibPending = false;
        CanConfig_ProcessCalibWrite();
    }

    /* ── Periodic TX scheduling ── */
    if ((tickMs - s_lastTxStatus1Ms) >= CAN_TX_PERIOD_STATUS1_MS)
    {
        s_lastTxStatus1Ms = tickMs;
        CanConfig_SendStatus1();
    }

    if ((tickMs - s_lastTxStatus2Ms) >= CAN_TX_PERIOD_STATUS2_MS)
    {
        s_lastTxStatus2Ms = tickMs;
        CanConfig_SendStatus2();
    }

    if ((tickMs - s_lastTxCalibMs) >= CAN_TX_PERIOD_CALIB_MS)
    {
        s_lastTxCalibMs = tickMs;
        CanConfig_SendCalibReadback();
    }
}

void CanConfig_SetTemperatures(float pcbTemp, float chipTemp)
{
    s_temperature.pcb_temperature_degc  = pcbTemp;
    s_temperature.chip_temperature_degc = chipTemp;
}

const can_calib_params_t *CanConfig_GetCalibParams(void)
{
    return &s_calibParams;
}

bool CanConfig_IsCalibCommitted(void)
{
    return s_calibParams.committed;
}

void CanConfig_AckCalibApplied(void)
{
    s_calibParams.committed = false;
    s_calibParams.pending   = false;
}
