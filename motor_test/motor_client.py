#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Control High-Level API
==============================
Provides a clean API for controlling the motor via CAN commands
and reading telemetry for test automation.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional

from can_interface import CanInterface


class MotorState(IntEnum):
    STOP = 0
    OFFSET_CAL = 1
    WIND_DETECT = 2
    COAST_DOWN = 3
    ALIGN = 4
    OPEN_LOOP_RAMP = 5
    CLOSED_LOOP = 6
    PARAM_IDENT = 7
    FAULT = 8
    ANGLE_MONITOR = 9
    HFI_IPD = 10


class MotorFault(IntEnum):
    NONE = 0
    ADC_OVERRUN = 1
    OVERCURRENT = 2
    VBUS_UNDERVOLTAGE = 3
    VBUS_OVERVOLTAGE = 4
    OBSERVER_LOSS = 5
    STARTUP_TIMEOUT = 6
    BAD_DIRECTION = 7
    CATCH_FAIL = 8
    STARTUP_FAIL = 9


class ControlMode(IntEnum):
    SPEED = 0
    CURRENT = 1


@dataclass
class MotorTelemetry:
    """Snapshot of motor telemetry at a point in time."""
    timestamp: float
    state: MotorState
    fault: MotorFault
    control_mode: ControlMode
    enabled: bool
    observer_locked: bool
    direction: int
    mechanical_rpm: float
    target_rpm: float
    bus_voltage_v: float
    iq_meas_a: float
    id_meas_a: float
    iq_target_a: float
    id_target_a: float
    elec_speed_rad_s: float
    elec_angle_rad: float
    phase_current_a: float
    phase_current_b: float
    phase_current_c: float
    observer_angle_rad: float
    observer_flux_vs: float
    pll_speed_rad_s: float
    phase_error_rad: float
    speed_pi_integrator: float
    volt_mod_ratio: float
    pcb_temperature: float
    chip_temperature: float
    startup_retry_count: int
    hfi_active: bool
    hfi_angle_valid: bool
    hfi_used: bool
    hfi_fallback_reason: int
    hfi_angle_rad: float
    hfi_confidence: float
    hfi_ripple_metric: float
    state_time_ms: int
    stall_div_count: int
    angle_mon_active: bool
    angle_mon_valid: bool
    angle_mon_angle_rad: float
    angle_mon_speed_rad_s: float
    angle_mon_bemf_mag: int


class MotorClient:
    """
    High-level motor control API over CAN FD.

    Encapsulates command encoding, telemetry decoding, and common
    test operations like start/stop, speed changes, and calibration
    parameter writes.
    """

    # Safety limits
    MAX_SAFE_RPM = 8500.0
    MAX_SAFE_IQ_A = 16.0
    MAX_SAFE_VBUS_V = 18.0    # Overvoltage threshold
    MIN_SAFE_VBUS_V = 7.0     # Undervoltage threshold

    def __init__(self, can: CanInterface):
        self.can = can

    # ── Motor Commands ──────────────────────────────────────────────────────

    def start(
        self,
        target_rpm: float = 1000.0,
        mode: ControlMode = ControlMode.SPEED,
        direction: int = 1,
        ramp_rpm_s: float = 600.0,
    ) -> None:
        """Send start command with specified parameters."""
        target_rpm = min(abs(target_rpm), self.MAX_SAFE_RPM)
        self.can.send_message("PC_MotorCommand", {
            "CmdEnable": 1,
            "CmdControlMode": int(mode),
            "CmdDirection": direction,
            "CmdTargetRpm": target_rpm,
            "CmdTargetIqA": 0.0,
            "CmdRpmRamp": int(ramp_rpm_s),
            "CmdReserved": 0,
        })

    def stop(self) -> None:
        """Send stop command."""
        self.can.send_message("PC_MotorCommand", {
            "CmdEnable": 0,
            "CmdControlMode": 0,
            "CmdDirection": 1,
            "CmdTargetRpm": 0.0,
            "CmdTargetIqA": 0.0,
            "CmdRpmRamp": 600,
            "CmdReserved": 0,
        })

    def set_speed(self, rpm: float, ramp_rpm_s: float = 600.0) -> None:
        """Change speed target while running."""
        rpm = min(abs(rpm), self.MAX_SAFE_RPM)
        self.can.send_message("PC_MotorCommand", {
            "CmdEnable": 1,
            "CmdControlMode": int(ControlMode.SPEED),
            "CmdDirection": 1 if rpm >= 0 else -1,
            "CmdTargetRpm": rpm,
            "CmdTargetIqA": 0.0,
            "CmdRpmRamp": int(ramp_rpm_s),
            "CmdReserved": 0,
        })

    def set_current(self, iq_a: float) -> None:
        """Set direct current control mode with specified Iq."""
        iq_a = max(-self.MAX_SAFE_IQ_A, min(iq_a, self.MAX_SAFE_IQ_A))
        self.can.send_message("PC_MotorCommand", {
            "CmdEnable": 1,
            "CmdControlMode": int(ControlMode.CURRENT),
            "CmdDirection": 1,
            "CmdTargetRpm": 0.0,
            "CmdTargetIqA": iq_a,
            "CmdRpmRamp": 600,
            "CmdReserved": 0,
        })

    def set_direction(self, direction: int) -> None:
        """Set rotation direction (+1=FWD, -1=REV). Motor must be stopped."""
        self.can.send_message("PC_MotorCommand", {
            "CmdEnable": 0,
            "CmdControlMode": 0,
            "CmdDirection": direction,
            "CmdTargetRpm": 0.0,
            "CmdTargetIqA": 0.0,
            "CmdRpmRamp": 600,
            "CmdReserved": 0,
        })

    # ── Calibration ─────────────────────────────────────────────────────────

    def write_calib(
        self,
        speed_kp: Optional[float] = None,
        speed_ki: Optional[float] = None,
        observer_gain: Optional[float] = None,
        pll_kp: Optional[float] = None,
        pll_ki: Optional[float] = None,
        max_iq_a: Optional[float] = None,
        open_loop_iq_a: Optional[float] = None,
        align_current_a: Optional[float] = None,
        fw_threshold: Optional[float] = None,
        apply: bool = False,
    ) -> None:
        """Write calibration parameters. Set apply=True to commit immediately."""
        # Read current active values as defaults
        current = self.can.get_all_signals()
        signals = {
            "CalibSelector": 0,
            "CalibSpeedKp": speed_kp if speed_kp is not None else current.get("ActiveSpeedKp", 0.003),
            "CalibSpeedKi": speed_ki if speed_ki is not None else current.get("ActiveSpeedKi", 0.006),
            "CalibObsGain": observer_gain if observer_gain is not None else current.get("ActiveObsGain", 1.8e6),
            "CalibPllKp": pll_kp if pll_kp is not None else current.get("ActivePllKp", 120.0),
            "CalibPllKi": pll_ki if pll_ki is not None else current.get("ActivePllKi", 6000.0),
            "CalibMaxIqA": max_iq_a if max_iq_a is not None else current.get("ActiveMaxIqA", 16.0),
            "CalibOpenLoopIqA": open_loop_iq_a if open_loop_iq_a is not None else current.get("ActiveOpenLoopIqA", 8.0),
            "CalibAlignCurrentA": align_current_a if align_current_a is not None else current.get("ActiveAlignCurrentA", 5.0),
            "CalibFwThreshold": fw_threshold if fw_threshold is not None else current.get("ActiveFwThreshold", 0.50),
            "CalibApply": 0xFF if apply else 0,
        }
        # Safety: clamp max Iq
        signals["CalibMaxIqA"] = min(signals["CalibMaxIqA"], self.MAX_SAFE_IQ_A)
        self.can.send_message("PC_CalibWrite", signals)

    def configure_hfi(
        self,
        enable: Optional[bool] = None,
        candidate_count: Optional[int] = None,
        pulse_pairs: Optional[int] = None,
        inject_v: Optional[float] = None,
        confidence_min: Optional[float] = None,
        polarity_v: Optional[float] = None,
        polarity_pulse_count: Optional[int] = None,
        align_hold_ms: Optional[int] = None,
    ) -> None:
        """Configure runtime HFI/IPD parameters via PC_TestCommand."""
        current = self.can.get_all_signals()
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x32,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
            "TcIdentPolePairs": 0,
            "TcIdentTargetRpm": 0.0,
            "TcIdentTestCurrentA": 0.0,
            "TcIdentIqMinA": 0.0,
            "TcIdentIqMaxA": 0.0,
            "TcIdentIqStepA": 0.0,
            "TcStepPreSamples": 0,
            "TcStepPostSamples": 0,
            "TcStepInjectIdA": 0.0,
            "TcStepDecimation": 0,
            "TcHfiEnable": 1 if (enable if enable is not None else bool(int(current.get("ActiveHfiFlags", 0)) & 0x01)) else 0,
            "TcHfiCandidateCount": candidate_count if candidate_count is not None else int(current.get("ActiveHfiCandidateCount", 4)),
            "TcHfiPulsePairs": pulse_pairs if pulse_pairs is not None else int(current.get("ActiveHfiPulsePairs", 8)),
            "TcHfiInjectV": inject_v if inject_v is not None else current.get("ActiveHfiInjectV", 1.2),
            "TcHfiConfidenceMin": confidence_min if confidence_min is not None else current.get("ActiveHfiConfidenceMin", 0.12),
            "TcHfiPolarityV": polarity_v if polarity_v is not None else current.get("ActiveHfiPolarityV", 1.8),
            "TcHfiPolarityPulseCount": polarity_pulse_count if polarity_pulse_count is not None else int(current.get("ActiveHfiPolarityPulseCount", 6)),
            "TcHfiAlignHoldMs": align_hold_ms if align_hold_ms is not None else int(current.get("ActiveHfiAlignHoldMs", 40)),
        })

    # ── Telemetry ───────────────────────────────────────────────────────────

    def get_telemetry(self) -> MotorTelemetry:
        """Get a complete telemetry snapshot."""
        s = self.can.get_all_signals()
        flags = int(s.get("Flags", 0))
        return MotorTelemetry(
            timestamp=time.monotonic(),
            state=MotorState(int(s.get("State", 0))),
            fault=MotorFault(int(s.get("FaultCode", 0))),
            control_mode=ControlMode(int(s.get("ControlMode", 0))),
            enabled=bool(flags & 0x01),
            observer_locked=bool(flags & 0x02),
            direction=1 if (flags & 0x04) else -1,
            mechanical_rpm=s.get("MechanicalRpm", 0.0),
            target_rpm=s.get("TargetRpm", 0.0),
            bus_voltage_v=s.get("BusVoltage", 0.0),
            iq_meas_a=s.get("IqMeas", 0.0),
            id_meas_a=s.get("IdMeas", 0.0),
            iq_target_a=s.get("IqTarget", 0.0),
            id_target_a=s.get("IdTarget", 0.0),
            elec_speed_rad_s=s.get("ElecSpeedRadS", 0.0),
            elec_angle_rad=s.get("ElecAngle", 0.0),
            phase_current_a=s.get("PhaseCurrentA", 0.0),
            phase_current_b=s.get("PhaseCurrentB", 0.0),
            phase_current_c=s.get("PhaseCurrentC", 0.0),
            observer_angle_rad=s.get("ObserverAngle", 0.0),
            observer_flux_vs=s.get("ObserverFluxVs", 0.0),
            pll_speed_rad_s=s.get("PLLSpeedRadS", 0.0),
            phase_error_rad=s.get("PhaseErrorRad", 0.0),
            speed_pi_integrator=s.get("SpeedPiIntegrator", 0.0),
            volt_mod_ratio=s.get("VoltModRatio", 0.0),
            pcb_temperature=s.get("PcbTemperature", 0.0),
            chip_temperature=s.get("ChipTemperature", 0.0),
            startup_retry_count=int(s.get("StartupRetryCount", 0)),
            hfi_active=bool(int(s.get("HfiFlags", 0)) & 0x01),
            hfi_angle_valid=bool(int(s.get("HfiFlags", 0)) & 0x02),
            hfi_used=bool(int(s.get("HfiFlags", 0)) & 0x04),
            hfi_fallback_reason=int(s.get("HfiFallbackReason", 0)),
            hfi_angle_rad=s.get("HfiAngle", 0.0),
            hfi_confidence=s.get("HfiConfidence", 0.0),
            hfi_ripple_metric=s.get("HfiRippleMetric", 0.0),
            state_time_ms=int(s.get("StateTimeMs", 0)),
            stall_div_count=int(s.get("StallDivCount", 0)),
            angle_mon_active=bool(int(s.get("AngleMonFlags", 0)) & 0x01),
            angle_mon_valid=bool(int(s.get("AngleMonFlags", 0)) & 0x02),
            angle_mon_angle_rad=s.get("AngleMonAngle", 0.0),
            angle_mon_speed_rad_s=s.get("AngleMonSpeedRadS", 0.0),
            angle_mon_bemf_mag=int(s.get("AngleMonBemfMag", 0)),
        )

    @property
    def state(self) -> MotorState:
        return MotorState(int(self.can.get_signal("State")))

    @property
    def fault(self) -> MotorFault:
        return MotorFault(int(self.can.get_signal("FaultCode")))

    @property
    def rpm(self) -> float:
        return self.can.get_signal("MechanicalRpm")

    @property
    def vbus(self) -> float:
        return self.can.get_signal("BusVoltage")

    @property
    def is_running(self) -> bool:
        return self.state == MotorState.CLOSED_LOOP

    @property
    def is_stopped(self) -> bool:
        return self.state == MotorState.STOP

    @property
    def is_faulted(self) -> bool:
        return self.state == MotorState.FAULT

    # ── Blocking Helpers ────────────────────────────────────────────────────

    def start_and_wait(
        self,
        target_rpm: float = 1000.0,
        timeout: float = 10.0,
        resend_interval: float = 0.1,
        **kwargs,
    ) -> bool:
        """Start motor and wait until closed-loop or fault.

        Periodically resends the start command every *resend_interval* seconds
        while the motor remains in STOP state, ensuring the firmware receives
        the enable request even if it requires periodic heartbeat commands.
        """
        self.start(target_rpm=target_rpm, **kwargs)
        deadline = time.monotonic() + timeout
        last_send = time.monotonic()
        while time.monotonic() < deadline:
            st = self.state
            if st == MotorState.CLOSED_LOOP:
                return True
            if st == MotorState.FAULT:
                return False
            # Resend command periodically while still in STOP
            now = time.monotonic()
            if st == MotorState.STOP and (now - last_send) >= resend_interval:
                self.start(target_rpm=target_rpm, **kwargs)
                last_send = now
            time.sleep(0.01)
        return False

    def stop_and_wait(self, timeout: float = 10.0, resend_interval: float = 0.5) -> bool:
        """Stop motor and wait until STOP state.

        Default timeout increased to 10s to accommodate high-inertia loads.
        Periodically resends the stop command to ensure reliable shutdown.
        """
        self.stop()
        deadline = time.monotonic() + timeout
        last_send = time.monotonic()
        while time.monotonic() < deadline:
            st = self.state
            if st == MotorState.STOP:
                return True
            # Resend stop command periodically
            now = time.monotonic()
            if (now - last_send) >= resend_interval:
                self.stop()
                last_send = now
            time.sleep(0.01)
        return False

    def wait_speed_settled(
        self,
        target_rpm: float,
        tolerance_pct: float = 5.0,
        settle_time_s: float = 1.0,
        timeout: float = 10.0,
    ) -> bool:
        """Wait until speed settles to within tolerance of target."""
        threshold = abs(target_rpm) * tolerance_pct / 100.0
        settle_start = None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rpm = abs(self.rpm)
            if abs(rpm - abs(target_rpm)) <= threshold:
                if settle_start is None:
                    settle_start = time.monotonic()
                elif (time.monotonic() - settle_start) >= settle_time_s:
                    return True
            else:
                settle_start = None
            time.sleep(0.01)
        return False

    # ── Safety Check ────────────────────────────────────────────────────────

    def check_safety(self) -> Optional[str]:
        """Check for dangerous conditions. Return description or None."""
        telem = self.get_telemetry()
        if telem.bus_voltage_v > self.MAX_SAFE_VBUS_V:
            return f"BUS OVERVOLTAGE: {telem.bus_voltage_v:.1f}V > {self.MAX_SAFE_VBUS_V}V"
        if telem.bus_voltage_v > 0 and telem.bus_voltage_v < self.MIN_SAFE_VBUS_V:
            return f"BUS UNDERVOLTAGE: {telem.bus_voltage_v:.1f}V < {self.MIN_SAFE_VBUS_V}V"
        iq = abs(telem.iq_meas_a)
        if iq > self.MAX_SAFE_IQ_A * 0.9:
            return f"NEAR OVERCURRENT: Iq={iq:.2f}A approaching {self.MAX_SAFE_IQ_A}A"
        return None

    # ── Recording ───────────────────────────────────────────────────────────

    def record_step_response(
        self,
        from_rpm: float,
        to_rpm: float,
        settle_s: float = 3.0,
        pre_s: float = 1.0,
        post_s: float = 3.0,
        sample_interval: float = 0.01,
    ) -> List[Dict[str, float]]:
        """Record a speed step response with pre/post settling data."""
        signals = [
            "MechanicalRpm", "TargetRpm", "IqMeas", "IdMeas",
            "IqTarget", "IdTarget", "BusVoltage", "PhaseErrorRad",
            "SpeedPiIntegrator", "VoltModRatio", "ElecSpeedRadS",
        ]

        # Ensure at starting speed
        self.set_speed(from_rpm)
        time.sleep(settle_s)

        # Pre-step data
        data = self.can.collect_signals(signals, pre_s, sample_interval)

        # Apply step
        step_time = time.monotonic()
        self.set_speed(to_rpm)

        # Post-step data
        post_data = self.can.collect_signals(signals, post_s, sample_interval)
        for d in post_data:
            d["_t"] += pre_s  # offset time
        data.extend(post_data)

        # Mark step point
        for d in data:
            d["_step_time"] = pre_s

        return data

    # ── Current Loop PI Tuning ─────────────────────────────────────────────

    def write_current_gains(
        self,
        id_kp: float,
        id_ki: float,
        iq_kp: float,
        iq_ki: float,
        apply: bool = True,
    ) -> None:
        """Write current loop PI gains via CalibWrite selector=2."""
        self.can.send_message("PC_CalibWrite", {
            "CalibSelector": 2,
            "CalibSpeedKp": id_kp,   # reuses byte positions
            "CalibSpeedKi": id_ki,
            "CalibObsGain": iq_kp,
            "CalibPllKp": iq_ki,
            "CalibPllKi": 0.0,
            "CalibMaxIqA": 0.0,
            "CalibOpenLoopIqA": 0.0,
            "CalibAlignCurrentA": 0.0,
            "CalibFwThreshold": 0.0,
            "CalibApply": 0xFF if apply else 0,
        })

    def reset_current_gains(self) -> None:
        """Reset current loop PI gains to firmware defaults via TestCommand."""
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x10,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
            "TcReserved": 0,
        })

    def get_current_gains(self) -> Dict[str, float]:
        """Read active current loop PI gains from CalibReadback."""
        s = self.can.get_all_signals()
        return {
            "id_kp": s.get("ActiveCurrentIdKp", 0.0),
            "id_ki": s.get("ActiveCurrentIdKi", 0.0),
            "iq_kp": s.get("ActiveCurrentIqKp", 0.0),
            "iq_ki": s.get("ActiveCurrentIqKi", 0.0),
        }

    # ── Waveform Capture (TestCommand + Status3) ───────────────────────────

    def start_waveform_capture(
        self,
        n_samples: int = 128,
        decimation: int = 1,
        trigger: int = 1,
    ) -> None:
        """Trigger waveform capture via TestCommand (0x01)."""
        n_samples = min(n_samples, 256)
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x01,
            "TcNSamples": n_samples,
            "TcDecimation": decimation,
            "TcTriggerSource": trigger,
            "TcReserved": 0,
        })

    def abort_waveform_capture(self) -> None:
        """Abort any active waveform capture via TestCommand (0x02)."""
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x02,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
            "TcReserved": 0,
        })

    def request_waveform_send(self) -> None:
        """Request MCU to burst-send captured waveform via Status3 (0x03)."""
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x03,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
            "TcReserved": 0,
        })

    def get_waveform_state(self) -> int:
        """Read waveform capture state: 0=Idle, 1=Capturing, 2=Sending, 3=Done."""
        return int(self.can.get_all_signals().get("WaveformState", 0))

    def get_waveform_sample_count(self) -> int:
        """Read number of captured waveform samples."""
        return int(self.can.get_all_signals().get("WaveformSampleCount", 0))

    def start_angle_monitor(self) -> None:
        """Enable passive BEMF angle monitoring for hand rotation."""
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x30,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
        })

    def stop_angle_monitor(self) -> None:
        """Disable passive BEMF angle monitoring."""
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x31,
            "TcNSamples": 0,
            "TcDecimation": 0,
            "TcTriggerSource": 0,
        })

    def capture_waveform(
        self,
        n_samples: int = 128,
        decimation: int = 1,
        capture_timeout: float = 5.0,
        send_timeout: float = 30.0,
    ) -> List[Dict[str, float]]:
        """
        Full waveform capture flow: trigger → wait → download.

        Returns list of dicts, each with keys:
        'Ia', 'Ib', 'Ic', 'Vbus', 'Id', 'Iq', 'ElecAngle', 'index'
        """
        self.start_waveform_capture(n_samples, decimation, trigger=1)

        # Wait for capture to complete (state transitions: Capturing → Idle)
        deadline = time.monotonic() + capture_timeout
        while time.monotonic() < deadline:
            ws = self.get_waveform_state()
            if ws != 1:  # Not CAPTURING
                break
            time.sleep(0.01)

        return self._download_waveform(capture_timeout, send_timeout)

    def capture_step_waveform(
        self,
        pre_samples: int = 64,
        post_samples: int = 128,
        inject_id_a: float = 1.0,
        decimation: int = 1,
        capture_timeout: float = 5.0,
        send_timeout: float = 30.0,
    ) -> List[Dict[str, float]]:
        """
        Trigger a step-synchronous waveform capture (TestCommand 0x04).
        """
        self.can.send_message("PC_TestCommand", {
            "TcCommandType": 0x04,
            "TcStepPreSamples": pre_samples,
            "TcStepPostSamples": post_samples,
            "TcStepInjectIdA": inject_id_a,
            "TcStepDecimation": decimation,
        })
        
        # Wait for capture to complete (StepPre -> StepPost -> Idle)
        deadline = time.monotonic() + capture_timeout
        while time.monotonic() < deadline:
            ws = self.get_waveform_state()
            if ws not in (4, 5, 1):
                break
            time.sleep(0.01)

        return self._download_waveform(capture_timeout, send_timeout)

    def _download_waveform(
        self,
        capture_timeout: float = 5.0,
        send_timeout: float = 30.0,
    ) -> List[Dict[str, float]]:

        # Request burst send
        self.request_waveform_send()
        time.sleep(0.05)

        # Collect Status3 burst frames
        total_expected = self.get_waveform_sample_count()
        if total_expected == 0:
            return []

        samples = []
        seen_indices = set()
        deadline = time.monotonic() + send_timeout

        while time.monotonic() < deadline:
            s = self.can.get_all_signals()
            idx = int(s.get("WfSampleIndex", 0))
            total = int(s.get("WfTotalSamples", 0))

            if idx not in seen_indices and total > 0:
                seen_indices.add(idx)
                samples.append({
                    "index": idx,
                    "Ia": s.get("WfCh0_Ia", 0.0),
                    "Ib": s.get("WfCh1_Ib", 0.0),
                    "Ic": s.get("WfCh2_Ic", 0.0),
                    "Vbus": s.get("WfCh3_Vbus", 0.0),
                    "Id": s.get("WfCh4_Id", 0.0),
                    "Iq": s.get("WfCh5_Iq", 0.0),
                    "ElecAngle": s.get("WfCh6_ElecAngle", 0.0),
                })

            ws = self.get_waveform_state()
            if ws == 3:  # DONE
                break
            time.sleep(0.002)

        samples.sort(key=lambda x: x["index"])
        return samples
