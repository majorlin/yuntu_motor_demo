#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Anomaly Analyzer
=================
Automatic fault detection, root-cause analysis, and improvement
suggestion engine for motor control CAN telemetry.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple

import numpy as np

from motor_client import MotorClient, MotorState, MotorFault, MotorTelemetry
from can_interface import CanInterface


class AnomalyType(Enum):
    STARTUP_FAILURE = auto()
    SPEED_OSCILLATION = auto()
    OBSERVER_LOSS = auto()
    CURRENT_OSCILLATION = auto()
    OVERCURRENT_TRIP = auto()
    OVERVOLTAGE_TRIP = auto()
    UNDERVOLTAGE_TRIP = auto()
    STARTUP_OVERSHOOT = auto()
    TEMPERATURE_ANOMALY = auto()
    STALL_DETECTED = auto()
    PHASE_ERROR_DIVERGENCE = auto()
    SPEED_TRACKING_ERROR = auto()


class Severity(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"


@dataclass
class Anomaly:
    """Detected anomaly with root cause analysis."""
    anomaly_type: AnomalyType
    severity: Severity
    timestamp: float
    description: str
    root_cause: str
    improvement: str
    signals: Dict[str, float] = field(default_factory=dict)
    waveform_context: Optional[List[Dict[str, float]]] = None


@dataclass
class AnalysisConfig:
    """Configurable thresholds for anomaly detection."""
    # Speed
    speed_oscillation_threshold_pct: float = 10.0   # RPM fluctuation %
    speed_oscillation_duration_s: float = 2.0
    speed_tracking_error_pct: float = 15.0
    startup_overshoot_pct: float = 20.0

    # Current
    current_oscillation_threshold_pct: float = 30.0
    current_oscillation_duration_s: float = 0.5
    overcurrent_threshold_a: float = 16.0

    # Observer
    phase_error_divergence_rad: float = 1.2
    phase_error_duration_s: float = 0.2
    observer_loss_count: int = 200

    # Voltage
    overvoltage_v: float = 18.0
    undervoltage_v: float = 7.0

    # Temperature
    temp_rise_rate_deg_per_min: float = 5.0
    max_chip_temp_c: float = 125.0
    max_pcb_temp_c: float = 85.0

    # Stall
    stall_div_count_threshold: int = 80

    # Startup
    startup_timeout_s: float = 5.0


class AnomalyAnalyzer:
    """
    Real-time and post-hoc anomaly detection engine.

    Monitors motor telemetry signals for known failure patterns,
    performs root-cause analysis, and generates improvement suggestions.
    """

    def __init__(
        self,
        can: CanInterface,
        motor: MotorClient,
        config: Optional[AnalysisConfig] = None,
    ):
        self.can = can
        self.motor = motor
        self.config = config or AnalysisConfig()
        self.anomalies: List[Anomaly] = []
        self._speed_history: List[Tuple[float, float]] = []
        self._iq_history: List[Tuple[float, float]] = []
        self._phase_err_history: List[Tuple[float, float]] = []
        self._temp_history: List[Tuple[float, float, float]] = []

    def clear(self):
        """Clear all accumulated anomaly data."""
        self.anomalies.clear()
        self._speed_history.clear()
        self._iq_history.clear()
        self._phase_err_history.clear()
        self._temp_history.clear()

    # ── Real-time Check ─────────────────────────────────────────────────────

    def check(self) -> List[Anomaly]:
        """Run a single-pass anomaly check on current telemetry. Returns new anomalies."""
        now = time.monotonic()
        telem = self.motor.get_telemetry()
        new_anomalies = []

        # Update histories
        self._speed_history.append((now, telem.mechanical_rpm))
        self._iq_history.append((now, telem.iq_meas_a))
        self._phase_err_history.append((now, telem.phase_error_rad))
        self._temp_history.append((now, telem.chip_temperature, telem.pcb_temperature))

        # Trim old history (keep last 30s)
        cutoff = now - 30.0
        self._speed_history = [(t, v) for t, v in self._speed_history if t > cutoff]
        self._iq_history = [(t, v) for t, v in self._iq_history if t > cutoff]
        self._phase_err_history = [(t, v) for t, v in self._phase_err_history if t > cutoff]
        self._temp_history = [(t, c, p) for t, c, p in self._temp_history if t > cutoff]

        # ── Fault-based detection ───────────────────────────────────────────
        if telem.state == MotorState.FAULT:
            a = self._analyze_fault(telem)
            if a:
                new_anomalies.append(a)

        # ── Speed oscillation ──────────────────────────────────────────────
        if telem.state == MotorState.CLOSED_LOOP:
            a = self._check_speed_oscillation(telem)
            if a:
                new_anomalies.append(a)

            a = self._check_speed_tracking(telem)
            if a:
                new_anomalies.append(a)

            a = self._check_current_oscillation(telem)
            if a:
                new_anomalies.append(a)

            a = self._check_phase_error(telem)
            if a:
                new_anomalies.append(a)

        # ── Temperature ────────────────────────────────────────────────────
        a = self._check_temperature(telem)
        if a:
            new_anomalies.append(a)

        # ── Stall ──────────────────────────────────────────────────────────
        if telem.stall_div_count >= self.config.stall_div_count_threshold:
            new_anomalies.append(Anomaly(
                anomaly_type=AnomalyType.STALL_DETECTED,
                severity=Severity.WARNING,
                timestamp=now,
                description=f"Stall divergence count = {telem.stall_div_count}",
                root_cause="转子受阻或开环电流不足以克服负载力矩",
                improvement="增大 MOTOR_CFG_OPEN_LOOP_IQ_A 或检查机械负载",
                signals={"StallDivCount": telem.stall_div_count},
            ))

        self.anomalies.extend(new_anomalies)
        return new_anomalies

    # ── Fault Analysis ──────────────────────────────────────────────────────

    def _analyze_fault(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        FAULT_ANALYSIS = {
            MotorFault.OVERCURRENT: (
                AnomalyType.OVERCURRENT_TRIP,
                Severity.CRITICAL,
                "相电流超过软件限制阈值",
                "堵转/负载突变/PI积分器饱和/采样偏移",
                "1. 降低 MOTOR_CFG_MAX_IQ_A\n"
                "2. 增加 PI Anti-windup\n"
                "3. 检查 ADC offset 校准\n"
                "4. 检查机械负载是否异常",
            ),
            MotorFault.VBUS_OVERVOLTAGE: (
                AnomalyType.OVERVOLTAGE_TRIP,
                Severity.CRITICAL,
                "母线电压超过阈值",
                "急减速回馈能量/无制动电阻/电源异常",
                "1. 增加减速斜坡时间 (SPEED_RAMP_RPM_PER_S)\n"
                "2. 降低制动Iq (COAST_BRAKE_IQ_A)\n"
                "3. 添加制动电阻硬件",
            ),
            MotorFault.VBUS_UNDERVOLTAGE: (
                AnomalyType.UNDERVOLTAGE_TRIP,
                Severity.WARNING,
                "母线电压低于阈值",
                "供电不足/线缆压降/电源掉电",
                "1. 检查电源容量和线缆阻抗\n"
                "2. 降低启动电流减少浪涌\n"
                "3. 适当降低 VBUS_UNDERVOLTAGE_V 阈值",
            ),
            MotorFault.OBSERVER_LOSS: (
                AnomalyType.OBSERVER_LOSS,
                Severity.CRITICAL,
                "观测器相位误差超限，失去对转子位置的跟踪",
                "磁链估计偏差/PLL带宽不足/负载突变/电机参数不匹配",
                "1. 放宽 FLUX_LINKAGE 范围 (LAMBDA_MIN/MAX)\n"
                "2. 增大 PLL_KP 提高跟踪带宽\n"
                "3. 增大 OBSERVER_LOSS_COUNT 减少误触发\n"
                "4. 校准电机 Rs/Ls 参数",
            ),
            MotorFault.STARTUP_TIMEOUT: (
                AnomalyType.STARTUP_FAILURE,
                Severity.CRITICAL,
                "启动超时，观测器未能在规定时间内锁定",
                "开环Iq不足/观测器增益不匹配/负载过重",
                "1. 增大 OPEN_LOOP_IQ_A\n"
                "2. 延长 STARTUP_TIMEOUT_MS\n"
                "3. 放宽 OBSERVER_LOCK_PHASE_ERR_RAD\n"
                "4. 减小 MIN_LOCK_ELEC_SPEED_RAD_S",
            ),
            MotorFault.STARTUP_FAIL: (
                AnomalyType.STARTUP_FAILURE,
                Severity.CRITICAL,
                "多次重试后启动仍然失败",
                "持续无法克服负载/电机参数严重不匹配",
                "1. 增大 STARTUP_IQ_BOOST_STEP_A\n"
                "2. 增大 STARTUP_MAX_RETRIES\n"
                "3. 检查机械连接和负载条件\n"
                "4. 重新标定电机参数",
            ),
            MotorFault.CATCH_FAIL: (
                AnomalyType.STARTUP_FAILURE,
                Severity.WARNING,
                "顺风捕获失败",
                "BEMF检测不准确/风速超出捕获范围",
                "1. 增大 CATCH_MAX_SPEED_RPM\n"
                "2. 调整 WIND_DETECT_TIMEOUT_MS\n"
                "3. 检查 BEMF 采样电路",
            ),
        }

        if telem.fault == MotorFault.NONE:
            return None

        info = FAULT_ANALYSIS.get(telem.fault)
        if info is None:
            return Anomaly(
                anomaly_type=AnomalyType.STARTUP_FAILURE,
                severity=Severity.WARNING,
                timestamp=time.monotonic(),
                description=f"Unknown fault: {telem.fault.name}",
                root_cause="未知",
                improvement="检查固件日志",
            )

        atype, severity, desc, cause, improve = info
        return Anomaly(
            anomaly_type=atype,
            severity=severity,
            timestamp=time.monotonic(),
            description=desc,
            root_cause=cause,
            improvement=improve,
            signals={
                "FaultCode": telem.fault.value,
                "State": telem.state.value,
                "BusVoltage": telem.bus_voltage_v,
                "IqMeas": telem.iq_meas_a,
                "MechanicalRpm": telem.mechanical_rpm,
                "PhaseErrorRad": telem.phase_error_rad,
            },
        )

    # ── Signal-based Checks ────────────────────────────────────────────────

    def _check_speed_oscillation(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        """Detect sustained speed oscillation."""
        if len(self._speed_history) < 50:
            return None

        window_s = self.config.speed_oscillation_duration_s
        now = time.monotonic()
        recent = [v for t, v in self._speed_history if t > (now - window_s)]
        if len(recent) < 20:
            return None

        mean_rpm = np.mean(recent)
        if abs(mean_rpm) < 10:
            return None

        std_rpm = np.std(recent)
        fluctuation_pct = (std_rpm / abs(mean_rpm)) * 100.0

        if fluctuation_pct > self.config.speed_oscillation_threshold_pct:
            return Anomaly(
                anomaly_type=AnomalyType.SPEED_OSCILLATION,
                severity=Severity.WARNING,
                timestamp=now,
                description=f"速度振荡: 波动 {fluctuation_pct:.1f}% (>{self.config.speed_oscillation_threshold_pct}%)",
                root_cause="Speed Kp 过高导致比例冲击 / Speed Ki 过高导致积分器振荡",
                improvement="1. 降低 Speed_Kp 50%\n2. 降低 Speed_Ki 30%\n3. 通过 CalibWrite 在线调参验证",
                signals={"MechanicalRpm": telem.mechanical_rpm, "TargetRpm": telem.target_rpm,
                         "SpeedPiIntegrator": telem.speed_pi_integrator},
            )
        return None

    def _check_speed_tracking(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        """Check steady-state speed tracking error."""
        if abs(telem.target_rpm) < 10:
            return None
        error_pct = abs(telem.mechanical_rpm - telem.target_rpm) / abs(telem.target_rpm) * 100
        if error_pct > self.config.speed_tracking_error_pct:
            return Anomaly(
                anomaly_type=AnomalyType.SPEED_TRACKING_ERROR,
                severity=Severity.WARNING,
                timestamp=time.monotonic(),
                description=f"速度跟踪误差 {error_pct:.1f}% > {self.config.speed_tracking_error_pct}%",
                root_cause="Speed Ki 不足/负载超出能力/弱磁区域失控",
                improvement="1. 适当增大 Speed_Ki\n2. 检查负载条件\n3. 检查弱磁控制参数",
                signals={"MechanicalRpm": telem.mechanical_rpm, "TargetRpm": telem.target_rpm},
            )
        return None

    def _check_current_oscillation(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        """Detect Iq current oscillation."""
        if len(self._iq_history) < 30:
            return None

        window_s = self.config.current_oscillation_duration_s
        now = time.monotonic()
        recent = [v for t, v in self._iq_history if t > (now - window_s)]
        if len(recent) < 10 or abs(np.mean(recent)) < 0.1:
            return None

        fluctuation_pct = np.std(recent) / abs(np.mean(recent)) * 100
        if fluctuation_pct > self.config.current_oscillation_threshold_pct:
            return Anomaly(
                anomaly_type=AnomalyType.CURRENT_OSCILLATION,
                severity=Severity.WARNING,
                timestamp=now,
                description=f"电流振荡: Iq 波动 {fluctuation_pct:.1f}%",
                root_cause="电流环带宽过高/ADC采样噪声/死区补偿异常",
                improvement="1. 降低 CURRENT_LOOP_BW_HZ\n2. 检查 ADC offset 校准\n3. 调整死区补偿增益",
                signals={"IqMeas": telem.iq_meas_a, "IqTarget": telem.iq_target_a},
            )
        return None

    def _check_phase_error(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        """Check observer phase error divergence."""
        if abs(telem.phase_error_rad) > self.config.phase_error_divergence_rad:
            return Anomaly(
                anomaly_type=AnomalyType.PHASE_ERROR_DIVERGENCE,
                severity=Severity.WARNING,
                timestamp=time.monotonic(),
                description=f"观测器相位误差 {telem.phase_error_rad:.4f} rad 接近失锁阈值",
                root_cause="电机参数不匹配/负载突变/磁链估计偏差",
                improvement="1. 增大 OBSERVER_GAIN\n2. 增大 PLL_KP\n3. 校准 Rs/Ls/Flux 参数",
                signals={"PhaseErrorRad": telem.phase_error_rad,
                         "ObserverAngle": telem.observer_angle_rad,
                         "PLLSpeedRadS": telem.pll_speed_rad_s},
            )
        return None

    def _check_temperature(self, telem: MotorTelemetry) -> Optional[Anomaly]:
        """Check for temperature anomalies."""
        if telem.chip_temperature > self.config.max_chip_temp_c:
            return Anomaly(
                anomaly_type=AnomalyType.TEMPERATURE_ANOMALY,
                severity=Severity.CRITICAL,
                timestamp=time.monotonic(),
                description=f"芯片温度 {telem.chip_temperature:.1f}°C 超限",
                root_cause="持续高电流运行/散热设计不足",
                improvement="1. 降低运行电流\n2. 改善散热设计\n3. 降低 PWM 频率减少开关损耗",
                signals={"ChipTemperature": telem.chip_temperature,
                         "PcbTemperature": telem.pcb_temperature},
            )

        # Temperature rise rate
        if len(self._temp_history) >= 60:
            recent_60 = self._temp_history[-60:]
            dt = recent_60[-1][0] - recent_60[0][0]
            if dt > 0:
                rate = (recent_60[-1][1] - recent_60[0][1]) / (dt / 60.0)
                if rate > self.config.temp_rise_rate_deg_per_min:
                    return Anomaly(
                        anomaly_type=AnomalyType.TEMPERATURE_ANOMALY,
                        severity=Severity.WARNING,
                        timestamp=time.monotonic(),
                        description=f"芯片温升速率 {rate:.1f}°C/min 异常",
                        root_cause="持续过流/散热不良",
                        improvement="降低运行电流，检查散热条件",
                        signals={"ChipTemperature": telem.chip_temperature,
                                 "rise_rate_deg_per_min": rate},
                    )
        return None

    # ── Post-hoc Analysis ───────────────────────────────────────────────────

    def analyze_history(self, history: List[Dict]) -> List[Anomaly]:
        """Analyze a recorded signal history for anomalies."""
        anomalies = []
        for i, record in enumerate(history):
            # Check for fault transitions
            state = int(record.get("State", 0))
            fault = int(record.get("FaultCode", 0))
            if state == MotorState.FAULT and fault != 0:
                telem = self._record_to_telem(record)
                a = self._analyze_fault(telem)
                if a:
                    anomalies.append(a)
        return anomalies

    @staticmethod
    def _record_to_telem(record: Dict) -> MotorTelemetry:
        """Convert a history record dict to MotorTelemetry."""
        flags = int(record.get("Flags", 0))
        return MotorTelemetry(
            timestamp=record.get("_t", 0),
            state=MotorState(int(record.get("State", 0))),
            fault=MotorFault(int(record.get("FaultCode", 0))),
            control_mode=0,
            enabled=bool(flags & 0x01),
            observer_locked=bool(flags & 0x02),
            direction=1 if (flags & 0x04) else -1,
            mechanical_rpm=record.get("MechanicalRpm", 0.0),
            target_rpm=record.get("TargetRpm", 0.0),
            bus_voltage_v=record.get("BusVoltage", 0.0),
            iq_meas_a=record.get("IqMeas", 0.0),
            id_meas_a=record.get("IdMeas", 0.0),
            iq_target_a=record.get("IqTarget", 0.0),
            id_target_a=record.get("IdTarget", 0.0),
            elec_speed_rad_s=record.get("ElecSpeedRadS", 0.0),
            elec_angle_rad=record.get("ElecAngle", 0.0),
            phase_current_a=record.get("PhaseCurrentA", 0.0),
            phase_current_b=record.get("PhaseCurrentB", 0.0),
            phase_current_c=record.get("PhaseCurrentC", 0.0),
            observer_angle_rad=record.get("ObserverAngle", 0.0),
            observer_flux_vs=record.get("ObserverFluxVs", 0.0),
            pll_speed_rad_s=record.get("PLLSpeedRadS", 0.0),
            phase_error_rad=record.get("PhaseErrorRad", 0.0),
            speed_pi_integrator=record.get("SpeedPiIntegrator", 0.0),
            volt_mod_ratio=record.get("VoltModRatio", 0.0),
            pcb_temperature=record.get("PcbTemperature", 0.0),
            chip_temperature=record.get("ChipTemperature", 0.0),
            startup_retry_count=int(record.get("StartupRetryCount", 0)),
            hfi_active=bool(int(record.get("HfiFlags", 0)) & 0x01),
            hfi_angle_valid=bool(int(record.get("HfiFlags", 0)) & 0x02),
            hfi_used=bool(int(record.get("HfiFlags", 0)) & 0x04),
            hfi_fallback_reason=int(record.get("HfiFallbackReason", 0)),
            hfi_angle_rad=record.get("HfiAngle", 0.0),
            hfi_confidence=record.get("HfiConfidence", 0.0),
            hfi_ripple_metric=record.get("HfiRippleMetric", 0.0),
            state_time_ms=int(record.get("StateTimeMs", 0)),
            stall_div_count=int(record.get("StallDivCount", 0)),
            angle_mon_active=bool(int(record.get("AngleMonFlags", 0)) & 0x01),
            angle_mon_valid=bool(int(record.get("AngleMonFlags", 0)) & 0x02),
            angle_mon_angle_rad=record.get("AngleMonAngle", 0.0),
            angle_mon_speed_rad_s=record.get("AngleMonSpeedRadS", 0.0),
            angle_mon_bemf_mag=int(record.get("AngleMonBemfMag", 0)),
        )

    # ── Report ──────────────────────────────────────────────────────────────

    def get_summary(self) -> str:
        """Get a text summary of all detected anomalies."""
        if not self.anomalies:
            return "✅ 未检测到异常"

        lines = [f"⚠️ 共检测到 {len(self.anomalies)} 个异常:\n"]
        for i, a in enumerate(self.anomalies, 1):
            lines.append(f"{'='*60}")
            lines.append(f"[{a.severity.value}] #{i}: {a.anomaly_type.name}")
            lines.append(f"  描述: {a.description}")
            lines.append(f"  根因: {a.root_cause}")
            lines.append(f"  改善: {a.improvement}")
            if a.signals:
                sig_str = ", ".join(f"{k}={v}" for k, v in a.signals.items())
                lines.append(f"  信号: {sig_str}")
        return "\n".join(lines)
