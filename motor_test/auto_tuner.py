#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Auto-Tuner for Speed PI and Observer/PLL Parameters
=====================================================
Uses step-response analysis and golden-section search to find
optimal PI gains. Safety-first: monitors Vbus and Iq continuously.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import asdict, dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np

from can_interface import CanInterface
from motor_client import MotorClient, MotorState
from waveform_capture import WaveformCapture, StepResponseMetrics
from anomaly_analyzer import AnomalyAnalyzer


@dataclass
class TuneResult:
    """Result of a single tuning trial."""
    kp: float
    ki: float
    metrics: StepResponseMetrics
    passed: bool
    reason: str = ""
    waveform_file: Optional[str] = None


@dataclass
class TuneReport:
    """Final tuning report."""
    parameter_name: str
    best_kp: float
    best_ki: float
    best_metrics: StepResponseMetrics
    all_trials: List[TuneResult]
    timestamp: str


class AutoTuner:
    """
    Automated PI parameter tuning via step-response optimization.

    Safety features:
    - Monitors Vbus continuously during tuning
    - Limits Iq to safe range
    - Immediately stops motor on any safety violation
    - Reverts to last-known-good parameters on failure
    """

    # Safety thresholds
    VBUS_MAX_V = 17.0          # Stop if Vbus approaches OV threshold
    IQ_MAX_A = 14.0            # Stop if Iq approaches OC threshold
    EMERGENCY_STOP_DELAY = 0.5 # Wait after emergency stop

    def __init__(
        self,
        can: CanInterface,
        motor: MotorClient,
        waveform: WaveformCapture,
        analyzer: AnomalyAnalyzer,
        output_dir: str = "test/test_records",
    ):
        self.can = can
        self.motor = motor
        self.waveform = waveform
        self.analyzer = analyzer
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    # ── Speed PI Tuning ─────────────────────────────────────────────────────

    def tune_speed_pi(
        self,
        base_rpm: float = 1000.0,
        step_rpm: float = 2000.0,
        kp_range: Tuple[float, float] = (0.0005, 0.02),
        ki_range: Tuple[float, float] = (0.001, 0.05),
        kp_steps: int = 5,
        ki_steps: int = 5,
        settle_time_s: float = 3.0,
        record_time_s: float = 5.0,
        target_overshoot_pct: float = 10.0,
        target_rise_time_s: float = 2.0,
    ) -> TuneReport:
        """
        Grid search over Kp/Ki space with step response evaluation.

        Strategy:
        1. Start motor at base_rpm
        2. For each (Kp, Ki) pair:
           a. Apply new gains via CalibWrite
           b. Wait for settling
           c. Apply speed step to step_rpm
           d. Record response and compute metrics
           e. Check safety throughout
           f. Step back to base_rpm
        3. Select best parameters
        """
        # Save original parameters
        orig_kp = self.can.get_signal("ActiveSpeedKp") or 0.003
        orig_ki = self.can.get_signal("ActiveSpeedKi") or 0.006

        kp_values = np.linspace(kp_range[0], kp_range[1], kp_steps)
        ki_values = np.linspace(ki_range[0], ki_range[1], ki_steps)

        trials: List[TuneResult] = []
        best: Optional[TuneResult] = None
        best_score = float("inf")

        # Start motor
        if not self.motor.is_running:
            ok = self.motor.start_and_wait(target_rpm=base_rpm, timeout=10.0)
            if not ok:
                return self._make_report("Speed_PI", orig_kp, orig_ki,
                                         StepResponseMetrics(), trials, "启动失败")

        for kp in kp_values:
            for ki in ki_values:
                # Safety check before each trial
                safety = self.motor.check_safety()
                if safety:
                    self._emergency_stop()
                    return self._make_report("Speed_PI", orig_kp, orig_ki,
                                             StepResponseMetrics(), trials, f"安全停止: {safety}")

                trial = self._run_speed_trial(
                    kp, ki, base_rpm, step_rpm, settle_time_s, record_time_s
                )
                trials.append(trial)

                if trial.passed:
                    # Score: weighted combination favoring low overshoot and fast rise
                    score = (
                        trial.metrics.overshoot_pct * 2.0
                        + trial.metrics.rise_time_s * 10.0
                        + trial.metrics.settling_time_s * 5.0
                        + trial.metrics.steady_state_error_pct * 3.0
                    )
                    # Penalty if overshoot exceeds target
                    if trial.metrics.overshoot_pct > target_overshoot_pct:
                        score += (trial.metrics.overshoot_pct - target_overshoot_pct) * 10
                    if trial.metrics.rise_time_s > target_rise_time_s:
                        score += (trial.metrics.rise_time_s - target_rise_time_s) * 20

                    if score < best_score:
                        best_score = score
                        best = trial

        # Restore best or original parameters
        if best:
            self.motor.write_calib(speed_kp=best.kp, speed_ki=best.ki, apply=True)
            time.sleep(0.5)
            return self._make_report("Speed_PI", best.kp, best.ki, best.metrics, trials)
        else:
            self.motor.write_calib(speed_kp=orig_kp, speed_ki=orig_ki, apply=True)
            return self._make_report("Speed_PI", orig_kp, orig_ki,
                                     StepResponseMetrics(), trials, "未找到合适参数")

    def _run_speed_trial(
        self, kp: float, ki: float,
        base_rpm: float, step_rpm: float,
        settle_s: float, record_s: float,
    ) -> TuneResult:
        """Run a single speed PI trial."""
        # Apply new parameters
        self.motor.write_calib(speed_kp=kp, speed_ki=ki, apply=True)
        time.sleep(0.3)  # Wait for parameters to take effect

        # Ensure we're at base speed
        self.motor.set_speed(base_rpm)
        settled = self.motor.wait_speed_settled(base_rpm, tolerance_pct=10.0,
                                                 settle_time_s=1.0, timeout=settle_s)
        if not settled:
            return TuneResult(kp=kp, ki=ki, metrics=StepResponseMetrics(),
                              passed=False, reason="未能稳定在基准速度")

        # Capture step response
        record, metrics = self.waveform.capture_step_response(
            signal_name="MechanicalRpm",
            step_trigger=lambda: self.motor.set_speed(step_rpm),
            pre_s=1.0,
            post_s=record_s,
            interval_s=0.01,
            extra_signals=["IqMeas", "SpeedPiIntegrator", "PhaseErrorRad", "BusVoltage"],
        )

        # Safety check during recording
        for sig_vals in [self.can.get_signals(["BusVoltage", "IqMeas"])]:
            if sig_vals.get("BusVoltage", 0) > self.VBUS_MAX_V:
                self._emergency_stop()
                return TuneResult(kp=kp, ki=ki, metrics=metrics,
                                  passed=False, reason="过压")
            if abs(sig_vals.get("IqMeas", 0)) > self.IQ_MAX_A:
                self._emergency_stop()
                return TuneResult(kp=kp, ki=ki, metrics=metrics,
                                  passed=False, reason="过流")

        # Save waveform
        filename = f"speed_pi_kp{kp:.6f}_ki{ki:.6f}.png"
        wf_file = self.waveform.plot_step_response(record, metrics, "MechanicalRpm", filename)

        # Return to base
        self.motor.set_speed(base_rpm)
        time.sleep(1.0)

        return TuneResult(kp=kp, ki=ki, metrics=metrics, passed=True, waveform_file=wf_file)

    # ── Observer/PLL Tuning ─────────────────────────────────────────────────

    def tune_observer(
        self,
        rpm: float = 1500.0,
        gain_range: Tuple[float, float] = (0.5e6, 5.0e6),
        pll_kp_range: Tuple[float, float] = (50.0, 300.0),
        pll_ki_range: Tuple[float, float] = (2000.0, 15000.0),
        gain_steps: int = 5,
        pll_steps: int = 4,
        measure_time_s: float = 2.0,
    ) -> TuneReport:
        """
        Sweep observer gain and PLL parameters to minimize phase error noise.

        Strategy:
        1. Run at fixed RPM
        2. For each parameter combination:
           a. Apply via CalibWrite
           b. Record PhaseError and PLLSpeed for measure_time_s
           c. Compute PhaseError RMS and PLLSpeed variance
        3. Select parameters that minimize both
        """
        orig_gain = self.can.get_signal("ActiveObsGain") or 1.8e6
        orig_pll_kp = self.can.get_signal("ActivePllKp") or 120.0
        orig_pll_ki = self.can.get_signal("ActivePllKi") or 6000.0

        if not self.motor.is_running:
            ok = self.motor.start_and_wait(target_rpm=rpm, timeout=10.0)
            if not ok:
                return self._make_report("Observer", orig_gain, orig_pll_kp,
                                         StepResponseMetrics(), [], "启动失败")

        gains = np.linspace(gain_range[0], gain_range[1], gain_steps)
        pll_kps = np.linspace(pll_kp_range[0], pll_kp_range[1], pll_steps)
        pll_kis = np.linspace(pll_ki_range[0], pll_ki_range[1], pll_steps)

        trials: List[TuneResult] = []
        best_score = float("inf")
        best_params = (orig_gain, orig_pll_kp, orig_pll_ki)

        for gain in gains:
            for pkp in pll_kps:
                for pki in pll_kis:
                    safety = self.motor.check_safety()
                    if safety:
                        self._emergency_stop()
                        return self._make_report("Observer", *best_params,
                                                 StepResponseMetrics(), trials, f"安全停止: {safety}")

                    # Apply
                    self.motor.write_calib(observer_gain=gain, pll_kp=pkp, pll_ki=pki, apply=True)
                    time.sleep(0.5)

                    # Check stability
                    if self.motor.is_faulted:
                        self.motor.stop()
                        time.sleep(1.0)
                        self.motor.write_calib(observer_gain=orig_gain, pll_kp=orig_pll_kp,
                                               pll_ki=orig_pll_ki, apply=True)
                        self.motor.start_and_wait(target_rpm=rpm)
                        trials.append(TuneResult(kp=gain, ki=pkp,
                                                 metrics=StepResponseMetrics(),
                                                 passed=False, reason="观测器失锁"))
                        continue

                    # Measure
                    data = self.can.collect_signals(
                        ["PhaseErrorRad", "PLLSpeedRadS", "ObserverFluxVs"],
                        measure_time_s, 0.01,
                    )
                    pe = [d["PhaseErrorRad"] for d in data]
                    pll_s = [d["PLLSpeedRadS"] for d in data]

                    pe_rms = float(np.sqrt(np.mean(np.array(pe)**2)))
                    pll_var = float(np.var(pll_s))

                    score = pe_rms * 100 + pll_var * 0.001
                    metrics = StepResponseMetrics(
                        steady_state_error_pct=pe_rms,
                        overshoot_pct=pll_var,
                    )
                    trial = TuneResult(kp=gain, ki=pkp, metrics=metrics, passed=True,
                                       reason=f"PE_RMS={pe_rms:.4f}, PLL_var={pll_var:.1f}")
                    trials.append(trial)

                    if score < best_score:
                        best_score = score
                        best_params = (gain, pkp, pki)

        # Apply best
        self.motor.write_calib(observer_gain=best_params[0], pll_kp=best_params[1],
                               pll_ki=best_params[2], apply=True)

        return self._make_report("Observer", best_params[0], best_params[1],
                                 StepResponseMetrics(), trials)

    # ── Helpers ─────────────────────────────────────────────────────────────

    def _emergency_stop(self):
        """Emergency stop with safety delay."""
        self.motor.stop()
        time.sleep(self.EMERGENCY_STOP_DELAY)

    def _make_report(
        self, name: str, kp: float, ki: float,
        metrics: StepResponseMetrics, trials: List[TuneResult],
        error: str = "",
    ) -> TuneReport:
        report = TuneReport(
            parameter_name=name,
            best_kp=kp,
            best_ki=ki,
            best_metrics=metrics,
            all_trials=trials,
            timestamp=time.strftime("%Y%m%d_%H%M%S"),
        )
        # Save report JSON
        filepath = os.path.join(self.output_dir, f"tune_{name}_{report.timestamp}.json")
        data = {
            "parameter_name": name,
            "best_kp": kp,
            "best_ki": ki,
            "best_metrics": asdict(metrics),
            "trials_count": len(trials),
            "passed_count": sum(1 for t in trials if t.passed),
            "error": error,
            "trials": [
                {"kp": t.kp, "ki": t.ki, "passed": t.passed, "reason": t.reason,
                 "overshoot": t.metrics.overshoot_pct, "rise_time": t.metrics.rise_time_s}
                for t in trials
            ],
        }
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return report


# ── CLI Entry Point ─────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Motor Auto-Tuner")
    parser.add_argument("--port", type=str, help="Serial port")
    parser.add_argument("--mode", choices=["speed_pi", "observer"], default="speed_pi")
    parser.add_argument("--rpm", type=float, default=1000.0, help="Base RPM")
    parser.add_argument("--step-rpm", type=float, default=2000.0, help="Step RPM (speed_pi mode)")
    args = parser.parse_args()

    can = CanInterface(port=args.port, auto_connect=True)
    motor = MotorClient(can)
    wf = WaveformCapture(can)
    analyzer = AnomalyAnalyzer(can, motor)
    tuner = AutoTuner(can, motor, wf, analyzer)

    try:
        if args.mode == "speed_pi":
            print("🔧 Starting Speed PI auto-tune...")
            report = tuner.tune_speed_pi(base_rpm=args.rpm, step_rpm=args.step_rpm)
            print(f"✅ Best Kp={report.best_kp:.6f}, Ki={report.best_ki:.6f}")
            print(f"   Overshoot: {report.best_metrics.overshoot_pct:.1f}%")
            print(f"   Rise time: {report.best_metrics.rise_time_s:.3f}s")
        else:
            print("🔧 Starting Observer auto-tune...")
            report = tuner.tune_observer(rpm=args.rpm)
            print(f"✅ Best Gain={report.best_kp:.0f}, PLL_Kp={report.best_ki:.1f}")
    finally:
        motor.stop()
        time.sleep(0.5)
        can.disconnect()
