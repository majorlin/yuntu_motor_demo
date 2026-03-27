#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-002: 速度响应测试
======================
验证速度环阶跃响应，评估上升时间、超调、稳态误差。
"""

import time
import pytest
from motor_client import MotorState
from report_generator import TestRecord, TestVerdict


class TestSpeedResponse:
    """TC-002: Speed step response characterization."""

    TEST_ID = "TC-002"

    @pytest.mark.parametrize("from_rpm,to_rpm", [
        (500, 1500),
        (1000, 2500),
        (800, 2000),
        (2500, 500),  # deceleration
    ])
    def test_002_speed_step(self, motor, can, waveform, analyzer, report,
                             csv_logger, from_rpm, to_rpm):
        """速度阶跃响应: from_rpm → to_rpm"""
        t0 = time.time()
        analyzer.clear()

        # Start and settle at from_rpm
        ok = motor.start_and_wait(target_rpm=from_rpm, timeout=10.0)
        assert ok, f"启动失败"
        settled = motor.wait_speed_settled(from_rpm, tolerance_pct=10, timeout=8.0)

        # Capture step response
        record_wf, metrics = waveform.capture_step_response(
            signal_name="MechanicalRpm",
            step_trigger=lambda: motor.set_speed(to_rpm),
            pre_s=1.0,
            post_s=6.0,
            interval_s=0.01,
            extra_signals=["IqMeas", "SpeedPiIntegrator", "BusVoltage", "PhaseErrorRad"],
        )

        # Plot
        plot_file = waveform.plot_step_response(
            record_wf, metrics, "MechanicalRpm",
            filename=f"tc002_step_{from_rpm}_{to_rpm}.png",
        )

        # Check anomalies
        anomalies = analyzer.check()
        duration = time.time() - t0

        # Verdict
        is_accel = to_rpm > from_rpm
        passed = True
        reasons = []
        if is_accel:
            if metrics.rise_time_s > 3.0:
                passed = False
                reasons.append(f"上升时间 {metrics.rise_time_s:.2f}s > 3.0s")
            if metrics.overshoot_pct > 20.0:
                passed = False
                reasons.append(f"超调 {metrics.overshoot_pct:.1f}% > 20%")
        if metrics.steady_state_error_pct > 10.0:
            passed = False
            reasons.append(f"稳态误差 {metrics.steady_state_error_pct:.1f}% > 10%")

        telem = motor.get_telemetry()
        record = TestRecord(
            test_id=f"{self.TEST_ID}",
            test_name=f"速度阶跃 {from_rpm}→{to_rpm} RPM",
            description=f"从{from_rpm}RPM阶跃到{to_rpm}RPM，分析响应品质",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            preconditions=f"电机稳定运行在{from_rpm}RPM",
            test_method=f"CAN设定目标{to_rpm}RPM，采集6s响应数据",
            pass_criteria="上升时间≤3s，超调≤20%，稳态误差≤10%",
            actual_result=(f"上升时间={metrics.rise_time_s:.2f}s, "
                          f"超调={metrics.overshoot_pct:.1f}%, "
                          f"稳态误差={metrics.steady_state_error_pct:.1f}%"),
            anomalies=[a.description for a in anomalies] + reasons,
            key_signals={
                "上升时间(s)": f"{metrics.rise_time_s:.3f}",
                "超调量(%)": f"{metrics.overshoot_pct:.1f}",
                "调节时间(s)": f"{metrics.settling_time_s:.3f}",
                "稳态误差(%)": f"{metrics.steady_state_error_pct:.1f}",
                "峰值(RPM)": f"{metrics.peak_value:.1f}",
                "终值(RPM)": f"{metrics.final_value:.1f}",
            },
            improvement_suggestions=[a.improvement for a in anomalies],
            waveform_files=[plot_file] if plot_file else [],
        )
        report.add_record(record)

        if not passed:
            pytest.fail(f"速度响应不达标: {'; '.join(reasons)}")
