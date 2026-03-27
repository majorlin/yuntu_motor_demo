#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-002: 速度响应测试
======================
验证速度环阶跃响应，评估上升时间、超调、稳态误差。
测试参数从 motor profile 中读取，自动适配不同电机。
"""

import time
import pytest
from motor_client import MotorState
from motor_profiles import get_speed_step_pairs, get_test_param
from report_generator import TestRecord, TestVerdict


class TestSpeedResponse:
    """TC-002: Speed step response characterization."""

    TEST_ID = "TC-002"

    def test_002_speed_step(self, motor, can, waveform, analyzer, report,
                             csv_logger, motor_profile, request, test_logger):
        """速度阶跃响应: 根据 motor profile 中的 speed_step_pairs 逐对测试"""
        log = test_logger
        step_pairs = get_speed_step_pairs(motor_profile)
        overshoot_limit = get_test_param(motor_profile, "overshoot_limit_pct", 20.0)
        rise_time_limit = get_test_param(motor_profile, "rise_time_limit_s", 3.0)
        settle_error_limit = get_test_param(motor_profile, "settle_error_limit_pct", 10.0)

        # ── Profile-driven timing ──
        start_timeout = get_test_param(motor_profile, "start_timeout_s", 10.0)
        settle_timeout = get_test_param(motor_profile, "settle_timeout_s", 8.0)
        capture_pre = get_test_param(motor_profile, "capture_pre_s", 1.0)
        capture_post = get_test_param(motor_profile, "capture_post_s", 6.0)
        capture_interval = get_test_param(motor_profile, "capture_interval_s", 0.01)
        stop_timeout = get_test_param(motor_profile, "stop_timeout_s", 5.0)
        inter_delay = get_test_param(motor_profile, "inter_pair_delay_s", 1.0)

        log.step("加载测试参数")
        log.info(f"速度阶跃对: {step_pairs}")
        log.info(f"超调限值: {overshoot_limit}%, 上升时间限值: {rise_time_limit}s, 稳态误差限值: {settle_error_limit}%")

        all_passed = True
        all_reasons = []

        for pair_idx, (from_rpm, to_rpm) in enumerate(step_pairs):
            log.separator(f"阶跃 {from_rpm}→{to_rpm} RPM")
            t0 = time.time()
            analyzer.clear()

            # Start and settle at from_rpm
            log.step(f"启动电机至 {from_rpm} RPM 并等待稳定")
            ok = motor.start_and_wait(target_rpm=from_rpm, timeout=start_timeout)
            assert ok, f"启动失败 (目标={from_rpm}RPM)"
            motor.wait_speed_settled(from_rpm, tolerance_pct=10, timeout=settle_timeout)
            log.info(f"电机已稳定在 {from_rpm} RPM")

            # Verify motor is in closed-loop before step
            log.step("确认电机处于闭环状态")
            assert motor.state == MotorState.CLOSED_LOOP, \
                f"电机未进入闭环 (state={motor.state.name})"
            log.info(f"✓ 状态 = {motor.state.name}")

            # Capture step response
            log.step(f"施加阶跃: {from_rpm}→{to_rpm} RPM, 采集 {capture_post}s 数据")
            expected_step = to_rpm - from_rpm
            record_wf, metrics = waveform.capture_step_response(
                signal_name="MechanicalRpm",
                step_trigger=lambda rpm=to_rpm: motor.set_speed(rpm),
                pre_s=capture_pre,
                post_s=capture_post,
                interval_s=capture_interval,
                extra_signals=["IqMeas", "SpeedPiIntegrator", "BusVoltage", "PhaseErrorRad"],
                expected_step=expected_step,
            )
            log.info(f"数据采集完成, 共 {len(record_wf.time_s)} 个采样点")

            # Post-step state check: detect stall / fault
            post_state = motor.state
            if post_state != MotorState.CLOSED_LOOP:
                metrics.motor_stalled = True
                log.warn(f"电机失步! 状态={post_state.name}")

            # Plot
            log.step("生成波形图")
            plot_file = waveform.plot_step_response(
                record_wf, metrics, "MechanicalRpm",
                filename=f"tc002_step_{from_rpm}_{to_rpm}.png",
            )
            if plot_file:
                log.info(f"波形图已保存: {plot_file}")

            # Check anomalies
            anomalies = analyzer.check()
            duration = time.time() - t0

            # Verdict
            passed = True
            reasons = []

            if metrics.motor_stalled:
                passed = False
                reasons.append(f"电机失步/停转 (state={post_state.name}, "
                              f"实际终值={metrics.final_value:.0f}RPM)")
            else:
                is_accel = to_rpm > from_rpm
                if is_accel:
                    if metrics.rise_time_s > rise_time_limit:
                        passed = False
                        reasons.append(f"上升时间 {metrics.rise_time_s:.2f}s > {rise_time_limit}s")
                    if metrics.overshoot_pct > overshoot_limit:
                        passed = False
                        reasons.append(f"超调 {metrics.overshoot_pct:.1f}% > {overshoot_limit}%")
                if metrics.steady_state_error_pct > settle_error_limit:
                    passed = False
                    reasons.append(f"稳态误差 {metrics.steady_state_error_pct:.1f}% > {settle_error_limit}%")

            log.step("分析测试结果")
            log.data("上升时间(s)", f"{metrics.rise_time_s:.3f}")
            log.data("超调量(%)", f"{metrics.overshoot_pct:.1f}")
            log.data("调节时间(s)", f"{metrics.settling_time_s:.3f}")
            log.data("稳态误差(%)", f"{metrics.steady_state_error_pct:.1f}")
            log.data("峰值(RPM)", f"{metrics.peak_value:.1f}")
            log.data("终值(RPM)", f"{metrics.final_value:.1f}")

            if not passed:
                all_passed = False
                all_reasons.extend([f"[{from_rpm}→{to_rpm}] {r}" for r in reasons])
                for r in reasons:
                    log.warn(r)

            telem = motor.get_telemetry()
            record = TestRecord(
                test_id=f"{self.TEST_ID}",
                test_name=f"速度阶跃 {from_rpm}→{to_rpm} RPM",
                description=f"从{from_rpm}RPM阶跃到{to_rpm}RPM，分析响应品质",
                verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
                duration_s=duration,
                preconditions=f"电机稳定运行在{from_rpm}RPM",
                test_method=f"CAN设定目标{to_rpm}RPM，采集{capture_post}s响应数据",
                pass_criteria=f"上升时间≤{rise_time_limit}s，超调≤{overshoot_limit}%，稳态误差≤{settle_error_limit}%",
                actual_result=(f"上升时间={metrics.rise_time_s:.2f}s, "
                              f"超调={metrics.overshoot_pct:.1f}%, "
                              f"稳态误差={metrics.steady_state_error_pct:.1f}%"
                              + (f", 电机失步!" if metrics.motor_stalled else "")),
                anomalies=[a.description for a in anomalies] + reasons,
                key_signals={
                    "上升时间(s)": f"{metrics.rise_time_s:.3f}",
                    "超调量(%)": f"{metrics.overshoot_pct:.1f}",
                    "调节时间(s)": f"{metrics.settling_time_s:.3f}",
                    "稳态误差(%)": f"{metrics.steady_state_error_pct:.1f}",
                    "峰值(RPM)": f"{metrics.peak_value:.1f}",
                    "终值(RPM)": f"{metrics.final_value:.1f}",
                    "电机状态": post_state.name if 'post_state' in dir() else "N/A",
                },
                improvement_suggestions=[a.improvement for a in anomalies],
                waveform_files=[plot_file] if plot_file else [],
                test_steps=log.get_steps(),
                test_logs=log.get_logs(),
            )
            report.add_record(record)

            # Stop and wait before next pair
            log.step("停机并等待下一组测试")
            motor.stop_and_wait(timeout=stop_timeout)
            time.sleep(inter_delay)

        if not all_passed:
            pytest.fail(f"速度响应不达标: {'; '.join(all_reasons)}")
