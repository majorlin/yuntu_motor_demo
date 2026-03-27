#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-007: 弱磁控制测试
======================
验证高速运行时弱磁控制的介入和退出。
"""

import time
import pytest
import numpy as np
from motor_client import MotorState
from report_generator import TestRecord, TestVerdict


class TestFieldWeakening:
    """TC-007: Field weakening control verification."""

    TEST_ID = "TC-007"

    @pytest.fixture(autouse=True)
    def _fan_only(self, motor_profile):
        if motor_profile.get("motor_id") != "fan":
            pytest.skip("TC-007 仅包含在鼓风机 profile 中")

    def test_007a_field_weakening_entry(self, motor, can, waveform, report,
                                         csv_logger, test_logger):
        """高速弱磁: 逐步升速至弱磁区域，观察Id注入"""
        log = test_logger
        t0 = time.time()

        log.step("启动电机至 1000 RPM")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        assert ok, "启动失败"

        log.step("逐步升速至弱磁区域")
        rpm_points = [1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000]
        fw_data = []

        for target in rpm_points:
            log.info(f"升速至 {target} RPM...")
            motor.set_speed(target, ramp_rpm_s=600)
            time.sleep(3.0)

            telem = motor.get_telemetry()
            if telem.state != MotorState.CLOSED_LOOP:
                log.warn(f"电机退出闭环 @ {target} RPM, 状态={telem.state.name}")
                break

            fw_id = can.get_signal("FwIdTarget")
            log.data(f"RPM={target}", f"ModRatio={telem.volt_mod_ratio:.4f}, FwId={fw_id:.3f}A")
            fw_data.append({
                "RPM": telem.mechanical_rpm,
                "TargetRPM": target,
                "VoltModRatio": telem.volt_mod_ratio,
                "FwIdTarget": fw_id,
                "BusVoltage": telem.bus_voltage_v,
                "IqMeas": telem.iq_meas_a,
            })

        duration = time.time() - t0

        log.step("采集高速运行波形")
        record_wf = waveform.capture(
            ["MechanicalRpm", "VoltModRatio", "FwIdTarget", "IqMeas", "BusVoltage"],
            duration_s=3.0, interval_s=0.01, name="field_weakening",
        )
        plot_file = waveform.plot_waveform(
            record_wf, title="Field Weakening Behavior",
            filename="tc007_field_weakening.png",
        )

        fw_engaged = any(d["FwIdTarget"] < -0.1 for d in fw_data)
        max_mod = max(d["VoltModRatio"] for d in fw_data) if fw_data else 0
        telem = motor.get_telemetry()

        log.data("弱磁状态", "已介入" if fw_engaged else "未介入")
        log.data("最大调制比", f"{max_mod:.4f}")

        verdict = TestVerdict.PASS if fw_engaged or max_mod < 0.5 else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="弱磁控制验证",
            description="逐步升速，验证弱磁控制在电压饱和时介入",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机闭环运行",
            test_method="从1000RPM逐步升至8000RPM (步长1000, 每步3s)，监控VoltModRatio和FwIdTarget",
            pass_criteria="VoltModRatio超过阈值时FwIdTarget出现负值",
            actual_result=f"弱磁{'已介入' if fw_engaged else '未介入'}, MaxMod={max_mod:.4f}",
            key_signals={
                "弱磁状态": "已介入" if fw_engaged else "未介入",
                "最大调制比": f"{max_mod:.4f}",
                "最高RPM": f"{max(d['RPM'] for d in fw_data):.0f}" if fw_data else "N/A",
            },
            waveform_files=[plot_file] if plot_file else [],
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

    def test_007b_field_weakening_exit(self, motor, can, waveform, report,
                                        csv_logger, test_logger):
        """弱磁退出: 从高速减速后弱磁Id应恢复为0"""
        log = test_logger
        t0 = time.time()

        log.step("启动电机至高速 6000 RPM")
        ok = motor.start_and_wait(target_rpm=6000.0, timeout=15.0)
        if not ok:
            pytest.skip("无法达到高速")
        time.sleep(3.0)
        log.info("高速运行稳定")

        log.step("阶跃减速至 1000 RPM, 采集弱磁退出波形")
        record_wf, metrics = waveform.capture_step_response(
            signal_name="FwIdTarget",
            step_trigger=lambda: motor.set_speed(1000.0),
            pre_s=1.0, post_s=5.0, interval_s=0.01,
            extra_signals=["MechanicalRpm", "VoltModRatio"],
        )

        duration = time.time() - t0
        time.sleep(2.0)
        fw_id = can.get_signal("FwIdTarget")
        log.data("FwIdTarget(A)", f"{fw_id:.3f}")

        verdict = TestVerdict.PASS if abs(fw_id) < 0.5 else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="弱磁退出",
            description="从高速减速，验证弱磁Id恢复到0",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机在6000RPM高速弱磁运行",
            test_method="从6000RPM阶跃减速至1000RPM, 采集FwIdTarget变化",
            pass_criteria="减速后FwIdTarget → 0",
            actual_result=f"FwIdTarget={fw_id:.3f}A",
            key_signals={"FwIdTarget(A)": f"{fw_id:.3f}"},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
