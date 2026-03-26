#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-006: 风检测/顺风捕获测试
==============================
验证 WIND_DETECT 和 COAST_DOWN 状态机行为。
需要手动旋转叶轮触发风检测。单独运行此测试。
"""

import time
import pytest
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestWindDetect:
    """TC-006: Wind detection and catch-spin (manual trigger required)."""

    TEST_ID = "TC-006"

    def test_006a_standstill_startup(self, motor, can, report, csv_logger):
        """静止状态启动: 风检测应快速判定静止并进入Align"""
        t0 = time.time()
        can.enable_history()

        ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()

        # Check that Wind Detect was traversed (via history)
        history = can.get_history()
        states_seen = set()
        for h in history:
            s = int(h.get("State", 0))
            states_seen.add(MotorState(s).name)

        wind_detect_seen = MotorState.WIND_DETECT.name in states_seen

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="静止风检测",
            description="电机静止时启动，验证风检测快速判定静止",
            verdict=TestVerdict.PASS if ok else TestVerdict.FAIL,
            duration_s=duration,
            preconditions="电机完全静止",
            test_method="启动电机，监控状态转换序列",
            pass_criteria="经过WIND_DETECT后正常进入闭环",
            actual_result=f"{'经过' if wind_detect_seen else '未经过'}WIND_DETECT, 最终={telem.state.name}",
            key_signals={
                "状态序列": ", ".join(sorted(states_seen)),
                "风检测": "经过" if wind_detect_seen else "跳过",
                "BEMF RPM": f"{can.get_signal('BemfDetectedRpm'):.0f}",
                "BEMF 过零": f"{can.get_signal('BemfCrossingCount'):.0f}",
            },
        )
        report.add_record(record)
        can.disable_history()

    def test_006b_wind_detect_bemf_signals(self, motor, can, waveform, report, csv_logger):
        """监控风检测阶段的BEMF信号质量"""
        t0 = time.time()

        # Capture BEMF signals during startup
        bemf_signals = [
            "BemfDetectedRpm", "BemfCrossingCount", "BemfPhaseSeq",
            "State", "MechanicalRpm",
        ]

        # Start motor and capture wind detect phase
        motor.start(target_rpm=1000.0)
        record_wf = waveform.capture(
            signal_names=bemf_signals,
            duration_s=5.0,
            interval_s=0.01,
            name="wind_detect_bemf",
        )

        # Wait for completion
        motor.wait_speed_settled(1000, tolerance_pct=15, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()

        plot_file = waveform.plot_waveform(
            record_wf, title="Wind Detect BEMF Signals",
            filename="tc006_bemf_signals.png",
        )

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="BEMF信号质量",
            description="监控风检测阶段BEMF采样和过零检测",
            verdict=TestVerdict.PASS if telem.state == MotorState.CLOSED_LOOP else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="BEMF信号有效，正常进入闭环",
            actual_result=f"状态={telem.state.name}",
            waveform_files=[plot_file] if plot_file else [],
        )
        report.add_record(record)

    @pytest.mark.skip(reason="需要手动旋转叶轮 — 请单独运行")
    def test_006c_tailwind_catch(self, motor, can, waveform, analyzer, report, csv_logger):
        """顺风捕获: 手动正向旋转叶轮后启动电机
        
        使用方法:
        1. 手动正向旋转叶轮
        2. 立即运行此测试
        """
        t0 = time.time()
        can.enable_history()

        print("⏳ 请在5秒内手动旋转叶轮...")
        time.sleep(5.0)

        ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()
        bemf_rpm = can.get_signal("BemfDetectedRpm")

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="顺风捕获",
            description="叶轮预旋转后启动，验证顺风检测和直接闭环捕获",
            verdict=TestVerdict.PASS if ok else TestVerdict.FAIL,
            duration_s=duration,
            preconditions="叶轮被手动旋转中",
            test_method="手动旋转叶轮→启动电机→观察BEMF检测和状态转换",
            pass_criteria="正确检测旋转方向和速度，成功进入闭环",
            actual_result=f"BEMF={bemf_rpm:.0f}RPM, 状态={telem.state.name}",
            key_signals={
                "BEMF检测RPM": f"{bemf_rpm:.0f}",
                "过零次数": f"{can.get_signal('BemfCrossingCount'):.0f}",
                "相序": f"{can.get_signal('BemfPhaseSeq'):.0f}",
            },
        )
        report.add_record(record)
        can.disable_history()
