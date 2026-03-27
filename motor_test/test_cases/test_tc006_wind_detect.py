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

    @pytest.fixture(autouse=True)
    def _fan_only(self, motor_profile):
        if motor_profile.get("motor_id") != "fan":
            pytest.skip("TC-006 仅包含在鼓风机 profile 中")

    def test_006a_standstill_startup(self, motor, can, report, csv_logger, test_logger):
        """静止状态启动: 风检测应快速判定静止并进入Align"""
        log = test_logger
        t0 = time.time()
        can.enable_history()

        log.step("从静止状态启动电机")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()

        log.step("分析状态转换序列")
        history = can.get_history()
        states_seen = set()
        for h in history:
            s = int(h.get("State", 0))
            states_seen.add(MotorState(s).name)

        wind_detect_seen = MotorState.WIND_DETECT.name in states_seen
        log.data("状态序列", ", ".join(sorted(states_seen)))
        log.data("风检测", "经过" if wind_detect_seen else "跳过")
        log.data("BEMF RPM", f"{can.get_signal('BemfDetectedRpm'):.0f}")

        verdict = TestVerdict.PASS if ok else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="静止风检测",
            description="电机静止时启动，验证风检测快速判定静止",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机完全静止",
            test_method="启动电机，通过CAN历史记录监控状态转换序列",
            pass_criteria="经过WIND_DETECT后正常进入闭环",
            actual_result=f"{'经过' if wind_detect_seen else '未经过'}WIND_DETECT, 最终={telem.state.name}",
            key_signals={
                "状态序列": ", ".join(sorted(states_seen)),
                "风检测": "经过" if wind_detect_seen else "跳过",
                "BEMF RPM": f"{can.get_signal('BemfDetectedRpm'):.0f}",
                "BEMF 过零": f"{can.get_signal('BemfCrossingCount'):.0f}",
            },
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        can.disable_history()

    def test_006b_wind_detect_bemf_signals(self, motor, can, waveform, report,
                                             csv_logger, test_logger):
        """监控风检测阶段的BEMF信号质量"""
        log = test_logger
        t0 = time.time()

        log.step("配置BEMF信号采集通道")
        bemf_signals = [
            "BemfDetectedRpm", "BemfCrossingCount", "BemfPhaseSeq",
            "State", "MechanicalRpm",
        ]

        log.step("启动电机并采集5秒风检测阶段波形")
        motor.start(target_rpm=1000.0)
        record_wf = waveform.capture(
            signal_names=bemf_signals,
            duration_s=5.0,
            interval_s=0.01,
            name="wind_detect_bemf",
        )
        log.info(f"采集完成, 共 {len(record_wf.time_s)} 个采样点")

        log.step("等待电机进入稳态")
        motor.wait_speed_settled(1000, tolerance_pct=15, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()

        log.step("生成波形图")
        plot_file = waveform.plot_waveform(
            record_wf, title="Wind Detect BEMF Signals",
            filename="tc006_bemf_signals.png",
        )

        verdict = TestVerdict.PASS if telem.state == MotorState.CLOSED_LOOP else TestVerdict.FAIL
        log.data("最终状态", telem.state.name)
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="BEMF信号质量",
            description="监控风检测阶段BEMF采样和过零检测",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止或低速旋转",
            test_method="启动电机，在风检测阶段以10ms间隔采集BEMF信号5秒",
            pass_criteria="BEMF信号有效，正常进入闭环",
            actual_result=f"状态={telem.state.name}",
            waveform_files=[plot_file] if plot_file else [],
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

    @pytest.mark.skip(reason="需要手动旋转叶轮 — 请单独运行")
    def test_006c_tailwind_catch(self, motor, can, waveform, analyzer, report,
                                  csv_logger, test_logger):
        """顺风捕获: 手动正向旋转叶轮后启动电机

        使用方法:
        1. 手动正向旋转叶轮
        2. 立即运行此测试
        """
        log = test_logger
        t0 = time.time()
        can.enable_history()

        log.step("等待手动旋转叶轮 (5秒)")
        log.info("⏳ 请在5秒内手动旋转叶轮...")
        print("⏳ 请在5秒内手动旋转叶轮...")
        time.sleep(5.0)

        log.step("启动电机，验证顺风检测")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
        duration = time.time() - t0
        telem = motor.get_telemetry()
        bemf_rpm = can.get_signal("BemfDetectedRpm")

        log.data("BEMF检测RPM", f"{bemf_rpm:.0f}")
        log.data("最终状态", telem.state.name)

        verdict = TestVerdict.PASS if ok else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="顺风捕获",
            description="叶轮预旋转后启动，验证顺风检测和直接闭环捕获",
            verdict=verdict,
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
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        can.disable_history()
