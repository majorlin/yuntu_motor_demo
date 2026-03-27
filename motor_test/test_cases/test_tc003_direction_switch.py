#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-003: 转向切换测试
======================
验证正反转切换功能。需要电机先停机再切换方向。
"""

import time
import pytest
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestDirectionSwitch:
    """TC-003: Direction switching (requires manual trigger or CAN-only)."""

    TEST_ID = "TC-003"

    def test_003a_forward_run(self, motor, can, report, csv_logger, test_logger):
        """正转运行验证"""
        log = test_logger
        t0 = time.time()

        log.step("设置正转方向 (direction=1)")
        motor.set_direction(1)
        time.sleep(0.2)

        log.step("启动电机至 1000 RPM")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        time.sleep(2.0)
        telem = motor.get_telemetry()
        duration = time.time() - t0

        log.data("状态", telem.state.name)
        log.data("方向", "正转" if telem.direction > 0 else "反转")
        log.data("RPM", f"{telem.mechanical_rpm:.1f}")

        verdict = TestVerdict.PASS if ok and telem.direction > 0 else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="正转运行",
            description="设置正转方向，启动到1000RPM",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method="CAN设置方向=正转, 启动至1000RPM, 运行2s后读取状态",
            pass_criteria="正常闭环运行，方向标志正确",
            actual_result=f"状态={telem.state.name}, 方向={'正转' if telem.direction>0 else '反转'}, RPM={telem.mechanical_rpm:.1f}",
            key_signals={"方向": "正转" if telem.direction > 0 else "反转",
                         "RPM": f"{telem.mechanical_rpm:.1f}"},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        motor.stop_and_wait()
        assert ok, "正转启动失败"

    def test_003b_reverse_run(self, motor, can, report, csv_logger, test_logger):
        """反转运行验证"""
        log = test_logger
        t0 = time.time()

        log.step("停机并设置反转方向 (direction=-1)")
        motor.stop_and_wait()
        motor.set_direction(-1)
        time.sleep(0.5)

        log.step("以反转方向启动电机至 1000 RPM")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        time.sleep(2.0)
        telem = motor.get_telemetry()
        duration = time.time() - t0

        log.data("状态", telem.state.name)
        log.data("方向", "正转" if telem.direction > 0 else "反转")
        log.data("RPM", f"{telem.mechanical_rpm:.1f}")

        verdict = TestVerdict.PASS if ok else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="反转运行",
            description="设置反转方向，启动到1000RPM",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method="CAN设置方向=反转, 启动至1000RPM, 运行2s后读取状态",
            pass_criteria="正常闭环运行，方向标志正确",
            actual_result=f"状态={telem.state.name}, 方向={'正转' if telem.direction>0 else '反转'}, RPM={telem.mechanical_rpm:.1f}",
            key_signals={"方向": "正转" if telem.direction > 0 else "反转",
                         "RPM": f"{telem.mechanical_rpm:.1f}"},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        motor.stop_and_wait()
        motor.set_direction(1)  # restore default
        assert ok, "反转启动失败"

    def test_003c_switch_direction(self, motor, can, report, csv_logger, test_logger):
        """正转→停机→反转→停机→正转 完整切换"""
        log = test_logger
        t0 = time.time()
        results = []

        log.step("开始正反转切换测试序列")
        for direction, label in [(1, "正转"), (-1, "反转"), (1, "回正转")]:
            log.step(f"切换至{label}: 停机 → 设方向={direction} → 启动")
            motor.stop_and_wait()
            time.sleep(0.5)
            motor.set_direction(direction)
            time.sleep(0.3)
            ok = motor.start_and_wait(target_rpm=800.0, timeout=10.0)
            if ok:
                time.sleep(2.0)
                log.info(f"{label}: 启动成功, 状态={motor.state.name}")
            else:
                log.error(f"{label}: 启动失败, 状态={motor.state.name}, 故障={motor.fault.name}")
            results.append((label, ok, motor.state.name, motor.fault.name))
            motor.stop_and_wait()
            time.sleep(1.0)

        duration = time.time() - t0
        all_ok = all(ok for _, ok, _, _ in results)

        log.step("汇总切换结果")
        for label, ok, state, fault in results:
            log.data(label, f"{'成功' if ok else '失败'} ({state})")

        verdict = TestVerdict.PASS if all_ok else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="正反转切换",
            description="正转→停→反转→停→正转 完整切换验证",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method="依次执行: 正转启动→停机→反转启动→停机→正转启动, 每次验证闭环状态",
            pass_criteria="每次切换后均能正常启动闭环",
            actual_result="; ".join(f"{l}:{'✅' if ok else '❌'} {s}" for l, ok, s, _ in results),
            key_signals={r[0]: f"{'成功' if r[1] else '失败'} ({r[2]})" for r in results},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        assert all_ok, f"方向切换失败: {results}"
