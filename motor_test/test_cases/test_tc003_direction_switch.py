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

    def test_003a_forward_run(self, motor, can, report, csv_logger):
        """正转运行验证"""
        t0 = time.time()
        motor.set_direction(1)
        time.sleep(0.2)
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        time.sleep(2.0)
        telem = motor.get_telemetry()
        duration = time.time() - t0

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="正转运行",
            description="设置正转方向，启动到1000RPM",
            verdict=TestVerdict.PASS if ok and telem.direction > 0 else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="正常闭环运行，方向标志正确",
            actual_result=f"状态={telem.state.name}, 方向={'正转' if telem.direction>0 else '反转'}, RPM={telem.mechanical_rpm:.1f}",
            key_signals={"方向": "正转" if telem.direction > 0 else "反转",
                         "RPM": f"{telem.mechanical_rpm:.1f}"},
        )
        report.add_record(record)
        motor.stop_and_wait()
        assert ok, "正转启动失败"

    def test_003b_reverse_run(self, motor, can, report, csv_logger):
        """反转运行验证"""
        t0 = time.time()
        motor.stop_and_wait()
        motor.set_direction(-1)
        time.sleep(0.5)
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        time.sleep(2.0)
        telem = motor.get_telemetry()
        duration = time.time() - t0

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="反转运行",
            description="设置反转方向，启动到1000RPM",
            verdict=TestVerdict.PASS if ok else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="正常闭环运行，方向标志正确",
            actual_result=f"状态={telem.state.name}, 方向={'正转' if telem.direction>0 else '反转'}, RPM={telem.mechanical_rpm:.1f}",
            key_signals={"方向": "正转" if telem.direction > 0 else "反转",
                         "RPM": f"{telem.mechanical_rpm:.1f}"},
        )
        report.add_record(record)
        motor.stop_and_wait()
        motor.set_direction(1)  # restore default
        assert ok, "反转启动失败"

    def test_003c_switch_direction(self, motor, can, report, csv_logger):
        """正转→停机→反转→停机→正转 完整切换"""
        t0 = time.time()
        results = []

        for direction, label in [(1, "正转"), (-1, "反转"), (1, "回正转")]:
            motor.stop_and_wait()
            time.sleep(0.5)
            motor.set_direction(direction)
            time.sleep(0.3)
            ok = motor.start_and_wait(target_rpm=800.0, timeout=10.0)
            if ok:
                time.sleep(2.0)
            results.append((label, ok, motor.state.name, motor.fault.name))
            motor.stop_and_wait()
            time.sleep(1.0)

        duration = time.time() - t0
        all_ok = all(ok for _, ok, _, _ in results)

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="正反转切换",
            description="正转→停→反转→停→正转 完整切换验证",
            verdict=TestVerdict.PASS if all_ok else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="每次切换后均能正常启动闭环",
            actual_result="; ".join(f"{l}:{'✅' if ok else '❌'} {s}" for l, ok, s, _ in results),
            key_signals={r[0]: f"{'成功' if r[1] else '失败'} ({r[2]})" for r in results},
        )
        report.add_record(record)
        assert all_ok, f"方向切换失败: {results}"
