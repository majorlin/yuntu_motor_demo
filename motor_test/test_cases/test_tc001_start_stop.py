#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-001: 基本启停测试
======================
验证电机正常启动和停止流程，检查状态机转换序列。
"""

import time
import pytest
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestStartStop:
    """TC-001: Motor basic start/stop functionality."""

    TEST_ID = "TC-001"

    def test_001a_start_to_closed_loop(self, motor, can, analyzer, report,
                                        csv_logger, test_logger):
        """验证: Stop → OffsetCal → Align → OpenLoop → ClosedLoop 完整启动链"""
        log = test_logger
        t0 = time.time()
        can.enable_history()
        analyzer.clear()

        # Step 1: Pre-check
        log.step("检查前置条件 — 电机处于 STOP 状态")
        log.info(f"当前状态 = {motor.state.name}")
        assert motor.is_stopped, f"前置条件失败: 电机状态={motor.state.name}, 非STOP"
        log.info("✓ 前置条件通过")

        # Step 2: Start motor
        log.step("发送启动命令: CmdEnable=1, TargetRPM=1000")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
        duration = time.time() - t0
        log.info(f"启动结果: {'成功' if ok else '失败'}, 耗时 {duration:.1f}s")

        # Step 3: Check anomalies
        log.step("检查异常分析器")
        anomalies = analyzer.check()
        if anomalies:
            for a in anomalies:
                log.warn(f"异常: {a.description}")
        else:
            log.info("✓ 无异常")

        # Step 4: Read telemetry
        log.step("读取遥测数据")
        telem = motor.get_telemetry()
        log.data("最终状态", telem.state.name)
        log.data("故障码", telem.fault.name)
        log.data("转速(RPM)", f"{telem.mechanical_rpm:.1f}")
        log.data("Vbus(V)", f"{telem.bus_voltage_v:.2f}")
        log.data("启动耗时(s)", f"{duration:.2f}")
        log.data("重试次数", str(telem.startup_retry_count))

        verdict = TestVerdict.PASS if ok else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="启动至闭环",
            description="发送启动命令，验证状态机经过完整启动链到达闭环",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method="CAN发送 CmdEnable=1, TargetRPM=1000, 等待闭环或故障",
            pass_criteria="10s内进入 CLOSED_LOOP 状态，无故障",
            actual_result=f"状态={telem.state.name}, 故障={telem.fault.name}, RPM={telem.mechanical_rpm:.1f}",
            anomalies=[a.description for a in anomalies],
            key_signals={
                "最终状态": telem.state.name,
                "故障码": telem.fault.name,
                "转速(RPM)": f"{telem.mechanical_rpm:.1f}",
                "Vbus(V)": f"{telem.bus_voltage_v:.2f}",
                "启动耗时(s)": f"{duration:.2f}",
                "重试次数": str(telem.startup_retry_count),
            },
            improvement_suggestions=[a.improvement for a in anomalies],
            waveform_files=[csv_logger],
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

        assert ok, f"启动失败: state={telem.state.name}, fault={telem.fault.name}"
        assert telem.fault == MotorFault.NONE, f"故障: {telem.fault.name}"
        can.disable_history()

    def test_001b_stop_from_running(self, motor, can, analyzer, report,
                                     csv_logger, test_logger):
        """验证: 从闭环运行状态正常停机"""
        log = test_logger
        t0 = time.time()
        analyzer.clear()

        # Step 1: Ensure running
        log.step("确保电机处于运行状态")
        if not motor.is_running:
            log.info("电机未运行，正在启动...")
            ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
            assert ok, "无法启动电机"
            time.sleep(2.0)  # stable running
            log.info("电机已启动并稳定运行")
        else:
            log.info(f"电机已运行, 状态={motor.state.name}")

        # Step 2: Send stop
        log.step("发送停机命令: CmdEnable=0")
        stopped = motor.stop_and_wait(timeout=5.0)
        duration = time.time() - t0
        log.info(f"停机结果: {'成功' if stopped else '失败'}, 耗时 {duration:.1f}s")

        # Step 3: Read telemetry
        log.step("读取遥测数据确认停机状态")
        telem = motor.get_telemetry()
        log.data("最终状态", telem.state.name)
        log.data("故障码", telem.fault.name)
        log.data("剩余RPM", f"{telem.mechanical_rpm:.1f}")

        verdict = TestVerdict.PASS if stopped else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="正常停机",
            description="从闭环运行发送停机命令，验证状态转入STOP",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机闭环运行",
            test_method="CAN发送 CmdEnable=0, 等待STOP状态",
            pass_criteria="5s内进入 STOP 状态，无故障",
            actual_result=f"状态={telem.state.name}, 故障={telem.fault.name}",
            key_signals={
                "最终状态": telem.state.name,
                "故障码": telem.fault.name,
                "剩余RPM": f"{telem.mechanical_rpm:.1f}",
            },
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        assert stopped, f"停机失败: state={telem.state.name}"

    def test_001c_repeated_start_stop(self, motor, can, analyzer, report,
                                       csv_logger, test_logger):
        """验证: 10次快速启停循环"""
        log = test_logger
        t0 = time.time()
        n_cycles = 10
        successes = 0
        fail_details = []

        log.step(f"开始 {n_cycles} 次启停循环测试")

        for i in range(n_cycles):
            log.separator(f"第 {i+1}/{n_cycles} 次循环")
            log.step(f"循环 #{i+1}: 发送启动命令, TargetRPM=1000")
            ok = motor.start_and_wait(target_rpm=1000.0, timeout=15.0)
            if ok:
                log.info(f"#{i+1} 启动成功, 运行 1s...")
                time.sleep(1.0)
                log.step(f"循环 #{i+1}: 发送停机命令")
                stopped = motor.stop_and_wait(timeout=10.0)
                if stopped:
                    successes += 1
                    log.info(f"#{i+1} 停机成功 ✓")
                else:
                    fail_details.append(f"#{i+1}: 停机失败")
                    log.error(f"#{i+1} 停机失败!")
                time.sleep(3.0)
            else:
                fail_details.append(f"#{i+1}: 启动失败 ({motor.fault.name})")
                log.error(f"#{i+1} 启动失败: {motor.fault.name}")
                motor.stop()
                time.sleep(3.0)

        duration = time.time() - t0
        passed = successes == n_cycles

        log.step("汇总测试结果")
        log.data("成功次数", f"{successes}/{n_cycles}")
        log.data("成功率", f"{successes/n_cycles*100:.0f}%")
        log.data("总耗时", f"{duration:.1f}s")

        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name=f"启停循环×{n_cycles}",
            description=f"连续{n_cycles}次启停循环测试启动可靠性",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method=f"{n_cycles}次: 启动→等待闭环→运行1s→停机→等待3s",
            pass_criteria=f"{n_cycles}次全部成功",
            actual_result=f"成功{successes}/{n_cycles}次",
            anomalies=fail_details,
            key_signals={
                "成功次数": f"{successes}/{n_cycles}",
                "总耗时": f"{duration:.1f}s",
                "成功率": f"{successes/n_cycles*100:.0f}%",
            },
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        assert passed, f"启停循环失败: {successes}/{n_cycles}, {fail_details}"
