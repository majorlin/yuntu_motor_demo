#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-004: 保护功能测试
======================
验证故障检测和保护响应（过流、过欠压、观测器失锁）。
部分测试需要手动操作电源。
"""

import time
import pytest
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestProtection:
    """TC-004: Protection and fault detection."""

    TEST_ID = "TC-004"

    def test_004a_fault_state_recovery(self, motor, can, report, csv_logger, test_logger):
        """验证: FAULT状态下发Stop可以清除故障恢复到STOP"""
        log = test_logger
        t0 = time.time()

        log.step("尝试触发故障状态")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        if not ok and motor.is_faulted:
            log.info(f"电机已处于故障状态: {motor.fault.name}")
        elif ok:
            log.info("电机启动成功, 尝试写入极端参数触发观测器失锁")
            motor.write_calib(observer_gain=100.0, pll_kp=1.0, apply=True)
            time.sleep(3.0)

        log.step("发送 Stop 命令清除故障")
        motor.stop()
        time.sleep(1.0)
        stopped = motor.is_stopped
        duration = time.time() - t0
        telem = motor.get_telemetry()
        log.data("最终状态", telem.state.name)
        log.data("故障码", telem.fault.name)
        log.info(f"恢复结果: {'成功' if stopped else '失败'}")

        log.step("恢复正常参数")
        motor.write_calib(observer_gain=1.8e6, pll_kp=120.0, pll_ki=6000.0, apply=True)
        log.info("✓ 参数已恢复")

        verdict = TestVerdict.PASS if stopped else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="故障恢复",
            description="从FAULT状态发送Stop，验证能恢复到STOP",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机处于 FAULT 状态（通过极端参数触发观测器失锁）",
            test_method="1. 启动电机 2. 写入极端参数触发故障 3. 发送Stop 4. 验证恢复到STOP 5. 恢复正常参数",
            pass_criteria="Stop命令后恢复到STOP状态",
            actual_result=f"状态={telem.state.name}, 故障={telem.fault.name}",
            key_signals={"状态": telem.state.name, "故障码": telem.fault.name},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

    def test_004b_overcurrent_monitoring(self, motor, can, analyzer, report,
                                          csv_logger, test_logger):
        """监控过流保护: 在正常运行中检查电流是否在安全范围"""
        log = test_logger
        t0 = time.time()

        log.step("启动电机至 2000 RPM")
        ok = motor.start_and_wait(target_rpm=2000.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")

        log.step("持续监控 Iq 电流 5 秒")
        max_iq = 0.0
        for i in range(500):
            iq = abs(can.get_signal("IqMeas"))
            max_iq = max(max_iq, iq)
            if iq > 16.0:
                log.error(f"过流! Iq={iq:.2f}A > 16A, 采样点 #{i}")
                break
            time.sleep(0.01)

        duration = time.time() - t0
        telem = motor.get_telemetry()
        passed = max_iq < 16.0 and telem.fault != MotorFault.OVERCURRENT

        log.data("最大Iq(A)", f"{max_iq:.2f}")
        log.data("故障码", telem.fault.name)
        log.info(f"过流检测: {'安全 ✓' if passed else '过流 ✗'}")

        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="过流监控",
            description="正常运行中监控Iq电流是否在安全范围",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机运行在 2000 RPM",
            test_method="每10ms采样Iq电流，持续5秒，检测是否超过16A阈值",
            pass_criteria="Iq < 16A, 无过流故障",
            actual_result=f"最大Iq={max_iq:.2f}A, 故障={telem.fault.name}",
            key_signals={"最大Iq(A)": f"{max_iq:.2f}", "故障": telem.fault.name},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

    def test_004c_bus_voltage_monitoring(self, motor, can, report, csv_logger, test_logger):
        """监控母线电压是否在正常范围"""
        log = test_logger
        t0 = time.time()

        log.step("启动电机至 1000 RPM")
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")

        log.step("持续监控 Vbus 电压 3 秒")
        vbus_min = 999.0
        vbus_max = 0.0
        for _ in range(300):
            v = can.get_signal("BusVoltage")
            if v > 0:
                vbus_min = min(vbus_min, v)
                vbus_max = max(vbus_max, v)
            time.sleep(0.01)

        duration = time.time() - t0
        telem = motor.get_telemetry()
        passed = 7.0 < vbus_min and vbus_max < 18.0

        log.data("Vbus_min(V)", f"{vbus_min:.2f}")
        log.data("Vbus_max(V)", f"{vbus_max:.2f}")
        log.info(f"电压范围: {vbus_min:.2f}~{vbus_max:.2f}V, {'正常 ✓' if passed else '异常 ✗'}")

        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="母线电压监控",
            description="运行中监控Vbus电压范围",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机运行在 1000 RPM",
            test_method="每10ms采样BusVoltage，持续3秒，记录最大/最小值",
            pass_criteria="7V < Vbus < 18V",
            actual_result=f"Vbus范围: {vbus_min:.2f}~{vbus_max:.2f}V",
            key_signals={"Vbus_min(V)": f"{vbus_min:.2f}", "Vbus_max(V)": f"{vbus_max:.2f}"},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

    def test_004d_observer_phase_error(self, motor, can, report, csv_logger, test_logger):
        """监控观测器相位误差是否在健康范围"""
        log = test_logger
        t0 = time.time()

        log.step("启动电机至 1500 RPM")
        ok = motor.start_and_wait(target_rpm=1500.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")
        time.sleep(2.0)
        log.info("等待 2s 进入稳态")

        log.step("持续监控 PhaseError 2 秒")
        pe_max = 0.0
        pe_values = []
        for _ in range(200):
            pe = abs(can.get_signal("PhaseErrorRad"))
            pe_max = max(pe_max, pe)
            pe_values.append(pe)
            time.sleep(0.01)

        import numpy as np
        pe_rms = float(np.sqrt(np.mean(np.array(pe_values)**2)))
        duration = time.time() - t0
        passed = pe_max < 1.2

        log.data("PE_max(rad)", f"{pe_max:.4f}")
        log.data("PE_RMS(rad)", f"{pe_rms:.4f}")
        log.info(f"相位误差: max={pe_max:.4f}, RMS={pe_rms:.4f}, {'健康 ✓' if passed else '异常 ✗'}")

        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=f"{self.TEST_ID}d",
            test_name="观测器相位误差",
            description="稳态运行中监控PhaseError是否在健康范围",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机稳定运行在 1500 RPM (已运行 2s 进入稳态)",
            test_method="每10ms采样PhaseErrorRad，持续2秒，计算最大值和RMS",
            pass_criteria="PhaseError < 1.2 rad",
            actual_result=f"最大={pe_max:.4f}rad, RMS={pe_rms:.4f}rad",
            key_signals={"PE_max(rad)": f"{pe_max:.4f}", "PE_RMS(rad)": f"{pe_rms:.4f}"},
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
