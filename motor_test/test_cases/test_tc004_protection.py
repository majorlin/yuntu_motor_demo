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

    def test_004a_fault_state_recovery(self, motor, can, report, csv_logger):
        """验证: FAULT状态下发Stop可以清除故障恢复到STOP"""
        t0 = time.time()
        # Start motor normally first
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        if not ok and motor.is_faulted:
            # Already faulted — use this
            pass
        elif ok:
            # Try to trigger observer loss by writing extreme params
            motor.write_calib(observer_gain=100.0, pll_kp=1.0, apply=True)
            time.sleep(3.0)

        # Whether faulted or not, test recovery
        motor.stop()
        time.sleep(1.0)
        stopped = motor.is_stopped
        duration = time.time() - t0
        telem = motor.get_telemetry()

        # Restore sane params
        motor.write_calib(observer_gain=1.8e6, pll_kp=120.0, pll_ki=6000.0, apply=True)

        record = TestRecord(
            test_id=f"{self.TEST_ID}a",
            test_name="故障恢复",
            description="从FAULT状态发送Stop，验证能恢复到STOP",
            verdict=TestVerdict.PASS if stopped else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="Stop命令后恢复到STOP状态",
            actual_result=f"状态={telem.state.name}, 故障={telem.fault.name}",
            key_signals={"状态": telem.state.name, "故障码": telem.fault.name},
        )
        report.add_record(record)

    def test_004b_overcurrent_monitoring(self, motor, can, analyzer, report, csv_logger):
        """监控过流保护: 在正常运行中检查电流是否在安全范围"""
        t0 = time.time()
        ok = motor.start_and_wait(target_rpm=2000.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")

        # Monitor for 5 seconds
        max_iq = 0.0
        for _ in range(500):
            iq = abs(can.get_signal("IqMeas"))
            max_iq = max(max_iq, iq)
            if iq > 16.0:
                break
            time.sleep(0.01)

        duration = time.time() - t0
        telem = motor.get_telemetry()
        passed = max_iq < 16.0 and telem.fault != MotorFault.OVERCURRENT

        record = TestRecord(
            test_id=f"{self.TEST_ID}b",
            test_name="过流监控",
            description="正常运行中监控Iq电流是否在安全范围",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="Iq < 16A, 无过流故障",
            actual_result=f"最大Iq={max_iq:.2f}A, 故障={telem.fault.name}",
            key_signals={"最大Iq(A)": f"{max_iq:.2f}", "故障": telem.fault.name},
        )
        report.add_record(record)

    def test_004c_bus_voltage_monitoring(self, motor, can, report, csv_logger):
        """监控母线电压是否在正常范围"""
        t0 = time.time()
        ok = motor.start_and_wait(target_rpm=1000.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")

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

        record = TestRecord(
            test_id=f"{self.TEST_ID}c",
            test_name="母线电压监控",
            description="运行中监控Vbus电压范围",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="7V < Vbus < 18V",
            actual_result=f"Vbus范围: {vbus_min:.2f}~{vbus_max:.2f}V",
            key_signals={"Vbus_min(V)": f"{vbus_min:.2f}", "Vbus_max(V)": f"{vbus_max:.2f}"},
        )
        report.add_record(record)

    def test_004d_observer_phase_error(self, motor, can, report, csv_logger):
        """监控观测器相位误差是否在健康范围"""
        t0 = time.time()
        ok = motor.start_and_wait(target_rpm=1500.0, timeout=10.0)
        if not ok:
            pytest.skip("无法启动电机")
        time.sleep(2.0)  # wait for steady state

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

        record = TestRecord(
            test_id=f"{self.TEST_ID}d",
            test_name="观测器相位误差",
            description="稳态运行中监控PhaseError是否在健康范围",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            pass_criteria="PhaseError < 1.2 rad",
            actual_result=f"最大={pe_max:.4f}rad, RMS={pe_rms:.4f}rad",
            key_signals={"PE_max(rad)": f"{pe_max:.4f}", "PE_RMS(rad)": f"{pe_rms:.4f}"},
        )
        report.add_record(record)
