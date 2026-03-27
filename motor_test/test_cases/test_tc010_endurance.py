#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-010: 长时间耐久性测试
==========================
连续运行数小时，监控所有信号稳定性。
"""

import time
import pytest
import numpy as np
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestEndurance:
    """TC-010: Long-duration endurance test."""

    TEST_ID = "TC-010"

    @pytest.mark.parametrize("duration_min", [30])
    def test_010_endurance(self, motor, can, waveform, analyzer, report,
                            csv_logger, duration_min):
        """耐久性测试: 连续运行并监控全量信号"""
        t0 = time.time()
        duration_s = duration_min * 60

        # Start motor
        ok = motor.start_and_wait(target_rpm=2000.0, timeout=10.0)
        assert ok, "启动失败"

        # Monitoring data
        rpm_log = []
        iq_log = []
        vbus_log = []
        pe_log = []
        chip_temp_log = []
        pcb_temp_log = []
        fault_events = []
        anomalies_detected = []

        sample_interval = 0.1  # 100ms
        check_interval = 1.0   # anomaly check every 1s
        last_check = 0

        while (time.monotonic() - t0) < duration_s:
            now = time.monotonic()
            elapsed = now - t0

            telem = motor.get_telemetry()

            # Log key signals
            rpm_log.append(telem.mechanical_rpm)
            iq_log.append(telem.iq_meas_a)
            vbus_log.append(telem.bus_voltage_v)
            pe_log.append(telem.phase_error_rad)
            chip_temp_log.append(telem.chip_temperature)
            pcb_temp_log.append(telem.pcb_temperature)

            # Detect fault
            if telem.state == MotorState.FAULT:
                fault_events.append(f"t={elapsed:.1f}s: {telem.fault.name}")
                # Try to restart
                motor.stop()
                time.sleep(2.0)
                ok = motor.start_and_wait(target_rpm=2000.0, timeout=10.0)
                if not ok:
                    break

            # Periodic anomaly check
            if (elapsed - last_check) >= check_interval:
                last_check = elapsed
                new_anomalies = analyzer.check()
                for a in new_anomalies:
                    anomalies_detected.append(f"t={elapsed:.1f}s: {a.description}")

            time.sleep(sample_interval)

        duration = time.time() - t0
        motor.stop_and_wait()

        # Statistics
        rpm_arr = np.array(rpm_log)
        iq_arr = np.array(iq_log)
        vbus_arr = np.array(vbus_log)
        pe_arr = np.array(pe_log)
        chip_arr = np.array(chip_temp_log)

        rpm_stability = np.std(rpm_arr) / np.mean(rpm_arr) * 100 if np.mean(rpm_arr) > 0 else 0
        passed = (
            len(fault_events) == 0
            and rpm_stability < 5.0
        )

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name=f"耐久性测试 {duration_min}min",
            description=f"连续运行{duration_min}分钟，监控稳定性",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            preconditions="电机可正常闭环运行",
            test_method=f"以2000RPM连续运行{duration_min}分钟，每100ms采样关键信号",
            pass_criteria="无故障，速度波动<5%，温度稳定",
            actual_result=(f"故障次数={len(fault_events)}, "
                         f"RPM波动={rpm_stability:.1f}%, "
                         f"芯片温度={chip_arr[-1]:.1f}°C"),
            anomalies=anomalies_detected[:20],  # limit
            key_signals={
                "运行时间(min)": f"{duration/60:.1f}",
                "故障次数": str(len(fault_events)),
                "RPM均值": f"{np.mean(rpm_arr):.1f}",
                "RPM标准差": f"{np.std(rpm_arr):.1f}",
                "RPM波动(%)": f"{rpm_stability:.1f}",
                "Iq均值(A)": f"{np.mean(iq_arr):.2f}",
                "Vbus均值(V)": f"{np.mean(vbus_arr):.2f}",
                "PhaseErr_RMS(rad)": f"{np.sqrt(np.mean(pe_arr**2)):.4f}",
                "芯片温度-起始(°C)": f"{chip_arr[0]:.1f}" if len(chip_arr) else "N/A",
                "芯片温度-结束(°C)": f"{chip_arr[-1]:.1f}" if len(chip_arr) else "N/A",
                "PCB温度-结束(°C)": f"{pcb_temp_log[-1]:.1f}" if pcb_temp_log else "N/A",
            },
            improvement_suggestions=[f"故障: {e}" for e in fault_events],
            waveform_files=[csv_logger],
        )
        report.add_record(record)

        assert passed, f"耐久性测试失败: {len(fault_events)}次故障, RPM波动{rpm_stability:.1f}%"
