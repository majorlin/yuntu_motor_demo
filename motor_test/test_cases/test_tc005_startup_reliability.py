#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-005: 启动可靠性测试
========================
大量重复启动，统计成功率和重试次数分布。
"""

import time
import pytest
import numpy as np
from motor_client import MotorState, MotorFault
from report_generator import TestRecord, TestVerdict


class TestStartupReliability:
    """TC-005: Startup reliability stress test."""

    TEST_ID = "TC-005"

    @pytest.mark.parametrize("n_cycles", [50])
    def test_005_startup_reliability(self, motor, can, waveform, analyzer, report,
                                      csv_logger, n_cycles):
        """重复启动测试: n_cycles次启停循环统计"""
        t0 = time.time()
        can.enable_history()

        successes = 0
        retries_list = []
        fault_counts = {}
        startup_times = []
        fail_details = []

        for i in range(n_cycles):
            iter_t0 = time.monotonic()

            ok = motor.start_and_wait(target_rpm=1000.0, timeout=12.0)
            startup_time = time.monotonic() - iter_t0

            telem = motor.get_telemetry()
            retries = telem.startup_retry_count

            if ok:
                successes += 1
                retries_list.append(retries)
                startup_times.append(startup_time)
                time.sleep(1.0)
            else:
                fault_name = telem.fault.name
                fault_counts[fault_name] = fault_counts.get(fault_name, 0) + 1
                fail_details.append(f"#{i+1}: {fault_name} (retries={retries})")

            motor.stop_and_wait(timeout=5.0)
            time.sleep(1.0)

        duration = time.time() - t0
        success_rate = successes / n_cycles * 100

        # Statistics
        key_signals = {
            "测试次数": str(n_cycles),
            "成功次数": str(successes),
            "成功率(%)": f"{success_rate:.1f}",
        }
        if startup_times:
            key_signals["平均启动时间(s)"] = f"{np.mean(startup_times):.2f}"
            key_signals["最大启动时间(s)"] = f"{np.max(startup_times):.2f}"
        if retries_list:
            key_signals["平均重试次数"] = f"{np.mean(retries_list):.1f}"
            key_signals["最大重试次数"] = str(max(retries_list))
        for fault, count in fault_counts.items():
            key_signals[f"故障-{fault}"] = str(count)

        passed = success_rate >= 95.0

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name=f"启动可靠性×{n_cycles}",
            description=f"连续{n_cycles}次启停循环，统计启动成功率",
            verdict=TestVerdict.PASS if passed else TestVerdict.FAIL,
            duration_s=duration,
            preconditions="电机静止",
            test_method=f"循环{n_cycles}次: 启动→等待闭环→运行1s→停机→等待1s",
            pass_criteria="成功率≥95%",
            actual_result=f"成功率={success_rate:.1f}%  ({successes}/{n_cycles})",
            anomalies=fail_details,
            key_signals=key_signals,
            improvement_suggestions=[
                "增大 OPEN_LOOP_IQ_A" if "STARTUP" in str(fault_counts) else "",
                "增大 STARTUP_MAX_RETRIES" if any(r > 2 for r in retries_list) else "",
            ],
            waveform_files=[csv_logger],
        )
        report.add_record(record)
        can.disable_history()

        assert passed, f"启动可靠性不足: {success_rate:.1f}% < 95%"
