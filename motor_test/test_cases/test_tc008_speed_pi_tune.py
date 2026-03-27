#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-008: Speed PI 自动调参
===========================
通过阶跃响应分析自动搜索最优 Speed Kp/Ki 参数。
"""

import time
import pytest
from motor_client import MotorState
from auto_tuner import AutoTuner
from report_generator import TestRecord, TestVerdict

pytestmark = pytest.mark.skip(reason="TC-008 属于参数整定流程，不纳入常规测试")


class TestSpeedPiTune:
    """TC-008: Automated speed PI tuning via step response."""

    TEST_ID = "TC-008"

    def test_008_speed_pi_auto_tune(self, motor, can, waveform, analyzer, report,
                                     csv_logger, test_logger):
        """Speed PI 自动调参: 网格搜索 Kp/Ki 空间"""
        log = test_logger
        t0 = time.time()
        can.enable_history()

        log.step("读取当前 Speed PI 参数")
        tuner = AutoTuner(can, motor, waveform, analyzer)
        orig_kp = can.get_signal("ActiveSpeedKp")
        orig_ki = can.get_signal("ActiveSpeedKi")
        log.data("原始 Kp", f"{orig_kp:.6f}")
        log.data("原始 Ki", f"{orig_ki:.6f}")

        log.step("开始 4×4 网格搜索")
        log.info("Kp=[0.001, 0.010], Ki=[0.002, 0.020], base=800RPM, step=2000RPM")
        tune_report = tuner.tune_speed_pi(
            base_rpm=800.0,
            step_rpm=2000.0,
            kp_range=(0.001, 0.010),
            ki_range=(0.002, 0.020),
            kp_steps=4,
            ki_steps=4,
            settle_time_s=3.0,
            record_time_s=5.0,
        )

        duration = time.time() - t0
        n_passed = sum(1 for t in tune_report.all_trials if t.passed)
        n_total = len(tune_report.all_trials)

        log.step("分析调参结果")
        log.data("试验总数", str(n_total))
        log.data("成功试验", str(n_passed))
        log.data("最优 Kp", f"{tune_report.best_kp:.6f}")
        log.data("最优 Ki", f"{tune_report.best_ki:.6f}")
        log.data("超调(%)", f"{tune_report.best_metrics.overshoot_pct:.1f}")
        log.data("上升时间(s)", f"{tune_report.best_metrics.rise_time_s:.3f}")

        passed = n_passed > 0 and tune_report.best_metrics.overshoot_pct < 20.0
        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name="Speed PI自动调参",
            description="网格搜索Kp/Ki空间，通过阶跃响应找最优参数",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机可正常闭环运行",
            test_method="4×4网格扫描Kp=[0.001,0.01], Ki=[0.002,0.02]",
            pass_criteria="找到超调<20%的参数组合",
            actual_result=(f"最优 Kp={tune_report.best_kp:.6f}, Ki={tune_report.best_ki:.6f}, "
                          f"超调={tune_report.best_metrics.overshoot_pct:.1f}%, "
                          f"上升时间={tune_report.best_metrics.rise_time_s:.3f}s"),
            key_signals={
                "原始Kp": f"{orig_kp:.6f}",
                "原始Ki": f"{orig_ki:.6f}",
                "最优Kp": f"{tune_report.best_kp:.6f}",
                "最优Ki": f"{tune_report.best_ki:.6f}",
                "超调(%)": f"{tune_report.best_metrics.overshoot_pct:.1f}",
                "上升时间(s)": f"{tune_report.best_metrics.rise_time_s:.3f}",
                "调节时间(s)": f"{tune_report.best_metrics.settling_time_s:.3f}",
                "试验总数": str(n_total),
                "成功试验": str(n_passed),
            },
            waveform_files=[t.waveform_file for t in tune_report.all_trials
                           if t.waveform_file],
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)
        can.disable_history()
