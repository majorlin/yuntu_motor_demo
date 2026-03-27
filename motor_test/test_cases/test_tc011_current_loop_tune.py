#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-011: Current Loop 自动调参
===========================
通过D轴阶跃注入，结合高频率波形缓冲捕获，扫描电流环带宽找出最优Kp/Ki参数。
"""

import time
import pytest
from motor_client import MotorState
from auto_tuner import AutoTuner
from report_generator import TestRecord, TestVerdict

pytestmark = pytest.mark.skip(reason="TC-011 属于参数整定流程，不纳入常规测试")


class TestCurrentLoopTune:
    """TC-011: Automated current loop tuning via step response."""

    TEST_ID = "TC-011"

    def test_011_current_loop_auto_tune(self, motor, can, waveform, analyzer, report,
                                         csv_logger, test_logger):
        """Current Loop 自动调参: 扫描带宽寻找最优解"""
        log = test_logger
        t0 = time.time()
        can.enable_history()

        log.step("读取当前电流环参数")
        tuner = AutoTuner(can, motor, waveform, analyzer)
        orig_id_kp = can.get_signal("ActiveCurrentIdKp") or 5.0
        orig_id_ki = can.get_signal("ActiveCurrentIdKi") or 5000.0
        log.data("原始 IdKp", f"{orig_id_kp:.4f}")
        log.data("原始 IdKi", f"{orig_id_ki:.1f}")

        # Determine Ls and Rs using motor configurations or defaults
        ls_h = 0.001       # 1mH
        rs_ohm = 0.5       # 0.5 Ohm

        try:
            from conftest import get_motor_config
            cfg = get_motor_config()
            ls_h = cfg.get("Rs", ls_h)
        except ImportError:
            pass

        log.step(f"开始 5 点带宽扫描 (1000~4000Hz)")
        log.info(f"Ls={ls_h*1e3:.2f}mH, Rs={rs_ohm:.2f}Ω, Inject={2.0}A")
        tune_report = tuner.tune_current_loop(
            ls_h=ls_h,
            rs_ohm=rs_ohm,
            bw_range=(1000.0, 4000.0),
            bw_steps=5,
            inject_a=2.0,
            target_overshoot_pct=5.0,
            target_rise_time_us=600.0,
            pre_samples=64,
            post_samples=128,
            decimation=1,
        )

        duration = time.time() - t0
        n_passed = sum(1 for t in tune_report.all_trials if t.passed)
        n_total = len(tune_report.all_trials)

        log.step("分析调参结果")
        log.data("试验总数", str(n_total))
        log.data("成功试验", str(n_passed))
        log.data("最优带宽(Hz)", f"{tune_report.best_bw_hz:.0f}")
        log.data("最优 IdKp", f"{tune_report.best_id_kp:.4f}")
        log.data("最优 IdKi", f"{tune_report.best_id_ki:.1f}")
        log.data("超调(%)", f"{tune_report.best_metrics.overshoot_pct:.1f}")
        log.data("上升时间(us)", f"{tune_report.best_metrics.rise_time_s*1e6:.0f}")

        passed = n_passed > 0 and tune_report.best_metrics.overshoot_pct < 20.0
        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name="电流环自动调参",
            description="注入D轴电流阶跃，分析波形并寻找最优电流环带宽",
            verdict=verdict,
            duration_s=duration,
            preconditions="电机静止",
            test_method=f"5点带宽扫描 (1000~4000Hz), Ls={ls_h*1e3:.2f}mH, Rs={rs_ohm:.2f}Ω",
            pass_criteria="找到超调<20%的参数组合",
            actual_result=(f"最优带宽={tune_report.best_bw_hz:.0f}Hz, Kp={tune_report.best_id_kp:.4f}, Ki={tune_report.best_id_ki:.1f}, "
                          f"超调={tune_report.best_metrics.overshoot_pct:.1f}%, "
                          f"上升时间={tune_report.best_metrics.rise_time_s*1e6:.0f}us"),
            key_signals={
                "原始IdKp": f"{orig_id_kp:.4f}",
                "原始IdKi": f"{orig_id_ki:.1f}",
                "最优带宽Hz": f"{tune_report.best_bw_hz:.0f}",
                "最优IdKp": f"{tune_report.best_id_kp:.4f}",
                "最优IdKi": f"{tune_report.best_id_ki:.1f}",
                "最优IqKp": f"{tune_report.best_iq_kp:.4f}",
                "最优IqKi": f"{tune_report.best_iq_ki:.1f}",
                "超调(%)": f"{tune_report.best_metrics.overshoot_pct:.1f}",
                "上升时间(us)": f"{tune_report.best_metrics.rise_time_s*1e6:.0f}",
                "调节时间(ms)": f"{tune_report.best_metrics.settling_time_s*1000.0:.1f}",
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
