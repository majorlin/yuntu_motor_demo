#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-009: Observer/PLL 参数自动调参
===================================
通过稳态相位误差和PLL速度噪声最小化搜索最优参数。
"""

import time
import pytest
from auto_tuner import AutoTuner
from report_generator import TestRecord, TestVerdict


class TestObserverTune:
    """TC-009: Observer and PLL parameter auto-tuning."""

    TEST_ID = "TC-009"

    def test_009_observer_auto_tune(self, motor, can, waveform, analyzer, report, csv_logger):
        """Observer/PLL 参数调优"""
        t0 = time.time()

        tuner = AutoTuner(can, motor, waveform, analyzer)

        orig_gain = can.get_signal("ActiveObsGain")
        orig_pll_kp = can.get_signal("ActivePllKp")
        orig_pll_ki = can.get_signal("ActivePllKi")

        tune_report = tuner.tune_observer(
            rpm=1500.0,
            gain_range=(0.8e6, 4.0e6),
            pll_kp_range=(60.0, 250.0),
            pll_ki_range=(3000.0, 12000.0),
            gain_steps=4,
            pll_steps=3,
            measure_time_s=2.0,
        )

        duration = time.time() - t0
        n_passed = sum(1 for t in tune_report.all_trials if t.passed)
        n_total = len(tune_report.all_trials)

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name="Observer/PLL调参",
            description="扫描ObsGain/PLL_Kp/PLL_Ki空间，最小化相位误差噪声",
            verdict=TestVerdict.PASS if n_passed > 0 else TestVerdict.FAIL,
            duration_s=duration,
            preconditions="电机稳定闭环运行",
            test_method="4×3×3网格搜索，每组参数记录2s PhaseError和PLLSpeed",
            pass_criteria="找到使PhaseError_RMS最小的参数组合",
            actual_result=f"最优 Gain={tune_report.best_kp:.0f}, PLL_Kp={tune_report.best_ki:.1f}",
            key_signals={
                "原始ObsGain": f"{orig_gain:.0f}",
                "原始PLL_Kp": f"{orig_pll_kp:.1f}",
                "原始PLL_Ki": f"{orig_pll_ki:.1f}",
                "最优ObsGain": f"{tune_report.best_kp:.0f}",
                "最优PLL_Kp": f"{tune_report.best_ki:.1f}",
                "试验总数": str(n_total),
                "成功试验": str(n_passed),
            },
        )
        report.add_record(record)
