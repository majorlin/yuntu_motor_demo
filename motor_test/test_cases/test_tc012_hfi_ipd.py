#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TC-012: HFI/IPD 静止初始角定位可行性验证
=========================================
面向座椅电机的 bench 评估用例。目标不是替代常规启动回归，而是回答：
1. HFI/IPD 是否真的在静止启动前给出了可用初始角；
2. 常见回退原因是什么；
3. 在当前参数下启动成功率和附加起动时间是否可接受。
"""

from __future__ import annotations

import time
from collections import Counter

import pytest

from motor_client import MotorState
from report_generator import TestRecord, TestVerdict


HFI_FALLBACK_LABELS = {
    0: "NONE",
    1: "DISABLED",
    2: "INVALID_VBUS",
    3: "LOW_CONFIDENCE",
    4: "LOW_REPEATABILITY",
    5: "POLARITY_AMBIGUOUS",
}


class TestHfiIpdFeasibility:
    """TC-012: Standstill HFI/IPD feasibility gate for the seat motor."""

    TEST_ID = "TC-012"

    @pytest.mark.parametrize("n_cycles", [100])
    def test_012_hfi_ipd_feasibility(self, require_hardware, motor, can, report,
                                     csv_logger, motor_profile, n_cycles,
                                     test_logger):
        if motor_profile.get("motor_id") != "seat":
            pytest.skip("TC-012 仅对座椅电机 profile 开放")

        log = test_logger
        t0 = time.time()

        start_rpm = float(motor_profile.get("test", {}).get("start_rpm", 2000.0))
        start_timeout_s = float(
            motor_profile.get("test", {}).get("start_timeout_s", 5.0)
        )
        stop_timeout_s = float(
            motor_profile.get("test", {}).get("stop_timeout_s", 3.0)
        )
        settle_s = float(
            motor_profile.get("test", {}).get("start_stop_settle_s", 1.0)
        )

        successes = 0
        hfi_used_count = 0
        direction_ok_count = 0
        startup_times_s = []
        hfi_confidence_samples = []
        hfi_ripple_samples = []
        fallback_counts: Counter[str] = Counter()
        fail_details = []

        log.step("配置 HFI/IPD 运行时参数")
        motor.stop_and_wait(timeout=stop_timeout_s)
        motor.configure_hfi(enable=True)
        time.sleep(0.2)
        current = can.get_all_signals()
        log.data("ActiveHfiFlags", f"{int(current.get('ActiveHfiFlags', 0))}")
        log.data("ActiveHfiCandidateCount", f"{int(current.get('ActiveHfiCandidateCount', 0))}")
        log.data("ActiveHfiPulsePairs", f"{int(current.get('ActiveHfiPulsePairs', 0))}")
        log.data("ActiveHfiInjectV", f"{current.get('ActiveHfiInjectV', 0.0):.2f} V")
        log.data("ActiveHfiConfidenceMin", f"{current.get('ActiveHfiConfidenceMin', 0.0):.3f}")
        log.data("ActiveHfiPolarityV", f"{current.get('ActiveHfiPolarityV', 0.0):.2f} V")
        log.data("ActiveHfiPolarityPulseCount", f"{int(current.get('ActiveHfiPolarityPulseCount', 0))}")
        log.data("ActiveHfiAlignHoldMs", f"{int(current.get('ActiveHfiAlignHoldMs', 0))} ms")

        log.step(f"执行 {n_cycles} 次静止启动评估")
        log.info(
            f"目标转速: {start_rpm:.0f} RPM, 启动超时: {start_timeout_s:.1f}s, 停机超时: {stop_timeout_s:.1f}s"
        )

        for i in range(n_cycles):
            iter_t0 = time.monotonic()
            ok = motor.start_and_wait(
                target_rpm=start_rpm,
                timeout=start_timeout_s,
                direction=1,
            )
            startup_time_s = time.monotonic() - iter_t0
            telem = motor.get_telemetry()

            startup_times_s.append(startup_time_s)

            if ok:
                successes += 1

            if telem.hfi_used and telem.hfi_angle_valid:
                hfi_used_count += 1
                hfi_confidence_samples.append(telem.hfi_confidence)
                hfi_ripple_samples.append(telem.hfi_ripple_metric)
            else:
                reason = HFI_FALLBACK_LABELS.get(
                    telem.hfi_fallback_reason,
                    f"UNKNOWN_{telem.hfi_fallback_reason}",
                )
                fallback_counts[reason] += 1

            if ok and (telem.elec_speed_rad_s > 0.0):
                direction_ok_count += 1

            log.info(
                f"#{i + 1}/{n_cycles}: ok={ok} state={telem.state.name} "
                f"hfi_used={telem.hfi_used} conf={telem.hfi_confidence:.3f} "
                f"fallback={HFI_FALLBACK_LABELS.get(telem.hfi_fallback_reason, telem.hfi_fallback_reason)} "
                f"t={startup_time_s:.2f}s"
            )

            if not ok:
                fail_details.append(
                    f"#{i + 1}: state={telem.state.name}, fallback="
                    f"{HFI_FALLBACK_LABELS.get(telem.hfi_fallback_reason, telem.hfi_fallback_reason)}, "
                    f"conf={telem.hfi_confidence:.3f}"
                )

            time.sleep(0.5)
            motor.stop_and_wait(timeout=stop_timeout_s)
            time.sleep(settle_s)

        duration_s = time.time() - t0
        success_rate = (100.0 * successes / n_cycles) if n_cycles else 0.0
        hfi_usage_rate = (100.0 * hfi_used_count / n_cycles) if n_cycles else 0.0
        direction_ok_rate = (100.0 * direction_ok_count / n_cycles) if n_cycles else 0.0
        avg_startup_time_s = (
            sum(startup_times_s) / len(startup_times_s) if startup_times_s else 0.0
        )
        avg_confidence = (
            sum(hfi_confidence_samples) / len(hfi_confidence_samples)
            if hfi_confidence_samples
            else 0.0
        )
        avg_ripple = (
            sum(hfi_ripple_samples) / len(hfi_ripple_samples)
            if hfi_ripple_samples
            else 0.0
        )

        log.step("汇总 HFI/IPD 可行性结果")
        log.data("启动成功率", f"{success_rate:.1f}% ({successes}/{n_cycles})")
        log.data("HFI 使用率", f"{hfi_usage_rate:.1f}% ({hfi_used_count}/{n_cycles})")
        log.data("方向正确率", f"{direction_ok_rate:.1f}% ({direction_ok_count}/{n_cycles})")
        log.data("平均启动时间", f"{avg_startup_time_s:.2f} s")
        log.data("平均 HFI 置信度", f"{avg_confidence:.3f}")
        log.data("平均 HFI Ripple", f"{avg_ripple:.3f} A")
        for label, count in sorted(fallback_counts.items()):
            log.data(f"Fallback-{label}", str(count))

        passed = (
            (success_rate >= 90.0) and
            (direction_ok_rate >= 95.0) and
            (hfi_used_count > 0) and
            (avg_confidence >= 0.05)
        )
        verdict = TestVerdict.PASS if passed else TestVerdict.FAIL
        log.print_summary(verdict.value)

        improvement_suggestions = []
        if hfi_used_count == 0:
            improvement_suggestions.append(
                "HFI 从未给出有效初始角，优先检查 saliency 是否足够、注入幅值是否过低、采样噪声是否过高。"
            )
        if fallback_counts.get("LOW_CONFIDENCE", 0) > 0:
            improvement_suggestions.append(
                "LOW_CONFIDENCE 偏多，可尝试增加注入电压、脉冲对数，或缩小候选角数量提升单点 SNR。"
            )
        if fallback_counts.get("LOW_REPEATABILITY", 0) > 0:
            improvement_suggestions.append(
                "LOW_REPEATABILITY 偏多，优先检查机械振动、采样同步和候选角细扫步距。"
            )
        if fallback_counts.get("POLARITY_AMBIGUOUS", 0) > 0:
            improvement_suggestions.append(
                "POLARITY_AMBIGUOUS 偏多，可适当增大极性判别脉冲幅值或脉冲个数。"
            )
        if success_rate < 90.0:
            improvement_suggestions.append(
                "启动成功率仍不足，建议同时复核 ALIGN 保持时间和 OPEN_LOOP_IQ_A，避免 HFI 失败回退后起动链路变脆弱。"
            )

        record = TestRecord(
            test_id=self.TEST_ID,
            test_name=f"HFI/IPD 静止启动评估×{n_cycles}",
            description="统计 HFI 使用率、回退原因、方向正确率和启动成功率，判断静止初始角定位是否具备继续推进价值。",
            verdict=verdict,
            duration_s=duration_s,
            preconditions="座椅电机静止，纯无感，无外加位置传感器",
            test_method=(
                f"使能 HFI/IPD 后执行 {n_cycles} 次静止启动，每次记录启动结果、"
                "HFI 使用标志、回退原因、置信度和起动时间。"
            ),
            pass_criteria="启动成功率≥90%，方向正确率≥95%，HFI 至少成功介入 1 次，平均 HFI 置信度≥0.05",
            actual_result=(
                f"success={success_rate:.1f}%, hfi_used={hfi_usage_rate:.1f}%, "
                f"dir_ok={direction_ok_rate:.1f}%, avg_conf={avg_confidence:.3f}"
            ),
            anomalies=fail_details,
            key_signals={
                "测试次数": str(n_cycles),
                "成功次数": str(successes),
                "启动成功率(%)": f"{success_rate:.1f}",
                "HFI 使用次数": str(hfi_used_count),
                "HFI 使用率(%)": f"{hfi_usage_rate:.1f}",
                "方向正确率(%)": f"{direction_ok_rate:.1f}",
                "平均启动时间(s)": f"{avg_startup_time_s:.2f}",
                "平均 HFI 置信度": f"{avg_confidence:.3f}",
                "平均 HFI Ripple(A)": f"{avg_ripple:.3f}",
                **{f"Fallback-{k}": str(v) for k, v in sorted(fallback_counts.items())},
            },
            improvement_suggestions=improvement_suggestions,
            waveform_files=[csv_logger],
            test_steps=log.get_steps(),
            test_logs=log.get_logs(),
        )
        report.add_record(record)

        assert passed, (
            "HFI/IPD 可行性未通过: "
            f"success={success_rate:.1f}%, hfi_used={hfi_usage_rate:.1f}%, "
            f"dir_ok={direction_ok_rate:.1f}%, avg_conf={avg_confidence:.3f}"
        )
