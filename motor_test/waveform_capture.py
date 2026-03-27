#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Waveform Capture & Analysis
=============================
Collects time-series signal data from CAN telemetry, performs
step-response analysis, and generates waveform plots.
"""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

from can_interface import CanInterface


@dataclass
class StepResponseMetrics:
    """Metrics extracted from a step response."""
    rise_time_s: float = 0.0       # 10%→90% of step
    settling_time_s: float = 0.0   # time to stay within tolerance band
    overshoot_pct: float = 0.0     # peak overshoot percentage
    undershoot_pct: float = 0.0    # peak undershoot percentage
    steady_state_error_pct: float = 0.0
    peak_value: float = 0.0
    final_value: float = 0.0
    initial_value: float = 0.0
    motor_stalled: bool = False    # True if motor did not respond to step


@dataclass
class WaveformRecord:
    """A recorded waveform with metadata."""
    name: str
    signals: Dict[str, List[float]]
    time_s: List[float]
    sample_interval_s: float
    metadata: Dict[str, str] = field(default_factory=dict)


class WaveformCapture:
    """Captures and analyzes motor control waveforms from CAN data."""

    def __init__(self, can: CanInterface, output_dir: str = "motor_test/test_records"):
        self.can = can
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)

    # ── Capture ─────────────────────────────────────────────────────────────

    def capture(
        self,
        signal_names: List[str],
        duration_s: float,
        interval_s: float = 0.01,
        name: str = "waveform",
    ) -> WaveformRecord:
        """Capture signal waveforms over a duration."""
        data: Dict[str, List[float]] = {n: [] for n in signal_names}
        timestamps: List[float] = []
        t0 = time.monotonic()

        while (time.monotonic() - t0) < duration_s:
            t = time.monotonic() - t0
            values = self.can.get_signals(signal_names)
            timestamps.append(t)
            for n in signal_names:
                data[n].append(values.get(n, 0.0))
            time.sleep(interval_s)

        return WaveformRecord(
            name=name,
            signals=data,
            time_s=timestamps,
            sample_interval_s=interval_s,
        )

    def capture_step_response(
        self,
        signal_name: str,
        step_trigger: callable,
        pre_s: float = 1.0,
        post_s: float = 5.0,
        interval_s: float = 0.01,
        extra_signals: Optional[List[str]] = None,
        expected_step: Optional[float] = None,
    ) -> Tuple[WaveformRecord, StepResponseMetrics]:
        """Capture a step response: record pre→trigger→post, analyze.

        Args:
            expected_step: Expected step magnitude (e.g. to_rpm - from_rpm).
                           Used for stall detection — if actual response is
                           far smaller, metrics.motor_stalled will be True.
        """
        all_signals = [signal_name]
        if extra_signals:
            all_signals.extend(extra_signals)

        data: Dict[str, List[float]] = {n: [] for n in all_signals}
        timestamps: List[float] = []

        # Pre-step
        t0 = time.monotonic()
        while (time.monotonic() - t0) < pre_s:
            t = time.monotonic() - t0
            vals = self.can.get_signals(all_signals)
            timestamps.append(t)
            for n in all_signals:
                data[n].append(vals.get(n, 0.0))
            time.sleep(interval_s)

        # Apply step
        step_time = time.monotonic() - t0
        step_trigger()

        # Post-step
        while (time.monotonic() - t0) < (pre_s + post_s):
            t = time.monotonic() - t0
            vals = self.can.get_signals(all_signals)
            timestamps.append(t)
            for n in all_signals:
                data[n].append(vals.get(n, 0.0))
            time.sleep(interval_s)

        record = WaveformRecord(
            name=f"step_{signal_name}",
            signals=data,
            time_s=timestamps,
            sample_interval_s=interval_s,
            metadata={"step_time": str(step_time)},
        )

        # Analyze primary signal
        metrics = self.analyze_step_response(
            timestamps, data[signal_name], step_time,
            expected_step=expected_step,
        )

        return record, metrics

    # ── Analysis ────────────────────────────────────────────────────────────

    @staticmethod
    def analyze_step_response(
        time_s: List[float],
        values: List[float],
        step_time: float,
        tolerance_pct: float = 5.0,
        expected_step: Optional[float] = None,
    ) -> StepResponseMetrics:
        """Analyze a step response curve."""
        t = np.array(time_s)
        y = np.array(values)

        # Find pre-step and post-step regions
        pre_mask = t < step_time
        post_mask = t >= step_time

        if not np.any(pre_mask) or not np.any(post_mask):
            return StepResponseMetrics()

        initial = float(np.mean(y[pre_mask][-min(20, np.sum(pre_mask)):]))
        # Final value: last 20% of post-step
        post_y = y[post_mask]
        post_t = t[post_mask] - step_time
        n_final = max(1, len(post_y) // 5)
        final = float(np.mean(post_y[-n_final:]))

        step_size = final - initial
        if abs(step_size) < 1e-6:
            return StepResponseMetrics(initial_value=initial, final_value=final)

        # Stall detection: if actual step is far smaller than expected,
        # the motor did not respond (stalled / lost sync)
        if expected_step is not None and abs(expected_step) > 1.0:
            if abs(step_size) < abs(expected_step) * 0.05:
                return StepResponseMetrics(
                    initial_value=initial,
                    final_value=final,
                    steady_state_error_pct=100.0,
                    motor_stalled=True,
                )

        # Normalize
        y_norm = (post_y - initial) / step_size  # 0→1 for ideal step

        # Rise time (10%→90%)
        t10_idx = np.argmax(y_norm >= 0.1) if np.any(y_norm >= 0.1) else 0
        t90_idx = np.argmax(y_norm >= 0.9) if np.any(y_norm >= 0.9) else len(y_norm) - 1
        rise_time = float(post_t[t90_idx] - post_t[t10_idx]) if t90_idx > t10_idx else 0.0

        # Overshoot
        peak = float(np.max(y_norm)) if step_size > 0 else float(np.min(y_norm))
        overshoot = max(0.0, (peak - 1.0) * 100.0) if step_size > 0 else max(0.0, (1.0 - peak) * 100.0)

        # Undershoot (in opposite direction)
        if step_size > 0:
            undershoot_val = float(np.min(y_norm))
            undershoot = max(0.0, -undershoot_val * 100.0)
        else:
            undershoot_val = float(np.max(y_norm))
            undershoot = max(0.0, (undershoot_val - 1.0) * 100.0)

        # Settling time (within tolerance band of final value)
        tol = tolerance_pct / 100.0
        settled = np.abs(y_norm - 1.0) <= tol
        settling_time = float(post_t[-1])
        # Find last time it was outside band
        outside = np.where(~settled)[0]
        if len(outside) > 0:
            settling_time = float(post_t[outside[-1]])
        else:
            settling_time = 0.0

        # Steady-state error
        ss_error = abs(final - (initial + step_size)) / abs(step_size) * 100.0 if abs(step_size) > 1e-6 else 0.0

        return StepResponseMetrics(
            rise_time_s=rise_time,
            settling_time_s=settling_time,
            overshoot_pct=overshoot,
            undershoot_pct=undershoot,
            steady_state_error_pct=ss_error,
            peak_value=float(initial + peak * step_size),
            final_value=final,
            initial_value=initial,
        )

    @staticmethod
    def compute_signal_stats(values: List[float]) -> Dict[str, float]:
        """Compute basic statistics for a signal."""
        arr = np.array(values)
        return {
            "mean": float(np.mean(arr)),
            "std": float(np.std(arr)),
            "min": float(np.min(arr)),
            "max": float(np.max(arr)),
            "p2p": float(np.ptp(arr)),
            "rms": float(np.sqrt(np.mean(arr**2))),
        }

    # ── Plotting ────────────────────────────────────────────────────────────

    def plot_waveform(
        self,
        record: WaveformRecord,
        title: str = "",
        filename: Optional[str] = None,
        step_time: Optional[float] = None,
        y_labels: Optional[Dict[str, str]] = None,
    ) -> Optional[str]:
        """Plot waveform to PNG file. Returns file path."""
        if not HAS_MPL:
            return None

        n_signals = len(record.signals)
        fig, axes = plt.subplots(n_signals, 1, figsize=(12, 3 * n_signals),
                                  sharex=True, facecolor="#1e1e2e")
        if n_signals == 1:
            axes = [axes]

        colors = ["#00e5ff", "#ff9100", "#00e676", "#7c4dff", "#ff5252",
                   "#f9e2af", "#89b4fa", "#f38ba8", "#a6e3a1", "#cba6f7"]

        for i, (sig_name, values) in enumerate(record.signals.items()):
            ax = axes[i]
            ax.set_facecolor("#181825")
            color = colors[i % len(colors)]
            ax.plot(record.time_s, values, color=color, linewidth=1.2, label=sig_name)
            ax.set_ylabel(y_labels.get(sig_name, sig_name) if y_labels else sig_name,
                          color="#cdd6f4", fontsize=9)
            ax.tick_params(colors="#6c7086", labelsize=8)
            ax.grid(True, color="#313244", linewidth=0.5, alpha=0.7)
            ax.legend(loc="upper right", fontsize=8, facecolor="#313244",
                      edgecolor="#45475a", labelcolor="#cdd6f4")
            for spine in ax.spines.values():
                spine.set_color("#45475a")
            if step_time is not None:
                ax.axvline(x=step_time, color="#f38ba8", linestyle="--",
                           linewidth=1, alpha=0.7, label="Step")

        axes[-1].set_xlabel("Time (s)", color="#cdd6f4", fontsize=9)
        if title:
            fig.suptitle(title, color="#89b4fa", fontsize=12, y=0.98)
        fig.tight_layout()

        if filename is None:
            filename = f"{record.name}_{int(time.time())}.png"
        filepath = os.path.join(self.output_dir, filename)
        fig.savefig(filepath, dpi=150, facecolor="#1e1e2e")
        plt.close(fig)
        return filepath

    def plot_step_response(
        self,
        record: WaveformRecord,
        metrics: StepResponseMetrics,
        signal_name: str,
        filename: Optional[str] = None,
    ) -> Optional[str]:
        """Plot step response with annotated metrics."""
        if not HAS_MPL:
            return None

        fig, ax = plt.subplots(1, 1, figsize=(12, 5), facecolor="#1e1e2e")
        ax.set_facecolor("#181825")

        values = record.signals[signal_name]
        ax.plot(record.time_s, values, color="#00e5ff", linewidth=1.5, label=signal_name)

        step_time = float(record.metadata.get("step_time", 0))
        ax.axvline(x=step_time, color="#f38ba8", linestyle="--", linewidth=1, label="Step")

        # Annotate metrics
        ax.axhline(y=metrics.final_value, color="#a6e3a1", linestyle=":", linewidth=0.8, alpha=0.7)
        ax.axhline(y=metrics.initial_value, color="#6c7086", linestyle=":", linewidth=0.8, alpha=0.7)
        if metrics.overshoot_pct > 0:
            ax.axhline(y=metrics.peak_value, color="#ff5252", linestyle=":", linewidth=0.8, alpha=0.7)

        info = (f"Rise: {metrics.rise_time_s:.3f}s  "
                f"Overshoot: {metrics.overshoot_pct:.1f}%  "
                f"Settle: {metrics.settling_time_s:.3f}s  "
                f"SSE: {metrics.steady_state_error_pct:.1f}%")
        ax.set_title(info, color="#f9e2af", fontsize=10, loc="left", pad=8)

        ax.set_xlabel("Time (s)", color="#cdd6f4")
        ax.set_ylabel(signal_name, color="#cdd6f4")
        ax.tick_params(colors="#6c7086")
        ax.grid(True, color="#313244", linewidth=0.5, alpha=0.7)
        ax.legend(loc="upper right", fontsize=8, facecolor="#313244",
                  edgecolor="#45475a", labelcolor="#cdd6f4")
        for spine in ax.spines.values():
            spine.set_color("#45475a")

        fig.tight_layout()
        if filename is None:
            filename = f"step_response_{signal_name}_{int(time.time())}.png"
        filepath = os.path.join(self.output_dir, filename)
        fig.savefig(filepath, dpi=150, facecolor="#1e1e2e")
        plt.close(fig)
        return filepath

    def plot_multi_signal(
        self,
        records: List[WaveformRecord],
        title: str = "",
        filename: Optional[str] = None,
    ) -> Optional[str]:
        """Overlay multiple waveform records on subplots grouped by signal."""
        if not HAS_MPL or not records:
            return None

        all_sigs = set()
        for r in records:
            all_sigs.update(r.signals.keys())
        sig_list = sorted(all_sigs)

        fig, axes = plt.subplots(len(sig_list), 1, figsize=(12, 3 * len(sig_list)),
                                  sharex=True, facecolor="#1e1e2e")
        if len(sig_list) == 1:
            axes = [axes]

        colors = ["#00e5ff", "#ff9100", "#00e676", "#7c4dff", "#ff5252"]
        for i, sig in enumerate(sig_list):
            ax = axes[i]
            ax.set_facecolor("#181825")
            for j, rec in enumerate(records):
                if sig in rec.signals:
                    c = colors[j % len(colors)]
                    ax.plot(rec.time_s, rec.signals[sig], color=c,
                            linewidth=1, label=rec.name, alpha=0.8)
            ax.set_ylabel(sig, color="#cdd6f4", fontsize=9)
            ax.tick_params(colors="#6c7086", labelsize=8)
            ax.grid(True, color="#313244", linewidth=0.5, alpha=0.7)
            ax.legend(loc="upper right", fontsize=7, facecolor="#313244",
                      edgecolor="#45475a", labelcolor="#cdd6f4")
            for spine in ax.spines.values():
                spine.set_color("#45475a")

        if title:
            fig.suptitle(title, color="#89b4fa", fontsize=12, y=0.98)
        fig.tight_layout()

        if filename is None:
            filename = f"multi_signal_{int(time.time())}.png"
        filepath = os.path.join(self.output_dir, filename)
        fig.savefig(filepath, dpi=150, facecolor="#1e1e2e")
        plt.close(fig)
        return filepath
