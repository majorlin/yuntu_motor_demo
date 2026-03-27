#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Logger
=============
Provides real-time console logging and log buffering for test cases.
Each test gets its own TestLogger instance via a pytest fixture.
Logs are printed to stdout in real time and also buffered for
inclusion in test reports.
"""

from __future__ import annotations

import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Tuple


class LogLevel(Enum):
    STEP = "STEP"
    INFO = "INFO"
    WARN = "WARN"
    ERROR = "ERROR"
    DATA = "DATA"


# ANSI color codes for terminal output
_COLORS = {
    LogLevel.STEP:  "\033[96m",   # cyan
    LogLevel.INFO:  "\033[37m",   # white
    LogLevel.WARN:  "\033[93m",   # yellow
    LogLevel.ERROR: "\033[91m",   # red
    LogLevel.DATA:  "\033[95m",   # magenta
}
_RESET = "\033[0m"
_BOLD = "\033[1m"

# Icons for HTML / report output
_ICONS = {
    LogLevel.STEP:  "▶",
    LogLevel.INFO:  "ℹ",
    LogLevel.WARN:  "⚠",
    LogLevel.ERROR: "✖",
    LogLevel.DATA:  "📊",
}


@dataclass
class LogEntry:
    """A single log entry."""
    timestamp: str
    elapsed_s: float
    level: LogLevel
    message: str
    step_number: int = 0  # non-zero for STEP entries


class TestLogger:
    """Real-time test logger with buffered output for reports.

    Usage::

        def test_example(test_logger):
            log = test_logger
            log.step("检查电机状态")
            log.info(f"当前状态 = {state.name}")
            log.data("转速", f"{rpm:.1f} RPM")
            log.step("发送启动命令")
            ...
            # At the end, retrieve for report
            steps = log.get_steps()      # list of step description strings
            logs = log.get_logs()        # list of formatted log strings
    """

    def __init__(self, test_name: str = ""):
        self.test_name = test_name
        self._entries: List[LogEntry] = []
        self._step_count = 0
        self._t0 = time.monotonic()
        self._start_time = time.strftime("%H:%M:%S")

        # Print header
        self._print_header()

    def _print_header(self) -> None:
        header = f"\n{'━' * 70}"
        header += f"\n{_BOLD}🧪 {self.test_name}{_RESET}"
        header += f"\n   开始时间: {self._start_time}"
        header += f"\n{'━' * 70}"
        print(header, flush=True)

    def _elapsed(self) -> float:
        return time.monotonic() - self._t0

    def _log(self, level: LogLevel, message: str, step_num: int = 0) -> None:
        ts = time.strftime("%H:%M:%S")
        elapsed = self._elapsed()
        entry = LogEntry(
            timestamp=ts,
            elapsed_s=elapsed,
            level=level,
            message=message,
            step_number=step_num,
        )
        self._entries.append(entry)

        # Print to console in real time
        color = _COLORS.get(level, "")
        icon = _ICONS.get(level, "")

        if level == LogLevel.STEP:
            line = f"  {color}{_BOLD}[{ts}] {icon} Step {step_num}: {message}{_RESET}"
        elif level == LogLevel.DATA:
            line = f"  {color}[{ts}] {icon} {message}{_RESET}"
        else:
            line = f"  {color}[{ts}] {icon} {message}{_RESET}"

        print(line, flush=True)

    # ── Public API ──────────────────────────────────────────────────────

    def step(self, message: str) -> None:
        """Record a test step (auto-numbered)."""
        self._step_count += 1
        self._log(LogLevel.STEP, message, step_num=self._step_count)

    def info(self, message: str) -> None:
        """Record an informational log."""
        self._log(LogLevel.INFO, message)

    def warn(self, message: str) -> None:
        """Record a warning."""
        self._log(LogLevel.WARN, message)

    def error(self, message: str) -> None:
        """Record an error."""
        self._log(LogLevel.ERROR, message)

    def data(self, key: str, value: str) -> None:
        """Record a key-value data point."""
        self._log(LogLevel.DATA, f"{key} = {value}")

    def separator(self, label: str = "") -> None:
        """Print a visual separator line."""
        if label:
            line = f"  {'─' * 10} {label} {'─' * (50 - len(label))}"
        else:
            line = f"  {'─' * 60}"
        print(line, flush=True)

    # ── Output for Reports ──────────────────────────────────────────────

    def get_steps(self) -> List[str]:
        """Return ordered list of step descriptions for the report."""
        return [
            e.message for e in self._entries
            if e.level == LogLevel.STEP
        ]

    def get_logs(self) -> List[str]:
        """Return all log entries as formatted strings for the report."""
        lines = []
        for e in self._entries:
            icon = _ICONS.get(e.level, "")
            if e.level == LogLevel.STEP:
                lines.append(f"[{e.timestamp}] {icon} Step {e.step_number}: {e.message}")
            else:
                lines.append(f"[{e.timestamp}] {icon} [{e.level.value}] {e.message}")
        return lines

    def get_duration_s(self) -> float:
        """Return total elapsed seconds since logger creation."""
        return self._elapsed()

    def get_data_dict(self) -> Dict[str, str]:
        """Return all DATA entries as a key-value dict."""
        result = {}
        for e in self._entries:
            if e.level == LogLevel.DATA:
                # Parse "key = value" format
                if " = " in e.message:
                    k, v = e.message.split(" = ", 1)
                    result[k] = v
        return result

    def print_summary(self, verdict: str = "") -> None:
        """Print a final summary line."""
        duration = self._elapsed()
        icon = "✅" if verdict == "PASS" else "❌" if verdict == "FAIL" else "⏭" if verdict == "SKIP" else "⚠️"
        summary = f"\n  {icon} 结果: {verdict}  |  步骤数: {self._step_count}  |  耗时: {duration:.1f}s"
        summary += f"\n{'━' * 70}\n"
        print(summary, flush=True)
