#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Report Generator
=======================
Generates Markdown + HTML test reports with embedded waveforms,
test results summary, and anomaly analysis.
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional

try:
    from jinja2 import Template
    HAS_JINJA = True
except ImportError:
    HAS_JINJA = False


class TestVerdict(Enum):
    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    ERROR = "ERROR"


@dataclass
class TestRecord:
    """Complete record of a single test case execution."""
    test_id: str
    test_name: str
    description: str
    verdict: TestVerdict
    duration_s: float = 0.0
    preconditions: str = ""
    test_method: str = ""
    pass_criteria: str = ""
    actual_result: str = ""
    anomalies: List[str] = field(default_factory=list)
    waveform_files: List[str] = field(default_factory=list)
    data_files: List[str] = field(default_factory=list)
    key_signals: Dict[str, str] = field(default_factory=dict)
    improvement_suggestions: List[str] = field(default_factory=list)
    timestamp: str = ""
    error_message: str = ""


class ReportGenerator:
    """Generates comprehensive test reports in Markdown and HTML."""

    def __init__(self, output_dir: str = "motor_test/test_reports"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.records: List[TestRecord] = []

    def add_record(self, record: TestRecord) -> None:
        if not record.timestamp:
            record.timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
        self.records.append(record)

    def clear(self) -> None:
        self.records.clear()

    # ── Markdown Report ─────────────────────────────────────────────────────

    def generate_markdown(self, title: str = "鼓风机电机测试报告") -> str:
        """Generate a complete Markdown test report."""
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        total = len(self.records)
        passed = sum(1 for r in self.records if r.verdict == TestVerdict.PASS)
        failed = sum(1 for r in self.records if r.verdict == TestVerdict.FAIL)
        skipped = sum(1 for r in self.records if r.verdict == TestVerdict.SKIP)
        errors = sum(1 for r in self.records if r.verdict == TestVerdict.ERROR)

        lines = [
            f"# {title}",
            f"",
            f"**生成时间**: {ts}  ",
            f"**测试平台**: YTM32 Sensorless FOC / CAN FD 500K/2M  ",
            f"**通信工具**: YuntuCanLin Debugger  ",
            f"",
            f"## 测试结果汇总",
            f"",
            f"| 指标 | 数量 |",
            f"|------|------|",
            f"| 总计 | {total} |",
            f"| ✅ 通过 | {passed} |",
            f"| ❌ 失败 | {failed} |",
            f"| ⏭ 跳过 | {skipped} |",
            f"| ⚠️ 错误 | {errors} |",
            f"| **通过率** | **{passed/total*100:.1f}%** |" if total > 0 else "",
            f"",
            f"---",
            f"",
        ]

        # Summary table
        lines.extend([
            "## 测试用例列表",
            "",
            "| ID | 测试项 | 结果 | 耗时 | 关键信息 |",
            "|-----|--------|------|------|----------|",
        ])
        for r in self.records:
            icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭", "ERROR": "⚠️"}[r.verdict.value]
            info = r.actual_result[:60] if r.actual_result else ""
            lines.append(f"| {r.test_id} | {r.test_name} | {icon} {r.verdict.value} | "
                         f"{r.duration_s:.1f}s | {info} |")
        lines.extend(["", "---", ""])

        # Detailed results
        lines.append("## 详细测试记录")
        lines.append("")
        for r in self.records:
            icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭", "ERROR": "⚠️"}[r.verdict.value]
            lines.extend([
                f"### {r.test_id}: {r.test_name} {icon}",
                f"",
                f"| 项目 | 内容 |",
                f"|------|------|",
                f"| **描述** | {r.description} |",
                f"| **前置条件** | {r.preconditions} |",
                f"| **测试方法** | {r.test_method} |",
                f"| **判定标准** | {r.pass_criteria} |",
                f"| **实际结果** | {r.actual_result} |",
                f"| **判定** | {icon} {r.verdict.value} |",
                f"| **耗时** | {r.duration_s:.1f}s |",
                f"| **时间** | {r.timestamp} |",
                f"",
            ])

            if r.key_signals:
                lines.append("**关键信号数据:**")
                lines.append("")
                lines.append("| 信号 | 值 |")
                lines.append("|------|-----|")
                for k, v in r.key_signals.items():
                    lines.append(f"| {k} | {v} |")
                lines.append("")

            if r.anomalies:
                lines.append("**检测到的异常:**")
                for a in r.anomalies:
                    lines.append(f"- ⚠️ {a}")
                lines.append("")

            if r.improvement_suggestions:
                lines.append("**改善建议:**")
                for s in r.improvement_suggestions:
                    lines.append(f"- 💡 {s}")
                lines.append("")

            if r.waveform_files:
                lines.append("**波形记录:**")
                for wf in r.waveform_files:
                    basename = os.path.basename(wf)
                    lines.append(f"![{basename}]({wf})")
                lines.append("")

            if r.error_message:
                lines.append(f"> ⚠️ 错误信息: {r.error_message}")
                lines.append("")

            lines.extend(["---", ""])

        return "\n".join(lines)

    def save_markdown(self, filename: Optional[str] = None, **kwargs) -> str:
        """Save Markdown report to file."""
        if filename is None:
            filename = f"test_report_{time.strftime('%Y%m%d_%H%M%S')}.md"
        filepath = os.path.join(self.output_dir, filename)
        content = self.generate_markdown(**kwargs)
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(content)
        return filepath

    # ── JSON Export ──────────────────────────────────────────────────────────

    def save_json(self, filename: Optional[str] = None) -> str:
        """Export all test records as JSON."""
        if filename is None:
            filename = f"test_results_{time.strftime('%Y%m%d_%H%M%S')}.json"
        filepath = os.path.join(self.output_dir, filename)
        data = {
            "timestamp": time.strftime("%Y-%m-%d %H:%M:%S"),
            "summary": {
                "total": len(self.records),
                "passed": sum(1 for r in self.records if r.verdict == TestVerdict.PASS),
                "failed": sum(1 for r in self.records if r.verdict == TestVerdict.FAIL),
            },
            "records": [
                {
                    "test_id": r.test_id,
                    "test_name": r.test_name,
                    "verdict": r.verdict.value,
                    "duration_s": r.duration_s,
                    "actual_result": r.actual_result,
                    "anomalies": r.anomalies,
                    "key_signals": r.key_signals,
                    "improvement_suggestions": r.improvement_suggestions,
                    "timestamp": r.timestamp,
                }
                for r in self.records
            ],
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return filepath

    # ── HTML Report ──────────────────────────────────────────────────────────

    def save_html(self, filename: Optional[str] = None, **kwargs) -> str:
        """Generate and save an HTML report with embedded styling."""
        if filename is None:
            filename = f"test_report_{time.strftime('%Y%m%d_%H%M%S')}.html"
        filepath = os.path.join(self.output_dir, filename)

        md_content = self.generate_markdown(**kwargs)

        html = f"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<title>鼓风机电机测试报告</title>
<style>
  body {{ font-family: 'Menlo', 'Consolas', monospace; background: #1e1e2e; color: #cdd6f4;
         margin: 2em auto; max-width: 1000px; padding: 0 2em; line-height: 1.6; }}
  h1 {{ color: #89b4fa; border-bottom: 2px solid #45475a; padding-bottom: 0.3em; }}
  h2 {{ color: #a6e3a1; }}
  h3 {{ color: #f9e2af; }}
  table {{ border-collapse: collapse; width: 100%; margin: 1em 0; }}
  th, td {{ border: 1px solid #45475a; padding: 8px 12px; text-align: left; }}
  th {{ background: #313244; color: #89b4fa; }}
  tr:nth-child(even) {{ background: #181825; }}
  code {{ background: #313244; padding: 2px 6px; border-radius: 3px; }}
  hr {{ border: 1px solid #45475a; }}
  img {{ max-width: 100%; margin: 1em 0; border-radius: 4px; }}
  blockquote {{ border-left: 3px solid #f38ba8; margin-left: 0; padding-left: 1em;
                color: #f38ba8; }}
  .pass {{ color: #a6e3a1; }} .fail {{ color: #f38ba8; }}
</style>
</head>
<body>
<pre style="white-space: pre-wrap;">{md_content}</pre>
</body>
</html>"""
        with open(filepath, "w", encoding="utf-8") as f:
            f.write(html)
        return filepath


# ── CLI Entry Point ─────────────────────────────────────────────────────────

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="Generate test report from JSON records")
    parser.add_argument("--input", type=str, default="motor_test/test_records",
                        help="Input directory with JSON test records")
    parser.add_argument("--output", type=str, default="motor_test/test_reports")
    args = parser.parse_args()

    gen = ReportGenerator(output_dir=args.output)

    # Load JSON records from input directory
    for fname in sorted(os.listdir(args.input)):
        if fname.endswith(".json"):
            with open(os.path.join(args.input, fname)) as f:
                data = json.load(f)
            if "records" in data:
                for rec in data["records"]:
                    gen.add_record(TestRecord(
                        test_id=rec.get("test_id", ""),
                        test_name=rec.get("test_name", ""),
                        description="",
                        verdict=TestVerdict(rec.get("verdict", "SKIP")),
                        actual_result=rec.get("actual_result", ""),
                        anomalies=rec.get("anomalies", []),
                        key_signals=rec.get("key_signals", {}),
                    ))

    md_path = gen.save_markdown()
    html_path = gen.save_html()
    json_path = gen.save_json()
    print(f"✅ Reports generated:")
    print(f"   Markdown: {md_path}")
    print(f"   HTML:     {html_path}")
    print(f"   JSON:     {json_path}")
