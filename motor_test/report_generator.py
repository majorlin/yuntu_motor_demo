#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Report Generator
=======================
Generates Markdown + HTML test reports with embedded waveforms,
test results summary, test procedures, process logs, and anomaly analysis.
"""

from __future__ import annotations

import base64
import json
import os
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional


class TestVerdict(Enum):
    __test__ = False

    PASS = "PASS"
    FAIL = "FAIL"
    SKIP = "SKIP"
    ERROR = "ERROR"


@dataclass
class TestRecord:
    """Complete record of a single test case execution."""
    __test__ = False

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
    # ── New fields for test process details ──
    test_steps: List[str] = field(default_factory=list)
    test_logs: List[str] = field(default_factory=list)


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

            # Test steps
            if r.test_steps:
                lines.append("**测试步骤:**")
                lines.append("")
                for i, step in enumerate(r.test_steps, 1):
                    lines.append(f"{i}. {step}")
                lines.append("")

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

            # Test logs
            if r.test_logs:
                lines.append("**测试过程日志:**")
                lines.append("```")
                for log_line in r.test_logs:
                    lines.append(log_line)
                lines.append("```")
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
                    "description": r.description,
                    "verdict": r.verdict.value,
                    "duration_s": r.duration_s,
                    "preconditions": r.preconditions,
                    "test_method": r.test_method,
                    "pass_criteria": r.pass_criteria,
                    "actual_result": r.actual_result,
                    "anomalies": r.anomalies,
                    "key_signals": r.key_signals,
                    "improvement_suggestions": r.improvement_suggestions,
                    "timestamp": r.timestamp,
                    "test_steps": r.test_steps,
                    "test_logs": r.test_logs,
                    "waveform_files": r.waveform_files,
                }
                for r in self.records
            ],
        }
        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return filepath

    # ── Helper: embed image ─────────────────────────────────────────────────

    @staticmethod
    def _embed_image_base64(filepath: str) -> str:
        """Read an image file and return a base64 data-URI string."""
        if not filepath or not os.path.isfile(filepath):
            return ""
        ext = os.path.splitext(filepath)[1].lower()
        mime = {
            ".png": "image/png",
            ".jpg": "image/jpeg",
            ".jpeg": "image/jpeg",
            ".svg": "image/svg+xml",
            ".gif": "image/gif",
        }.get(ext, "image/png")
        try:
            with open(filepath, "rb") as f:
                data = base64.b64encode(f.read()).decode("ascii")
            return f"data:{mime};base64,{data}"
        except Exception:
            return ""

    # ── HTML Report ──────────────────────────────────────────────────────────

    def save_html(self, filename: Optional[str] = None, **kwargs) -> str:
        """Generate and save a rich HTML report with embedded styling and images."""
        if filename is None:
            filename = f"test_report_{time.strftime('%Y%m%d_%H%M%S')}.html"
        filepath = os.path.join(self.output_dir, filename)

        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        total = len(self.records)
        passed = sum(1 for r in self.records if r.verdict == TestVerdict.PASS)
        failed = sum(1 for r in self.records if r.verdict == TestVerdict.FAIL)
        skipped = sum(1 for r in self.records if r.verdict == TestVerdict.SKIP)
        errors = sum(1 for r in self.records if r.verdict == TestVerdict.ERROR)
        pass_rate = f"{passed/total*100:.1f}" if total > 0 else "0.0"

        # Build nav items and detail cards
        nav_items = []
        detail_cards = []
        for idx, r in enumerate(self.records):
            icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭", "ERROR": "⚠️"}[r.verdict.value]
            verdict_class = r.verdict.value.lower()
            card_id = f"tc-{idx}"

            nav_items.append(
                f'<a href="#{card_id}" class="nav-item {verdict_class}">'
                f'{icon} {r.test_id}: {r.test_name}</a>'
            )

            # Build card sections
            card_html = f'''
            <div class="card" id="{card_id}">
              <div class="card-header {verdict_class}">
                <h3>{icon} {r.test_id}: {r.test_name}</h3>
                <span class="badge {verdict_class}">{r.verdict.value}</span>
              </div>
              <div class="card-body">
                <table class="info-table">
                  <tr><th>描述</th><td>{self._esc(r.description)}</td></tr>
                  <tr><th>前置条件</th><td>{self._esc(r.preconditions)}</td></tr>
                  <tr><th>测试方法</th><td>{self._esc(r.test_method)}</td></tr>
                  <tr><th>判定标准</th><td>{self._esc(r.pass_criteria)}</td></tr>
                  <tr><th>实际结果</th><td><strong>{self._esc(r.actual_result)}</strong></td></tr>
                  <tr><th>判定</th><td class="{verdict_class}">{icon} {r.verdict.value}</td></tr>
                  <tr><th>耗时</th><td>{r.duration_s:.1f}s</td></tr>
                  <tr><th>测试时间</th><td>{r.timestamp}</td></tr>
                </table>'''

            # Test steps
            if r.test_steps:
                card_html += '''
                <div class="section">
                  <h4>📋 测试步骤</h4>
                  <ol class="steps-list">'''
                for step in r.test_steps:
                    card_html += f'<li>{self._esc(step)}</li>'
                card_html += '</ol></div>'

            # Key signals
            if r.key_signals:
                card_html += '''
                <div class="section">
                  <h4>📊 关键信号数据</h4>
                  <table class="data-table">
                    <tr><th>信号</th><th>值</th></tr>'''
                for k, v in r.key_signals.items():
                    card_html += f'<tr><td>{self._esc(k)}</td><td>{self._esc(v)}</td></tr>'
                card_html += '</table></div>'

            # Anomalies
            if r.anomalies:
                card_html += '''
                <div class="section">
                  <h4>⚠️ 检测到的异常</h4>
                  <ul class="anomaly-list">'''
                for a in r.anomalies:
                    card_html += f'<li>{self._esc(a)}</li>'
                card_html += '</ul></div>'

            # Improvement suggestions
            if r.improvement_suggestions:
                card_html += '''
                <div class="section">
                  <h4>💡 改善建议</h4>
                  <ul class="suggestion-list">'''
                for s in r.improvement_suggestions:
                    card_html += f'<li>{self._esc(s)}</li>'
                card_html += '</ul></div>'

            # Waveform images (embedded as base64)
            if r.waveform_files:
                card_html += '''
                <div class="section">
                  <h4>📈 波形记录</h4>
                  <div class="waveform-gallery">'''
                for wf in r.waveform_files:
                    data_uri = self._embed_image_base64(wf)
                    basename = os.path.basename(wf) if wf else ""
                    if data_uri:
                        card_html += f'''
                    <div class="waveform-item">
                      <img src="{data_uri}" alt="{self._esc(basename)}" />
                      <p class="waveform-caption">{self._esc(basename)}</p>
                    </div>'''
                    else:
                        card_html += f'<p class="waveform-missing">📎 {self._esc(basename)} (文件未找到)</p>'
                card_html += '</div></div>'

            # Test logs (collapsible)
            if r.test_logs:
                card_html += f'''
                <div class="section">
                  <details>
                    <summary><h4 style="display:inline">📝 测试过程日志 ({len(r.test_logs)} 条)</h4></summary>
                    <pre class="log-block">'''
                for log_line in r.test_logs:
                    card_html += self._esc(log_line) + '\n'
                card_html += '</pre></details></div>'

            # Error message
            if r.error_message:
                card_html += f'''
                <div class="error-block">
                  ⚠️ 错误信息: {self._esc(r.error_message)}
                </div>'''

            card_html += '</div></div>'
            detail_cards.append(card_html)

        nav_html = "\n".join(nav_items)
        cards_html = "\n".join(detail_cards)

        # Summary table rows
        summary_rows = ""
        for r in self.records:
            icon = {"PASS": "✅", "FAIL": "❌", "SKIP": "⏭", "ERROR": "⚠️"}[r.verdict.value]
            verdict_class = r.verdict.value.lower()
            info = self._esc(r.actual_result[:80]) if r.actual_result else ""
            idx = self.records.index(r)
            summary_rows += f'''
            <tr class="{verdict_class}-row">
              <td><a href="#tc-{idx}">{self._esc(r.test_id)}</a></td>
              <td>{self._esc(r.test_name)}</td>
              <td class="{verdict_class}">{icon} {r.verdict.value}</td>
              <td>{r.duration_s:.1f}s</td>
              <td class="info-cell">{info}</td>
            </tr>'''

        html = f'''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>电机测试报告 - {ts}</title>
<style>
  :root {{
    --bg-primary: #0f0f1a;
    --bg-secondary: #1a1a2e;
    --bg-card: #16213e;
    --bg-card-header: #1a1a3e;
    --border: #2a2a4a;
    --text-primary: #e0e0f0;
    --text-secondary: #8888aa;
    --accent-blue: #4fc3f7;
    --accent-green: #66bb6a;
    --accent-red: #ef5350;
    --accent-yellow: #ffa726;
    --accent-purple: #ab47bc;
  }}

  * {{ margin: 0; padding: 0; box-sizing: border-box; }}

  body {{
    font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', 'PingFang SC',
                 'Hiragino Sans GB', 'Microsoft YaHei', sans-serif;
    background: var(--bg-primary);
    color: var(--text-primary);
    line-height: 1.6;
  }}

  .layout {{
    display: flex;
    min-height: 100vh;
  }}

  /* ── Sidebar Navigation ── */
  .sidebar {{
    width: 280px;
    background: var(--bg-secondary);
    border-right: 1px solid var(--border);
    padding: 20px 0;
    position: fixed;
    top: 0;
    left: 0;
    bottom: 0;
    overflow-y: auto;
    z-index: 10;
  }}

  .sidebar h2 {{
    color: var(--accent-blue);
    font-size: 14px;
    padding: 0 16px 12px;
    border-bottom: 1px solid var(--border);
    margin-bottom: 8px;
  }}

  .nav-item {{
    display: block;
    padding: 8px 16px;
    color: var(--text-secondary);
    text-decoration: none;
    font-size: 13px;
    border-left: 3px solid transparent;
    transition: all 0.2s;
  }}

  .nav-item:hover {{
    background: rgba(79, 195, 247, 0.08);
    color: var(--text-primary);
  }}

  .nav-item.pass {{ border-left-color: var(--accent-green); }}
  .nav-item.fail {{ border-left-color: var(--accent-red); }}
  .nav-item.skip {{ border-left-color: var(--accent-yellow); }}
  .nav-item.error {{ border-left-color: var(--accent-yellow); }}

  /* ── Main Content ── */
  .main {{
    flex: 1;
    margin-left: 280px;
    padding: 32px 40px;
    max-width: 1100px;
  }}

  .report-header {{
    margin-bottom: 32px;
  }}

  .report-header h1 {{
    color: var(--accent-blue);
    font-size: 24px;
    margin-bottom: 8px;
  }}

  .report-meta {{
    color: var(--text-secondary);
    font-size: 13px;
    line-height: 1.8;
  }}

  /* ── Stats Cards ── */
  .stats-row {{
    display: flex;
    gap: 16px;
    margin-bottom: 32px;
    flex-wrap: wrap;
  }}

  .stat-card {{
    flex: 1;
    min-width: 120px;
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: 8px;
    padding: 16px;
    text-align: center;
  }}

  .stat-card .stat-value {{
    font-size: 28px;
    font-weight: 700;
  }}

  .stat-card .stat-label {{
    font-size: 12px;
    color: var(--text-secondary);
    margin-top: 4px;
  }}

  .stat-card.total .stat-value {{ color: var(--accent-blue); }}
  .stat-card.pass .stat-value {{ color: var(--accent-green); }}
  .stat-card.fail .stat-value {{ color: var(--accent-red); }}
  .stat-card.rate .stat-value {{ color: var(--accent-purple); }}

  /* ── Summary Table ── */
  .summary-table {{
    width: 100%;
    border-collapse: collapse;
    margin-bottom: 32px;
    font-size: 14px;
  }}

  .summary-table th {{
    background: var(--bg-card-header);
    color: var(--accent-blue);
    padding: 10px 12px;
    text-align: left;
    border-bottom: 2px solid var(--border);
    font-weight: 600;
  }}

  .summary-table td {{
    padding: 10px 12px;
    border-bottom: 1px solid var(--border);
  }}

  .summary-table tr:hover {{
    background: rgba(79, 195, 247, 0.04);
  }}

  .summary-table a {{
    color: var(--accent-blue);
    text-decoration: none;
  }}

  .summary-table a:hover {{
    text-decoration: underline;
  }}

  .info-cell {{
    color: var(--text-secondary);
    font-size: 12px;
    max-width: 300px;
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }}

  /* ── Detail Cards ── */
  .card {{
    background: var(--bg-card);
    border: 1px solid var(--border);
    border-radius: 8px;
    margin-bottom: 24px;
    overflow: hidden;
  }}

  .card-header {{
    display: flex;
    justify-content: space-between;
    align-items: center;
    padding: 14px 20px;
    border-bottom: 1px solid var(--border);
  }}

  .card-header h3 {{
    font-size: 16px;
    font-weight: 600;
  }}

  .card-header.pass {{ border-left: 4px solid var(--accent-green); }}
  .card-header.fail {{ border-left: 4px solid var(--accent-red); }}
  .card-header.skip {{ border-left: 4px solid var(--accent-yellow); }}
  .card-header.error {{ border-left: 4px solid var(--accent-yellow); }}

  .badge {{
    padding: 4px 12px;
    border-radius: 12px;
    font-size: 12px;
    font-weight: 600;
  }}

  .badge.pass {{ background: rgba(102,187,106,0.15); color: var(--accent-green); }}
  .badge.fail {{ background: rgba(239,83,80,0.15); color: var(--accent-red); }}
  .badge.skip {{ background: rgba(255,167,38,0.15); color: var(--accent-yellow); }}
  .badge.error {{ background: rgba(255,167,38,0.15); color: var(--accent-yellow); }}

  .card-body {{
    padding: 20px;
  }}

  .info-table {{
    width: 100%;
    border-collapse: collapse;
    font-size: 14px;
  }}

  .info-table th {{
    width: 120px;
    text-align: right;
    padding: 8px 16px 8px 8px;
    color: var(--text-secondary);
    font-weight: 500;
    vertical-align: top;
    border-bottom: 1px solid rgba(42,42,74,0.5);
  }}

  .info-table td {{
    padding: 8px;
    border-bottom: 1px solid rgba(42,42,74,0.5);
  }}

  .pass {{ color: var(--accent-green); }}
  .fail {{ color: var(--accent-red); }}
  .skip {{ color: var(--accent-yellow); }}
  .error {{ color: var(--accent-yellow); }}

  .section {{
    margin-top: 16px;
    padding-top: 16px;
    border-top: 1px solid rgba(42,42,74,0.5);
  }}

  .section h4 {{
    color: var(--accent-blue);
    font-size: 14px;
    margin-bottom: 10px;
  }}

  /* ── Steps ── */
  .steps-list {{
    padding-left: 24px;
    font-size: 14px;
  }}

  .steps-list li {{
    padding: 4px 0;
    color: var(--text-primary);
  }}

  .steps-list li::marker {{
    color: var(--accent-blue);
    font-weight: 600;
  }}

  /* ── Data Table ── */
  .data-table {{
    width: auto;
    border-collapse: collapse;
    font-size: 13px;
  }}

  .data-table th {{
    background: rgba(26,26,62,0.5);
    color: var(--accent-blue);
    padding: 6px 16px;
    text-align: left;
    font-weight: 600;
  }}

  .data-table td {{
    padding: 6px 16px;
    border-bottom: 1px solid rgba(42,42,74,0.3);
    font-family: 'Menlo', 'Consolas', monospace;
  }}

  /* ── Anomalies & Suggestions ── */
  .anomaly-list li {{
    color: var(--accent-yellow);
    font-size: 13px;
    padding: 2px 0;
  }}

  .suggestion-list li {{
    color: var(--accent-purple);
    font-size: 13px;
    padding: 2px 0;
  }}

  /* ── Waveforms ── */
  .waveform-gallery {{
    display: flex;
    flex-direction: column;
    gap: 16px;
  }}

  .waveform-item img {{
    max-width: 100%;
    border-radius: 6px;
    border: 1px solid var(--border);
  }}

  .waveform-caption {{
    font-size: 12px;
    color: var(--text-secondary);
    margin-top: 4px;
    text-align: center;
  }}

  .waveform-missing {{
    color: var(--text-secondary);
    font-size: 13px;
    font-style: italic;
  }}

  /* ── Log Block ── */
  .log-block {{
    background: #0a0a14;
    border: 1px solid var(--border);
    border-radius: 6px;
    padding: 12px 16px;
    font-family: 'Menlo', 'Consolas', monospace;
    font-size: 12px;
    line-height: 1.5;
    overflow-x: auto;
    color: #a0a0c0;
    max-height: 400px;
    overflow-y: auto;
  }}

  details {{
    cursor: pointer;
  }}

  details summary {{
    color: var(--accent-blue);
    font-size: 14px;
    padding: 4px 0;
  }}

  details summary:hover {{
    color: var(--text-primary);
  }}

  /* ── Error Block ── */
  .error-block {{
    margin-top: 12px;
    padding: 10px 16px;
    background: rgba(239,83,80,0.08);
    border-left: 3px solid var(--accent-red);
    border-radius: 4px;
    color: var(--accent-red);
    font-size: 13px;
  }}

  /* ── Section Title ── */
  .section-title {{
    color: var(--accent-blue);
    font-size: 18px;
    margin: 32px 0 16px;
    padding-bottom: 8px;
    border-bottom: 1px solid var(--border);
  }}

  /* ── Responsive ── */
  @media (max-width: 768px) {{
    .sidebar {{ display: none; }}
    .main {{ margin-left: 0; padding: 16px; }}
    .stats-row {{ flex-direction: column; }}
  }}

  /* ── Pass/Fail row highlights ── */
  .pass-row td {{ }}
  .fail-row td {{ background: rgba(239,83,80,0.04); }}
</style>
</head>
<body>
<div class="layout">
  <!-- Sidebar -->
  <nav class="sidebar">
    <h2>📋 测试导航</h2>
    {nav_html}
  </nav>

  <!-- Main Content -->
  <div class="main">
    <div class="report-header">
      <h1>🔧 鼓风机电机测试报告</h1>
      <div class="report-meta">
        生成时间: {ts}<br>
        测试平台: YTM32 Sensorless FOC / CAN FD 500K/2M<br>
        通信工具: YuntuCanLin Debugger
      </div>
    </div>

    <!-- Stats -->
    <div class="stats-row">
      <div class="stat-card total">
        <div class="stat-value">{total}</div>
        <div class="stat-label">总计</div>
      </div>
      <div class="stat-card pass">
        <div class="stat-value">{passed}</div>
        <div class="stat-label">✅ 通过</div>
      </div>
      <div class="stat-card fail">
        <div class="stat-value">{failed}</div>
        <div class="stat-label">❌ 失败</div>
      </div>
      <div class="stat-card rate">
        <div class="stat-value">{pass_rate}%</div>
        <div class="stat-label">通过率</div>
      </div>
    </div>

    <!-- Summary Table -->
    <h2 class="section-title">测试用例汇总</h2>
    <table class="summary-table">
      <thead>
        <tr>
          <th>ID</th>
          <th>测试项</th>
          <th>结果</th>
          <th>耗时</th>
          <th>关键信息</th>
        </tr>
      </thead>
      <tbody>
        {summary_rows}
      </tbody>
    </table>

    <!-- Detail Cards -->
    <h2 class="section-title">详细测试记录</h2>
    {cards_html}

    <footer style="text-align:center; color:var(--text-secondary); font-size:12px; padding:32px 0;">
      Motor Test Report Generator &copy; Yuntu Motor • {ts}
    </footer>
  </div>
</div>
</body>
</html>'''

        with open(filepath, "w", encoding="utf-8") as f:
            f.write(html)
        return filepath

    @staticmethod
    def _esc(text: str) -> str:
        """Escape HTML special characters."""
        if not text:
            return ""
        return (text
                .replace("&", "&amp;")
                .replace("<", "&lt;")
                .replace(">", "&gt;")
                .replace('"', "&quot;"))


# ── CLI Entry Point ─────────────────────────────────────────────────────────

def _iter_report_json_files(*roots: str) -> List[str]:
    """Collect candidate report JSON files recursively from one or more roots."""
    json_files: List[str] = []
    seen = set()

    for root in roots:
        if not root or not os.path.isdir(root):
            continue
        for dirpath, _, filenames in os.walk(root):
            for fname in sorted(filenames):
                if not fname.endswith(".json"):
                    continue
                full_path = os.path.join(dirpath, fname)
                if full_path in seen:
                    continue
                seen.add(full_path)
                json_files.append(full_path)

    return sorted(json_files)


def _record_from_dict(rec: Dict[str, object]) -> TestRecord:
    """Convert one JSON record dict into a TestRecord."""
    verdict_raw = str(rec.get("verdict", "SKIP"))
    try:
        verdict = TestVerdict(verdict_raw)
    except ValueError:
        verdict = TestVerdict.SKIP

    return TestRecord(
        test_id=str(rec.get("test_id", "")),
        test_name=str(rec.get("test_name", "")),
        description=str(rec.get("description", "")),
        verdict=verdict,
        duration_s=float(rec.get("duration_s", 0.0) or 0.0),
        preconditions=str(rec.get("preconditions", "")),
        test_method=str(rec.get("test_method", "")),
        pass_criteria=str(rec.get("pass_criteria", "")),
        actual_result=str(rec.get("actual_result", "")),
        anomalies=list(rec.get("anomalies", []) or []),
        waveform_files=list(rec.get("waveform_files", []) or []),
        data_files=list(rec.get("data_files", []) or []),
        key_signals=dict(rec.get("key_signals", {}) or {}),
        improvement_suggestions=list(rec.get("improvement_suggestions", []) or []),
        timestamp=str(rec.get("timestamp", "")),
        error_message=str(rec.get("error_message", "")),
        test_steps=list(rec.get("test_steps", []) or []),
        test_logs=list(rec.get("test_logs", []) or []),
    )


def _record_dedup_key(record: TestRecord) -> tuple:
    """Stable key for collapsing duplicate records across multiple JSON reports."""
    return (
        record.test_id,
        record.test_name,
        record.timestamp,
        record.verdict.value,
        record.actual_result,
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description="Generate an aggregate test report from JSON result files"
    )
    parser.add_argument("--input", type=str, default="motor_test/test_records",
                        help="Primary directory to scan recursively for JSON result files")
    parser.add_argument("--output", type=str, default="motor_test/test_reports")
    parser.add_argument("--title", type=str, default="电机测试汇总报告")
    args = parser.parse_args()

    gen = ReportGenerator(output_dir=args.output)
    json_files = _iter_report_json_files(args.input)

    # Common usage is --input test_records --output test_reports. The raw
    # records directory often contains CSV/PNG only, while the actual JSON
    # session summaries live under test_reports. Fall back automatically.
    if not json_files:
      json_files = _iter_report_json_files(args.output)

    seen_keys = set()
    loaded_count = 0

    for json_path in json_files:
        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            print(f"⚠️ Skip unreadable JSON: {json_path} ({exc})")
            continue

        if "records" not in data or not isinstance(data["records"], list):
            continue

        for rec in data["records"]:
            record = _record_from_dict(rec)
            dedup_key = _record_dedup_key(record)
            if dedup_key in seen_keys:
                continue
            seen_keys.add(dedup_key)
            gen.add_record(record)
            loaded_count += 1

    if not gen.records:
        print("⚠️ No JSON test records found to aggregate.")
        print(f"   scanned input:  {args.input}")
        print(f"   scanned output: {args.output}")
        raise SystemExit(1)

    md_path = gen.save_markdown(title=args.title)
    html_path = gen.save_html(title=args.title)
    json_path = gen.save_json()
    print(f"✅ Aggregate reports generated from {loaded_count} unique records")
    print(f"   JSON sources scanned: {len(json_files)}")
    print(f"   Markdown: {md_path}")
    print(f"   HTML:     {html_path}")
    print(f"   JSON:     {json_path}")
