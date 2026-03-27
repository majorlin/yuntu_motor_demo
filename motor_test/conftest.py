#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pytest Shared Fixtures
========================
Provides CAN connection, motor client, motor profile, report generator,
and test logger as shared fixtures for all test cases.

Motor profile selection:
    pytest motor_test/ --motor-profile seat
    pytest motor_test/ --motor-profile fan

Use ``-s`` to see real-time test logs in the console:
    pytest motor_test/ -s --motor-profile fan
"""

import os
import sys
import time

import pytest

# Ensure motor_test/ directory is on path for local imports
TEST_DIR = os.path.dirname(os.path.abspath(__file__))
if TEST_DIR not in sys.path:
    sys.path.insert(0, TEST_DIR)

from can_interface import CanInterface
from motor_client import MotorClient
from waveform_capture import WaveformCapture
from anomaly_analyzer import AnomalyAnalyzer
from report_generator import ReportGenerator
from motor_profiles import load_profile, list_profiles
from test_logger import TestLogger


def pytest_addoption(parser):
    parser.addoption("--port", action="store", default=None, help="Serial port for CAN")
    parser.addoption("--channel", action="store", default=0, type=int, help="CAN channel")
    parser.addoption("--no-hardware", action="store_true", help="Skip hardware-dependent tests")
    parser.addoption(
        "--motor-profile",
        action="store",
        default="seat",
        help=f"Motor profile name (available: {list_profiles()})",
    )


@pytest.fixture(scope="session")
def motor_profile(request):
    """Session-scoped motor profile loaded from YAML.

    Usage in tests:
        def test_example(motor_profile):
            max_rpm = motor_profile["limits"]["max_rpm"]
    """
    name = request.config.getoption("--motor-profile")
    profile = load_profile(name)
    print(f"\n🔧 Motor profile: {profile['motor_name']} ({name})")
    return profile


@pytest.fixture(scope="session")
def can(request) -> CanInterface:
    """Session-scoped CAN interface. Auto-connects."""
    port = request.config.getoption("--port")
    channel = request.config.getoption("--channel")
    ci = CanInterface(port=port, channel=channel, auto_connect=True)
    # Wait for initial data
    time.sleep(0.5)
    yield ci
    ci.disconnect()


@pytest.fixture(scope="session")
def motor(can) -> MotorClient:
    """Session-scoped motor client."""
    return MotorClient(can)


@pytest.fixture(scope="session")
def waveform(can) -> WaveformCapture:
    """Session-scoped waveform capture."""
    return WaveformCapture(can, output_dir=os.path.join(TEST_DIR, "test_records"))


@pytest.fixture(scope="session")
def analyzer(can, motor) -> AnomalyAnalyzer:
    """Session-scoped anomaly analyzer."""
    return AnomalyAnalyzer(can, motor)


@pytest.fixture(scope="session")
def report(request) -> ReportGenerator:
    """Session-scoped report generator. Auto-generates reports at session end."""
    gen = ReportGenerator(output_dir=os.path.join(TEST_DIR, "test_reports"))

    def _finalize():
        if gen.records:
            md_path = gen.save_markdown()
            html_path = gen.save_html()
            json_path = gen.save_json()
            print(f"\n{'━' * 70}")
            print(f"📋 测试报告已生成:")
            print(f"   HTML:     {html_path}")
            print(f"   Markdown: {md_path}")
            print(f"   JSON:     {json_path}")
            print(f"{'━' * 70}\n")

    request.addfinalizer(_finalize)
    return gen


@pytest.fixture
def test_logger(request) -> TestLogger:
    """Per-test logger providing real-time console output and log buffering.

    Usage in tests::

        def test_example(test_logger):
            test_logger.step("检查前置条件")
            test_logger.info(f"当前状态 = {state}")
            test_logger.step("执行操作")
            ...
            # Retrieve for report
            steps = test_logger.get_steps()
            logs = test_logger.get_logs()
    """
    test_name = request.node.name
    return TestLogger(test_name=test_name)


@pytest.fixture(autouse=True)
def ensure_stopped(motor):
    """Ensure motor is stopped before and after each test."""
    yield
    # Post-test cleanup: stop motor if running
    if not motor.is_stopped:
        motor.stop()
        time.sleep(1.0)


@pytest.fixture
def require_hardware(request):
    """Skip test if --no-hardware flag is set."""
    if request.config.getoption("--no-hardware"):
        pytest.skip("Skipped: --no-hardware flag set")


@pytest.fixture
def csv_logger(can, request):
    """Per-test CSV logger. File named after test function."""
    test_name = request.node.name
    filepath = os.path.join(TEST_DIR, "test_records", f"{test_name}_{int(time.time())}.csv")
    can.start_csv_log(filepath)
    yield filepath
    can.stop_csv_log()
