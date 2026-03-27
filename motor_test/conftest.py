#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pytest Shared Fixtures
========================
Provides CAN connection, motor client, and report generator
as shared fixtures for all test cases.
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


def pytest_addoption(parser):
    parser.addoption("--port", action="store", default=None, help="Serial port for CAN")
    parser.addoption("--channel", action="store", default=0, type=int, help="CAN channel")
    parser.addoption("--no-hardware", action="store_true", help="Skip hardware-dependent tests")


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
def report() -> ReportGenerator:
    """Session-scoped report generator."""
    return ReportGenerator(output_dir=os.path.join(TEST_DIR, "test_reports"))


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
