#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CAN FD Communication Interface
================================
Wraps YuntuCanBus + cantools DBC for motor telemetry and command.
Provides thread-safe signal access and CSV logging.
"""

from __future__ import annotations

import csv
import glob
import os
import sys
import threading
import time
import types
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

if "asammdf" not in sys.modules:
    # python-can optionally imports asammdf for MF4 logging support. In this
    # workstation the installed asammdf/PySide stack aborts during import, but
    # the CAN test tools do not use MF4 support. A tiny stub forces python-can
    # to treat MF4 as unavailable instead of crashing the process.
    sys.modules["asammdf"] = types.ModuleType("asammdf")

import can
import cantools

# ── Path setup ──────────────────────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DEBUGGER_DIR = os.path.normpath(
    os.path.join(PROJECT_DIR, "..", "..", "Tools", "2_YuntuCanLinDebugger", "python")
)
if DEBUGGER_DIR not in sys.path:
    sys.path.insert(0, DEBUGGER_DIR)

from yuntu_debugger.can_bus import YuntuCanBus      # noqa: E402
from yuntu_debugger.transport import SerialTransport # noqa: E402

DEFAULT_DBC = os.path.join(PROJECT_DIR, "docs", "motor_control.dbc")


@dataclass
class SignalSnapshot:
    """Thread-safe snapshot of all decoded CAN signals at a point in time."""
    timestamp: float = 0.0
    values: Dict[str, float] = field(default_factory=dict)
    rx_count: int = 0


class CanInterface:
    """
    CAN FD communication wrapper with DBC decoding, CSV logging,
    and signal subscription.
    """

    def __init__(
        self,
        port: Optional[str] = None,
        channel: int = 0,
        dbc_path: str = DEFAULT_DBC,
        auto_connect: bool = False,
    ):
        self.port = port or self._auto_detect_port()
        self.channel = channel
        self.db: cantools.Database = cantools.database.load_file(dbc_path)
        self.bus: Optional[YuntuCanBus] = None
        self.transport: Optional[SerialTransport] = None

        # Thread-safe signal store
        self._lock = threading.Lock()
        self._signals: Dict[str, float] = {}
        self._rx_count = 0
        self._tx_count = 0
        self._t0: Optional[float] = None

        # RX thread
        self._rx_running = False
        self._rx_thread: Optional[threading.Thread] = None

        # Signal history for waveform capture
        self._history: List[Dict[str, Any]] = []
        self._history_enabled = False
        self._max_history = 100_000

        # CSV logger
        self._csv_file = None
        self._csv_writer = None

        # Callbacks
        self._callbacks: List[Callable[[str, Dict[str, float]], None]] = []

        if auto_connect:
            self.connect()

    # ── Connection ──────────────────────────────────────────────────────────

    def connect(self) -> None:
        """Open serial transport and start CAN FD communication."""
        self.transport = SerialTransport(port=self.port, baudrate=2_000_000)
        self.transport.start()
        self.bus = YuntuCanBus(
            channel=self.channel,
            transport=self.transport,
            bitrate=500_000,
            fd=True,
            data_bitrate=2_000_000,
            sample_point=0.70,
            data_sample_point=0.70,
        )
        self._rx_running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()

    def disconnect(self) -> None:
        """Stop RX and close CAN bus."""
        self._rx_running = False
        if self._rx_thread:
            self._rx_thread.join(timeout=2.0)
        try:
            if self.bus:
                self.bus.shutdown()
        except Exception:
            pass
        try:
            if self.transport:
                self.transport.close()
        except Exception:
            pass
        self.bus = None
        self.transport = None
        self.stop_csv_log()

    @property
    def connected(self) -> bool:
        return self.bus is not None and self._rx_running

    # ── Signal Access ───────────────────────────────────────────────────────

    def get_signal(self, name: str, default: float = 0.0) -> float:
        """Get latest value of a decoded signal."""
        with self._lock:
            return self._signals.get(name, default)

    def get_signals(self, names: List[str]) -> Dict[str, float]:
        """Get multiple signals atomically."""
        with self._lock:
            return {n: self._signals.get(n, 0.0) for n in names}

    def get_all_signals(self) -> Dict[str, float]:
        """Get snapshot of all signals."""
        with self._lock:
            return dict(self._signals)

    def snapshot(self) -> SignalSnapshot:
        """Get a thread-safe snapshot."""
        with self._lock:
            return SignalSnapshot(
                timestamp=time.monotonic() - (self._t0 or time.monotonic()),
                values=dict(self._signals),
                rx_count=self._rx_count,
            )

    @property
    def rx_count(self) -> int:
        with self._lock:
            return self._rx_count

    @property
    def tx_count(self) -> int:
        with self._lock:
            return self._tx_count

    # ── History / Waveform ──────────────────────────────────────────────────

    def enable_history(self, max_points: int = 100_000) -> None:
        """Start buffering signal history for waveform analysis."""
        self._max_history = max_points
        with self._lock:
            self._history.clear()
            self._history_enabled = True

    def disable_history(self) -> None:
        self._history_enabled = False

    def get_history(self) -> List[Dict[str, Any]]:
        """Return a copy of the signal history."""
        with self._lock:
            return list(self._history)

    def clear_history(self) -> None:
        with self._lock:
            self._history.clear()

    # ── Callbacks ───────────────────────────────────────────────────────────

    def add_callback(self, cb: Callable[[str, Dict[str, float]], None]) -> None:
        """Register a callback: cb(msg_name, decoded_signals)."""
        self._callbacks.append(cb)

    # ── TX ──────────────────────────────────────────────────────────────────

    def send_message(self, msg_name: str, signals: Dict[str, Any]) -> None:
        """Encode and send a CAN FD message by DBC name."""
        if not self.bus:
            raise RuntimeError("CAN bus not connected")
        db_msg = self.db.get_message_by_name(msg_name)
        full_signals = {}
        for sig in db_msg.signals:
            if sig.name in signals:
                full_signals[sig.name] = signals[sig.name]
            else:
                full_signals[sig.name] = 0.0 if sig.is_float else 0
        data = db_msg.encode(full_signals)
        msg = can.Message(
            arbitration_id=db_msg.frame_id,
            data=data,
            is_extended_id=False,
            is_fd=True,
            bitrate_switch=True,
        )
        self.bus.send(msg)
        with self._lock:
            self._tx_count += 1

    # ── CSV Logging ─────────────────────────────────────────────────────────

    def start_csv_log(self, filepath: str, signal_names: Optional[List[str]] = None):
        """Start logging decoded signals to CSV."""
        self._csv_signal_names = signal_names
        self._csv_file = open(filepath, "w", newline="")
        # Header written on first record
        self._csv_writer = None
        self._csv_path = filepath

    def stop_csv_log(self) -> Optional[str]:
        """Stop CSV logging, return file path."""
        path = getattr(self, "_csv_path", None)
        if self._csv_file:
            self._csv_file.close()
            self._csv_file = None
            self._csv_writer = None
        return path

    # ── RX Loop ─────────────────────────────────────────────────────────────

    def _rx_loop(self):
        while self._rx_running and self.bus is not None:
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is None:
                    continue
                self._decode(msg)
            except Exception:
                if self._rx_running:
                    continue
                break

    def _decode(self, msg: can.Message):
        try:
            db_msg = self.db.get_message_by_frame_id(msg.arbitration_id)
        except KeyError:
            return
        try:
            decoded = db_msg.decode(msg.data, decode_choices=False)
        except Exception:
            return

        now = time.monotonic()
        with self._lock:
            if self._t0 is None:
                self._t0 = now
            t = now - self._t0
            self._rx_count += 1

            for sig_name, value in decoded.items():
                self._signals[sig_name] = float(value)

            # History
            if self._history_enabled:
                record = {"_t": t, "_msg": db_msg.name}
                record.update({k: float(v) for k, v in decoded.items()})
                self._history.append(record)
                if len(self._history) > self._max_history:
                    self._history.pop(0)

            # CSV
            if self._csv_file is not None:
                row = {"_timestamp": t, "_msg": db_msg.name}
                if self._csv_signal_names:
                    row.update({k: self._signals.get(k, "") for k in self._csv_signal_names})
                else:
                    row.update(dict(self._signals))
                if self._csv_writer is None:
                    self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=list(row.keys()))
                    self._csv_writer.writeheader()
                self._csv_writer.writerow(row)
                self._csv_file.flush()

        # Fire callbacks outside lock
        for cb in self._callbacks:
            try:
                cb(db_msg.name, decoded)
            except Exception:
                pass

    # ── Utility ─────────────────────────────────────────────────────────────

    @staticmethod
    def _auto_detect_port() -> str:
        patterns = ["/dev/cu.usbserial-*", "/dev/ttyUSB*", "/dev/ttyACM*"]
        for p in patterns:
            found = sorted(glob.glob(p))
            if found:
                return found[0]
        raise RuntimeError("No serial port detected. Specify --port explicitly.")

    def wait_for_signal(
        self, name: str, predicate: Callable[[float], bool],
        timeout: float = 10.0, poll_interval: float = 0.01,
    ) -> bool:
        """Block until signal satisfies predicate or timeout."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            val = self.get_signal(name)
            if predicate(val):
                return True
            time.sleep(poll_interval)
        return False

    def wait_for_state(self, target_state: int, timeout: float = 10.0) -> bool:
        """Wait until motor State signal equals target_state."""
        return self.wait_for_signal("State", lambda v: int(v) == target_state, timeout)

    def collect_signals(
        self, names: List[str], duration_s: float, interval_s: float = 0.01,
    ) -> List[Dict[str, float]]:
        """Collect signal samples over a duration. Returns list of {signal: value, _t: time}."""
        samples = []
        t0 = time.monotonic()
        while (time.monotonic() - t0) < duration_s:
            row = self.get_signals(names)
            row["_t"] = time.monotonic() - t0
            samples.append(row)
            time.sleep(interval_s)
        return samples
