#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Motor Control CAN FD Debug GUI
===============================
Real-time motor telemetry viewer, command sender, and PI calibration tool.

Uses:
  - yuntu_debugger (YuntuCanBus) for UART-to-CAN FD communication
  - cantools for DBC signal encoding / decoding
  - tkinter for GUI framework
  - matplotlib for real-time signal curves

Usage:
  python3 motor_debugger_gui.py
  python3 motor_debugger_gui.py --port /dev/cu.usbserial-1130
  python3 motor_debugger_gui.py --dbc path/to/motor_control.dbc
"""

from __future__ import annotations

import argparse
import collections
import glob
import os
import struct
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
from typing import Any, Dict, Optional

import can
import cantools
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
import matplotlib.animation as animation

# Allow running from repo root
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEBUGGER_PYTHON_DIR = os.path.normpath(
    os.path.join(SCRIPT_DIR, "..", "..", "Tools", "2_YuntuCanLinDebugger", "python")
)
if DEBUGGER_PYTHON_DIR not in sys.path:
    sys.path.insert(0, DEBUGGER_PYTHON_DIR)

from yuntu_debugger.can_bus import YuntuCanBus
from yuntu_debugger.transport import SerialTransport

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
DEFAULT_DBC = os.path.join(SCRIPT_DIR, "docs", "motor_control.dbc")

STATE_MAP = {
    0: "STOP", 1: "OFFSET_CAL", 2: "WIND_DETECT", 3: "COAST_DOWN",
    4: "ALIGN", 5: "OPEN_LOOP", 6: "CLOSED_LOOP", 7: "PARAM_IDENT",
    8: "FAULT",
}

# Background-color hints per state for the banner
STATE_COLOR_MAP = {
    0: "#6c7086",    # STOP        — grey
    1: "#fab387",    # OFFSET_CAL  — peach
    2: "#89dceb",    # WIND_DETECT — teal
    3: "#f9e2af",    # COAST_DOWN  — yellow
    4: "#f9e2af",    # ALIGN       — yellow
    5: "#fab387",    # OPEN_LOOP   — peach/orange
    6: "#a6e3a1",    # CLOSED_LOOP — green (running OK)
    7: "#cba6f7",    # PARAM_IDENT — lavender
    8: "#f38ba8",    # FAULT       — red
}

FAULT_MAP = {
    0: "None",
    1: "ADC Overrun",
    2: "OverCurrent",
    3: "Vbus UnderVolt",
    4: "Vbus OverVolt",
    5: "Observer Loss",
    6: "Startup Timeout",
    7: "Bad Direction",
    8: "Catch Fail",
    9: "Startup Fail",
}
CTRL_MODE_MAP = {0: "Speed", 1: "Current"}

CHART_WINDOW_S = 10.0   # scrolling window width (seconds)
CHART_FPS = 20           # target chart update rate

# Curve group definitions: (subplot_index, signal_name, label, color)
CURVE_DEFS = [
    # Subplot 0: Speed
    (0, "MechanicalRpm",    "Mech RPM",      "#00e5ff"),
    (0, "TargetRpm",        "Target RPM",    "#ff9100"),
    # Subplot 1: Idq Current
    (1, "IqMeas",           "Iq Meas",       "#00e676"),
    (1, "IdMeas",           "Id Meas",       "#7c4dff"),
    (1, "IqTarget",         "Iq Target",     "#ff5252"),
    (1, "IdTarget",         "Id Target",     "#ffd740"),
    # Subplot 2: ABC Phase Current
    (2, "PhaseCurrentA",    "Ia",            "#f38ba8"),
    (2, "PhaseCurrentB",    "Ib",            "#a6e3a1"),
    (2, "PhaseCurrentC",    "Ic",            "#89b4fa"),
    # Subplot 3: Voltage & Modulation
    (3, "BusVoltage",       "Vbus",          "#18ffff"),
    (3, "VoltModRatio",     "Mod Ratio",     "#ff80ab"),
    # Subplot 4: Observer
    (4, "ObserverAngle",    "Obs Angle",     "#b388ff"),
    (4, "ElecAngle",        "Elec Angle",    "#69f0ae"),
    (4, "PhaseErrorRad",    "Phase Err",     "#ff6e40"),
]

# Status panel signal list: (signal_name, display_label, unit, format)
STATUS_SIGNALS = [
    ("State",               "State",            "",     None),
    ("FaultCode",           "Fault",            "",     None),
    ("ControlMode",         "Mode",             "",     None),
    ("MechanicalRpm",       "Mech RPM",         "rpm",  ".1f"),
    ("TargetRpm",           "Target RPM",       "rpm",  ".1f"),
    ("BusVoltage",          "Vbus",             "V",    ".2f"),
    ("IqMeas",              "Iq",               "A",    ".2f"),
    ("IdMeas",              "Id",               "A",    ".2f"),
    ("IqTarget",            "Iq*",              "A",    ".2f"),
    ("IdTarget",            "Id*",              "A",    ".2f"),
    ("ElecSpeedRadS",       "ωe",               "rad/s",".1f"),
    ("ElecAngle",           "θe",               "rad",  ".4f"),
    ("ObserverAngle",       "θ_obs",            "rad",  ".4f"),
    ("PLLSpeedRadS",        "ω_PLL",            "rad/s",".1f"),
    ("VoltModRatio",        "Mod Ratio",        "",     ".4f"),
    ("PhaseErrorRad",       "Phase Err",        "rad",  ".4f"),
    ("PhaseCurrentA",       "Ia",               "A",    ".2f"),
    ("PhaseCurrentB",       "Ib",               "A",    ".2f"),
    ("PhaseCurrentC",       "Ic",               "A",    ".2f"),
    ("DutyU",               "Duty U",           "",     ".4f"),
    ("DutyV",               "Duty V",           "",     ".4f"),
    ("DutyW",               "Duty W",           "",     ".4f"),
    ("SpeedPiIntegrator",   "Spd PI Int",       "A",    ".3f"),
    ("IqPiIntegrator",      "Iq PI Int",        "V",    ".3f"),
    ("IdPiIntegrator",      "Id PI Int",        "V",    ".3f"),
    ("ObserverFluxVs",      "Flux",             "Vs",   ".6f"),
    ("ClosedLoopBlend",     "CL Blend",         "",     ".4f"),
    ("OpenLoopSpeedRadS",   "OL Speed",         "rad/s",".1f"),
    ("FwIdTarget",          "FW Id*",           "A",    ".2f"),
    ("ChipTemperature",     "T_chip",           "°C",   ".1f"),
    ("PcbTemperature",      "T_pcb",            "°C",   ".1f"),
    ("StallDivCount",       "Stall Cnt",        "",     "d"),
    ("StateTimeMs",         "State Time",       "ms",   "d"),
    ("Timestamp",           "Timestamp",        "ms",   "d"),
    ("StartupRetryCount",   "Retry Cnt",        "",     "d"),
    ("Flags",               "Flags",            "",     "02X"),
]


# ---------------------------------------------------------------------------
# Application
# ---------------------------------------------------------------------------
class MotorDebuggerApp:
    def __init__(self, root: tk.Tk, dbc_path: str, initial_port: Optional[str] = None):
        self.root = root
        self.root.title("Motor Control CAN FD Debugger")
        self.root.geometry("1600x960")
        self.root.minsize(1200, 700)

        # State
        self.db: cantools.Database = cantools.database.load_file(dbc_path)
        self.bus: Optional[YuntuCanBus] = None
        self.transport: Optional[SerialTransport] = None
        self.connected = False
        self.rx_thread: Optional[threading.Thread] = None
        self.rx_running = False

        # Decoded signal values (thread-safe access via lock)
        self.lock = threading.Lock()
        self.signals: Dict[str, float] = {}
        self.rx_count = 0
        self.tx_count = 0

        # Curve data: signal_name -> deque of (time, value)
        self.curve_data: Dict[str, collections.deque] = {}
        self.t0: Optional[float] = None
        self.chart_frozen = False  # True when user has zoomed/panned manually
        self.hidden_signals: set = set()  # signal names toggled off via legend
        for _, sig_name, _, _ in CURVE_DEFS:
            self.curve_data[sig_name] = collections.deque(maxlen=2000)

        # Build GUI
        self._build_style()
        self._build_menu_bar()
        self._build_toolbar(initial_port)
        self._build_main_area()
        self._build_status_bar()

        # Start chart animation
        self.anim = animation.FuncAnimation(
            self.fig, self._update_chart, interval=1000 // CHART_FPS, blit=False, cache_frame_data=False,
        )

        # Periodic UI update for status values
        self._schedule_status_update()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---- Style -----------------------------------------------------------
    def _build_style(self):
        style = ttk.Style()
        style.theme_use("clam")
        BG = "#1e1e2e"
        FG = "#cdd6f4"
        ACCENT = "#89b4fa"
        style.configure(".", background=BG, foreground=FG, fieldbackground="#313244")
        style.configure("TLabel", background=BG, foreground=FG, font=("Menlo", 11))
        style.configure("TButton", background="#45475a", foreground=FG, font=("Menlo", 11))
        style.map("TButton", background=[("active", ACCENT)])
        style.configure("Header.TLabel", font=("Menlo", 12, "bold"), foreground=ACCENT)
        style.configure("Val.TLabel", font=("Menlo", 11), foreground="#a6e3a1")
        style.configure("Unit.TLabel", font=("Menlo", 10), foreground="#6c7086")
        style.configure("Status.TLabel", font=("Menlo", 10), foreground="#a6adc8", background="#181825")
        style.configure("Connect.TButton", foreground="#a6e3a1")
        style.configure("Disconnect.TButton", foreground="#f38ba8")
        style.configure("Send.TButton", foreground="#f9e2af")
        style.configure("TFrame", background=BG)
        style.configure("TLabelframe", background=BG, foreground=ACCENT)
        style.configure("TLabelframe.Label", background=BG, foreground=ACCENT, font=("Menlo", 11, "bold"))
        style.configure("TCombobox", fieldbackground="#313244", foreground=FG)
        style.configure("TEntry", fieldbackground="#313244", foreground=FG)
        style.configure("TSpinbox", fieldbackground="#313244", foreground=FG)
        style.configure("TCheckbutton", background=BG, foreground=FG)

        self.root.configure(bg="#1e1e2e")

    # ---- Menu Bar --------------------------------------------------------
    def _build_menu_bar(self):
        menubar = tk.Menu(self.root, bg="#181825", fg="#cdd6f4", activebackground="#89b4fa")
        file_menu = tk.Menu(menubar, tearoff=0, bg="#181825", fg="#cdd6f4")
        file_menu.add_command(label="Clear Charts", command=self._clear_charts)
        file_menu.add_command(label="Reset Zoom (Live)", command=self._reset_chart_zoom)
        file_menu.add_separator()
        file_menu.add_command(label="Quit", command=self._on_close)
        menubar.add_cascade(label="File", menu=file_menu)
        self.root.config(menu=menubar)

    # ---- Toolbar ---------------------------------------------------------
    def _build_toolbar(self, initial_port: Optional[str]):
        toolbar = ttk.Frame(self.root)
        toolbar.pack(fill=tk.X, padx=6, pady=4)

        ttk.Label(toolbar, text="Port:").pack(side=tk.LEFT, padx=2)
        self.port_var = tk.StringVar()
        ports = self._detect_serial_ports()
        self.port_combo = ttk.Combobox(toolbar, textvariable=self.port_var, values=ports, width=28)
        if initial_port:
            self.port_var.set(initial_port)
        elif ports:
            self.port_var.set(ports[0])
        self.port_combo.pack(side=tk.LEFT, padx=2)

        ttk.Button(toolbar, text="🔄", width=3, command=self._refresh_ports).pack(side=tk.LEFT, padx=2)

        ttk.Label(toolbar, text="Ch:").pack(side=tk.LEFT, padx=(10, 2))
        self.channel_var = tk.IntVar(value=0)
        ch_spin = ttk.Spinbox(toolbar, from_=0, to=1, textvariable=self.channel_var, width=3)
        ch_spin.pack(side=tk.LEFT, padx=2)

        self.connect_btn = ttk.Button(toolbar, text="▶ Connect", style="Connect.TButton", command=self._connect)
        self.connect_btn.pack(side=tk.LEFT, padx=8)
        self.disconnect_btn = ttk.Button(toolbar, text="■ Disconnect", style="Disconnect.TButton",
                                          command=self._disconnect, state=tk.DISABLED)
        self.disconnect_btn.pack(side=tk.LEFT, padx=2)

        # RX/TX counter
        self.counter_var = tk.StringVar(value="RX: 0  TX: 0")
        ttk.Label(toolbar, textvariable=self.counter_var, font=("Menlo", 10)).pack(side=tk.RIGHT, padx=10)

    # ---- Main Area -------------------------------------------------------
    def _build_main_area(self):
        main = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill=tk.BOTH, expand=True, padx=4, pady=2)

        # Left: Command + Calibration
        left_frame = ttk.Frame(main, width=300)
        main.add(left_frame, weight=0)

        self._build_command_panel(left_frame)
        self._build_calibration_panel(left_frame)

        # Center: Charts
        center_frame = ttk.Frame(main)
        main.add(center_frame, weight=3)
        self._build_chart(center_frame)

        # Right: Status
        right_frame = ttk.Frame(main, width=280)
        main.add(right_frame, weight=0)
        self._build_status_panel(right_frame)

    # ---- Command Panel ---------------------------------------------------
    def _build_command_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="  ⚡ Motor Command (0x200)  ", padding=8)
        frame.pack(fill=tk.X, padx=4, pady=4)

        # Enable
        r = 0
        self.cmd_enable_var = tk.IntVar(value=0)
        ttk.Label(frame, text="Enable:").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        enable_frame = ttk.Frame(frame)
        enable_frame.grid(row=r, column=1, sticky=tk.W, padx=2)
        ttk.Radiobutton(enable_frame, text="Stop", variable=self.cmd_enable_var, value=0).pack(side=tk.LEFT)
        ttk.Radiobutton(enable_frame, text="Run", variable=self.cmd_enable_var, value=1).pack(side=tk.LEFT)

        # Control Mode
        r += 1
        self.cmd_mode_var = tk.IntVar(value=0)
        ttk.Label(frame, text="Mode:").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        mode_frame = ttk.Frame(frame)
        mode_frame.grid(row=r, column=1, sticky=tk.W, padx=2)
        ttk.Radiobutton(mode_frame, text="Speed", variable=self.cmd_mode_var, value=0).pack(side=tk.LEFT)
        ttk.Radiobutton(mode_frame, text="Current", variable=self.cmd_mode_var, value=1).pack(side=tk.LEFT)

        # Direction
        r += 1
        self.cmd_dir_var = tk.IntVar(value=1)
        ttk.Label(frame, text="Direction:").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        dir_frame = ttk.Frame(frame)
        dir_frame.grid(row=r, column=1, sticky=tk.W, padx=2)
        ttk.Radiobutton(dir_frame, text="FWD", variable=self.cmd_dir_var, value=1).pack(side=tk.LEFT)
        ttk.Radiobutton(dir_frame, text="REV", variable=self.cmd_dir_var, value=-1).pack(side=tk.LEFT)

        # Target RPM
        r += 1
        self.cmd_rpm_var = tk.DoubleVar(value=1000.0)
        ttk.Label(frame, text="Target RPM:").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Entry(frame, textvariable=self.cmd_rpm_var, width=10).grid(row=r, column=1, sticky=tk.W, padx=2)

        # Target Iq
        r += 1
        self.cmd_iq_var = tk.DoubleVar(value=0.0)
        ttk.Label(frame, text="Target Iq (A):").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Entry(frame, textvariable=self.cmd_iq_var, width=10).grid(row=r, column=1, sticky=tk.W, padx=2)

        # RPM Ramp
        r += 1
        self.cmd_ramp_var = tk.DoubleVar(value=500.0)
        ttk.Label(frame, text="RPM Ramp:").grid(row=r, column=0, sticky=tk.W, padx=2, pady=2)
        ttk.Entry(frame, textvariable=self.cmd_ramp_var, width=10).grid(row=r, column=1, sticky=tk.W, padx=2)

        # Send button
        r += 1
        ttk.Button(frame, text="📡 Send Command", style="Send.TButton",
                   command=self._send_motor_command).grid(row=r, column=0, columnspan=2, pady=8, sticky=tk.EW)

        # Quick buttons
        r += 1
        quick_frame = ttk.Frame(frame)
        quick_frame.grid(row=r, column=0, columnspan=2, sticky=tk.EW, pady=2)
        ttk.Button(quick_frame, text="▶ Quick Start", style="Connect.TButton",
                   command=self._quick_start).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(quick_frame, text="⏹ Quick Stop", style="Disconnect.TButton",
                   command=self._quick_stop).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

    # ---- Calibration Panel -----------------------------------------------
    def _build_calibration_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="  🔧 PI Calibration (0x201)  ", padding=8)
        frame.pack(fill=tk.X, padx=4, pady=4)

        calib_fields = [
            ("Selector",       "CalibSelector",       0,      "d",   "0=SpeedPI, 1=Obs, 2=Lim"),
            ("Speed Kp",       "CalibSpeedKp",        0.0,    ".6f", "float32"),
            ("Speed Ki",       "CalibSpeedKi",        0.0,    ".6f", "float32"),
            ("Obs Gain",       "CalibObsGain",        0.0,    ".6f", "float32"),
            ("PLL Kp",         "CalibPllKp",          0.0,    ".6f", "float32"),
            ("PLL Ki",         "CalibPllKi",          0.0,    ".6f", "float32"),
            ("Max Iq (A)",     "CalibMaxIqA",         5.0,    ".2f", ""),
            ("OL Iq (A)",      "CalibOpenLoopIqA",    1.0,    ".2f", ""),
            ("Align I (A)",    "CalibAlignCurrentA",   2.0,    ".2f", ""),
            ("FW Thr",         "CalibFwThreshold",    0.95,   ".4f", ""),
            ("Apply",          "CalibApply",          0,      "d",   "0xFF to commit"),
        ]

        self.calib_vars: Dict[str, tk.Variable] = {}
        for i, (label, sig_name, default, fmt, tooltip) in enumerate(calib_fields):
            ttk.Label(frame, text=f"{label}:").grid(row=i, column=0, sticky=tk.W, padx=2, pady=1)
            if isinstance(default, float):
                var = tk.DoubleVar(value=default)
            else:
                var = tk.IntVar(value=default)
            self.calib_vars[sig_name] = var
            entry = ttk.Entry(frame, textvariable=var, width=12)
            entry.grid(row=i, column=1, sticky=tk.W, padx=2, pady=1)
            if tooltip:
                ttk.Label(frame, text=tooltip, style="Unit.TLabel").grid(row=i, column=2, sticky=tk.W, padx=2)

        r = len(calib_fields)
        btn_frame = ttk.Frame(frame)
        btn_frame.grid(row=r, column=0, columnspan=3, sticky=tk.EW, pady=6)
        ttk.Button(btn_frame, text="📡 Send Calib", style="Send.TButton",
                   command=self._send_calib).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_frame, text="📡 Send & Apply", style="Send.TButton",
                   command=self._send_calib_apply).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self._build_param_ident_panel(parent)

    # ---- Chart -----------------------------------------------------------
    def _build_chart(self, parent):
        n_subplots = 5
        self.fig = Figure(figsize=(10, 9), facecolor="#1e1e2e")
        self.axes = []
        titles = ["Speed (rpm)", "Idq Current (A)", "ABC Phase Current (A)",
                  "Voltage / Modulation", "Observer Angle (rad)"]
        for i in range(n_subplots):
            ax = self.fig.add_subplot(n_subplots, 1, i + 1)
            ax.set_facecolor("#181825")
            ax.set_title(titles[i], fontsize=10, color="#89b4fa", loc="left", pad=4)
            ax.tick_params(colors="#6c7086", labelsize=8)
            ax.grid(True, color="#313244", linewidth=0.5, alpha=0.7)
            for spine in ax.spines.values():
                spine.set_color("#45475a")
            self.axes.append(ax)
        self.fig.subplots_adjust(hspace=0.55, left=0.08, right=0.96, top=0.97, bottom=0.03)

        # Create line objects
        self.lines: Dict[str, Any] = {}
        for subplot_idx, sig_name, label, color in CURVE_DEFS:
            ax = self.axes[subplot_idx]
            line, = ax.plot([], [], label=label, color=color, linewidth=1.2)
            self.lines[sig_name] = line

        # Build legends with pick support (click to toggle curves)
        self.legend_line_map: Dict[Any, str] = {}  # legend_line -> signal_name
        for ax in self.axes:
            ax.legend(loc="upper right", fontsize=7, facecolor="#313244",
                      edgecolor="#45475a", labelcolor="#cdd6f4", framealpha=0.9)

        # Map legend artist -> signal_name for pick events
        for ax_idx, ax in enumerate(self.axes):
            leg = ax.get_legend()
            if leg is None:
                continue
            # Collect signal names that belong to this subplot
            ax_signals = [sn for si, sn, _, _ in CURVE_DEFS if si == ax_idx]
            for leg_line, sig_name in zip(leg.get_lines(), ax_signals):
                leg_line.set_picker(8)  # 8pt pick radius
                self.legend_line_map[leg_line] = sig_name

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Connect interactive events
        self.fig.canvas.mpl_connect('pick_event', self._on_legend_pick)
        self.fig.canvas.mpl_connect('scroll_event', self._on_chart_scroll)
        self.fig.canvas.mpl_connect('button_press_event', self._on_chart_dblclick)

    def _on_legend_pick(self, event):
        """Toggle curve visibility when its legend entry is clicked."""
        leg_line = event.artist
        sig_name = self.legend_line_map.get(leg_line)
        if sig_name is None:
            return
        orig_line = self.lines[sig_name]
        visible = not orig_line.get_visible()
        orig_line.set_visible(visible)
        # Dim legend entry for hidden lines
        leg_line.set_alpha(1.0 if visible else 0.25)
        if visible:
            self.hidden_signals.discard(sig_name)
        else:
            self.hidden_signals.add(sig_name)
        self.canvas.draw_idle()

    def _on_chart_scroll(self, event):
        """Scroll-wheel zoom on the subplot under the cursor."""
        if event.inaxes is None:
            return
        ax = event.inaxes
        scale_factor = 0.8 if event.button == 'up' else 1.25

        # X-axis zoom centred on cursor
        x_min, x_max = ax.get_xlim()
        x_data = event.xdata
        new_xmin = x_data - (x_data - x_min) * scale_factor
        new_xmax = x_data + (x_max - x_data) * scale_factor
        ax.set_xlim(new_xmin, new_xmax)

        # Y-axis zoom centred on cursor
        y_min, y_max = ax.get_ylim()
        y_data = event.ydata
        new_ymin = y_data - (y_data - y_min) * scale_factor
        new_ymax = y_data + (y_max - y_data) * scale_factor
        ax.set_ylim(new_ymin, new_ymax)

        self.chart_frozen = True
        self.canvas.draw_idle()

    def _on_chart_dblclick(self, event):
        """Double-click to reset to live auto-scroll mode."""
        if event.dblclick:
            self._reset_chart_zoom()

    def _reset_chart_zoom(self):
        """Menu action: reset zoom and return to live auto-scroll."""
        self.chart_frozen = False
        for ax in self.axes:
            ax.autoscale(enable=True, axis='both', tight=None)
        self.canvas.draw_idle()

    # ---- Status Panel ----------------------------------------------------
    def _build_status_panel(self, parent):
        container = ttk.Frame(parent)
        container.pack(fill=tk.BOTH, expand=True)

        # ── Prominent Motor State Banner ──────────────────────────────────
        banner_frame = tk.Frame(container, bg="#313244", bd=0, highlightthickness=1,
                                highlightbackground="#45475a")
        banner_frame.pack(fill=tk.X, padx=4, pady=(4, 2))

        # State label (large)
        self.state_banner_label = tk.Label(
            banner_frame, text="── STOP ──", font=("Menlo", 16, "bold"),
            bg="#313244", fg="#6c7086", anchor=tk.CENTER, pady=6)
        self.state_banner_label.pack(fill=tk.X)

        # Fault info (visible only on fault)
        self.fault_banner_frame = tk.Frame(banner_frame, bg="#45475a")
        self.fault_banner_frame.pack(fill=tk.X, padx=4, pady=(0, 4))

        self.fault_reason_label = tk.Label(
            self.fault_banner_frame, text="", font=("Menlo", 11, "bold"),
            bg="#45475a", fg="#f38ba8", anchor=tk.W, padx=6, pady=2)
        self.fault_reason_label.pack(fill=tk.X)

        self.retry_count_label = tk.Label(
            self.fault_banner_frame, text="", font=("Menlo", 10),
            bg="#45475a", fg="#f9e2af", anchor=tk.W, padx=6, pady=1)
        self.retry_count_label.pack(fill=tk.X)

        # Initially hide fault detail
        self.fault_banner_frame.pack_forget()

        # ── Header ────────────────────────────────────────────────────────
        ttk.Label(container, text="📊 Live Status", style="Header.TLabel").pack(anchor=tk.W, padx=4, pady=4)

        # Scrollable canvas for many signals
        canvas = tk.Canvas(container, bg="#1e1e2e", highlightthickness=0)
        scrollbar = ttk.Scrollbar(container, orient=tk.VERTICAL, command=canvas.yview)
        scroll_frame = ttk.Frame(canvas)

        scroll_frame.bind("<Configure>", lambda e: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=scroll_frame, anchor=tk.NW)
        canvas.configure(yscrollcommand=scrollbar.set)

        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Bind mouse wheel
        def _on_mousewheel(event):
            canvas.yview_scroll(-1 * (event.delta // 120), "units")
        canvas.bind_all("<MouseWheel>", _on_mousewheel)

        self.status_labels: Dict[str, ttk.Label] = {}
        for i, (sig_name, disp_label, unit, fmt) in enumerate(STATUS_SIGNALS):
            ttk.Label(scroll_frame, text=disp_label, width=14, anchor=tk.W).grid(
                row=i, column=0, sticky=tk.W, padx=4, pady=1)
            val_label = ttk.Label(scroll_frame, text="---", style="Val.TLabel", width=12, anchor=tk.E)
            val_label.grid(row=i, column=1, sticky=tk.E, padx=2, pady=1)
            self.status_labels[sig_name] = val_label
            if unit:
                ttk.Label(scroll_frame, text=unit, style="Unit.TLabel", width=6).grid(
                    row=i, column=2, sticky=tk.W, padx=2, pady=1)

    # ---- Status Bar ------------------------------------------------------
    def _build_status_bar(self):
        bar = ttk.Frame(self.root, style="TFrame")
        bar.pack(fill=tk.X, side=tk.BOTTOM, padx=2, pady=2)
        self.status_var = tk.StringVar(value="Ready. Select a serial port and click Connect.")
        ttk.Label(bar, textvariable=self.status_var, style="Status.TLabel").pack(side=tk.LEFT, padx=6)

    # ---- Connection ------------------------------------------------------
    def _detect_serial_ports(self) -> list:
        patterns = ["/dev/cu.usbserial-*", "/dev/cu.usbmodem-*", "/dev/ttyUSB*", "/dev/ttyACM*"]
        ports = []
        for p in patterns:
            ports.extend(sorted(glob.glob(p)))
        # On macOS also check for other common patterns
        if not ports:
            ports.extend(sorted(glob.glob("/dev/cu.*")))
        return ports

    def _refresh_ports(self):
        ports = self._detect_serial_ports()
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showerror("Error", "Please select a serial port.")
            return
        ch = self.channel_var.get()
        try:
            self.transport = SerialTransport(port=port, baudrate=2_000_000)
            self.transport.start()
            self.bus = YuntuCanBus(
                channel=ch,
                transport=self.transport,
                bitrate=500_000,
                fd=True,
                data_bitrate=2_000_000,
                sample_point=0.70,
                data_sample_point=0.70,
            )
            self.connected = True
            self.rx_running = True
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()

            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            self.status_var.set(f"Connected to {port} (CAN FD 500K/2M, ch={ch})")
            self._log(f"Connected to {port} ch={ch}")
        except Exception as e:
            messagebox.showerror("Connection Error", str(e))
            self._cleanup_connection()

    def _disconnect(self):
        self._cleanup_connection()
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        self.status_var.set("Disconnected.")
        self._log("Disconnected.")
        self._update_chart(force=True)

    def _cleanup_connection(self):
        self.rx_running = False
        self.connected = False
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
        self.bus = None
        # Transport is owned by bus when not shared; cleaned up by bus.shutdown()
        # But if bus creation failed, we clean transport ourselves
        try:
            if self.transport is not None:
                self.transport.close()
        except Exception:
            pass
        self.transport = None

    # ---- RX Thread -------------------------------------------------------
    def _rx_loop(self):
        """Background thread: receive CAN frames and decode via DBC."""
        while self.rx_running and self.bus is not None:
            try:
                msg = self.bus.recv(timeout=0.05)
                if msg is None:
                    continue
                self._decode_message(msg)
            except Exception:
                if self.rx_running:
                    continue
                break

    def _decode_message(self, msg: can.Message):
        """Decode a received CAN message using the DBC database."""
        try:
            db_msg = self.db.get_message_by_frame_id(msg.arbitration_id)
        except KeyError:
            return  # Unknown message ID

        try:
            decoded = db_msg.decode(msg.data, decode_choices=False)
        except Exception:
            return

        now = time.monotonic()
        with self.lock:
            if self.t0 is None:
                self.t0 = now
            t = now - self.t0
            self.rx_count += 1

            for sig_name, value in decoded.items():
                self.signals[sig_name] = value
                if sig_name in self.curve_data:
                    self.curve_data[sig_name].append((t, float(value)))

    # ---- TX: Motor Command -----------------------------------------------
    def _send_motor_command(self):
        if not self.connected or self.bus is None:
            messagebox.showwarning("Warning", "Not connected.")
            return
        try:
            cmd_msg = self.db.get_message_by_name("PC_MotorCommand")
            data_dict = {
                "CmdEnable": self.cmd_enable_var.get(),
                "CmdControlMode": self.cmd_mode_var.get(),
                "CmdDirection": self.cmd_dir_var.get(),
                "CmdTargetRpm": self.cmd_rpm_var.get(),
                "CmdTargetIqA": self.cmd_iq_var.get(),
                "CmdRpmRamp": int(self.cmd_ramp_var.get()),
                "CmdReserved": 0,
            }
            data = cmd_msg.encode(data_dict)
            msg = can.Message(
                arbitration_id=cmd_msg.frame_id,
                data=data,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
            )
            self.bus.send(msg)
            with self.lock:
                self.tx_count += 1
            self._log(f"TX MotorCommand: {data_dict}")
        except Exception as e:
            messagebox.showerror("TX Error", str(e))

    def _quick_start(self):
        """Set enable=1 and send."""
        self.cmd_enable_var.set(1)
        self._send_motor_command()

    def _quick_stop(self):
        """Set enable=0 and send."""
        self.cmd_enable_var.set(0)
        self._send_motor_command()

    # ---- TX: Calibration --------------------------------------------------
    def _send_calib(self):
        if not self.connected or self.bus is None:
            messagebox.showwarning("Warning", "Not connected.")
            return
        try:
            calib_msg = self.db.get_message_by_name("PC_CalibWrite")
            data_dict = {}
            for sig_name, var in self.calib_vars.items():
                val = var.get()
                # Float32 fields need special handling: cantools encodes them
                # using the DBC factor/offset, but our float32 fields use
                # SIG_VALTYPE_ = 1 (IEEE float). cantools handles this.
                data_dict[sig_name] = val
            data = calib_msg.encode(data_dict)
            msg = can.Message(
                arbitration_id=calib_msg.frame_id,
                data=data,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
            )
            self.bus.send(msg)
            with self.lock:
                self.tx_count += 1
            self._log(f"TX CalibWrite: sel={data_dict.get('CalibSelector')}")
        except Exception as e:
            messagebox.showerror("TX Error", str(e))

    def _send_calib_apply(self):
        """Send calibration with Apply=0xFF to commit."""
        self.calib_vars["CalibApply"].set(0xFF)
        self._send_calib()

    # ---- Parameter Identification Panel ----------------------------------
    def _build_param_ident_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="  🔬 Param Identification  ", padding=8)
        frame.pack(fill=tk.X, padx=4, pady=4)

        # ── Ident config inputs ──
        cfg_frame = ttk.Frame(frame)
        cfg_frame.pack(fill=tk.X, pady=(0, 6))
        self.ident_cfg_vars = {}
        ident_cfg_fields = [
            ("Pole Pairs",  "ident_pp",    "4"),
            ("Target RPM",  "ident_rpm",   "300"),
            ("Test Iq (A)", "ident_iq",    "0.5"),
            ("λ Iq Min",    "ident_iqmin", "0.3"),
            ("λ Iq Max",    "ident_iqmax", "2.0"),
            ("λ Iq Step",   "ident_iqstp", "0.2"),
        ]
        for i, (label_text, key, default) in enumerate(ident_cfg_fields):
            row, col = divmod(i, 2)
            ttk.Label(cfg_frame, text=label_text, width=10, anchor=tk.W).grid(
                row=row, column=col * 2, sticky=tk.W, padx=2, pady=1)
            var = tk.StringVar(value=default)
            self.ident_cfg_vars[key] = var
            ttk.Entry(cfg_frame, textvariable=var, width=7).grid(
                row=row, column=col * 2 + 1, sticky=tk.W, padx=2, pady=1)

        # Buttons
        btn_frame = ttk.Frame(frame)
        btn_frame.pack(fill=tk.X, pady=(0, 6))
        ttk.Button(btn_frame, text="▶ Start Ident", style="Connect.TButton",
                   command=self._start_param_ident).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)
        ttk.Button(btn_frame, text="⏹ Abort", style="Disconnect.TButton",
                   command=self._abort_param_ident).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        # Phase & progress
        info_frame = ttk.Frame(frame)
        info_frame.pack(fill=tk.X)

        ttk.Label(info_frame, text="Phase:", width=10, anchor=tk.W).grid(row=0, column=0, sticky=tk.W, padx=2, pady=1)
        self.ident_phase_label = ttk.Label(info_frame, text="Idle", style="Val.TLabel", width=14, anchor=tk.W)
        self.ident_phase_label.grid(row=0, column=1, sticky=tk.W, padx=2, pady=1)

        ttk.Label(info_frame, text="Progress:", width=10, anchor=tk.W).grid(row=1, column=0, sticky=tk.W, padx=2, pady=1)
        self.ident_progress_label = ttk.Label(info_frame, text="0%", style="Val.TLabel", width=14, anchor=tk.W)
        self.ident_progress_label.grid(row=1, column=1, sticky=tk.W, padx=2, pady=1)

        # Progress bar
        self.ident_progressbar = ttk.Progressbar(frame, mode='determinate', maximum=100)
        self.ident_progressbar.pack(fill=tk.X, pady=(2, 6))

        # Results
        result_frame = ttk.Frame(frame)
        result_frame.pack(fill=tk.X)

        result_fields = [
            ("Rs", "Ω"),
            ("Ls", "mH"),
            ("λ",  "mVs"),
        ]
        self.ident_result_labels = {}
        for i, (name, unit) in enumerate(result_fields):
            ttk.Label(result_frame, text=f"{name}:", width=6, anchor=tk.W).grid(
                row=i, column=0, sticky=tk.W, padx=2, pady=1)
            val_lbl = ttk.Label(result_frame, text="---", style="Val.TLabel", width=12, anchor=tk.E)
            val_lbl.grid(row=i, column=1, sticky=tk.E, padx=2, pady=1)
            self.ident_result_labels[name] = val_lbl
            ttk.Label(result_frame, text=unit, style="Unit.TLabel", width=6).grid(
                row=i, column=2, sticky=tk.W, padx=2, pady=1)

    def _send_test_command(self, cmd_type: int, extra_signals=None):
        """Send a PC_TestCommand frame with the given command type byte."""
        if not self.connected or self.bus is None:
            messagebox.showwarning("Warning", "Not connected.")
            return
        try:
            tc_msg = self.db.get_message_by_name("PC_TestCommand")
            if extra_signals is None:
                extra_signals = {}
            data_dict = {
                "TcCommandType": cmd_type,
                "TcNSamples": 0,
                "TcDecimation": 0,
                "TcTriggerSource": 0,
                "TcIdentPolePairs": 0,
                "TcIdentTargetRpm": 0.0,
                "TcIdentTestCurrentA": 0.0,
                "TcIdentIqMinA": 0.0,
                "TcIdentIqMaxA": 0.0,
                "TcIdentIqStepA": 0.0,
            }
            data_dict.update(extra_signals)
            data = tc_msg.encode(data_dict)
            msg = can.Message(
                arbitration_id=tc_msg.frame_id,
                data=data,
                is_extended_id=False,
                is_fd=True,
                bitrate_switch=True,
            )
            self.bus.send(msg)
            with self.lock:
                self.tx_count += 1
            self._log(f"TX TestCommand: type=0x{cmd_type:02X}")
        except Exception as e:
            messagebox.showerror("TX Error", str(e))

    def _start_param_ident(self):
        """Send TestCommand 0x20 with user ident config."""
        try:
            pp  = int(self.ident_cfg_vars["ident_pp"].get())
            rpm = float(self.ident_cfg_vars["ident_rpm"].get())
            iq  = float(self.ident_cfg_vars["ident_iq"].get())
            iq_min  = float(self.ident_cfg_vars["ident_iqmin"].get())
            iq_max  = float(self.ident_cfg_vars["ident_iqmax"].get())
            iq_step = float(self.ident_cfg_vars["ident_iqstp"].get())
        except ValueError:
            messagebox.showerror("Config Error", "Invalid ident config value.")
            return
        self._send_test_command(0x20, {
            "TcIdentPolePairs": pp,
            "TcIdentTargetRpm": rpm,
            "TcIdentTestCurrentA": iq,
            "TcIdentIqMinA": iq_min,
            "TcIdentIqMaxA": iq_max,
            "TcIdentIqStepA": iq_step,
        })

    def _abort_param_ident(self):
        """Send TestCommand 0x21 to abort parameter identification."""
        self._send_test_command(0x21)

    # ---- Chart Update ----------------------------------------------------
    def _update_chart(self, frame_num=None, force=False):
        if not self.connected and not force:
            return

        with self.lock:
            for subplot_idx, sig_name, _, _ in CURVE_DEFS:
                if sig_name in self.hidden_signals:
                    continue
                d = self.curve_data[sig_name]
                if len(d) == 0:
                    self.lines[sig_name].set_data([], [])
                    continue
                ts = [p[0] for p in d]
                vs = [p[1] for p in d]
                self.lines[sig_name].set_data(ts, vs)

        # When frozen (user zoomed), skip auto-scale to keep the user's view
        if not self.chart_frozen:
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view(scalex=True, scaley=True)
                # Scrolling window
                with self.lock:
                    if self.t0 is not None:
                        t_now = time.monotonic() - self.t0
                        ax.set_xlim(max(0, t_now - CHART_WINDOW_S), t_now + 0.2)

        self.canvas.draw_idle()

    def _clear_charts(self):
        with self.lock:
            for d in self.curve_data.values():
                d.clear()
            self.t0 = None
        self._update_chart(force=True)

    # ---- Status Update ---------------------------------------------------
    def _schedule_status_update(self):
        self._update_status_labels()
        self.root.after(100, self._schedule_status_update)

    def _update_status_labels(self):
        with self.lock:
            signals = dict(self.signals)
            rx = self.rx_count
            tx = self.tx_count

        self.counter_var.set(f"RX: {rx}  TX: {tx}")

        # ── Update prominent state banner ──
        state_val = signals.get("State")
        fault_val = signals.get("FaultCode")
        retry_val = signals.get("StartupRetryCount")

        if state_val is not None:
            state_int = int(state_val)
            state_name = STATE_MAP.get(state_int, f"UNKNOWN({state_int})")
            color = STATE_COLOR_MAP.get(state_int, "#cdd6f4")
            self.state_banner_label.config(
                text=f"── {state_name} ──", fg=color)

            # Show / hide fault detail
            if state_int == 8:  # FAULT
                fault_int = int(fault_val) if fault_val is not None else 0
                fault_name = FAULT_MAP.get(fault_int, f"Unknown({fault_int})")
                self.fault_reason_label.config(text=f"⚠ Fault: {fault_name}")
                retry_cnt = int(retry_val) if retry_val is not None else 0
                self.retry_count_label.config(
                    text=f"Startup Retries: {retry_cnt}")
                self.fault_banner_frame.pack(fill=tk.X, padx=4, pady=(0, 4))
            else:
                self.fault_banner_frame.pack_forget()
                # Show retry count in banner subtitle during startup phases
                if state_int in (4, 5) and retry_val is not None and int(retry_val) > 0:
                    self.retry_count_label.config(
                        text=f"Retry #{int(retry_val)}")
                    self.fault_reason_label.config(text="")
                    self.fault_banner_frame.pack(fill=tk.X, padx=4, pady=(0, 4))

        # ── Update signal value labels ──
        for sig_name, disp_label, unit, fmt in STATUS_SIGNALS:
            val = signals.get(sig_name)
            label_widget = self.status_labels.get(sig_name)
            if label_widget is None:
                continue

            if val is None:
                label_widget.config(text="---")
                continue

            # Special formatting for enum-like fields
            if sig_name == "State":
                txt = STATE_MAP.get(int(val), f"?({int(val)})")
            elif sig_name == "FaultCode":
                txt = FAULT_MAP.get(int(val), f"?({int(val)})")
            elif sig_name == "ControlMode":
                txt = CTRL_MODE_MAP.get(int(val), f"?({int(val)})")
            elif fmt:
                try:
                    txt = f"{val:{fmt}}"
                except (ValueError, TypeError):
                    txt = str(val)
            else:
                txt = str(val)

            label_widget.config(text=txt)

        # ── Update parameter identification panel ──
        ident_phase_val = signals.get("IdentPhase")
        ident_progress_val = signals.get("IdentProgress")
        ident_rs = signals.get("IdentRsOhm")
        ident_ls = signals.get("IdentLsH")
        ident_lambda = signals.get("IdentLambdaVs")

        if ident_phase_val is not None:
            ident_phase_names = {
                0: "Idle", 1: "Rs (Resistance)", 2: "Ls (Inductance)",
                3: "λ (Flux Linkage)", 4: "✅ Complete",
                5: "⚠ Drag Retry", 6: "❌ Error",
            }
            phase_int = int(ident_phase_val)
            phase_text = ident_phase_names.get(phase_int, f"?({phase_int})")
            self.ident_phase_label.config(text=phase_text)

            # Color the phase label
            phase_colors = {
                0: "#6c7086", 1: "#fab387", 2: "#89dceb",
                3: "#cba6f7", 4: "#a6e3a1", 5: "#f9e2af", 6: "#f38ba8",
            }
            self.ident_phase_label.config(foreground=phase_colors.get(phase_int, "#cdd6f4"))

        if ident_progress_val is not None:
            prog = int(ident_progress_val)
            self.ident_progress_label.config(text=f"{prog}%")
            self.ident_progressbar['value'] = prog

        if ident_rs is not None:
            rs_val = float(ident_rs)
            self.ident_result_labels["Rs"].config(
                text=f"{rs_val:.4f}" if rs_val > 0 else "---")
        if ident_ls is not None:
            ls_val = float(ident_ls)
            self.ident_result_labels["Ls"].config(
                text=f"{ls_val * 1000.0:.4f}" if ls_val > 0 else "---")
        if ident_lambda is not None:
            lam_val = float(ident_lambda)
            self.ident_result_labels["λ"].config(
                text=f"{lam_val * 1000.0:.4f}" if lam_val > 0 else "---")

    # ---- Logging ---------------------------------------------------------
    def _log(self, text: str):
        ts = time.strftime("%H:%M:%S")
        self.status_var.set(f"[{ts}] {text}")

    # ---- Cleanup ---------------------------------------------------------
    def _on_close(self):
        self.rx_running = False
        self._cleanup_connection()
        try:
            self.anim.event_source.stop()
        except Exception:
            pass
        self.root.quit()
        self.root.destroy()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description="Motor Control CAN FD Debug GUI")
    parser.add_argument("--port", "-p", default=None, help="Serial port (e.g. /dev/cu.usbserial-1130)")
    parser.add_argument("--dbc", default=DEFAULT_DBC, help=f"DBC file path (default: {DEFAULT_DBC})")
    parser.add_argument("--channel", "-c", type=int, default=0, help="CAN channel (default: 0)")
    args = parser.parse_args()

    if not os.path.isfile(args.dbc):
        print(f"ERROR: DBC file not found: {args.dbc}")
        sys.exit(1)

    root = tk.Tk()
    app = MotorDebuggerApp(root, dbc_path=args.dbc, initial_port=args.port)
    if args.channel != 0:
        app.channel_var.set(args.channel)
    root.mainloop()


if __name__ == "__main__":
    main()
