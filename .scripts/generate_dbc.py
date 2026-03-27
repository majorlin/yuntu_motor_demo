#!/usr/bin/env python3
"""
Motor Control CAN FD DBC Generator & Validator
================================================
Generates a Vector DBC file for YTM32 sensorless FOC motor control telemetry,
command, and calibration interface.

Usage:
    python3 .scripts/generate_dbc.py

Output:
    docs/motor_control.dbc  — Valid DBC file for CAN FD (500K/2M)
"""

import os
import sys

import cantools
from cantools.database import Message, Signal, Database, Node
from cantools.database.conversion import LinearConversion

# ─── Nodes ──────────────────────────────────────────────────────────────────────

NODE_MCU = Node(name="MCU", comment="YTM32 Motor Controller ECU")
NODE_PC  = Node(name="PC",  comment="Test PC / Calibration Tool")

# ─── Helper ─────────────────────────────────────────────────────────────────────

def sig(name, start, length, is_signed=False, scale=1, offset=0, minimum=None,
        maximum=None, unit="", comment="", byte_order="little_endian",
        is_float=False):
    """Shorthand signal constructor compatible with cantools >= 39."""
    conv = LinearConversion(scale=scale, offset=offset, is_float=is_float)
    return Signal(
        name=name,
        start=start,
        length=length,
        byte_order=byte_order,
        is_signed=is_signed,
        conversion=conv,
        minimum=minimum,
        maximum=maximum,
        unit=unit if unit else None,
        comment=comment,
    )

# ═══════════════════════════════════════════════════════════════════════════════
# Message 1: MCU_MotorStatus1  (0x180, 10ms, MCU→PC)
#   Core real-time telemetry for data logging
# ═══════════════════════════════════════════════════════════════════════════════

msg_status1 = Message(
    frame_id=0x180,
    name="MCU_MotorStatus1",
    length=64,
    senders=["MCU"],
    comment="Core motor telemetry: state, fault, RPM, Idq, Vbus, phase currents, PWM duty, timestamp. Cycle: 10ms.",
    is_fd=True,
    signals=[
        sig("State",              0,   8, comment="Motor state machine (0=Stop..8=Fault)"),
        sig("FaultCode",          8,   8, comment="Active fault code (0=None)"),
        sig("ControlMode",       16,   8, comment="0=Speed, 1=Current"),
        sig("Flags",             24,   8, comment="Bit0=Enabled, Bit1=ObsLocked, Bit2=Direction(1=fwd)"),
        sig("MechanicalRpm",     32,  16, is_signed=True, scale=0.5, unit="rpm",
            comment="Estimated mechanical speed"),
        sig("BusVoltage",        48,  16, scale=0.01, unit="V",
            comment="DC bus voltage"),
        sig("IdMeas",            64,  16, is_signed=True, scale=0.01, unit="A",
            comment="Measured d-axis current"),
        sig("IqMeas",            80,  16, is_signed=True, scale=0.01, unit="A",
            comment="Measured q-axis current"),
        sig("IdTarget",          96,  16, is_signed=True, scale=0.01, unit="A",
            comment="Commanded d-axis current target"),
        sig("IqTarget",         112,  16, is_signed=True, scale=0.01, unit="A",
            comment="Commanded q-axis current target"),
        sig("TargetRpm",        128,  16, is_signed=True, scale=0.5, unit="rpm",
            comment="Ramped speed target"),
        sig("ElecAngle",        144,  16, scale=0.0001, unit="rad",
            comment="Electrical angle [0, 2pi)"),
        sig("ElecSpeedRadS",    160,  16, is_signed=True, scale=0.1, unit="rad/s",
            comment="Electrical speed"),
        sig("PhaseCurrentA",    176,  16, is_signed=True, scale=0.01, unit="A",
            comment="Phase A current"),
        sig("PhaseCurrentB",    192,  16, is_signed=True, scale=0.01, unit="A",
            comment="Phase B current"),
        sig("PhaseCurrentC",    208,  16, is_signed=True, scale=0.01, unit="A",
            comment="Phase C current"),
        sig("StartupRetryCount",224,   8, comment="Startup retry attempts"),
        sig("DutyU",            232,  16, scale=0.0001, comment="Phase U PWM duty [0..1]"),
        sig("DutyV",            248,  16, scale=0.0001, comment="Phase V PWM duty [0..1]"),
        sig("DutyW",            264,  16, scale=0.0001, comment="Phase W PWM duty [0..1]"),
        sig("Timestamp",        280,  32, unit="ms", comment="System tick milliseconds"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 2: MCU_MotorStatus2  (0x181, 50ms, MCU→PC)
#   Observer & PI diagnostics, temperature sensors
# ═══════════════════════════════════════════════════════════════════════════════

msg_status2 = Message(
    frame_id=0x181,
    name="MCU_MotorStatus2",
    length=64,
    senders=["MCU"],
    comment="Observer/PI diagnostics, temperature, BEMF, param ident telemetry. Cycle: 10ms.",
    is_fd=True,
    signals=[
        sig("ObserverAngle",      0,  16, scale=0.0001, unit="rad",
            comment="Observer estimated electrical angle"),
        sig("ObserverFluxVs",    16,  16, scale=0.000001, unit="Vs",
            comment="Adaptive flux linkage estimate"),
        sig("PLLSpeedRadS",      32,  16, is_signed=True, scale=0.1, unit="rad/s",
            comment="PLL estimated electrical speed"),
        sig("PhaseErrorRad",     48,  16, is_signed=True, scale=0.0001, unit="rad",
            comment="Control-to-observer phase error"),
        sig("SpeedPiIntegrator", 64,  16, is_signed=True, scale=0.01, unit="A",
            comment="Speed PI integrator state"),
        sig("IdPiIntegrator",    80,  16, is_signed=True, scale=0.01, unit="V",
            comment="D-axis current PI integrator"),
        sig("IqPiIntegrator",    96,  16, is_signed=True, scale=0.01, unit="V",
            comment="Q-axis current PI integrator"),
        sig("VoltModRatio",     112,  16, scale=0.0001,
            comment="|Vab|/Vbus modulation ratio"),
        sig("FwIdTarget",       128,  16, is_signed=True, scale=0.01, unit="A",
            comment="Field weakening d-axis current"),
        sig("OpenLoopAngle",    144,  16, scale=0.0001, unit="rad",
            comment="Forced open-loop angle"),
        sig("OpenLoopSpeedRadS",160,  16, is_signed=True, scale=0.1, unit="rad/s",
            comment="Open-loop forced speed"),
        sig("ClosedLoopBlend",  176,  16, scale=0.0001,
            comment="Closed-loop blend factor [0..1]"),
        sig("StateTimeMs",      192,  32, unit="ms",
            comment="Time in current state"),
        sig("ObsLockResidualRad",224, 16, is_signed=True, scale=0.0001, unit="rad",
            comment="Observer lock residual angle"),
        sig("StallDivCount",    240,  16, comment="Consecutive stall divergence count"),
        sig("PcbTemperature",   256,  16, is_signed=True, scale=0.1, unit="degC",
            comment="PCB board temperature"),
        sig("ChipTemperature",  272,  16, is_signed=True, scale=0.1, unit="degC",
            comment="MCU die temperature from TMU"),
        # ── BEMF wind detection diagnostics (bytes 36-48) ──
        sig("BemfURaw",         288,  16, comment="BEMF U raw ADC count"),
        sig("BemfVRaw",         304,  16, comment="BEMF V raw ADC count"),
        sig("BemfWRaw",         320,  16, comment="BEMF W raw ADC count"),
        sig("BemfComRaw",       336,  16, comment="BEMF COM raw ADC count"),
        sig("BemfCrossingCount",352,  16, comment="Zero-crossing count"),
        sig("BemfDetectedRpm",  368,  16, is_signed=True, unit="rpm",
            comment="BEMF-detected mechanical RPM"),
        sig("BemfPhaseSeq",     384,   8, is_signed=True,
            comment="Phase sequence: +1=fwd, -1=rev, 0=unknown"),
        # ── Parameter identification telemetry (bytes 49-63) ──
        sig("IdentPhase",       392,   8,
            comment="0=Idle,1=Rs,2=Ls,3=Lambda,4=Complete,5=DragFail,6=Error"),
        sig("IdentProgress",    400,   8, unit="%",
            comment="Current phase progress 0-100"),
        sig("IdentRsOhm",       408,  32, is_float=True, unit="Ohm",
            comment="Identified stator resistance"),
        sig("IdentLsH",         440,  32, is_float=True, unit="H",
            comment="Identified stator inductance"),
        sig("IdentLambdaVs",    472,  32, is_float=True, unit="Vs",
            comment="Identified PM flux linkage"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 3: PC_MotorCommand (0x200, event, PC→MCU)
# ═══════════════════════════════════════════════════════════════════════════════

msg_command = Message(
    frame_id=0x200,
    name="PC_MotorCommand",
    length=64,
    senders=["PC"],
    comment="Motor control commands from test PC. Event-driven.",
    is_fd=True,
    signals=[
        sig("CmdEnable",         0,   8, comment="0=Stop, 1=Run"),
        sig("CmdControlMode",    8,   8, comment="0=Speed, 1=Current"),
        sig("CmdDirection",     16,   8, is_signed=True, comment="+1=Forward, -1=Reverse"),
        sig("CmdTargetRpm",     24,  16, is_signed=True, scale=0.5, unit="rpm",
            comment="Speed reference"),
        sig("CmdTargetIqA",     40,  16, is_signed=True, scale=0.01, unit="A",
            comment="Iq current reference"),
        sig("CmdRpmRamp",       56,  16, scale=1, unit="rpm/s",
            comment="Speed ramp rate override"),
        sig("CmdReserved",      72,  56, comment="Reserved for future extension"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 4: PC_CalibWrite (0x201, event, PC→MCU)
# ═══════════════════════════════════════════════════════════════════════════════

msg_calib_write = Message(
    frame_id=0x201,
    name="PC_CalibWrite",
    length=64,
    senders=["PC"],
    comment="Online calibration parameter write. Event-driven.",
    is_fd=True,
    signals=[
        sig("CalibSelector",       0,   8,
            comment="Parameter group selector (0=speed PI, 1=observer, 2=limits)"),
        sig("CalibSpeedKp",        8,  32, is_float=True,
            comment="Speed PI proportional gain"),
        sig("CalibSpeedKi",       40,  32, is_float=True,
            comment="Speed PI integral gain"),
        sig("CalibObsGain",       72,  32, is_float=True,
            comment="Ortega observer correction gain"),
        sig("CalibPllKp",        104,  32, is_float=True,
            comment="PLL proportional gain"),
        sig("CalibPllKi",        136,  32, is_float=True,
            comment="PLL integral gain"),
        sig("CalibMaxIqA",       168,  16, scale=0.01, unit="A",
            comment="Maximum Iq clamp"),
        sig("CalibOpenLoopIqA",  184,  16, scale=0.01, unit="A",
            comment="Open-loop Iq target"),
        sig("CalibAlignCurrentA",200,  16, scale=0.01, unit="A",
            comment="Alignment d-axis current"),
        sig("CalibFwThreshold",  216,  16, scale=0.0001,
            comment="Field weakening voltage threshold"),
        sig("CalibApply",        232,   8,
            comment="0xFF = commit parameters to runtime"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 5: MCU_CalibReadback  (0x182, 200ms, MCU→PC)
# ═══════════════════════════════════════════════════════════════════════════════

msg_calib_readback = Message(
    frame_id=0x182,
    name="MCU_CalibReadback",
    length=64,
    senders=["MCU"],
    comment="Echo active calibration parameters. Cycle: 200ms.",
    is_fd=True,
    signals=[
        sig("ActiveSpeedKp",       0,  32, is_float=True,
            comment="Active speed PI Kp"),
        sig("ActiveSpeedKi",      32,  32, is_float=True,
            comment="Active speed PI Ki"),
        sig("ActiveObsGain",      64,  32, is_float=True,
            comment="Active observer gain"),
        sig("ActivePllKp",        96,  32, is_float=True,
            comment="Active PLL Kp"),
        sig("ActivePllKi",       128,  32, is_float=True,
            comment="Active PLL Ki"),
        sig("ActiveMaxIqA",      160,  16, scale=0.01, unit="A",
            comment="Active Iq clamp"),
        sig("ActiveOpenLoopIqA", 176,  16, scale=0.01, unit="A",
            comment="Active open-loop Iq"),
        sig("ActiveAlignCurrentA",192, 16, scale=0.01, unit="A",
            comment="Active alignment current"),
        sig("ActiveFwThreshold", 208,  16, scale=0.0001,
            comment="Active FW voltage threshold"),
        sig("ActiveRpmRamp",     224,  16, scale=1, unit="rpm/s",
            comment="Active speed ramp rate"),
        sig("ActiveCurrentIdKp", 240,  32, is_float=True, unit="V/A",
            comment="Active D-axis current PI Kp"),
        sig("ActiveCurrentIdKi", 272,  32, is_float=True, unit="V/A/s",
            comment="Active D-axis current PI Ki"),
        sig("ActiveCurrentIqKp", 304,  32, is_float=True, unit="V/A",
            comment="Active Q-axis current PI Kp"),
        sig("ActiveCurrentIqKi", 336,  32, is_float=True, unit="V/A/s",
            comment="Active Q-axis current PI Ki"),
        sig("WaveformState",     368,   8,
            comment="0=Idle,1=Capturing,2=Sending,3=Done"),
        sig("WaveformSampleCount",376, 16,
            comment="Number of waveform samples captured"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 6: MCU_MotorStatus3  (0x183, burst, MCU→PC)
#   PWM-rate waveform data, one sample per CAN FD frame
# ═══════════════════════════════════════════════════════════════════════════════

msg_status3 = Message(
    frame_id=0x183,
    name="MCU_MotorStatus3",
    length=64,
    senders=["MCU"],
    comment="PWM-rate waveform burst data. Sent after TestCommand trigger.",
    is_fd=True,
    signals=[
        sig("WfSampleIndex",       0,  16,
            comment="Current sample index in burst"),
        sig("WfTotalSamples",     16,  16,
            comment="Total samples in this capture"),
        sig("WfTriggerSource",    32,   8,
            comment="Trigger: 0=none,1=manual,2=startup,3=fault"),
        sig("WfDecimation",       40,   8,
            comment="Decimation factor (sample every N ticks)"),
        sig("WfCh0_Ia",           64,  32, is_float=True, unit="A",
            comment="Phase A current"),
        sig("WfCh1_Ib",           96,  32, is_float=True, unit="A",
            comment="Phase B current"),
        sig("WfCh2_Ic",          128,  32, is_float=True, unit="A",
            comment="Phase C current"),
        sig("WfCh3_Vbus",        160,  32, is_float=True, unit="V",
            comment="DC bus voltage"),
        sig("WfCh4_Id",          192,  32, is_float=True, unit="A",
            comment="D-axis current"),
        sig("WfCh5_Iq",          224,  32, is_float=True, unit="A",
            comment="Q-axis current"),
        sig("WfCh6_ElecAngle",   256,  32, is_float=True, unit="rad",
            comment="Electrical angle"),
    ],
)

# ═══════════════════════════════════════════════════════════════════════════════
# Message 7: PC_TestCommand (0x202, event, PC→MCU)
#   PC-triggered waveform capture and current loop reset
# ═══════════════════════════════════════════════════════════════════════════════

msg_test_command = Message(
    frame_id=0x202,
    name="PC_TestCommand",
    length=64,
    senders=["PC"],
    comment="Test command from PC. Event-driven.",
    is_fd=True,
    signals=[
        sig("TcCommandType",       0,   8,
            comment="0x01=CaptureStart,0x02=Abort,0x03=Send,0x10=ResetCurrentPI,0x20=StartParamIdent,0x21=AbortParamIdent"),
        sig("TcNSamples",          8,  16,
            comment="Number of samples to capture"),
        sig("TcDecimation",       24,   8,
            comment="Decimation factor"),
        sig("TcTriggerSource",    32,   8,
            comment="Trigger source for capture"),
        # ── Ident config (used when TcCommandType=0x20) ──
        sig("TcIdentPolePairs",   40,   8,
            comment="Motor pole pairs (for cmd 0x20)"),
        sig("TcIdentTargetRpm",   48,  32, is_float=True, unit="rpm",
            comment="Target mechanical RPM for lambda ident"),
        sig("TcIdentTestCurrentA",80,  32, is_float=True, unit="A",
            comment="Rs/Ls test current"),
        sig("TcIdentIqMinA",     112,  32, is_float=True, unit="A",
            comment="Lambda Iq initial current"),
        sig("TcIdentIqMaxA",     144,  32, is_float=True, unit="A",
            comment="Lambda Iq max current"),
        sig("TcIdentIqStepA",    176,  32, is_float=True, unit="A",
            comment="Lambda Iq retry increment"),
    ],
)

# ─── Build Database ─────────────────────────────────────────────────────────────

db = Database(
    messages=[msg_status1, msg_status2, msg_command, msg_calib_write,
              msg_calib_readback, msg_status3, msg_test_command],
    nodes=[NODE_MCU, NODE_PC],
    version="1.0",
)

# ─── Write DBC File ─────────────────────────────────────────────────────────────

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
OUTPUT_DIR  = os.path.join(PROJECT_ROOT, "docs")
OUTPUT_PATH = os.path.join(OUTPUT_DIR, "motor_control.dbc")

os.makedirs(OUTPUT_DIR, exist_ok=True)

dbc_string = db.as_dbc_string()

header = (
    '// =============================================================================\n'
    '// Motor Control CAN FD DBC - YTM32 Sensorless FOC\n'
    '// Generated by .scripts/generate_dbc.py using cantools\n'
    '// CAN FD: 500 kbit/s arbitration, 2 Mbit/s data, 64-byte payload\n'
    '// PE Clock: 120 MHz (Peripheral Clock)\n'
    '// =============================================================================\n'
    '\n'
)

bus_type_attr = '\nBA_DEF_ "BusType" STRING;\nBA_ "BusType" "CAN FD";\n'

with open(OUTPUT_PATH, "w") as f:
    f.write(header)
    f.write(dbc_string)
    f.write(bus_type_attr)

print(f"✅ DBC file written to: {OUTPUT_PATH}")
print(f"   File size: {os.path.getsize(OUTPUT_PATH)} bytes")

# ─── Validation Phase 1: Re-parse ──────────────────────────────────────────────

print("\n─── Validation Phase 1: Structural Parse ───")
try:
    db_check = cantools.database.load_file(OUTPUT_PATH)
    print(f"  ✅ Parsed successfully: {len(db_check.messages)} messages, "
          f"{sum(len(m.signals) for m in db_check.messages)} total signals")
    for m in db_check.messages:
        print(f"     📨 {m.name:25s}  ID=0x{m.frame_id:03X}  DLC={m.length:2d}B  "
              f"signals={len(m.signals):2d}  senders={m.senders}")
except Exception as e:
    print(f"  ❌ Parse FAILED: {e}")
    sys.exit(1)

# ─── Validation Phase 2: Encode/Decode Round-Trip ───────────────────────────────

print("\n─── Validation Phase 2: Encode/Decode Round-Trip Tests ───")

test_vectors = {
    "MCU_MotorStatus1": {
        "State": 6, "FaultCode": 0, "ControlMode": 0, "Flags": 0x07,
        "MechanicalRpm": 3000.0, "BusVoltage": 12.50,
        "IdMeas": -0.50, "IqMeas": 3.25, "IdTarget": 0.0, "IqTarget": 3.50,
        "TargetRpm": 3000.0, "ElecAngle": 3.1415, "ElecSpeedRadS": 942.0,
        "PhaseCurrentA": 5.12, "PhaseCurrentB": -2.56, "PhaseCurrentC": -2.56,
        "StartupRetryCount": 0,
        "DutyU": 0.6500, "DutyV": 0.3200, "DutyW": 0.4100,
        "Timestamp": 123456,
    },
    "MCU_MotorStatus2": {
        "ObserverAngle": 3.1415, "ObserverFluxVs": 0.005300,
        "PLLSpeedRadS": 942.0, "PhaseErrorRad": 0.1234,
        "SpeedPiIntegrator": 2.50, "IdPiIntegrator": -0.30, "IqPiIntegrator": 1.20,
        "VoltModRatio": 0.4500, "FwIdTarget": -1.50,
        "OpenLoopAngle": 1.5708, "OpenLoopSpeedRadS": 120.0, "ClosedLoopBlend": 1.0000,
        "StateTimeMs": 5000, "ObsLockResidualRad": 0.0500, "StallDivCount": 0,
        "PcbTemperature": 45.0, "ChipTemperature": 62.5,
        "BemfURaw": 2048, "BemfVRaw": 2100, "BemfWRaw": 1980,
        "BemfComRaw": 2048, "BemfCrossingCount": 12, "BemfDetectedRpm": 300,
        "BemfPhaseSeq": 1,
        "IdentPhase": 4, "IdentProgress": 100,
        "IdentRsOhm": 0.235, "IdentLsH": 0.000180, "IdentLambdaVs": 0.00530,
    },
    "PC_MotorCommand": {
        "CmdEnable": 1, "CmdControlMode": 0, "CmdDirection": 1,
        "CmdTargetRpm": 2000.0, "CmdTargetIqA": 0.0, "CmdRpmRamp": 600,
        "CmdReserved": 0,
    },
    "PC_CalibWrite": {
        "CalibSelector": 0,
        "CalibSpeedKp": 0.003, "CalibSpeedKi": 0.006,
        "CalibObsGain": 1800000.0, "CalibPllKp": 120.0, "CalibPllKi": 6000.0,
        "CalibMaxIqA": 16.0, "CalibOpenLoopIqA": 8.0, "CalibAlignCurrentA": 5.0,
        "CalibFwThreshold": 0.5000, "CalibApply": 0xFF,
    },
    "MCU_CalibReadback": {
        "ActiveSpeedKp": 0.003, "ActiveSpeedKi": 0.006,
        "ActiveObsGain": 1800000.0, "ActivePllKp": 120.0, "ActivePllKi": 6000.0,
        "ActiveMaxIqA": 16.0, "ActiveOpenLoopIqA": 8.0, "ActiveAlignCurrentA": 5.0,
        "ActiveFwThreshold": 0.5000, "ActiveRpmRamp": 600,
        "ActiveCurrentIdKp": 12.5, "ActiveCurrentIdKi": 2400.0,
        "ActiveCurrentIqKp": 18.0, "ActiveCurrentIqKi": 3600.0,
        "WaveformState": 0, "WaveformSampleCount": 0,
    },
    "MCU_MotorStatus3": {
        "WfSampleIndex": 42, "WfTotalSamples": 128,
        "WfTriggerSource": 1, "WfDecimation": 2,
        "WfCh0_Ia": 3.14, "WfCh1_Ib": -1.57, "WfCh2_Ic": -1.57,
        "WfCh3_Vbus": 12.0, "WfCh4_Id": 0.01, "WfCh5_Iq": 2.50,
        "WfCh6_ElecAngle": 1.5708,
    },
    "PC_TestCommand": {
        "TcCommandType": 0x20, "TcNSamples": 0,
        "TcDecimation": 0, "TcTriggerSource": 0,
        "TcIdentPolePairs": 4, "TcIdentTargetRpm": 300.0,
        "TcIdentTestCurrentA": 0.5, "TcIdentIqMinA": 0.3,
        "TcIdentIqMaxA": 2.0, "TcIdentIqStepA": 0.2,
    },
}

all_pass = True
for msg_name, test_values in test_vectors.items():
    msg_obj = db_check.get_message_by_name(msg_name)
    try:
        encoded = msg_obj.encode(test_values)
        decoded = msg_obj.decode(encoded)

        errors = []
        for sig_name, original in test_values.items():
            got = decoded[sig_name]
            sig_obj = msg_obj.get_signal_by_name(sig_name)

            conv = sig_obj.conversion
            if hasattr(conv, 'is_float') and conv.is_float:
                if abs(got - original) > 1e-6 * max(abs(original), 1.0):
                    errors.append(f"    {sig_name}: sent={original}, got={got}")
            else:
                scale = conv.scale if hasattr(conv, 'scale') else 1
                tol = abs(scale) * 0.6
                if abs(got - original) > tol:
                    errors.append(f"    {sig_name}: sent={original}, got={got}, tol={tol}")

        if errors:
            print(f"  ⚠️  {msg_name}:")
            for e in errors:
                print(e)
            all_pass = False
        else:
            print(f"  ✅ {msg_name}: {len(test_values)} signals round-trip OK")

    except Exception as e:
        print(f"  ❌ {msg_name}: encode/decode FAILED: {e}")
        all_pass = False

# ─── Summary ────────────────────────────────────────────────────────────────────

print("\n" + "=" * 60)
if all_pass:
    print("🎉 ALL VALIDATIONS PASSED — DBC is ready for use!")
else:
    print("⚠️  Some validations had issues — review above.")
print("=" * 60)

print("\n─── CAN FD Bitrate Config (PE Clock = 120 MHz) ───")
print("  Arbitration 500 kbit/s: preDivider=14, propSeg=5, phaseSeg1=5, phaseSeg2=4, rJumpwidth=3")
print("    → Tq = (14+1)/120MHz = 125ns, NBT = 1+5+5+4+1 = 16 Tq, Bitrate = 500 kbit/s")
print("  Data 2 Mbit/s:          preDivider=2,  propSeg=5, phaseSeg1=5, phaseSeg2=4, rJumpwidth=3")
print("    → Tq = (2+1)/120MHz = 25ns, DBT = 1+7+5+7 = 20 Tq, Bitrate = 2 Mbit/s")
