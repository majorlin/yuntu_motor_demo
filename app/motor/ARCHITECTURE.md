# Motor Control Architecture

## Goals

- Primary control path: sensorless FOC with dual-shunt current sampling and SMO observer.
- Startup path for this phase: forced-drag startup with observer handover.
- Protection path: over-current, under-voltage, over-voltage, over-temperature, phase-loss.
- Extension points reserved for Hall sensor, PLL observer, single-shunt reconstruction, and zero-speed closed-loop startup.

## Layering

1. `mc_hal_*`
   - Owns YTM32-specific ADC, eTMR, GPIO, SysTick, and fault latch handling.
   - Presents a narrow control-oriented interface to the control core.
   - Keeps direct-register PWM update in one place for efficiency.

2. `mc_motor`
   - Owns the fast-loop and slow-loop state machine.
   - Coordinates startup, observer, current loop, speed loop, protection, and PWM update.
   - Uses configuration enums instead of hard-coding one sensor or one startup path.

3. `mc_observer`
   - Dispatch layer for observer backends.
   - SMO is implemented now.
   - PLL and Hall are defined as future-capable observer types so the upper-layer state machine does not need redesign.

4. `mc_startup`
   - Dispatch layer for startup backends.
   - Forced-drag is implemented now.
   - Zero-speed closed-loop startup already has a reserved type and API slot.

5. `mc_math`, `mc_control`, `mc_protection`
   - Reusable algorithm blocks: Clarke/Park, fast sin/cos, PI, LPF, ramp, SVPWM, safety checks.

## Runtime Flow

- `SysTick` runs at 1 kHz and provides the slow-loop heartbeat.
- `eTMR0` runs continuously, even while outputs are masked.
- `eTMR0` generates ADC trigger pulses near the PWM center.
- `ADC0_IRQHandler` is the fast loop:
  - Read dual-shunt samples and DC bus / temperature samples
  - Reconstruct currents
  - Run protection
  - Run startup + observer
  - Run current FOC
  - Update PWM compare registers through eTMR shadow registers

- `MotorApp_Run()` is the background task:
  - Poll start/stop, direction, Hall pins
  - Run speed ramp / speed loop scheduling
  - Drive status LEDs

## Extension Points

- `mc_shunt_mode_t`
  - `MC_SHUNT_SINGLE`, `MC_SHUNT_DUAL`, `MC_SHUNT_TRIPLE`
- `mc_observer_type_t`
  - `MC_OBSERVER_SMO`, `MC_OBSERVER_PLL`, `MC_OBSERVER_HALL`
- `mc_startup_mode_t`
  - `MC_STARTUP_FORCED_DRAG`, `MC_STARTUP_ZERO_SPEED_CLOSED_LOOP`

The current implementation fully wires the `DUAL + SMO + FORCED_DRAG` path and keeps the upper architecture stable for later extension.
