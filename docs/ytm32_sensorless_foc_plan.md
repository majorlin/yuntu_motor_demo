# YTM32 Sensorless FOC Design

## Goal

This implementation adds a first-pass sensorless FOC stack for the YTM32 motor
board:

- `ADC0` samples phase currents A/B/C and DC bus voltage.
- `eTMR0` generates three complementary PWM pairs on channel pairs
  `(0,1)`, `(4,5)`, `(6,7)`.
- `TMU` routes `eTMR0_EXT_TRIG` to `ADC0_EXT_TRIG`.
- `pTMR0_CH0` runs a 1 kHz speed/state loop.
- All inner control uses electrical angle and electrical speed. Mechanical RPM
  only appears in the public API and status reporting.

## Runtime Flow

1. `main()` performs board clock/pin init.
2. `MotorControl_Init()` enables the required runtime clocks and initializes
   ADC, TMU, eTMR and pTMR in the order `ADC -> TMU -> eTMR -> pTMR`.
3. The public API arms the controller with a target RPM and direction.
4. The speed loop moves the controller through:
   `STOP -> OFFSET_CAL -> ALIGN -> OPEN_LOOP_RAMP -> CLOSED_LOOP -> FAULT`.
5. The fast loop runs from `ADC0_IRQHandler()` and executes:
   `sample -> observer/pll -> Clarke/Park -> dq PI -> inverse Park -> SVM ->
   eTMR shadow update`.

## Files

- `app/motor_user_config.h`
  User-visible board, motor and control parameters with derived macros.
- `app/motor_hw_ytm32.[ch]`
  Runtime peripheral setup and fast-path register writes.
- `app/motor_foc.[ch]`
  Fast trig helpers, Clarke/Park, current PI, Ortega-style observer, PLL and
  SVM.
- `app/motor_control.[ch]`
  Public API, state machine, interrupt handlers and fault handling.

## Current Limitations

- This is a v1 control stack intended to bring up sensorless FOC structure and
  timing on the target MCU. The default board and motor numbers are templates
  until real hardware values are supplied.
- No field weakening, MTPA, CAN/UART runtime tuning, Hall/BEMF fallback or
  six-step mode is included.
- Direction changes are only accepted while stopped.

## Verification Targets

- Build stays green with the existing `cmake --build build -j4` flow.
- `ADC0_IRQHandler()` is the fast-loop entry and has higher NVIC priority than
  `pTMR0_CH0_IRQHandler()`.
- PWM outputs are masked low in `STOP` and `FAULT`.
- Faults are latched for ADC overrun, software overcurrent, DC bus out of
  range, observer loss and startup timeout.
