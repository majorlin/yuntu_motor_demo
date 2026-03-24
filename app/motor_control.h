#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_STATE_STOP = 0,
    MOTOR_STATE_OFFSET_CAL,
    MOTOR_STATE_ALIGN,
    MOTOR_STATE_OPEN_LOOP_RAMP,
    MOTOR_STATE_CLOSED_LOOP,
    MOTOR_STATE_FAULT
} motor_control_state_t;

typedef enum
{
    MOTOR_FAULT_NONE = 0,
    MOTOR_FAULT_ADC_OVERRUN = 1,
    MOTOR_FAULT_OVERCURRENT = 2,
    MOTOR_FAULT_VBUS_UNDERVOLTAGE = 3,
    MOTOR_FAULT_VBUS_OVERVOLTAGE = 4,
    MOTOR_FAULT_OBSERVER_LOSS = 5,
    MOTOR_FAULT_STARTUP_TIMEOUT = 6,
    MOTOR_FAULT_BAD_DIRECTION = 7
} motor_fault_t;

typedef enum
{
    MOTOR_CONTROL_MODE_SPEED = 0,
    MOTOR_CONTROL_MODE_CURRENT
} motor_control_mode_t;

typedef struct
{
    motor_control_state_t state;
    motor_fault_t fault;
    motor_control_mode_t control_mode;
    bool observer_locked;
    bool enabled;
    int8_t direction;
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    float mechanical_rpm;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float bus_voltage_v;
    float id_a;
    float iq_a;
    float id_target_a;
    float iq_target_a;
    float target_rpm;
} motor_status_t;

typedef struct
{
    uint32_t last_cycles;
    uint32_t min_cycles;
    uint32_t max_cycles;
    uint32_t avg_cycles;
    uint64_t total_cycles;
    uint32_t sample_count;
} motor_cycle_stat_t;

typedef struct
{
    bool dwt_enabled;
    uint32_t core_clock_hz;
    uint32_t fast_loop_hz;
    motor_cycle_stat_t adc_irq_total;
    motor_cycle_stat_t foc_total;
    motor_cycle_stat_t foc_observer_pll;
    motor_cycle_stat_t foc_current_loop;
    motor_cycle_stat_t foc_svm;
    motor_cycle_stat_t pwm_update;
} motor_fast_loop_profile_t;

extern volatile motor_fast_loop_profile_t g_motorFastLoopProfile;

void MotorControl_Init(void);
void MotorControl_Enable(bool enable);
bool MotorControl_SetControlMode(motor_control_mode_t mode);
bool MotorControl_SetTargetRpm(float targetRpm);
bool MotorControl_SetTargetIqA(float targetIqA);
bool MotorControl_SetDirection(int8_t direction);
const motor_status_t *MotorControl_GetStatus(void);
const volatile motor_fast_loop_profile_t *MotorControl_GetFastLoopProfile(void);
void MotorControl_ResetFastLoopProfile(void);
uint32_t MotorControl_GetTickMs(void);

#endif /* MOTOR_CONTROL_H */
