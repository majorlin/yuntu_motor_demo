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
    MOTOR_FAULT_ADC_OVERRUN,
    MOTOR_FAULT_OVERCURRENT,
    MOTOR_FAULT_VBUS_UNDERVOLTAGE,
    MOTOR_FAULT_VBUS_OVERVOLTAGE,
    MOTOR_FAULT_OBSERVER_LOSS,
    MOTOR_FAULT_STARTUP_TIMEOUT,
    MOTOR_FAULT_BAD_DIRECTION
} motor_fault_t;

typedef struct
{
    motor_control_state_t state;
    motor_fault_t fault;
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
    uint32_t control_state;
    uint32_t fault_code;
    bool observer_locked;
    bool outputs_unmasked;
    bool fast_loop_running;
    uint32_t startup_time_ms;
    uint32_t state_time_ms;
    uint32_t observer_lock_count;
    uint32_t observer_loss_count;
    float closed_loop_blend;
    float open_loop_angle_rad;
    float open_loop_speed_rad_s;
    float control_angle_rad;
    float observer_angle_rad;
    float observer_speed_rad_s;
    float observer_phase_offset_rad;
    float observer_lock_residual_rad;
    float phase_error_rad;
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    float target_electrical_speed_rad_s;
    float mechanical_rpm;
    float bus_voltage_v;
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float id_a;
    float iq_a;
    float id_target_a;
    float iq_target_a;
} motor_debug_t;

extern volatile motor_debug_t g_motorDebug;

void MotorControl_Init(void);
void MotorControl_Enable(bool enable);
bool MotorControl_SetTargetRpm(float targetRpm);
bool MotorControl_SetDirection(int8_t direction);
const motor_status_t *MotorControl_GetStatus(void);
const volatile motor_debug_t *MotorControl_GetDebug(void);

#endif /* MOTOR_CONTROL_H */
