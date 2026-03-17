#ifndef MC_MOTOR_H
#define MC_MOTOR_H

#include "mc_common.h"
#include "mc_control.h"
#include "mc_hal.h"
#include "mc_observer.h"
#include "mc_protection.h"
#include "mc_startup.h"
#include "mc_svpwm.h"

typedef struct
{
    const mc_hal_ops_t *hal;
    mc_user_config_t config;
    mc_state_t state;
    uint32_t fault_mask;
    uint16_t current_a_offset_raw;
    uint16_t current_b_offset_raw;
    uint32_t calibration_accum_a;
    uint32_t calibration_accum_b;
    uint16_t calibration_count;
    bool outputs_enabled;
    bool handover_active;
    float handover_blend;
    float speed_command_rpm;
    float speed_feedback_rpm;
    float iq_reference_a;
    float id_reference_a;
    float electrical_angle_rad;
    float electrical_speed_rad_s;
    float last_slow_loop_ts_s;
    uint32_t last_slow_loop_tick_ms;
    mc_input_request_t input_request;
    mc_adc_sample_t last_sample;
    mc_abc_t phase_currents_abc;
    mc_alpha_beta_t phase_currents_ab;
    mc_dq_t phase_currents_dq;
    mc_alpha_beta_t voltage_command_ab;
    mc_dq_t voltage_command_dq;
    float vbus_v;
    float temperature_c;
    mc_pi_controller_t id_pi;
    mc_pi_controller_t iq_pi;
    mc_pi_controller_t speed_pi;
    mc_ramp_t speed_ramp;
    mc_observer_t observer;
    mc_startup_t startup;
    mc_protection_t protection;
} mc_motor_t;

void MC_Motor_Init(mc_motor_t *motor, const mc_hal_ops_t *hal, const mc_user_config_t *config);
void MC_Motor_FastLoop(mc_motor_t *motor, const mc_adc_sample_t *sample);
void MC_Motor_Background(mc_motor_t *motor);
void MC_Motor_ReportFault(mc_motor_t *motor, uint32_t fault_mask);

#endif
