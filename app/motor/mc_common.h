#ifndef MC_COMMON_H
#define MC_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include "status.h"

#define MC_PI_F 3.14159265358979323846f
#define MC_TWO_PI_F 6.28318530717958647692f
#define MC_HALF_PI_F 1.57079632679489661923f
#define MC_ONE_BY_SQRT3_F 0.57735026919f
#define MC_SQRT3_BY_2_F 0.86602540378f

typedef struct
{
    float a;
    float b;
    float c;
} mc_abc_t;

typedef struct
{
    float alpha;
    float beta;
} mc_alpha_beta_t;

typedef struct
{
    float d;
    float q;
} mc_dq_t;

typedef struct
{
    float duty_a;
    float duty_b;
    float duty_c;
} mc_duty_cycle_t;

typedef struct
{
    uint16_t current_a_raw;
    uint16_t current_b_raw;
    uint16_t vbus_raw;
    uint16_t temp_raw;
} mc_adc_sample_t;

typedef struct
{
    bool run_request;
    int8_t direction;
    uint8_t hall_state;
} mc_input_request_t;

typedef enum
{
    MC_SHUNT_SINGLE = 0U,
    MC_SHUNT_DUAL = 1U,
    MC_SHUNT_TRIPLE = 2U
} mc_shunt_mode_t;

typedef enum
{
    MC_OBSERVER_SMO = 0U,
    MC_OBSERVER_PLL = 1U,
    MC_OBSERVER_HALL = 2U
} mc_observer_type_t;

typedef enum
{
    MC_STARTUP_FORCED_DRAG = 0U,
    MC_STARTUP_ZERO_SPEED_CLOSED_LOOP = 1U
} mc_startup_mode_t;

typedef enum
{
    MC_STATE_BOOT = 0U,
    MC_STATE_CALIBRATION = 1U,
    MC_STATE_IDLE = 2U,
    MC_STATE_STARTUP = 3U,
    MC_STATE_RUN = 4U,
    MC_STATE_FAULT = 5U
} mc_state_t;

typedef enum
{
    MC_FAULT_NONE = 0U,
    MC_FAULT_OVERCURRENT = (1UL << 0),
    MC_FAULT_UNDERVOLTAGE = (1UL << 1),
    MC_FAULT_OVERVOLTAGE = (1UL << 2),
    MC_FAULT_OVERTEMPERATURE = (1UL << 3),
    MC_FAULT_PHASE_LOSS = (1UL << 4),
    MC_FAULT_HW_TRIP = (1UL << 5),
    MC_FAULT_ADC_OVERRUN = (1UL << 6),
    MC_FAULT_OBSERVER_LOSS = (1UL << 7),
    MC_FAULT_UNSUPPORTED = (1UL << 8)
} mc_fault_bits_t;

typedef struct
{
    uint8_t pole_pairs;
    float rs_ohm;
    float ld_h;
    float lq_h;
    float flux_linkage_v_s;
    float max_phase_current_a;
    float rated_bus_voltage_v;
    float max_mech_speed_rpm;
} mc_motor_params_t;

typedef struct
{
    float adc_reference_voltage_v;
    uint16_t adc_full_scale;
    float current_shunt_resistor_ohm;
    float current_amplifier_gain;
    float vbus_divider_upper_resistor_ohm;
    float vbus_divider_lower_resistor_ohm;
    float temperature_sensor_voltage_at_25c_v;
    float temperature_sensor_slope_v_per_c;
} mc_feedback_hw_params_t;

typedef struct
{
    float kp;
    float ki;
    float output_limit;
} mc_pi_gains_t;

typedef struct
{
    mc_pi_gains_t id;
    mc_pi_gains_t iq;
    mc_pi_gains_t speed;
    float current_pi_bandwidth_hz;
    float current_loop_hz;
    float speed_loop_hz;
    float voltage_utilization;
} mc_control_config_t;

typedef struct
{
    float slide_gain;
    float slide_boundary_a;
    float emf_filter_hz;
    float speed_filter_hz;
    float valid_bemf_v;
} mc_smo_config_t;

typedef struct
{
    float align_current_a;
    float align_time_s;
    float drag_current_a;
    float start_electrical_hz;
    float target_electrical_hz;
    float acceleration_hz_per_s;
    float observer_blend_time_s;
    float handover_electrical_hz;
} mc_forced_drag_config_t;

typedef struct
{
    float over_current_a;
    float under_voltage_v;
    float over_voltage_v;
    float over_temp_c;
    float phase_loss_current_a;
    float phase_loss_speed_rpm;
    float phase_loss_hold_time_s;
} mc_protection_config_t;

typedef struct
{
    float default_speed_rpm;
    float speed_ramp_rpm_per_s;
} mc_command_config_t;

typedef struct
{
    uint32_t etmr_instance;
    uint32_t adc_instance;
    uint32_t pwm_frequency_hz;
    float deadtime_ns;
    float min_duty;
    uint16_t calibration_samples;
} mc_hardware_config_t;

typedef struct
{
    mc_shunt_mode_t shunt_mode;
    mc_observer_type_t observer_type;
    mc_startup_mode_t startup_mode;
    mc_motor_params_t motor;
    mc_feedback_hw_params_t feedback;
    mc_control_config_t control;
    mc_smo_config_t smo;
    mc_forced_drag_config_t forced_drag;
    mc_protection_config_t protection;
    mc_command_config_t command;
    mc_hardware_config_t hardware;
} mc_config_params_t;

typedef struct
{
    float adc_lsb_voltage_v;
    float current_a_per_count;
    float vbus_v_per_count;
    float temp_c_per_count;
    float temp_c_offset;
    float current_loop_dt_s;
    float speed_loop_dt_s;
    float ls_avg_h;
    mc_pi_gains_t id_pi;
    mc_pi_gains_t iq_pi;
    uint16_t current_offset_default_raw;
    uint16_t deadtime_ticks;
} mc_derived_params_t;

typedef struct
{
    mc_config_params_t user;
    mc_derived_params_t derived;
} mc_user_config_t;

#endif
