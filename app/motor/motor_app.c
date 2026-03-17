#include "motor/motor_app.h"

#include "motor/mc_motor.h"
#include "motor/ytm32/mc_hal_ytm32.h"

static mc_motor_t s_motor;

static const mc_user_config_t s_motor_config =
{
    .shunt_mode = MC_SHUNT_DUAL,
    .observer_type = MC_OBSERVER_SMO,
    .startup_mode = MC_STARTUP_FORCED_DRAG,
    .motor =
    {
        .pole_pairs = 7U,
        .rs_ohm = 0.18f,
        .ld_h = 0.00018f,
        .lq_h = 0.00018f,
        .flux_linkage_v_s = 0.012f,
        .max_phase_current_a = 10.0f,
        .rated_bus_voltage_v = 24.0f,
        .max_mech_speed_rpm = 4000.0f
    },
    .adc_scale =
    {
        .current_a_per_count = 0.0040f,
        .vbus_v_per_count = 0.0161f,
        .temp_c_per_count = 0.0806f,
        .temp_c_offset = -50.0f,
        .adc_full_scale = 4095U
    },
    .control =
    {
        .id = {2.0f, 400.0f, 12.0f},
        .iq = {2.0f, 400.0f, 12.0f},
        .speed = {0.012f, 0.40f, 6.0f},
        .current_loop_hz = 20000.0f,
        .speed_loop_hz = 1000.0f,
        .voltage_utilization = 0.92f
    },
    .smo =
    {
        .slide_gain = 18.0f,
        .slide_boundary_a = 1.0f,
        .emf_filter_hz = 1200.0f,
        .speed_filter_hz = 250.0f,
        .valid_bemf_v = 0.4f
    },
    .forced_drag =
    {
        .align_current_a = 1.8f,
        .align_time_s = 0.18f,
        .drag_current_a = 2.4f,
        .start_electrical_hz = 2.0f,
        .target_electrical_hz = 45.0f,
        .acceleration_hz_per_s = 220.0f,
        .observer_blend_time_s = 0.05f,
        .handover_electrical_hz = 20.0f
    },
    .protection =
    {
        .over_current_a = 12.0f,
        .under_voltage_v = 10.0f,
        .over_voltage_v = 30.0f,
        .over_temp_c = 110.0f,
        .phase_loss_current_a = 2.0f,
        .phase_loss_speed_rpm = 600.0f,
        .phase_loss_hold_time_s = 0.08f
    },
    .command =
    {
        .default_speed_rpm = 1200.0f,
        .speed_ramp_rpm_per_s = 3000.0f
    },
    .hardware =
    {
        .etmr_instance = 0U,
        .adc_instance = 0U,
        .adc_trigger_source = 0U,
        .pwm_frequency_hz = 20000U,
        .deadtime_ticks = 80U,
        .min_duty = 0.03f,
        .calibration_samples = 1024U
    }
};

static void MotorApp_FastLoopCallback(const mc_adc_sample_t *sample)
{
    MC_Motor_FastLoop(&s_motor, sample);
}

static void MotorApp_FaultCallback(uint32_t fault_mask)
{
    MC_Motor_ReportFault(&s_motor, fault_mask);
}

void MotorApp_Init(void)
{
    MC_Motor_Init(&s_motor, &g_mc_hal_ytm32_ops, &s_motor_config);
    g_mc_hal_ytm32_ops.bind_callbacks(MotorApp_FastLoopCallback, MotorApp_FaultCallback);
}

void MotorApp_Run(void)
{
    MC_Motor_Background(&s_motor);
}
