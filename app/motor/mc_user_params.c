#include "mc_user_params.h"

#include "clock.h"
#include "device_registers.h"

static const clock_names_t s_etmr_clock_names[eTMR_INSTANCE_COUNT] = eTMR_IPC_CLK;

/*
 * This file is the single entry point for motor-control parameter editing.
 *
 * Rule of thumb:
 * 1. Only edit s_user_params below.
 * 2. Do not hand-edit derived values in other modules.
 * 3. When hardware changes, first update feedback/hardware group parameters.
 *
 * Hardware parameters you usually need to confirm from the schematic:
 * - adc_reference_voltage_v:
 *   ADC reference voltage actually used by the chip.
 * - current_shunt_resistor_ohm:
 *   Phase current shunt resistor value.
 * - current_amplifier_gain:
 *   Gain from shunt voltage to ADC input.
 * - vbus_divider_upper_resistor_ohm / vbus_divider_lower_resistor_ohm:
 *   DC bus divider network to ADC.
 * - temperature_sensor_voltage_at_25c_v / temperature_sensor_slope_v_per_c:
 *   Linear temperature sensor model seen by ADC.
 *   If your board uses NTC, this can be replaced later by a table/model module.
 * - pwm_frequency_hz:
 *   PWM carrier frequency.
 * - deadtime_ns:
 *   Required high/low complementary deadtime on gate drive side.
 *
 * Current board trigger route:
 * - ADC hardware trigger is fixed to eTMR0 CH1 matching event through TMU.
 * - In complementary PWM mode, CH1 VAL0/VAL1 are reserved for ADC trigger timing.
 * - User no longer needs to configure an ADC trigger mux parameter.
 */
static const mc_config_params_t s_user_params =
{
    /* Current implementation path: dual-shunt + SMO + forced-drag startup. */
    .shunt_mode = MC_SHUNT_DUAL,
    .observer_type = MC_OBSERVER_SMO,
    .startup_mode = MC_STARTUP_FORCED_DRAG,

    .motor =
    {
        /* Number of motor pole pairs. Mechanical/electrical speed conversion depends on it. */
        .pole_pairs = 4U,
        /* Phase resistance in ohm, line-neutral equivalent used by observer/decoupling. */
        .rs_ohm = 0.05f,
        /* D-axis inductance in henry. Here Ld uses the measured phase inductance value. */
        .ld_h = 0.00045158f,
        /* Q-axis inductance in henry. Here Lq uses the measured phase inductance value. */
        .lq_h = 0.00045158f,
        /*
         * Flux linkage estimated from line-line back-EMF test:
         * lambda ~= Vab_pp / (2 * sqrt(3) * 2 * pi * fe)
         *        ~= 28.6 / (2 * 1.732 * 2 * pi * 385)
         *        ~= 0.00341 V*s
         */
        .flux_linkage_v_s = 0.00341f,
        /* Continuous or software-limited peak phase current. */
        .max_phase_current_a = 10.0f,
        /* Nominal DC bus voltage of your power stage. */
        .rated_bus_voltage_v = 24.0f,
        /*
         * Reference maximum mechanical speed from the same back-EMF test:
         * n ~= fe / pole_pairs * 60 = 385 / 4 * 60 = 5775 rpm
         */
        .max_mech_speed_rpm = 5800.0f
    },

    .feedback =
    {
        /* ADC reference voltage. */
        .adc_reference_voltage_v = 5.0f,
        /* ADC full-scale count for 12-bit converter. */
        .adc_full_scale = 4095U,
        /* Current sampling shunt resistor. */
        .current_shunt_resistor_ohm = 0.005f,
        /* Current amplifier gain from shunt to ADC input. */
        .current_amplifier_gain = 10.0f,
        /* DC bus divider top resistor. */
        .vbus_divider_upper_resistor_ohm = 4000.0f,
        /* DC bus divider bottom resistor. */
        .vbus_divider_lower_resistor_ohm = 1000.0f,
        /* ADC input voltage when temperature is 25 C. */
        .temperature_sensor_voltage_at_25c_v = 2.015f,
        /* Sensor slope in V/C. Positive means voltage rises with temperature. */
        .temperature_sensor_slope_v_per_c = 0.001f
    },

    .control =
    {
        /*
         * Id/Iq current PI gains are auto-derived in MC_UserParams_Load():
         * Kp = L * 2 * pi * fc
         * Ki = R * 2 * pi * fc
         * Only the output_limit here is user-configured.
         */
        .id = {0.0f, 0.0f, 12.0f},
        .iq = {0.0f, 0.0f, 12.0f},
        .speed = {0.012f, 0.40f, 6.0f},
        /* Conservative initial current-loop design bandwidth for first power-up debug. */
        .current_pi_bandwidth_hz = 1000.0f,
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
        .align_current_a = 0.4f,
        .align_time_s = 0.18f,
        .drag_current_a = 0.8f,
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
        .over_voltage_v = 24.0f,
        .over_temp_c = 110.0f,
        .phase_loss_current_a = 2.0f,
        .phase_loss_speed_rpm = 600.0f,
        .phase_loss_hold_time_s = 0.08f,
        /* Debug stage: keep current/bus protection, disable temperature fault temporarily. */
        .enable_over_temperature = false,
        /* Debug stage: phase-loss diagnosis is disabled until current reconstruction is stable. */
        .enable_phase_loss = false
    },

    .command =
    {
        .default_speed_rpm = 1200.0f,
        .speed_ramp_rpm_per_s = 3000.0f
    },

    .hardware =
    {
        /* Current board uses eTMR0 for three-phase PWM. */
        .etmr_instance = 0U,
        /* Current board uses ADC0 for current/bus/temp sampling. */
        .adc_instance = 0U,
        /* PWM carrier frequency. */
        .pwm_frequency_hz = 20000U,
        /* Deadtime requirement at gate drive, in nanoseconds. */
        .deadtime_ns = 666.0f,
        /* Minimum duty clamp to preserve sampling window and bootstrap margin. */
        .min_duty = 0.03f,
        /* Current offset auto-calibration sample count at startup. */
        .calibration_samples = 1024U
    }
};

status_t MC_UserParams_Load(mc_user_config_t *config)
{
    float adc_lsb_voltage_v;
    float current_denominator;
    float temperature_slope;
    float current_pi_omega_rad_s;
    float deadtime_tick_f;
    float etmr_clock_hz_f;
    uint32_t etmr_clock_hz;

    if (config == 0)
    {
        return STATUS_ERROR;
    }

    config->user = s_user_params;

    adc_lsb_voltage_v =
        config->user.feedback.adc_reference_voltage_v / (float)config->user.feedback.adc_full_scale;
    current_denominator =
        config->user.feedback.current_shunt_resistor_ohm * config->user.feedback.current_amplifier_gain;
    temperature_slope = config->user.feedback.temperature_sensor_slope_v_per_c;

    if ((current_denominator <= 0.0f) ||
        (config->user.feedback.vbus_divider_lower_resistor_ohm <= 0.0f) ||
        (temperature_slope == 0.0f) ||
        (config->user.control.current_pi_bandwidth_hz <= 0.0f) ||
        (config->user.control.current_loop_hz <= 0.0f) ||
        (config->user.control.speed_loop_hz <= 0.0f))
    {
        return STATUS_ERROR;
    }

    config->derived.adc_lsb_voltage_v = adc_lsb_voltage_v;
    config->derived.current_a_per_count = adc_lsb_voltage_v / current_denominator;
    config->derived.vbus_v_per_count =
        adc_lsb_voltage_v *
        ((config->user.feedback.vbus_divider_upper_resistor_ohm +
          config->user.feedback.vbus_divider_lower_resistor_ohm) /
         config->user.feedback.vbus_divider_lower_resistor_ohm);
    config->derived.temp_c_per_count = adc_lsb_voltage_v / temperature_slope;
    config->derived.temp_c_offset =
        25.0f - (config->user.feedback.temperature_sensor_voltage_at_25c_v / temperature_slope);
    config->derived.current_loop_dt_s = 1.0f / config->user.control.current_loop_hz;
    config->derived.speed_loop_dt_s = 1.0f / config->user.control.speed_loop_hz;
    config->derived.electrical_speed_to_mech_rpm =
        (config->user.motor.pole_pairs > 0U) ?
        (60.0f / (MC_TWO_PI_F * (float)config->user.motor.pole_pairs)) :
        0.0f;
    config->derived.mech_rpm_to_electrical_speed_rad_s =
        (config->user.motor.pole_pairs > 0U) ?
        (MC_TWO_PI_F * (float)config->user.motor.pole_pairs / 60.0f) :
        0.0f;
    config->derived.ls_avg_h = 0.5f * (config->user.motor.ld_h + config->user.motor.lq_h);
    current_pi_omega_rad_s = MC_TWO_PI_F * config->user.control.current_pi_bandwidth_hz;
    config->derived.id_pi.kp = config->user.motor.ld_h * current_pi_omega_rad_s;
    config->derived.id_pi.ki = config->user.motor.rs_ohm * current_pi_omega_rad_s;
    config->derived.id_pi.output_limit = config->user.control.id.output_limit;
    config->derived.iq_pi.kp = config->user.motor.lq_h * current_pi_omega_rad_s;
    config->derived.iq_pi.ki = config->user.motor.rs_ohm * current_pi_omega_rad_s;
    config->derived.iq_pi.output_limit = config->user.control.iq.output_limit;
    config->derived.speed_pi.kp =
        config->user.control.speed.kp * config->derived.electrical_speed_to_mech_rpm;
    config->derived.speed_pi.ki =
        config->user.control.speed.ki * config->derived.electrical_speed_to_mech_rpm;
    config->derived.speed_pi.output_limit = config->user.control.speed.output_limit;
    config->derived.current_offset_default_raw = config->user.feedback.adc_full_scale / 2U;

    etmr_clock_hz = 0U;
    if (CLOCK_SYS_GetFreq(s_etmr_clock_names[config->user.hardware.etmr_instance], &etmr_clock_hz) != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    etmr_clock_hz_f = (float)etmr_clock_hz;
    deadtime_tick_f = (config->user.hardware.deadtime_ns * 1.0e-9f) * etmr_clock_hz_f;
    if (deadtime_tick_f < 1.0f)
    {
        deadtime_tick_f = 1.0f;
    }
    config->derived.deadtime_ticks = (uint16_t)(deadtime_tick_f + 0.5f);

    return STATUS_SUCCESS;
}
