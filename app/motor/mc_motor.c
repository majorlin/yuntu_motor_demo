#include "mc_motor.h"
#include "mc_math.h"

static void MC_Motor_ResetControllers(mc_motor_t *motor)
{
    MC_PI_Reset(&motor->id_pi);
    MC_PI_Reset(&motor->iq_pi);
    MC_PI_Reset(&motor->speed_pi);
}

static void MC_Motor_SetIdleOutput(mc_motor_t *motor)
{
    mc_duty_cycle_t idle_duty;

    idle_duty.duty_a = 0.5f;
    idle_duty.duty_b = 0.5f;
    idle_duty.duty_c = 0.5f;
    motor->voltage_command_ab.alpha = 0.0f;
    motor->voltage_command_ab.beta = 0.0f;
    motor->voltage_command_dq.d = 0.0f;
    motor->voltage_command_dq.q = 0.0f;
    motor->hal->apply_pwm(&idle_duty);
}

static void MC_Motor_EnterState(mc_motor_t *motor, mc_state_t state)
{
    if (state == MC_STATE_IDLE)
    {
        motor->outputs_enabled = false;
        motor->hal->enable_outputs(false);
        MC_Motor_SetIdleOutput(motor);
        MC_Motor_ResetControllers(motor);
        MC_Startup_Reset(&motor->startup);
    }
    else if (state == MC_STATE_STARTUP)
    {
        motor->handover_active = false;
        motor->handover_blend = 0.0f;
        MC_Motor_ResetControllers(motor);
        MC_Startup_Reset(&motor->startup);
        MC_Observer_Reset(&motor->observer);
        motor->outputs_enabled = true;
        motor->hal->enable_outputs(true);
    }
    else if (state == MC_STATE_FAULT)
    {
        motor->outputs_enabled = false;
        motor->hal->enable_outputs(false);
        MC_Motor_SetIdleOutput(motor);
    }

    motor->state = state;
}

static void MC_Motor_UpdateMeasurements(mc_motor_t *motor, const mc_adc_sample_t *sample)
{
    motor->phase_currents_abc.a =
        ((float)sample->current_a_raw - (float)motor->current_a_offset_raw) * motor->config.derived.current_a_per_count;
    motor->phase_currents_abc.b =
        ((float)sample->current_b_raw - (float)motor->current_b_offset_raw) * motor->config.derived.current_a_per_count;
    motor->phase_currents_abc.c = -motor->phase_currents_abc.a - motor->phase_currents_abc.b;
    motor->vbus_v = (float)sample->vbus_raw * motor->config.derived.vbus_v_per_count;
    motor->temperature_c =
        ((float)sample->temp_raw * motor->config.derived.temp_c_per_count) + motor->config.derived.temp_c_offset;
    MC_Math_Clarke(&motor->phase_currents_abc, &motor->phase_currents_ab);
}

static void MC_Motor_RunCalibration(mc_motor_t *motor, const mc_adc_sample_t *sample)
{
    motor->calibration_accum_a += sample->current_a_raw;
    motor->calibration_accum_b += sample->current_b_raw;
    motor->calibration_count++;

    if (motor->calibration_count >= motor->config.user.hardware.calibration_samples)
    {
        motor->current_a_offset_raw = (uint16_t)(motor->calibration_accum_a / motor->calibration_count);
        motor->current_b_offset_raw = (uint16_t)(motor->calibration_accum_b / motor->calibration_count);
        MC_Motor_EnterState(motor, MC_STATE_IDLE);
    }
}

static void MC_Motor_RunCurrentLoop(mc_motor_t *motor, float electrical_angle_rad)
{
    float sin_theta;
    float cos_theta;
    float voltage_limit_v;
    static mc_svpwm_result_t svm_result;

    MC_Math_FastSinCos(electrical_angle_rad, &sin_theta, &cos_theta);
    MC_Math_Park(&motor->phase_currents_ab, sin_theta, cos_theta, &motor->phase_currents_dq);

    voltage_limit_v = motor->config.user.control.voltage_utilization * motor->vbus_v * MC_ONE_BY_SQRT3_F;
    motor->id_pi.out_min = -voltage_limit_v;
    motor->id_pi.out_max = voltage_limit_v;
    motor->iq_pi.out_min = -voltage_limit_v;
    motor->iq_pi.out_max = voltage_limit_v;

    motor->voltage_command_dq.d = MC_PI_Run(&motor->id_pi,
                                            motor->id_reference_a,
                                            motor->phase_currents_dq.d,
                                            0.0f,
                                            motor->config.derived.current_loop_dt_s);
    motor->voltage_command_dq.q = MC_PI_Run(&motor->iq_pi,
                                            motor->iq_reference_a,
                                            motor->phase_currents_dq.q,
                                            0.0f,
                                            motor->config.derived.current_loop_dt_s);

    MC_Math_LimitMagnitude(&motor->voltage_command_dq.d, &motor->voltage_command_dq.q, voltage_limit_v);
    MC_Math_InvPark(&motor->voltage_command_dq, sin_theta, cos_theta, &motor->voltage_command_ab);
    MC_Svpwm_Generate(&motor->voltage_command_ab,
                      motor->vbus_v,
                      motor->config.user.hardware.min_duty,
                      &svm_result);
    motor->hal->apply_pwm(&svm_result.duty);
}

static void MC_Motor_UpdateSpeedFeedback(mc_motor_t *motor)
{
    motor->speed_feedback_rpm =
        motor->electrical_speed_rad_s * motor->config.derived.electrical_speed_to_mech_rpm;
}

static void MC_Motor_RunSpeedLoop(mc_motor_t *motor)
{
    motor->id_reference_a = 0.0f;
    motor->iq_reference_a = MC_PI_Run(&motor->speed_pi,
                                      motor->speed_command_rad_s,
                                      motor->electrical_speed_rad_s,
                                      0.0f,
                                      motor->last_slow_loop_ts_s);
    motor->iq_reference_a = MC_Math_Clamp(motor->iq_reference_a,
                                          -motor->config.user.motor.max_phase_current_a,
                                          motor->config.user.motor.max_phase_current_a);
}

void MC_Motor_Init(mc_motor_t *motor, const mc_hal_ops_t *hal, const mc_user_config_t *config)
{
    motor->hal = hal;
    motor->config = *config;
    motor->state = MC_STATE_BOOT;
    motor->fault_mask = MC_FAULT_NONE;
    motor->current_a_offset_raw = config->derived.current_offset_default_raw;
    motor->current_b_offset_raw = config->derived.current_offset_default_raw;
    motor->calibration_accum_a = 0U;
    motor->calibration_accum_b = 0U;
    motor->calibration_count = 0U;
    motor->outputs_enabled = false;
    motor->handover_active = false;
    motor->handover_blend = 0.0f;
    motor->speed_command_rad_s = 0.0f;
    motor->speed_feedback_rpm = 0.0f;
    motor->iq_reference_a = 0.0f;
    motor->id_reference_a = 0.0f;
    motor->electrical_angle_rad = 0.0f;
    motor->electrical_speed_rad_s = 0.0f;
    motor->last_slow_loop_tick_ms = 0U;
    motor->last_slow_loop_ts_s = config->derived.speed_loop_dt_s;
    motor->input_request.run_request = false;
    motor->input_request.direction = 1;
    motor->input_request.hall_state = 0U;
    motor->vbus_v = 0.0f;
    motor->temperature_c = 25.0f;
    MC_PI_Init(&motor->id_pi, &config->derived.id_pi);
    MC_PI_Init(&motor->iq_pi, &config->derived.iq_pi);
    MC_PI_Init(&motor->speed_pi, &config->derived.speed_pi);
    MC_Ramp_Init(&motor->speed_ramp,
                 config->user.command.speed_ramp_rpm_per_s,
                 config->user.command.speed_ramp_rpm_per_s,
                 0.0f);
    MC_Observer_Init(&motor->observer, config);
    MC_Startup_Init(&motor->startup, config);
    MC_Protection_Init(&motor->protection, config);
    (void)motor->hal->init(config);
    motor->hal->enable_outputs(false);
    MC_Motor_SetIdleOutput(motor);
    MC_Motor_EnterState(motor, MC_STATE_CALIBRATION);
}

void MC_Motor_FastLoop(mc_motor_t *motor, const mc_adc_sample_t *sample)
{
    static mc_startup_output_t startup_output;
    float electrical_angle;
    float handover_error;
    float dt_s;

    dt_s = motor->config.derived.current_loop_dt_s;
    motor->last_sample = *sample;

    if (motor->state == MC_STATE_CALIBRATION)
    {
        MC_Motor_RunCalibration(motor, sample);
        return;
    }

    MC_Motor_UpdateMeasurements(motor, sample);
    MC_Observer_Run(&motor->observer, &motor->voltage_command_ab, &motor->phase_currents_ab, dt_s);
    motor->electrical_speed_rad_s = motor->observer.output.speed_rad_s;

    motor->fault_mask |= MC_Protection_Run(&motor->protection,
                                           &motor->phase_currents_abc,
                                           motor->iq_reference_a,
                                           motor->electrical_speed_rad_s,
                                           motor->vbus_v,
                                           motor->temperature_c,
                                           dt_s);
    if (motor->fault_mask != MC_FAULT_NONE)
    {
        // MC_Motor_EnterState(motor, MC_STATE_FAULT);
        // return;
    }

    if (!motor->input_request.run_request)
    {
        if (motor->state != MC_STATE_IDLE)
        {
            MC_Motor_EnterState(motor, MC_STATE_IDLE);
        }
        return;
    }

    if (motor->state == MC_STATE_IDLE)
    {
        MC_Motor_EnterState(motor, MC_STATE_STARTUP);
    }

    if (motor->state == MC_STATE_STARTUP)
    {
        MC_Startup_Run(&motor->startup, dt_s, motor->observer.output.valid, &startup_output);
        electrical_angle = startup_output.angle_rad;
        if (motor->input_request.direction < 0)
        {
            electrical_angle = MC_Math_WrapAngle(-electrical_angle);
            startup_output.speed_rad_s = -startup_output.speed_rad_s;
            startup_output.iq_ref_a = -startup_output.iq_ref_a;
        }

        if (startup_output.observer_handover_ready)
        {
            if (!motor->handover_active)
            {
                motor->handover_active = true;
                motor->handover_blend = 0.0f;
            }

            handover_error = MC_Math_WrapDelta(motor->observer.output.theta_rad - electrical_angle);
            motor->handover_blend += dt_s / motor->config.user.forced_drag.observer_blend_time_s;
            motor->handover_blend = MC_Math_Clamp(motor->handover_blend, 0.0f, 1.0f);
            electrical_angle = MC_Math_WrapAngle(electrical_angle + (motor->handover_blend * handover_error));
            if (motor->handover_blend >= 1.0f)
            {
                MC_Motor_EnterState(motor, MC_STATE_RUN);
                electrical_angle = motor->observer.output.theta_rad;
            }
        }

        motor->electrical_angle_rad = electrical_angle;
        motor->electrical_speed_rad_s = startup_output.speed_rad_s;
        motor->id_reference_a = startup_output.id_ref_a;
        motor->iq_reference_a = startup_output.iq_ref_a;
        MC_Motor_RunCurrentLoop(motor, motor->electrical_angle_rad);
        return;
    }

    if (motor->state == MC_STATE_RUN)
    {
        if (!motor->observer.output.valid)
        {
            motor->fault_mask |= MC_FAULT_OBSERVER_LOSS;
            MC_Motor_EnterState(motor, MC_STATE_FAULT);
            return;
        }

        motor->electrical_angle_rad = motor->observer.output.theta_rad;
        MC_Motor_RunCurrentLoop(motor, motor->electrical_angle_rad);
    }
}

void MC_Motor_Background(mc_motor_t *motor)
{
    uint32_t now_ms;
    float dt_s;
    float target_speed_rpm;

    now_ms = motor->hal->get_tick_ms();
    if (now_ms == motor->last_slow_loop_tick_ms)
    {
        return;
    }

    dt_s = (float)(now_ms - motor->last_slow_loop_tick_ms) * 0.001f;
    motor->last_slow_loop_tick_ms = now_ms;
    motor->last_slow_loop_ts_s = dt_s;

    motor->hal->poll_inputs(&motor->input_request);
    target_speed_rpm = motor->input_request.run_request ?
        ((float)motor->input_request.direction * motor->config.user.command.default_speed_rpm) :
        0.0f;
    target_speed_rpm = MC_Ramp_Run(&motor->speed_ramp, target_speed_rpm, dt_s);
    motor->speed_command_rad_s =
        target_speed_rpm * motor->config.derived.mech_rpm_to_electrical_speed_rad_s;
    MC_Motor_UpdateSpeedFeedback(motor);

    if (motor->state == MC_STATE_RUN)
    {
        MC_Motor_RunSpeedLoop(motor);
    }

    if ((motor->state == MC_STATE_FAULT) && !motor->input_request.run_request)
    {
        motor->hal->clear_fault_latch();
        motor->fault_mask = MC_FAULT_NONE;
        MC_Protection_Reset(&motor->protection);
        MC_Observer_Reset(&motor->observer);
        MC_Motor_EnterState(motor, MC_STATE_IDLE);
    }

    motor->hal->set_status(motor->state, motor->fault_mask);
}

void MC_Motor_ReportFault(mc_motor_t *motor, uint32_t fault_mask)
{
    motor->fault_mask |= fault_mask;
    MC_Motor_EnterState(motor, MC_STATE_FAULT);
}
