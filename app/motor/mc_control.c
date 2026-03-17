#include "mc_control.h"
#include "mc_math.h"

void MC_PI_Init(mc_pi_controller_t *controller, const mc_pi_gains_t *gains)
{
    controller->kp = gains->kp;
    controller->ki = gains->ki;
    controller->integrator = 0.0f;
    controller->out_min = -gains->output_limit;
    controller->out_max = gains->output_limit;
}

void MC_PI_Reset(mc_pi_controller_t *controller)
{
    controller->integrator = 0.0f;
}

float MC_PI_Run(mc_pi_controller_t *controller, float reference, float feedback, float feedforward, float dt_s)
{
    float error;
    float output;
    float unsaturated;

    error = reference - feedback;
    controller->integrator += controller->ki * error * dt_s;
    controller->integrator = MC_Math_Clamp(controller->integrator, controller->out_min, controller->out_max);

    unsaturated = (controller->kp * error) + controller->integrator + feedforward;
    output = MC_Math_Clamp(unsaturated, controller->out_min, controller->out_max);

    if (unsaturated != output)
    {
        controller->integrator += 0.1f * (output - unsaturated);
        controller->integrator = MC_Math_Clamp(controller->integrator, controller->out_min, controller->out_max);
    }

    return output;
}

void MC_LPF_Init(mc_lpf_t *filter, float cutoff_hz, float dt_s, float initial_value)
{
    float tau;

    if (cutoff_hz <= 0.0f)
    {
        filter->alpha = 1.0f;
    }
    else
    {
        tau = 1.0f / (MC_TWO_PI_F * cutoff_hz);
        filter->alpha = dt_s / (tau + dt_s);
    }
    filter->state = initial_value;
}

float MC_LPF_Run(mc_lpf_t *filter, float sample)
{
    filter->state += filter->alpha * (sample - filter->state);
    return filter->state;
}

void MC_Ramp_Init(mc_ramp_t *ramp, float rise_rate, float fall_rate, float initial_value)
{
    ramp->value = initial_value;
    ramp->rise_rate = rise_rate;
    ramp->fall_rate = fall_rate;
}

float MC_Ramp_Run(mc_ramp_t *ramp, float target, float dt_s)
{
    float delta;
    float max_step;

    delta = target - ramp->value;
    if (delta >= 0.0f)
    {
        max_step = ramp->rise_rate * dt_s;
        delta = MC_Math_Clamp(delta, -max_step, max_step);
    }
    else
    {
        max_step = ramp->fall_rate * dt_s;
        delta = MC_Math_Clamp(delta, -max_step, max_step);
    }

    ramp->value += delta;
    return ramp->value;
}
