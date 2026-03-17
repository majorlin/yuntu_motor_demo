#include "mc_svpwm.h"
#include "mc_math.h"

void MC_Svpwm_Generate(const mc_alpha_beta_t *voltage_ab,
                       float vbus_v,
                       float min_duty,
                       mc_svpwm_result_t *result)
{
    float u;
    float v;
    float w;
    float min_value;
    float max_value;
    float common;
    float inv_vbus;
    float max_duty;

    if (vbus_v <= 0.1f)
    {
        result->duty.duty_a = 0.5f;
        result->duty.duty_b = 0.5f;
        result->duty.duty_c = 0.5f;
        result->saturated = true;
        return;
    }

    inv_vbus = 1.0f / vbus_v;
    u = voltage_ab->alpha * inv_vbus;
    v = ((-0.5f * voltage_ab->alpha) + (MC_SQRT3_BY_2_F * voltage_ab->beta)) * inv_vbus;
    w = ((-0.5f * voltage_ab->alpha) - (MC_SQRT3_BY_2_F * voltage_ab->beta)) * inv_vbus;

    min_value = u;
    if (v < min_value)
    {
        min_value = v;
    }
    if (w < min_value)
    {
        min_value = w;
    }

    max_value = u;
    if (v > max_value)
    {
        max_value = v;
    }
    if (w > max_value)
    {
        max_value = w;
    }

    common = -0.5f * (max_value + min_value);
    max_duty = 1.0f - min_duty;

    result->duty.duty_a = MC_Math_Clamp(0.5f + u + common, min_duty, max_duty);
    result->duty.duty_b = MC_Math_Clamp(0.5f + v + common, min_duty, max_duty);
    result->duty.duty_c = MC_Math_Clamp(0.5f + w + common, min_duty, max_duty);
    result->saturated =
        (result->duty.duty_a <= min_duty) || (result->duty.duty_a >= max_duty) ||
        (result->duty.duty_b <= min_duty) || (result->duty.duty_b >= max_duty) ||
        (result->duty.duty_c <= min_duty) || (result->duty.duty_c >= max_duty);
}
