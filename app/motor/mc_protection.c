#include "mc_protection.h"
#include "mc_math.h"

void MC_Protection_Init(mc_protection_t *protection, const mc_user_config_t *config)
{
    protection->config = config->user.protection;
    MC_Protection_Reset(protection);
}

void MC_Protection_Reset(mc_protection_t *protection)
{
    protection->fault_mask = MC_FAULT_NONE;
    protection->phase_loss_counter = 0U;
}

uint32_t MC_Protection_Run(mc_protection_t *protection,
                           const mc_abc_t *currents_abc,
                           float iq_reference_a,
                           float speed_rpm,
                           float vbus_v,
                           float temperature_c,
                           float dt_s)
{
    float ia_abs;
    float ib_abs;
    float ic_abs;
    float phase_loss_limit_count;
    bool phase_loss_candidate;

    ia_abs = MC_Math_Abs(currents_abc->a);
    ib_abs = MC_Math_Abs(currents_abc->b);
    ic_abs = MC_Math_Abs(currents_abc->c);

    if ((ia_abs > protection->config.over_current_a) ||
        (ib_abs > protection->config.over_current_a) ||
        (ic_abs > protection->config.over_current_a))
    {
        protection->fault_mask |= MC_FAULT_OVERCURRENT;
    }

    if (vbus_v < protection->config.under_voltage_v)
    {
        protection->fault_mask |= MC_FAULT_UNDERVOLTAGE;
    }

    if (vbus_v > protection->config.over_voltage_v)
    {
        protection->fault_mask |= MC_FAULT_OVERVOLTAGE;
    }

    if (temperature_c > protection->config.over_temp_c)
    {
        protection->fault_mask |= MC_FAULT_OVERTEMPERATURE;
    }

    phase_loss_candidate =
        (MC_Math_Abs(iq_reference_a) >= protection->config.phase_loss_current_a) &&
        (MC_Math_Abs(speed_rpm) >= protection->config.phase_loss_speed_rpm) &&
        ((ia_abs < (0.4f * protection->config.phase_loss_current_a)) ||
         (ib_abs < (0.4f * protection->config.phase_loss_current_a)) ||
         (ic_abs < (0.4f * protection->config.phase_loss_current_a)));

    if (phase_loss_candidate)
    {
        phase_loss_limit_count = protection->config.phase_loss_hold_time_s / dt_s;
        protection->phase_loss_counter++;
        if ((float)protection->phase_loss_counter >= phase_loss_limit_count)
        {
            protection->fault_mask |= MC_FAULT_PHASE_LOSS;
        }
    }
    else
    {
        protection->phase_loss_counter = 0U;
    }

    return protection->fault_mask;
}
