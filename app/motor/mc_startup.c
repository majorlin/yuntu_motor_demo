#include "mc_startup.h"
#include "mc_math.h"

void MC_Startup_Init(mc_startup_t *startup, const mc_user_config_t *config)
{
    startup->type = config->user.startup_mode;
    startup->config = config->user.forced_drag;
    MC_Startup_Reset(startup);
}

void MC_Startup_Reset(mc_startup_t *startup)
{
    startup->elapsed_s = 0.0f;
    startup->angle_rad = 0.0f;
    startup->electrical_hz = startup->config.start_electrical_hz;
}

void MC_Startup_Run(mc_startup_t *startup, float dt_s, bool observer_valid, mc_startup_output_t *output)
{
    float speed_rad_s;

    startup->elapsed_s += dt_s;
    output->observer_handover_ready = false;

    if (startup->type != MC_STARTUP_FORCED_DRAG)
    {
        output->angle_rad = startup->angle_rad;
        output->speed_rad_s = 0.0f;
        output->id_ref_a = 0.0f;
        output->iq_ref_a = 0.0f;
        return;
    }

    if (startup->elapsed_s < startup->config.align_time_s)
    {
        output->angle_rad = 0.0f;
        output->speed_rad_s = 0.0f;
        output->id_ref_a = startup->config.align_current_a;
        output->iq_ref_a = 0.0f;
        startup->angle_rad = 0.0f;
        startup->electrical_hz = startup->config.start_electrical_hz;
        return;
    }

    startup->electrical_hz += startup->config.acceleration_hz_per_s * dt_s;
    if (startup->electrical_hz > startup->config.target_electrical_hz)
    {
        startup->electrical_hz = startup->config.target_electrical_hz;
    }

    speed_rad_s = startup->electrical_hz * MC_TWO_PI_F;
    startup->angle_rad = MC_Math_WrapAngle(startup->angle_rad + (speed_rad_s * dt_s));

    output->angle_rad = startup->angle_rad;
    output->speed_rad_s = speed_rad_s;
    output->id_ref_a = 0.0f;
    output->iq_ref_a = startup->config.drag_current_a;
    output->observer_handover_ready =
        observer_valid && (startup->electrical_hz >= startup->config.handover_electrical_hz);
}
