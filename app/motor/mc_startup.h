#ifndef MC_STARTUP_H
#define MC_STARTUP_H

#include "mc_common.h"

typedef struct
{
    float angle_rad;
    float speed_rad_s;
    float id_ref_a;
    float iq_ref_a;
    bool observer_handover_ready;
} mc_startup_output_t;

typedef struct
{
    mc_startup_mode_t type;
    mc_forced_drag_config_t config;
    float elapsed_s;
    float angle_rad;
    float electrical_hz;
} mc_startup_t;

void MC_Startup_Init(mc_startup_t *startup, const mc_user_config_t *config);
void MC_Startup_Reset(mc_startup_t *startup);
void MC_Startup_Run(mc_startup_t *startup, float dt_s, bool observer_valid, mc_startup_output_t *output);

#endif
