#ifndef MC_OBSERVER_H
#define MC_OBSERVER_H

#include "mc_common.h"
#include "mc_control.h"

typedef struct
{
    float theta_rad;
    float speed_rad_s;
    mc_alpha_beta_t bemf;
    bool valid;
} mc_observer_output_t;

typedef struct
{
    float rs_ohm;
    float ls_h;
    float dt_s;
    float slide_gain;
    float slide_boundary_a;
    float valid_bemf_v;
    float theta_offset_rad;
    mc_alpha_beta_t current_hat;
    mc_alpha_beta_t bemf;
    mc_lpf_t bemf_alpha_lpf;
    mc_lpf_t bemf_beta_lpf;
    mc_lpf_t speed_lpf;
    float theta_rad;
    bool valid;
} mc_smo_observer_t;

typedef struct
{
    mc_observer_type_t type;
    mc_smo_observer_t smo;
    mc_observer_output_t output;
} mc_observer_t;

void MC_Observer_Init(mc_observer_t *observer, const mc_user_config_t *config);
void MC_Observer_Reset(mc_observer_t *observer);
void MC_Observer_Run(mc_observer_t *observer,
                     const mc_alpha_beta_t *voltage_ab,
                     const mc_alpha_beta_t *current_ab,
                     float dt_s);

#endif
