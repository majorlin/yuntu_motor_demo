#ifndef MC_SVPWM_H
#define MC_SVPWM_H

#include <stdbool.h>
#include "mc_common.h"

typedef struct
{
    mc_duty_cycle_t duty;
    bool saturated;
} mc_svpwm_result_t;

void MC_Svpwm_Generate(const mc_alpha_beta_t *voltage_ab,
                       float vbus_v,
                       float min_duty,
                       mc_svpwm_result_t *result);

#endif
