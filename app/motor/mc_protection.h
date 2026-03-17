#ifndef MC_PROTECTION_H
#define MC_PROTECTION_H

#include "mc_common.h"

typedef struct
{
    mc_protection_config_t config;
    uint32_t fault_mask;
    uint32_t phase_loss_counter;
} mc_protection_t;

void MC_Protection_Init(mc_protection_t *protection, const mc_user_config_t *config);
void MC_Protection_Reset(mc_protection_t *protection);
uint32_t MC_Protection_Run(mc_protection_t *protection,
                           const mc_abc_t *currents_abc,
                           float iq_reference_a,
                           float speed_rpm,
                           float vbus_v,
                           float temperature_c,
                           float dt_s);

#endif
