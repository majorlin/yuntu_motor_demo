#ifndef MC_CONTROL_H
#define MC_CONTROL_H

#include "mc_common.h"

typedef struct
{
    float kp;
    float ki;
    float integrator;
    float out_min;
    float out_max;
} mc_pi_controller_t;

typedef struct
{
    float alpha;
    float state;
} mc_lpf_t;

typedef struct
{
    float value;
    float rise_rate;
    float fall_rate;
} mc_ramp_t;

void MC_PI_Init(mc_pi_controller_t *controller, const mc_pi_gains_t *gains);
void MC_PI_Reset(mc_pi_controller_t *controller);
float MC_PI_Run(mc_pi_controller_t *controller, float reference, float feedback, float feedforward, float dt_s);

void MC_LPF_Init(mc_lpf_t *filter, float cutoff_hz, float dt_s, float initial_value);
float MC_LPF_Run(mc_lpf_t *filter, float sample);

void MC_Ramp_Init(mc_ramp_t *ramp, float rise_rate, float fall_rate, float initial_value);
float MC_Ramp_Run(mc_ramp_t *ramp, float target, float dt_s);

#endif
