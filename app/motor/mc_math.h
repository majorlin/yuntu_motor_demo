#ifndef MC_MATH_H
#define MC_MATH_H

#include "mc_common.h"

float MC_Math_Abs(float x);
float MC_Math_Clamp(float x, float min_value, float max_value);
float MC_Math_WrapAngle(float angle_rad);
float MC_Math_WrapDelta(float delta_rad);
float MC_Math_FastSqrt(float x);
void MC_Math_FastSinCos(float angle_rad, float *sin_value, float *cos_value);
float MC_Math_FastAtan2(float y, float x);
void MC_Math_Clarke(const mc_abc_t *abc, mc_alpha_beta_t *alpha_beta);
void MC_Math_Park(const mc_alpha_beta_t *alpha_beta, float sin_theta, float cos_theta, mc_dq_t *dq);
void MC_Math_InvPark(const mc_dq_t *dq, float sin_theta, float cos_theta, mc_alpha_beta_t *alpha_beta);
void MC_Math_LimitMagnitude(float *x, float *y, float max_magnitude);

#endif
