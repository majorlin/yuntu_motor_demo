#include "mc_math.h"

float MC_Math_Abs(float x)
{
    return (x >= 0.0f) ? x : -x;
}

float MC_Math_Clamp(float x, float min_value, float max_value)
{
    if (x < min_value)
    {
        return min_value;
    }
    if (x > max_value)
    {
        return max_value;
    }
    return x;
}

float MC_Math_WrapAngle(float angle_rad)
{
    while (angle_rad >= MC_PI_F)
    {
        angle_rad -= MC_TWO_PI_F;
    }
    while (angle_rad < -MC_PI_F)
    {
        angle_rad += MC_TWO_PI_F;
    }
    return angle_rad;
}

float MC_Math_WrapDelta(float delta_rad)
{
    return MC_Math_WrapAngle(delta_rad);
}

float MC_Math_FastSqrt(float x)
{
    float guess;
    uint8_t i;

    if (x <= 0.0f)
    {
        return 0.0f;
    }

    guess = (x > 1.0f) ? x : 1.0f;
    for (i = 0U; i < 4U; i++)
    {
        guess = 0.5f * (guess + (x / guess));
    }
    return guess;
}

void MC_Math_FastSinCos(float angle_rad, float *sin_value, float *cos_value)
{
    const float b = 1.27323954474f;
    const float c = -0.40528473457f;
    const float p = 0.225f;
    float x;
    float y;
    float sin_x;
    float cos_x;

    x = MC_Math_WrapAngle(angle_rad);
    y = (b * x) + (c * x * MC_Math_Abs(x));
    sin_x = p * ((y * MC_Math_Abs(y)) - y) + y;

    x = MC_Math_WrapAngle(angle_rad + MC_HALF_PI_F);
    y = (b * x) + (c * x * MC_Math_Abs(x));
    cos_x = p * ((y * MC_Math_Abs(y)) - y) + y;

    if (sin_value != 0)
    {
        *sin_value = sin_x;
    }
    if (cos_value != 0)
    {
        *cos_value = cos_x;
    }
}

float MC_Math_FastAtan2(float y, float x)
{
    const float coeff_1 = MC_PI_F / 4.0f;
    const float coeff_2 = 3.0f * coeff_1;
    const float tiny = 1.0e-12f;
    float abs_y;
    float angle;
    float r;

    if ((x == 0.0f) && (y == 0.0f))
    {
        return 0.0f;
    }

    abs_y = MC_Math_Abs(y) + tiny;
    if (x >= 0.0f)
    {
        r = (x - abs_y) / (x + abs_y);
        angle = coeff_1 - (coeff_1 * r);
    }
    else
    {
        r = (x + abs_y) / (abs_y - x);
        angle = coeff_2 - (coeff_1 * r);
    }

    return (y < 0.0f) ? -angle : angle;
}

void MC_Math_Clarke(const mc_abc_t *abc, mc_alpha_beta_t *alpha_beta)
{
    alpha_beta->alpha = abc->a;
    alpha_beta->beta = (abc->a + (2.0f * abc->b)) * MC_ONE_BY_SQRT3_F;
}

void MC_Math_Park(const mc_alpha_beta_t *alpha_beta, float sin_theta, float cos_theta, mc_dq_t *dq)
{
    dq->d = (alpha_beta->alpha * cos_theta) + (alpha_beta->beta * sin_theta);
    dq->q = (-alpha_beta->alpha * sin_theta) + (alpha_beta->beta * cos_theta);
}

void MC_Math_InvPark(const mc_dq_t *dq, float sin_theta, float cos_theta, mc_alpha_beta_t *alpha_beta)
{
    alpha_beta->alpha = (dq->d * cos_theta) - (dq->q * sin_theta);
    alpha_beta->beta = (dq->d * sin_theta) + (dq->q * cos_theta);
}

void MC_Math_LimitMagnitude(float *x, float *y, float max_magnitude)
{
    float magnitude;
    float scale;

    magnitude = MC_Math_FastSqrt((*x * *x) + (*y * *y));
    if ((magnitude > max_magnitude) && (magnitude > 0.0f))
    {
        scale = max_magnitude / magnitude;
        *x *= scale;
        *y *= scale;
    }
}
