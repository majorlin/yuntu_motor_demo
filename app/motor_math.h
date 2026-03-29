/**
 * @file motor_math.h
 * @brief Shared inline math utilities for the FOC motor control stack.
 *
 * Provides lightweight, ISR-safe mathematical helper functions used across
 * the FOC engine, state machine, and parameter identification modules.
 * All functions are declared @c static @c inline for zero call overhead.
 *
 * @defgroup motor_math  Math Utilities
 * @{
 */

#ifndef MOTOR_MATH_H
#define MOTOR_MATH_H

#include <stdbool.h>
#include <stdint.h>

#include "motor_user_config.h"

/* ────────────────────────────────────────────────────────────────────────── */
/*  Constants (re-exported from motor_user_config.h for convenience)         */
/* ────────────────────────────────────────────────────────────────────────── */

/** @brief Small epsilon to avoid division-by-zero in float comparisons. */
#define MOTOR_MATH_EPSILON_F  (1.0e-6f)

/* ────────────────────────────────────────────────────────────────────────── */
/*  Scalar helpers                                                           */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Clamp a float value between a minimum and maximum.
 *
 * @param[in] value     Input value.
 * @param[in] minVal    Lower bound.
 * @param[in] maxVal    Upper bound.
 * @return Clamped value in [minVal, maxVal].
 */
static inline float MotorMath_Clamp(float value, float minVal, float maxVal)
{
    float result = value;

    if (result < minVal)
    {
        result = minVal;
    }
    else if (result > maxVal)
    {
        result = maxVal;
    }

    return result;
}

/**
 * @brief Fast floating-point absolute value.
 *
 * @param[in] value  Input value.
 * @return Absolute value of @p value.
 */
static inline float MotorMath_Abs(float value)
{
    return __builtin_fabsf(value);
}

/**
 * @brief Non-negative square root with negative-input clamping.
 *
 * @param[in] value  Input value (negative values are clamped to zero).
 * @return Square root of @p value, or 0 if @p value < 0.
 */
static inline float MotorMath_Sqrt(float value)
{
    float safe = value;

    if (safe < 0.0f)
    {
        safe = 0.0f;
    }

    return __builtin_sqrtf(safe);
}

/**
 * @brief Maximum of three float values.
 *
 * @param[in] a  First value.
 * @param[in] b  Second value.
 * @param[in] c  Third value.
 * @return Maximum of a, b, and c.
 */
static inline float MotorMath_Max3(float a, float b, float c)
{
    float result = a;

    if (b > result)
    {
        result = b;
    }
    if (c > result)
    {
        result = c;
    }

    return result;
}

/**
 * @brief Minimum of three float values.
 *
 * @param[in] a  First value.
 * @param[in] b  Second value.
 * @param[in] c  Third value.
 * @return Minimum of a, b, and c.
 */
static inline float MotorMath_Min3(float a, float b, float c)
{
    float result = a;

    if (b < result)
    {
        result = b;
    }
    if (c < result)
    {
        result = c;
    }

    return result;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Angle utilities                                                          */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Wrap an angle to the range [0, 2*pi).
 *
 * Assumes the input is within one period of the target range (i.e. the
 * deviation from [0, 2*pi) is at most one full revolution).
 *
 * @param[in] angleRad  Input angle in radians.
 * @return Wrapped angle in [0, 2*pi).
 */
static inline float MotorMath_WrapAngle0To2Pi(float angleRad)
{
    float w = angleRad;

    if (w >= MOTOR_CFG_TWO_PI_F)
    {
        w -= MOTOR_CFG_TWO_PI_F;
    }
    else if (w < 0.0f)
    {
        w += MOTOR_CFG_TWO_PI_F;
    }

    return w;
}

/**
 * @brief Compute the shortest signed angle difference (target - measured).
 *
 * Both inputs should already be approximately wrapped. The result lies in
 * the range (-pi, pi].
 *
 * @param[in] targetRad    Target angle in radians.
 * @param[in] measuredRad  Measured angle in radians.
 * @return Signed angle difference in radians.
 */
static inline float MotorMath_AngleDiff(float targetRad, float measuredRad)
{
    float diff = targetRad - measuredRad;

    if (diff > MOTOR_CFG_PI_F)
    {
        diff -= MOTOR_CFG_TWO_PI_F;
    }
    else if (diff < -MOTOR_CFG_PI_F)
    {
        diff += MOTOR_CFG_TWO_PI_F;
    }

    return diff;
}

/**
 * @brief Wrap an angle to the range (-pi, pi].
 *
 * @param[in] angleRad  Input angle in radians.
 * @return Wrapped angle in (-pi, pi].
 */
static inline float MotorMath_WrapAngleSigned(float angleRad)
{
    float w = MotorMath_WrapAngle0To2Pi(angleRad);

    if (w > MOTOR_CFG_PI_F)
    {
        w -= MOTOR_CFG_TWO_PI_F;
    }

    return w;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Trigonometric approximations                                             */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Fast sine/cosine approximation (Bhaskara-style parabolic).
 *
 * Maximum error ~0.001. The input angle is first wrapped to [0, 2*pi).
 *
 * @param[in]  angleRad  Angle in radians.
 * @param[out] sinOut    Pointer to store the sine result.
 * @param[out] cosOut    Pointer to store the cosine result.
 */
static inline void MotorMath_SinCos(float angleRad, float *sinOut, float *cosOut)
{
    const float b = 4.0f / MOTOR_CFG_PI_F;
    const float c = -4.0f / (MOTOR_CFG_PI_F * MOTOR_CFG_PI_F);
    const float p = 0.225f;
    float wrapped = MotorMath_WrapAngle0To2Pi(angleRad);
    float sinIn = wrapped;
    float cosIn = wrapped + MOTOR_CFG_HALF_PI_F;
    float sinY;
    float cosY;

    if (sinIn > MOTOR_CFG_PI_F)
    {
        sinIn -= MOTOR_CFG_TWO_PI_F;
    }

    if (cosIn >= MOTOR_CFG_TWO_PI_F)
    {
        cosIn -= MOTOR_CFG_TWO_PI_F;
    }
    if (cosIn > MOTOR_CFG_PI_F)
    {
        cosIn -= MOTOR_CFG_TWO_PI_F;
    }

    sinY = (b * sinIn) + (c * sinIn * MotorMath_Abs(sinIn));
    sinY = (p * ((sinY * MotorMath_Abs(sinY)) - sinY)) + sinY;
    cosY = (b * cosIn) + (c * cosIn * MotorMath_Abs(cosIn));
    cosY = (p * ((cosY * MotorMath_Abs(cosY)) - cosY)) + cosY;

    *sinOut = sinY;
    *cosOut = cosY;
}

/**
 * @brief Fast two-argument arctangent approximation.
 *
 * Uses a first-order rational approximation. Maximum error ~0.07 rad.
 *
 * @param[in] y  Y coordinate (beta-axis component).
 * @param[in] x  X coordinate (alpha-axis component).
 * @return Angle in radians [-pi, pi].
 */
static inline float MotorMath_Atan2(float y, float x)
{
    const float qtrPi   = 0.78539816339744830962f;
    const float tqtrPi  = 2.35619449019234492885f;
    const float absY    = MotorMath_Abs(y) + MOTOR_MATH_EPSILON_F;
    float angle;
    float ratio;

    if (x >= 0.0f)
    {
        ratio = (x - absY) / (x + absY);
        angle = qtrPi - (qtrPi * ratio);
    }
    else
    {
        ratio = (x + absY) / (absY - x);
        angle = tqtrPi - (qtrPi * ratio);
    }

    if (y < 0.0f)
    {
        angle = -angle;
    }

    return angle;
}

/* ────────────────────────────────────────────────────────────────────────── */
/*  Slew-rate limiter                                                        */
/* ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Step a value toward a target with a bounded slew rate.
 *
 * @param[in] current  Current value.
 * @param[in] target   Desired target value.
 * @param[in] step     Maximum step magnitude per call (must be >= 0).
 * @return Updated value after one step toward @p target.
 */
static inline float MotorMath_SlewTowards(float current, float target, float step)
{
    float result = current;

    if (result < target)
    {
        result += step;
        if (result > target)
        {
            result = target;
        }
    }
    else if (result > target)
    {
        result -= step;
        if (result < target)
        {
            result = target;
        }
    }

    return result;
}

/** @} */ /* end of motor_math group */

#endif /* MOTOR_MATH_H */
