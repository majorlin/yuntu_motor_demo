#ifndef MOTOR_FOC_H
#define MOTOR_FOC_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTOR_OBSERVER_BACKEND_ORTEGA = 0,
    MOTOR_OBSERVER_BACKEND_MXLEMMING,
    MOTOR_OBSERVER_BACKEND_MXV
} motor_observer_backend_t;

typedef struct
{
    float alpha;
    float beta;
} motor_ab_frame_t;

typedef struct
{
    float d;
    float q;
} motor_dq_frame_t;

typedef struct
{
    float phase_current_a;
    float phase_current_b;
    float phase_current_c;
    float bus_voltage_v;
    float control_angle_rad;
    float id_target_a;
    float iq_target_a;
    bool deadtime_comp_enable;
} motor_foc_fast_input_t;

typedef struct
{
    float duty_u;
    float duty_v;
    float duty_w;
    float id_a;
    float iq_a;
    float observer_angle_rad;
    float observer_speed_rad_s;
    float phase_error_rad;
    float bus_voltage_used_v;
    motor_ab_frame_t commanded_vab_v;
} motor_foc_fast_output_t;

typedef struct
{
    motor_observer_backend_t backend;
    float current_pi_d_integrator_v;
    float current_pi_q_integrator_v;
    float speed_pi_integrator_a;
    float pll_phase_rad;
    float pll_speed_rad_s;
    float observer_x1;
    float observer_x2;
    float observer_lambda_vs;
    float phase_error_rad;
    float last_i_alpha_a;
    float last_i_beta_a;
    float last_v_alpha_v;
    float last_v_beta_v;
    int8_t deadtime_comp_sign_a;
    int8_t deadtime_comp_sign_b;
    int8_t deadtime_comp_sign_c;
} motor_foc_state_t;

void MotorFoc_Init(motor_foc_state_t *state);
void MotorFoc_Reset(motor_foc_state_t *state);
void MotorFoc_RunFast(motor_foc_state_t *state,
                      const motor_foc_fast_input_t *input,
                      motor_foc_fast_output_t *output);
float MotorFoc_RunSpeedPi(motor_foc_state_t *state,
                          float targetElectricalSpeedRadS,
                          float measuredElectricalSpeedRadS,
                          bool resetIntegrator);
float MotorFoc_WrapAngle0ToTwoPi(float angleRad);
float MotorFoc_AngleDiff(float targetAngleRad, float measuredAngleRad);

#endif /* MOTOR_FOC_H */
