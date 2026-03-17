#include "motor/motor_app.h"

#include "motor/mc_motor.h"
#include "motor/mc_user_params.h"
#include "motor/ytm32/mc_hal_ytm32.h"

static mc_motor_t s_motor;
static mc_user_config_t s_motor_config;

static void MotorApp_FastLoopCallback(const mc_adc_sample_t *sample)
{
    MC_Motor_FastLoop(&s_motor, sample);
}

static void MotorApp_FaultCallback(uint32_t fault_mask)
{
    MC_Motor_ReportFault(&s_motor, fault_mask);
}

void MotorApp_Init(void)
{
    if (MC_UserParams_Load(&s_motor_config) != STATUS_SUCCESS)
    {
        while (1)
        {
        }
    }
    MC_Motor_Init(&s_motor, &g_mc_hal_ytm32_ops, &s_motor_config);
    g_mc_hal_ytm32_ops.bind_callbacks(MotorApp_FastLoopCallback, MotorApp_FaultCallback);
}

void MotorApp_Run(void)
{
    MC_Motor_Background(&s_motor);
}
