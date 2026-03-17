#ifndef MC_HAL_H
#define MC_HAL_H

#include "mc_common.h"

typedef void (*mc_fast_loop_callback_t)(const mc_adc_sample_t *sample);
typedef void (*mc_fault_callback_t)(uint32_t fault_mask);

typedef struct
{
    status_t (*init)(const mc_user_config_t *config);
    void (*bind_callbacks)(mc_fast_loop_callback_t fast_loop_cb, mc_fault_callback_t fault_cb);
    void (*enable_outputs)(bool enable);
    void (*apply_pwm)(const mc_duty_cycle_t *duty_cycle);
    void (*poll_inputs)(mc_input_request_t *request);
    void (*set_status)(mc_state_t state, uint32_t fault_mask);
    void (*clear_fault_latch)(void);
    uint32_t (*get_tick_ms)(void);
    uint32_t (*get_pwm_period_ticks)(void);
} mc_hal_ops_t;

#endif
