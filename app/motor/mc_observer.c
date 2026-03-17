#include "mc_observer.h"
#include "mc_math.h"

static float MC_Observer_Sat(float value, float boundary)
{
    if (boundary <= 0.0f)
    {
        return 0.0f;
    }
    return MC_Math_Clamp(value / boundary, -1.0f, 1.0f);
}

void MC_Observer_Init(mc_observer_t *observer, const mc_user_config_t *config)
{
    observer->type = config->user.observer_type;
    observer->smo.rs_ohm = config->user.motor.rs_ohm;
    observer->smo.ls_h = config->derived.ls_avg_h;
    observer->smo.dt_s = config->derived.current_loop_dt_s;
    observer->smo.slide_gain = config->user.smo.slide_gain;
    observer->smo.slide_boundary_a = config->user.smo.slide_boundary_a;
    observer->smo.valid_bemf_v = config->user.smo.valid_bemf_v;
    MC_LPF_Init(&observer->smo.bemf_alpha_lpf, config->user.smo.emf_filter_hz, observer->smo.dt_s, 0.0f);
    MC_LPF_Init(&observer->smo.bemf_beta_lpf, config->user.smo.emf_filter_hz, observer->smo.dt_s, 0.0f);
    MC_LPF_Init(&observer->smo.speed_lpf, config->user.smo.speed_filter_hz, observer->smo.dt_s, 0.0f);
    MC_Observer_Reset(observer);
}

void MC_Observer_Reset(mc_observer_t *observer)
{
    observer->smo.current_hat.alpha = 0.0f;
    observer->smo.current_hat.beta = 0.0f;
    observer->smo.bemf.alpha = 0.0f;
    observer->smo.bemf.beta = 0.0f;
    observer->smo.theta_rad = 0.0f;
    observer->smo.valid = false;
    observer->smo.bemf_alpha_lpf.state = 0.0f;
    observer->smo.bemf_beta_lpf.state = 0.0f;
    observer->smo.speed_lpf.state = 0.0f;
    observer->output.theta_rad = 0.0f;
    observer->output.speed_rad_s = 0.0f;
    observer->output.bemf.alpha = 0.0f;
    observer->output.bemf.beta = 0.0f;
    observer->output.valid = false;
}

void MC_Observer_Run(mc_observer_t *observer,
                     const mc_alpha_beta_t *voltage_ab,
                     const mc_alpha_beta_t *current_ab,
                     float dt_s)
{
    float error_alpha;
    float error_beta;
    float z_alpha;
    float z_beta;
    float theta_next;
    float delta_theta;
    float speed_raw;
    float bemf_mag;
    mc_smo_observer_t *smo;

    if (observer->type != MC_OBSERVER_SMO)
    {
        observer->output.valid = false;
        return;
    }

    smo = &observer->smo;
    error_alpha = current_ab->alpha - smo->current_hat.alpha;
    error_beta = current_ab->beta - smo->current_hat.beta;
    z_alpha = smo->slide_gain * MC_Observer_Sat(error_alpha, smo->slide_boundary_a);
    z_beta = smo->slide_gain * MC_Observer_Sat(error_beta, smo->slide_boundary_a);

    if (smo->ls_h > 0.0f)
    {
        smo->current_hat.alpha += dt_s * ((voltage_ab->alpha - (smo->rs_ohm * smo->current_hat.alpha) - smo->bemf.alpha + z_alpha) / smo->ls_h);
        smo->current_hat.beta += dt_s * ((voltage_ab->beta - (smo->rs_ohm * smo->current_hat.beta) - smo->bemf.beta + z_beta) / smo->ls_h);
    }

    smo->bemf.alpha = MC_LPF_Run(&smo->bemf_alpha_lpf, z_alpha);
    smo->bemf.beta = MC_LPF_Run(&smo->bemf_beta_lpf, z_beta);

    theta_next = MC_Math_FastAtan2(-smo->bemf.alpha, smo->bemf.beta);
    delta_theta = MC_Math_WrapDelta(theta_next - smo->theta_rad);
    speed_raw = delta_theta / dt_s;
    smo->theta_rad = theta_next;
    bemf_mag = MC_Math_FastSqrt((smo->bemf.alpha * smo->bemf.alpha) + (smo->bemf.beta * smo->bemf.beta));
    smo->valid = (bemf_mag >= smo->valid_bemf_v);

    observer->output.theta_rad = smo->theta_rad;
    observer->output.speed_rad_s = MC_LPF_Run(&smo->speed_lpf, speed_raw);
    observer->output.bemf = smo->bemf;
    observer->output.valid = smo->valid;
}
