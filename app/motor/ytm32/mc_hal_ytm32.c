#include "motor/ytm32/mc_hal_ytm32.h"

#include "adc_driver.h"
#include "clock.h"
#include "etmr_common.h"
#include "etmr_hw_access.h"
#include "interrupt_manager.h"
#include "osif.h"
#include "pins_driver.h"
#include "sdk_project_config.h"
#include "system_YTM32B1ME0.h"
#include "tmu_driver.h"

#define MC_HAL_PWM_A_HIGH_CH 0U
#define MC_HAL_PWM_A_LOW_CH 1U
#define MC_HAL_PWM_B_HIGH_CH 4U
#define MC_HAL_PWM_B_LOW_CH 5U
#define MC_HAL_PWM_C_HIGH_CH 6U
#define MC_HAL_PWM_C_LOW_CH 7U
#define MC_HAL_ADC_TRIGGER_CH MC_HAL_PWM_A_LOW_CH
#define MC_HAL_TMU_INSTANCE 0U

#define MC_HAL_LED1_PIN 3U
#define MC_HAL_LED2_PIN 2U
#define MC_HAL_START_STOP_PIN 11U
#define MC_HAL_DIRECTION_PIN 3U
#define MC_HAL_HALL1_PIN 17U
#define MC_HAL_HALL2_PIN 16U
#define MC_HAL_HALL3_PIN 3U

#define MC_HAL_ADC_CH_IA ADC_INPUTCHAN_EXT2
#define MC_HAL_ADC_CH_IB ADC_INPUTCHAN_EXT3
#define MC_HAL_ADC_CH_VBUS ADC_INPUTCHAN_EXT5
#define MC_HAL_ADC_CH_TEMP ADC_INPUTCHAN_EXT6

static mc_fast_loop_callback_t s_fast_loop_callback;
static mc_fault_callback_t s_fault_callback;
static uint32_t s_pwm_period_ticks;
static uint32_t s_etmr_instance;
static uint32_t s_adc_instance;
static eTMR_Type *s_etmr_base;
static etmr_state_t s_etmr_state;
static etmr_trig_ch_param_t s_adc_trigger_channel_config[1] =
{
    {
        .channelId = MC_HAL_ADC_TRIGGER_CH,
        .channelVal0MatchTrigEn = true,
        .channelVal1MatchTrigEn = false
    }
};

static void MC_HAL_YTM32_SetAdcTriggerPoint(void)
{

    eTMR_SetChnVal0(s_etmr_base, MC_HAL_ADC_TRIGGER_CH, 0);
    eTMR_SetChnVal1(s_etmr_base, MC_HAL_ADC_TRIGGER_CH, 0);
}

static void MC_HAL_YTM32_WritePwmChannel(uint8_t pwm_channel, float duty_cycle)
{
    uint32_t half_period;
    uint32_t pulse_half_ticks;
    uint32_t center_ticks;
    uint32_t val0;
    uint32_t val1;

    half_period = s_pwm_period_ticks / 2U;
    center_ticks = half_period;
    pulse_half_ticks = (uint32_t)(duty_cycle * (float)half_period);
    if (pulse_half_ticks > half_period)
    {
        pulse_half_ticks = half_period;
    }

    if (center_ticks >= pulse_half_ticks)
    {
        val0 = center_ticks - pulse_half_ticks;
    }
    else
    {
        val0 = 0U;
    }
    val1 = center_ticks + pulse_half_ticks;
    if (val1 > s_pwm_period_ticks)
    {
        val1 = s_pwm_period_ticks;
    }

    eTMR_SetChnVal0(s_etmr_base, pwm_channel, val0);
    eTMR_SetChnVal1(s_etmr_base, pwm_channel, val1);
}

static status_t MC_HAL_YTM32_InitTmu(void)
{
    return TMU_DRV_Init(MC_HAL_TMU_INSTANCE, &tmu_config0);
}

static status_t MC_HAL_YTM32_InitPwm(const mc_user_config_t *config)
{
    etmr_pwm_sync_t sync_method;
    etmr_trig_config_t trig_config;
    etmr_user_config_t etmr_config;
    uint32_t pwm_clk_hz;
    uint8_t channel;

    s_etmr_instance = config->user.hardware.etmr_instance;
    s_etmr_base = g_etmrBase[s_etmr_instance];

    sync_method.regSyncFreq = 1U;
    sync_method.regSyncSel = REG_SYNC_WITH_MOD;
    sync_method.cntInitSyncSel = CNT_SYNC_WITH_REG;
    sync_method.maskOutputSyncSel = CHMASK_SYNC_WITH_REG;
    sync_method.regSyncTrigSrc = DISABLE_TRIGGER;
    sync_method.cntInitSyncTrigSrc = DISABLE_TRIGGER;
    sync_method.maskOutputSyncTrigSrc = DISABLE_TRIGGER;
    sync_method.hwTrigFromTmuEnable = false;
    sync_method.hwTrigFromCimEnable = false;
    sync_method.hwTrigFromPadEnable = false;

    trig_config.trigSrc = TRIGGER_FROM_MATCHING_EVENT;
    trig_config.pwmOutputChannel = MC_HAL_PWM_A_HIGH_CH;
    trig_config.outputTrigWidth = 1U;
    trig_config.outputTrigFreq = 1U;
    trig_config.modMatchTrigEnable = false;
    trig_config.midMatchTrigEnable = false;
    trig_config.initMatchTrigEnable = false;
    trig_config.numOfChannels = 1U;
    trig_config.channelTrigParamConfig = s_adc_trigger_channel_config;

    etmr_config.etmrClockSource = eTMR_CLOCK_SOURCE_INTERNALCLK;
    etmr_config.etmrPrescaler = 1U;
    etmr_config.debugMode = true;
    etmr_config.syncMethod = &sync_method;
    etmr_config.outputTrigConfig = &trig_config;
    etmr_config.isTofIntEnabled = false;

    if (eTMR_DRV_Init(s_etmr_instance, &etmr_config, &s_etmr_state) != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    pwm_clk_hz = eTMR_DRV_GetFrequency(s_etmr_instance);
    if ((pwm_clk_hz == 0U) || (config->user.hardware.pwm_frequency_hz == 0U))
    {
        return STATUS_ERROR;
    }

    s_pwm_period_ticks = pwm_clk_hz / (config->user.hardware.pwm_frequency_hz);
    if (s_pwm_period_ticks == 0U)
    {
        s_pwm_period_ticks = 1U;
    }

    s_etmr_base->MOD = s_pwm_period_ticks;
    s_etmr_base->INIT = 0U;
#if FEATURE_eTMR_HAS_MID
    s_etmr_base->MID = s_pwm_period_ticks / 2U;
#endif
    s_etmr_base->CHFV = 0x0000U;

    for (channel = 0U; channel < FEATURE_eTMR_CHANNEL_MAX_COUNT; channel++)
    {
        eTMR_SetChnMode(s_etmr_base, channel, eTMR_CHANNEL_DISABLE);
        eTMR_SetChnPwmSrc(s_etmr_base, channel, 0U);
        eTMR_IsInvertChnOutputPol(s_etmr_base, channel, false);
    }

    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_A_HIGH_CH, eTMR_PWM_MODE);
    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_A_LOW_CH, eTMR_PWM_MODE);
    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_B_HIGH_CH, eTMR_PWM_MODE);
    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_B_LOW_CH, eTMR_PWM_MODE);
    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_C_HIGH_CH, eTMR_PWM_MODE);
    eTMR_SetChnMode(s_etmr_base, MC_HAL_PWM_C_LOW_CH, eTMR_PWM_MODE);

    eTMR_DRV_SetChnCompMode(s_etmr_instance, 0U, PWM_COMPLEMENTARY_MODE);
    eTMR_DRV_SetChnCompMode(s_etmr_instance, 2U, PWM_COMPLEMENTARY_MODE);
    eTMR_DRV_SetChnCompMode(s_etmr_instance, 3U, PWM_COMPLEMENTARY_MODE);

    eTMR_SetChnDeadtime(s_etmr_base, MC_HAL_PWM_A_HIGH_CH, config->derived.deadtime_ticks);
    eTMR_SetChnDeadtime(s_etmr_base, MC_HAL_PWM_B_HIGH_CH, config->derived.deadtime_ticks);
    eTMR_SetChnDeadtime(s_etmr_base, MC_HAL_PWM_C_HIGH_CH, config->derived.deadtime_ticks);

    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_A_HIGH_CH, 0.5f);
    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_B_HIGH_CH, 0.5f);
    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_C_HIGH_CH, 0.5f);
    MC_HAL_YTM32_SetAdcTriggerPoint();

    eTMR_SetLdok(s_etmr_base);
    eTMR_GenSoftwareTrigger(s_etmr_base, true);
    (void)eTMR_DRV_SetChnOutMask(s_etmr_instance, 0xFFU, 0x0000U, true);
    (void)eTMR_DRV_EnableInterrupts(s_etmr_instance, eTMR_FAULT_INT_ENABLE);
    INT_SYS_SetPriority(eTMR0_Fault_IRQn, 0U);
    INT_SYS_EnableIRQ(eTMR0_Fault_IRQn);
    eTMR_DRV_Enable(s_etmr_instance);
    return STATUS_SUCCESS;
}

static status_t MC_HAL_YTM32_InitAdc(const mc_user_config_t *config)
{
    adc_converter_config_t adc_config;
    /* FOR DEBUG */
    PINS_DRV_SetMuxModeSel(PCTRLC, 7, PCTRL_MUX_AS_GPIO);
    PINS_DRV_SetPinDirection(GPIOC, 7, GPIO_OUTPUT_DIRECTION);

    s_adc_instance = config->user.hardware.adc_instance;
    ADC_DRV_InitConverterStruct(&adc_config);
    adc_config.trigger = ADC_TRIGGER_HARDWARE;
    adc_config.clockDivider = 0U;
    adc_config.startTime = 40U;
    adc_config.sampleTime = 4U;
    adc_config.sequenceConfig.channels[0] = MC_HAL_ADC_CH_IA;
    adc_config.sequenceConfig.channels[1] = MC_HAL_ADC_CH_IB;
    adc_config.sequenceConfig.channels[2] = MC_HAL_ADC_CH_VBUS;
    adc_config.sequenceConfig.channels[3] = MC_HAL_ADC_CH_TEMP;
    adc_config.sequenceConfig.totalChannels = 4U;
    adc_config.sequenceConfig.sequenceMode = ADC_CONV_LOOP;
    adc_config.sequenceConfig.sequenceIntEnable = true;
    adc_config.sequenceConfig.convIntEnable = false;
    adc_config.sequenceConfig.ovrunIntEnable = true;
    /* Enable temp sensor */
    ADC_DRV_ConfigConverter(s_adc_instance, &adc_config);
    ADC_DRV_ClearEoseqFlagCmd(s_adc_instance);
    ADC_DRV_ClearOvrFlagCmd(s_adc_instance);
    INT_SYS_SetPriority(ADC_DRV_GetInterruptNumber(s_adc_instance), 1U);
    INT_SYS_EnableIRQ(ADC_DRV_GetInterruptNumber(s_adc_instance));
    ADC0->CTRL |= ADC_CTRL_TSEN(1);
    ADC_DRV_Start(s_adc_instance);
    return STATUS_SUCCESS;
}

static status_t MC_HAL_YTM32_Init(const mc_user_config_t *config)
{
    SystemCoreClockUpdate();
    s_fast_loop_callback = 0;
    s_fault_callback = 0;
    CIM->CTRL |= CIM_CTRL_ADC0_TRIG_SEL(1);
    if (MC_HAL_YTM32_InitAdc(config) != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    if (MC_HAL_YTM32_InitPwm(config) != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }

    if (MC_HAL_YTM32_InitTmu() != STATUS_SUCCESS)
    {
        return STATUS_ERROR;
    }


    OSIF_TimeDelay(0U);
    return STATUS_SUCCESS;
}

static void MC_HAL_YTM32_BindCallbacks(mc_fast_loop_callback_t fast_loop_cb, mc_fault_callback_t fault_cb)
{
    s_fast_loop_callback = fast_loop_cb;
    s_fault_callback = fault_cb;
}

static void MC_HAL_YTM32_EnableOutputs(bool enable)
{
    if (enable)
    {
        (void)eTMR_DRV_SetChnOutMask(s_etmr_instance, 0x00U, 0x0000U, true);
    }
    else
    {
        (void)eTMR_DRV_SetChnOutMask(s_etmr_instance, 0xFFU, 0x0000U, true);
    }
}

static void MC_HAL_YTM32_ApplyPwm(const mc_duty_cycle_t *duty_cycle)
{
    eTMR_ClearLdok(s_etmr_base);
    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_A_HIGH_CH, duty_cycle->duty_a);
    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_B_HIGH_CH, duty_cycle->duty_b);
    MC_HAL_YTM32_WritePwmChannel(MC_HAL_PWM_C_HIGH_CH, duty_cycle->duty_c);
    MC_HAL_YTM32_SetAdcTriggerPoint();
    eTMR_SetLdok(s_etmr_base);
}

static void MC_HAL_YTM32_PollInputs(mc_input_request_t *request)
{
    uint8_t hall_state;

    request->run_request = (PINS_DRV_ReadPin(GPIOE, MC_HAL_START_STOP_PIN) == 0U);
    request->direction = (PINS_DRV_ReadPin(GPIOE, MC_HAL_DIRECTION_PIN) == 0U) ? -1 : 1;

    hall_state = (uint8_t)(PINS_DRV_ReadPin(GPIOC, MC_HAL_HALL1_PIN) << 2U);
    hall_state |= (uint8_t)(PINS_DRV_ReadPin(GPIOC, MC_HAL_HALL2_PIN) << 1U);
    hall_state |= (uint8_t)PINS_DRV_ReadPin(GPIOB, MC_HAL_HALL3_PIN);
    request->hall_state = hall_state;
}

static void MC_HAL_YTM32_SetStatus(mc_state_t state, uint32_t fault_mask)
{
    bool led1_on;
    bool led2_on;

    led1_on = (state == MC_STATE_RUN) || (state == MC_STATE_STARTUP);
    led2_on = (fault_mask != MC_FAULT_NONE);

    PINS_DRV_WritePin(GPIOD, MC_HAL_LED1_PIN, led1_on ? 1U : 0U);
    PINS_DRV_WritePin(GPIOD, MC_HAL_LED2_PIN, led2_on ? 1U : 0U);
}

static void MC_HAL_YTM32_ClearFaultLatch(void)
{
    uint8_t fault_channel;

    for (fault_channel = 0U; fault_channel < FEATURE_eTMR_FAULT_CHANNELS; fault_channel++)
    {
        eTMR_DRV_ClearFaultFlag(s_etmr_instance, fault_channel);
    }
}

static uint32_t MC_HAL_YTM32_GetTickMs(void)
{
    return OSIF_GetMilliseconds();
}

static uint32_t MC_HAL_YTM32_GetPwmPeriodTicks(void)
{
    return s_pwm_period_ticks;
}

const mc_hal_ops_t g_mc_hal_ytm32_ops =
{
    MC_HAL_YTM32_Init,
    MC_HAL_YTM32_BindCallbacks,
    MC_HAL_YTM32_EnableOutputs,
    MC_HAL_YTM32_ApplyPwm,
    MC_HAL_YTM32_PollInputs,
    MC_HAL_YTM32_SetStatus,
    MC_HAL_YTM32_ClearFaultLatch,
    MC_HAL_YTM32_GetTickMs,
    MC_HAL_YTM32_GetPwmPeriodTicks
};

void ADC0_IRQHandler(void)
{
    mc_adc_sample_t sample;
    /* FOR DEBUG */
    PINS_DRV_WritePin(GPIOC, 7, 1U);

    if (ADC_DRV_GetOvrRunOfConversionFlag(s_adc_instance))
    {
        ADC_DRV_ClearOvrFlagCmd(s_adc_instance);
        if (s_fault_callback != 0)
        {
            s_fault_callback(MC_FAULT_ADC_OVERRUN);
        }
    }

    if (!ADC_DRV_GetEndOfSequenceFlag(s_adc_instance))
    {
        return;
    }

    sample.current_a_raw = ADC_DRV_ReadFIFO(s_adc_instance);
    sample.current_b_raw = ADC_DRV_ReadFIFO(s_adc_instance);
    sample.vbus_raw = ADC_DRV_ReadFIFO(s_adc_instance);
    sample.temp_raw = ADC_DRV_ReadFIFO(s_adc_instance);
    ADC_DRV_ClearEoseqFlagCmd(s_adc_instance);
    ADC_DRV_ClearEocFlagCmd(s_adc_instance);

    if (s_fast_loop_callback != 0)
    {
        s_fast_loop_callback(&sample);
    }
    /* FOR DEBUG */
    PINS_DRV_WritePin(GPIOC, 7, 0U);
}

void eTMR0_Fault_IRQHandler(void)
{
    uint8_t fault_channel;

    MC_HAL_YTM32_EnableOutputs(false);
    for (fault_channel = 0U; fault_channel < FEATURE_eTMR_FAULT_CHANNELS; fault_channel++)
    {
        eTMR_DRV_ClearFaultFlag(s_etmr_instance, fault_channel);
    }
    if (s_fault_callback != 0)
    {
        s_fault_callback(MC_FAULT_HW_TRIP);
    }
}
