#include "motor_hw_ytm32.h"

#include "sdk_project_config.h"
#include "motor_user_config.h"
#include "etmr_hw_access.h"

#define MOTOR_HW_ADC_INSTANCE                    (0U)
#define MOTOR_HW_TMU_INSTANCE                    (0U)
#define MOTOR_HW_ETMR_INSTANCE                   (0U)
#define MOTOR_HW_PTMR_INSTANCE                   (0U)
#define MOTOR_HW_PTMR_CHANNEL                    (0U)

#define MOTOR_HW_ADC_CURRENT_A_CH                (ADC_INPUTCHAN_EXT2)
#define MOTOR_HW_ADC_CURRENT_B_CH                (ADC_INPUTCHAN_EXT3)
#define MOTOR_HW_ADC_CURRENT_C_CH                (ADC_INPUTCHAN_EXT4)
#define MOTOR_HW_ADC_VBUS_CH                     (ADC_INPUTCHAN_EXT5)

#define MOTOR_HW_PWM_U_HIGH_CH                   (0U)
#define MOTOR_HW_PWM_U_LOW_CH                    (1U)
#define MOTOR_HW_PWM_V_HIGH_CH                   (4U)
#define MOTOR_HW_PWM_V_LOW_CH                    (5U)
#define MOTOR_HW_PWM_W_HIGH_CH                   (6U)
#define MOTOR_HW_PWM_W_LOW_CH                    (7U)

#define MOTOR_HW_ADC_IRQ_PRIORITY                (1U)
#define MOTOR_HW_SPEED_IRQ_PRIORITY              (2U)
#define MOTOR_HW_SOFTWARE_ADC_TIMEOUT            (200000U)

#if ((MOTOR_CFG_ETMR_CLOCK_HZ / MOTOR_CFG_PWM_FREQUENCY_HZ) < 100UL)
#error "PWM period is too small for reliable center-aligned updates."
#endif

#if (((MOTOR_CFG_ETMR_CLOCK_HZ / MOTOR_CFG_PWM_FREQUENCY_HZ) / 2UL) >= \
     (MOTOR_CFG_ETMR_CLOCK_HZ / MOTOR_CFG_PWM_FREQUENCY_HZ))
#error "PWM MID tick must stay strictly inside the PWM period."
#endif

#if (((1ULL * MOTOR_CFG_ETMR_CLOCK_HZ * MOTOR_CFG_DEADTIME_NS) / 1000000000ULL) == 0ULL)
#error "Deadtime must resolve to at least one eTMR tick."
#endif

#if (MOTOR_CFG_USED_PWM_CHANNEL_MASK != 0xF3U)
#error "This implementation assumes channel mask 0xF3 for CH0/1/4/5/6/7."
#endif

static etmr_state_t s_motorEtmrState;
static etmr_pwm_sync_t s_motorEtmrSync;
static etmr_trig_config_t s_motorEtmrTrig;
static etmr_trig_ch_param_t s_motorEtmrTrigChannel[1];
static bool s_adcConfiguredForHardwareTrigger = true;

static void MotorHwYtm32_SelectAdc0ExternalTriggerFromTmu(void)
{
    CIM->CTRL = (CIM->CTRL & ~CIM_CTRL_ADC0_TRIG_SEL_MASK) | CIM_CTRL_ADC0_TRIG_SEL(1U);
}

static void MotorHwYtm32_ConfigClocks(void)
{
    CLOCK_DRV_SetModuleClock(ADC0_CLK, true, CLK_SRC_FIRC, DIV_BY_3);
    CLOCK_DRV_SetModuleClock(TMU_CLK, true, CLK_SRC_DISABLED, DIV_BY_1);
    CLOCK_DRV_SetModuleClock(eTMR0_CLK, true, CLK_SRC_DISABLED, DIV_BY_1);
    CLOCK_DRV_SetModuleClock(pTMR0_CLK, true, CLK_SRC_DISABLED, DIV_BY_1);
}

static void MotorHwYtm32_SetAdcTriggerPoint(eTMR_Type *const etmrBase)
{
    eTMR_SetChnVal0(etmrBase, MOTOR_HW_PWM_U_LOW_CH, 0U);
    eTMR_SetChnVal1(etmrBase, MOTOR_HW_PWM_U_LOW_CH, 0U);
}

static void MotorHwYtm32_WriteHighSidePwm(eTMR_Type *const etmrBase, uint8_t channel, float duty)
{
    const uint32_t halfPeriodTicks = MOTOR_CFG_PWM_HALF_PERIOD_TICKS;
    const float clampedDuty = (duty < 0.0f) ? 0.0f : ((duty > 1.0f) ? 1.0f : duty);
    const uint32_t edgeDelta = (uint32_t)(clampedDuty * (float)halfPeriodTicks);

    eTMR_SetChnVal0(etmrBase, channel, MOTOR_CFG_PWM_MID_TICKS - edgeDelta);
    eTMR_SetChnVal1(etmrBase, channel, MOTOR_CFG_PWM_MID_TICKS + edgeDelta);
}

static void MotorHwYtm32_SetNeutralPwm(eTMR_Type *const etmrBase)
{
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_U_HIGH_CH, 0.5f);
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_V_HIGH_CH, 0.5f);
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_W_HIGH_CH, 0.5f);
    MotorHwYtm32_SetAdcTriggerPoint(etmrBase);
}


static void MotorHwYtm32_ConfigAdc(bool hardwareTrigger, bool sequenceInterrupt, bool overrunInterrupt)
{
    adc_converter_config_t adcConfig;

    ADC_DRV_Disable(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_InitConverterStruct(&adcConfig);

    adcConfig.trigger = hardwareTrigger ? ADC_TRIGGER_HARDWARE : ADC_TRIGGER_SOFTWARE;
    adcConfig.clockDivider = 0U;
    adcConfig.sampleTime = 4U;
    adcConfig.sequenceConfig.channels[0] = MOTOR_HW_ADC_CURRENT_A_CH;
    adcConfig.sequenceConfig.channels[1] = MOTOR_HW_ADC_CURRENT_B_CH;
    adcConfig.sequenceConfig.channels[2] = MOTOR_HW_ADC_CURRENT_C_CH;
    adcConfig.sequenceConfig.channels[3] = MOTOR_HW_ADC_VBUS_CH;
    adcConfig.sequenceConfig.totalChannels = 4U;
    adcConfig.sequenceConfig.sequenceMode = ADC_CONV_LOOP;
    adcConfig.sequenceConfig.sequenceIntEnable = sequenceInterrupt;
    adcConfig.sequenceConfig.convIntEnable = false;
    adcConfig.sequenceConfig.ovrunIntEnable = overrunInterrupt;
    adcConfig.sequenceConfig.sampIntEnable = false;
    adcConfig.sequenceConfig.readyIntEnable = false;

    ADC_DRV_ConfigConverter(MOTOR_HW_ADC_INSTANCE, &adcConfig);
    ADC_DRV_ClearEocFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_ClearReadyFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_ClearSampEndFlagCmd(MOTOR_HW_ADC_INSTANCE);
    s_adcConfiguredForHardwareTrigger = hardwareTrigger;
}

static void MotorHwYtm32_InitTmu(void)
{
    const tmu_inout_mapping_config_t mapping = {
        .triggerSource = TMU_TRIG_SOURCE_eTMR0_EXT_TRIG,
        .targetModule = TMU_TARGET_MODULE_ADC0_EXT_TRIG
    };
    const tmu_user_config_t config = {
        .numInOutMappingConfigs = 1U,
        .inOutMappingConfig = &mapping
    };

    (void)TMU_DRV_Init(MOTOR_HW_TMU_INSTANCE, &config);
}

static void MotorHwYtm32_CommitShadowNow(void)
{
    eTMR_Type *const etmrBase = g_etmrBase[MOTOR_HW_ETMR_INSTANCE];

    eTMR_SetRegLoadOpportunity(etmrBase, REG_SYNC_WITH_TRIG);
    eTMR_SetRegLoadTrigSrc(etmrBase, SW_TRIGGER);
    eTMR_SetLdok(etmrBase);
    eTMR_GenSoftwareTrigger(etmrBase, true);
    eTMR_SetRegLoadOpportunity(etmrBase, REG_SYNC_WITH_MOD);
    eTMR_SetRegLoadTrigSrc(etmrBase, DISABLE_TRIGGER);
}

static void MotorHwYtm32_InitEtmr(void)
{
    etmr_user_config_t etmrConfig;
    eTMR_Type *const etmrBase = g_etmrBase[MOTOR_HW_ETMR_INSTANCE];
    const uint8_t usedChannels[] = {
        MOTOR_HW_PWM_U_HIGH_CH,
        MOTOR_HW_PWM_U_LOW_CH,
        MOTOR_HW_PWM_V_HIGH_CH,
        MOTOR_HW_PWM_V_LOW_CH,
        MOTOR_HW_PWM_W_HIGH_CH,
        MOTOR_HW_PWM_W_LOW_CH
    };
    uint32_t index;

    etmrConfig.syncMethod = &s_motorEtmrSync;
    eTMR_DRV_GetDefaultConfig(&etmrConfig);

    s_motorEtmrSync.regSyncFreq = 1U;
    s_motorEtmrSync.regSyncSel = REG_SYNC_WITH_MOD;
    s_motorEtmrSync.cntInitSyncSel = CNT_SYNC_WITH_REG;
    s_motorEtmrSync.maskOutputSyncSel = CHMASK_SYNC_WITH_REG;
    s_motorEtmrSync.regSyncTrigSrc = DISABLE_TRIGGER;
    s_motorEtmrSync.cntInitSyncTrigSrc = DISABLE_TRIGGER;
    s_motorEtmrSync.maskOutputSyncTrigSrc = DISABLE_TRIGGER;
    s_motorEtmrSync.hwTrigFromTmuEnable = false;
    s_motorEtmrSync.hwTrigFromCimEnable = false;
    s_motorEtmrSync.hwTrigFromPadEnable = false;

    s_motorEtmrTrig.trigSrc = TRIGGER_FROM_MATCHING_EVENT;
    s_motorEtmrTrig.pwmOutputChannel = MOTOR_HW_PWM_U_HIGH_CH;
    s_motorEtmrTrig.outputTrigWidth = 1U;
    s_motorEtmrTrig.outputTrigFreq = 1U;
    s_motorEtmrTrig.modMatchTrigEnable = false;
    s_motorEtmrTrig.midMatchTrigEnable = false;
    s_motorEtmrTrig.initMatchTrigEnable = false;
    s_motorEtmrTrig.numOfChannels = 1U;
    s_motorEtmrTrig.channelTrigParamConfig = s_motorEtmrTrigChannel;
    s_motorEtmrTrigChannel[0].channelId = MOTOR_HW_PWM_U_LOW_CH;
    s_motorEtmrTrigChannel[0].channelVal0MatchTrigEn = true;
    s_motorEtmrTrigChannel[0].channelVal1MatchTrigEn = false;

    etmrConfig.outputTrigConfig = &s_motorEtmrTrig;
    etmrConfig.etmrClockSource = eTMR_CLOCK_SOURCE_INTERNALCLK;
    etmrConfig.etmrPrescaler = 1U;
    etmrConfig.debugMode = true;
    etmrConfig.isTofIntEnabled = false;

    (void)eTMR_DRV_Init(MOTOR_HW_ETMR_INSTANCE, &etmrConfig, &s_motorEtmrState);
    etmrBase->INIT = 0U;
    etmrBase->MOD = MOTOR_CFG_PWM_PERIOD_TICKS;
    etmrBase->MID = MOTOR_CFG_PWM_MID_TICKS;
    etmrBase->CHFV = 0U;

    for (index = 0U; index < (sizeof(usedChannels) / sizeof(usedChannels[0])); index++)
    {
        const uint8_t channel = usedChannels[index];

        eTMR_SetChnMode(etmrBase, channel, eTMR_PWM_MODE);
        eTMR_SetChnPwmSrc(etmrBase, channel, 0U);
        eTMR_IsInvertChnOutputPol(etmrBase, channel, false);
        eTMR_SetChnOutInitVal(etmrBase, channel, 0U);
        eTMR_InitChnOutput(etmrBase, channel);
        eTMR_SetChnDeadtime(etmrBase, channel, MOTOR_CFG_DEADTIME_TICKS);

        eTMR_SetChnVal0(etmrBase, channel, 0U);
        eTMR_SetChnVal1(etmrBase, channel, 0U);
    }

    eTMR_DRV_SetChnCompMode(MOTOR_HW_ETMR_INSTANCE, 0U, PWM_COMPLEMENTARY_MODE);
    eTMR_DRV_SetChnCompMode(MOTOR_HW_ETMR_INSTANCE, 2U, PWM_COMPLEMENTARY_MODE);
    eTMR_DRV_SetChnCompMode(MOTOR_HW_ETMR_INSTANCE, 3U, PWM_COMPLEMENTARY_MODE);
#if defined(FEATURE_eTMR_HAS_DOUBLE_SWITCH) && (FEATURE_eTMR_HAS_DOUBLE_SWITCH == 1U)
    eTMR_DRV_SetChnDoubleSwitch(MOTOR_HW_ETMR_INSTANCE, 0U, false);
    eTMR_DRV_SetChnDoubleSwitch(MOTOR_HW_ETMR_INSTANCE, 2U, false);
    eTMR_DRV_SetChnDoubleSwitch(MOTOR_HW_ETMR_INSTANCE, 3U, false);
#endif

    MotorHwYtm32_SetNeutralPwm(etmrBase);
    MotorHwYtm32_CommitShadowNow();
    (void)eTMR_DRV_SetChnOutMask(MOTOR_HW_ETMR_INSTANCE, MOTOR_CFG_USED_PWM_CHANNEL_MASK, 0U, true);
    eTMR_DRV_Disable(MOTOR_HW_ETMR_INSTANCE);
}

static void MotorHwYtm32_InitPtmr(void)
{
    ptmr_user_config_t ptmrConfig;
    ptmr_user_channel_config_t ptmrChannelConfig;

    pTMR_DRV_GetDefaultConfig(&ptmrConfig);
    pTMR_DRV_Init(MOTOR_HW_PTMR_INSTANCE, &ptmrConfig);

    pTMR_DRV_GetDefaultChanConfig(&ptmrChannelConfig);
    ptmrChannelConfig.periodUnits = pTMR_PERIOD_UNITS_MICROSECONDS;
    ptmrChannelConfig.period = 1000000UL / MOTOR_CFG_SPEED_LOOP_HZ;
    ptmrChannelConfig.chainChannel = false;
    ptmrChannelConfig.isInterruptEnabled = true;
    (void)pTMR_DRV_InitChannel(MOTOR_HW_PTMR_INSTANCE, MOTOR_HW_PTMR_CHANNEL, &ptmrChannelConfig);
}

static bool MotorHwYtm32_ReadFifoFrame(motor_adc_raw_frame_t *frame)
{
    frame->current_a_raw = ADC_DRV_ReadFIFO(MOTOR_HW_ADC_INSTANCE);
    frame->current_b_raw = ADC_DRV_ReadFIFO(MOTOR_HW_ADC_INSTANCE);
    frame->current_c_raw = ADC_DRV_ReadFIFO(MOTOR_HW_ADC_INSTANCE);
    frame->vbus_raw = ADC_DRV_ReadFIFO(MOTOR_HW_ADC_INSTANCE);

    return true;
}

void MotorHwYtm32_Init(void)
{
    MotorHwYtm32_ConfigClocks();
    MotorHwYtm32_SelectAdc0ExternalTriggerFromTmu();
    INT_SYS_DisableIRQ(ADC0_IRQn);
    MotorHwYtm32_ConfigAdc(true, false, false);
    MotorHwYtm32_InitEtmr();
    MotorHwYtm32_InitTmu();
    MotorHwYtm32_InitPtmr();
    INT_SYS_SetPriority(ADC0_IRQn, MOTOR_HW_ADC_IRQ_PRIORITY);
    INT_SYS_SetPriority(pTMR0_Ch0_IRQn, MOTOR_HW_SPEED_IRQ_PRIORITY);
}

void MotorHwYtm32_StartSpeedLoopTimer(void)
{
    pTMR_DRV_ClearInterruptFlagTimerChannels(MOTOR_HW_PTMR_INSTANCE, MOTOR_HW_PTMR_CHANNEL);
    pTMR_DRV_StartTimerChannels(MOTOR_HW_PTMR_INSTANCE, MOTOR_HW_PTMR_CHANNEL);
}

void MotorHwYtm32_ClearSpeedLoopIrq(void)
{
    pTMR_DRV_ClearInterruptFlagTimerChannels(MOTOR_HW_PTMR_INSTANCE, MOTOR_HW_PTMR_CHANNEL);
}

bool MotorHwYtm32_ReadSoftwareFrame(motor_adc_raw_frame_t *frame)
{
    uint32_t timeout = MOTOR_HW_SOFTWARE_ADC_TIMEOUT;

    if (frame == NULL)
    {
        return false;
    }

    if (s_adcConfiguredForHardwareTrigger)
    {
        MotorHwYtm32_ConfigAdc(false, false, false);
    }

    frame->overrun = false;
    ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
    ADC_DRV_Start(MOTOR_HW_ADC_INSTANCE);

    while ((timeout > 0U) && (!ADC_DRV_GetEndOfSequenceFlag(MOTOR_HW_ADC_INSTANCE)))
    {
        timeout--;
    }

    if (timeout == 0U)
    {
        ADC_DRV_Stop(MOTOR_HW_ADC_INSTANCE);
        return false;
    }

    frame->overrun = ADC_DRV_GetOvrRunOfConversionFlag(MOTOR_HW_ADC_INSTANCE);
    if (!MotorHwYtm32_ReadFifoFrame(frame))
    {
        ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
        ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
        return false;
    }

    ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
    if (frame->overrun)
    {
        ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
    }

    return true;
}

void MotorHwYtm32_EnableFastLoopSampling(void)
{
    INT_SYS_DisableIRQ(ADC0_IRQn);
    MotorHwYtm32_StopPwmTimeBase();
    MotorHwYtm32_ConfigAdc(true, true, true);
    ADC_DRV_Start(MOTOR_HW_ADC_INSTANCE);
    INT_SYS_EnableIRQ(ADC0_IRQn);
    MotorHwYtm32_StartPwmTimeBase();
}

void MotorHwYtm32_DisableFastLoopSampling(void)
{
    INT_SYS_DisableIRQ(ADC0_IRQn);
    ADC_DRV_Stop(MOTOR_HW_ADC_INSTANCE);
    MotorHwYtm32_StopPwmTimeBase();
    MotorHwYtm32_SetOutputsMasked(true);
    MotorHwYtm32_ConfigAdc(true, false, false);
}

bool MotorHwYtm32_ReadTriggeredFrame(motor_adc_raw_frame_t *frame)
{
    if ((frame == NULL) || (!ADC_DRV_GetEndOfSequenceFlag(MOTOR_HW_ADC_INSTANCE)))
    {
        return false;
    }

    frame->overrun = ADC_DRV_GetOvrRunOfConversionFlag(MOTOR_HW_ADC_INSTANCE);
    if (!MotorHwYtm32_ReadFifoFrame(frame))
    {
        ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
        ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
        return false;
    }

    ADC_DRV_ClearEoseqFlagCmd(MOTOR_HW_ADC_INSTANCE);
    if (frame->overrun)
    {
        ADC_DRV_ClearOvrFlagCmd(MOTOR_HW_ADC_INSTANCE);
    }

    return true;
}

void MotorHwYtm32_StartPwmTimeBase(void)
{
    eTMR_DRV_ClearTofFlag(MOTOR_HW_ETMR_INSTANCE);
    eTMR_DRV_Enable(MOTOR_HW_ETMR_INSTANCE);
}

void MotorHwYtm32_StopPwmTimeBase(void)
{
    eTMR_DRV_Disable(MOTOR_HW_ETMR_INSTANCE);
    eTMR_DRV_ClearTofFlag(MOTOR_HW_ETMR_INSTANCE);
}

void MotorHwYtm32_SetOutputsMasked(bool masked)
{
    (void)eTMR_DRV_SetChnOutMask(MOTOR_HW_ETMR_INSTANCE,
                                 masked ? MOTOR_CFG_USED_PWM_CHANNEL_MASK : 0U,
                                 0U,
                                 true);
}

void MotorHwYtm32_ApplyPhaseDuty(float dutyU, float dutyV, float dutyW)
{
    eTMR_Type *const etmrBase = g_etmrBase[MOTOR_HW_ETMR_INSTANCE];

    eTMR_ClearLdok(etmrBase);
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_U_HIGH_CH, dutyU);
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_V_HIGH_CH, dutyV);
    MotorHwYtm32_WriteHighSidePwm(etmrBase, MOTOR_HW_PWM_W_HIGH_CH, dutyW);
    MotorHwYtm32_SetAdcTriggerPoint(etmrBase);

    (void)eTMR_DRV_SetLdok(MOTOR_HW_ETMR_INSTANCE);
}
