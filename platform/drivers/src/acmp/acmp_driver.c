/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file acmp_driver.c
 * @version 1.4.0
 */

#include "acmp_driver.h"
#include "acmp_hw_access.h"
#include <stddef.h>

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Table of base addresses for ACMP instances. */
static ACMP_Type *const g_acmpBase[] = ACMP_BASE_PTRS;

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_Reset
 * Description   : This function set all ACMP registers to reset values.
 *
 * Implements : ACMP_DRV_Reset_Activity
 *END**************************************************************************/
status_t ACMP_DRV_Reset(const uint32_t instance)
{
    status_t status = STATUS_SUCCESS;
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    IPC->CTRL[instance + IPC_ACMP0_INDEX] |= IPC_CTRL_SWREN_MASK;
    IPC->CTRL[instance + IPC_ACMP0_INDEX] &= ~IPC_CTRL_SWREN_MASK;

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetDefaultConfig
 * Description   : Return a default configuration structure for all components
 * from comparator module.
 * Implements : ACMP_DRV_GetDefaultConfig_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetDefaultConfig(const acmp_config_t *config)
{
    status_t status = STATUS_SUCCESS;
    DEV_ASSERT(config != NULL);

#if FEATURE_ACMP_HAS_CLK_SRC_SEL
    (config->comparatorConfig)->acmpClkSrc = ACMP_CLK_SRC_SIRC;
#endif
    (config->comparatorConfig)->sampleMode = ACMP_COMMON_MODE;
    (config->comparatorConfig)->edgeSelection = ACMP_RISING_EDGE;
#if FEATURE_ACMP_HAS_OFFSET_CONTROL
    (config->comparatorConfig)->hardBlockOffset = false;
#endif
    (config->comparatorConfig)->hysteresisLevel = ACMP_HYS_LEVEL_0;
    (config->comparatorConfig)->outputSelect = ACMP_OUTPUT_DIRECTLY;
    (config->comparatorConfig)->outputPolarity = ACMP_OUT_POL_NOT_INVERT;
    (config->comparatorConfig)->filterEnable = false;
#if FEATURE_ACMP_HAS_AUTODIS
    (config->comparatorConfig)->autoDisableHardBlock = false;
#endif
    (config->comparatorConfig)->powerMode = ACMP_LOW_POWER;
    (config->comparatorConfig)->filterClkSrc = ACMP_FILTER_SEL_FUNC_CLK;
    (config->comparatorConfig)->interruptEnable = true;
    (config->comparatorConfig)->dmaTriggerEnable = false;

    (config->dacConfig)->enable = true;
    (config->dacConfig)->voltage = 127U;

    (config->muxConfig)->positiveInputSrc = ACMP_INPUT_SRC_MUX;
    (config->muxConfig)->negativeInputSrc = ACMP_INPUT_SRC_MUX;
    (config->muxConfig)->positiveInputChnSel = 0U;
    (config->muxConfig)->negativeInputChnSel = 1U;

    (config->continuousConfig)->continuousEnable = false;
    (config->continuousConfig)->continuousMode = ACMP_CONTINUOUS_LOOP_MODE;
    (config->continuousConfig)->continuousInterruptEnable = false;
    (config->continuousConfig)->fixedPort = ACMP_FIXED_NEG_PORT;
    (config->continuousConfig)->samplePeriod = FEATURE_ACMP_DEFAULT_CONT_PERIOD;
    (config->continuousConfig)->samplePosition = FEATURE_ACMP_DEFAULT_CONT_POS;
    for (int8_t i = 0; i < 8; i++)
    {
        (config->continuousConfig)->channelConfig[i].enable = false;
        (config->continuousConfig)->channelConfig[i].expectation = ACMP_EXPECT_POS_LESS_THAN_NEG;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_Init
 * Description   : Initialize Analog comparator configuration.
 *
 * Implements : ACMP_DRV_Init_Activity
 *END**************************************************************************/
status_t ACMP_DRV_Init(const uint32_t instance, const acmp_config_t *config)
{
    status_t status = STATUS_SUCCESS;
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);

    /* Configure acmp comparator mode */
    if (config->comparatorConfig != NULL)
    {
        (void)ACMP_DRV_ConfigComparator(instance, config->comparatorConfig);
    }
    /* Configure acmp dac */
    if (config->dacConfig != NULL)
    {
        (void)ACMP_DRV_ConfigDac(instance, config->dacConfig);
    }
    /* Configure acmp multiplexer */
    if (config->muxConfig != NULL)
    {
        (void)ACMP_DRV_ConfigMux(instance, config->muxConfig);
    }
    /* Clear all flags*/
    (void)ACMP_DRV_ClearOutputFlags(instance);
    (void)ACMP_DRV_ClearChannelFlags(instance);
    /* Configure continuous mode if it is enabled */
    if (config->continuousConfig != NULL)
    {
        if (config->continuousConfig->continuousEnable)
        {
            (void)ACMP_DRV_ConfigContinuous(instance, config->continuousConfig);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_SetExpectation
 * Description   : This function set channel expectation
 * Implements : ACMP_DRV_SetExpectation_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_SetExpectation(const uint32_t instance, uint8_t state)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    /* Set channels expectation */
    ACMP_SetExpectation(base, state);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_EnableContinuous
 * Description   : This function enable continuous mode
 * Implements : ACMP_DRV_EnableContinuous_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_EnableContinuous(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    /* Set continuous mode enable state */
    ACMP_SetContinuousModeEnState(base, true);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_DisableContinuous
 * Description   : This function disable continuous mode
 * Implements : ACMP_DRV_DisableContinuous_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_DisableContinuous(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    /* Disable continuous mode */
    ACMP_SetContinuousModeEnState(base, false);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetConfigAll
 * Description   : This function returns the configuration for all components
 * from comparator module.
 * Implements : ACMP_DRV_GetConfigAll_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetConfigAll(const uint32_t instance, const acmp_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);

    /* Get ACMP sample mode configuration */
    (void)ACMP_DRV_GetComparatorConfig(instance, config->comparatorConfig);
    /* Get DAC configuration */
    (void)ACMP_DRV_GetDacConfig(instance, config->dacConfig);
    /* Get multiplexer configuration */
    (void)ACMP_DRV_GetMuxConfig(instance, config->muxConfig);
    /* Get continuous mode configuration */
    (void)ACMP_DRV_GetContinuousConfig(instance, config->continuousConfig);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_ConfigComparator
 * Description   : Configure only comparator features (functional mode, power mode,
 * inverter, hysteresis, offset, filter sampling period and samples count).
 * Implements : ACMP_DRV_ConfigComparator_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_ConfigComparator(const uint32_t instance, const acmp_comparator_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];

#if FEATURE_ACMP_HAS_CLK_SRC_SEL
    /* Set ACMP clock source */
    ACMP_SetClockSrc(base, (uint8_t)config->acmpClkSrc);
#endif
    /* Set ACMP sample mode */
    ACMP_SetSampleMode(base, config->sampleMode);
    /* Set edge selection */
    ACMP_SetEdgeSelection(base, config->edgeSelection);
#if FEATURE_ACMP_HAS_OFFSET_CONTROL
    /* Set hard block offset control */
    ACMP_SetHardBlockOffset(base, (config->hardBlockOffset ? 1U : 0U));
#endif
    /* Set hysteresis */
    ACMP_SetHysteresis(base, config->hysteresisLevel);
    /* Set output selection for filter or not */
    ACMP_SetOutputFilterSelection(base, config->outputSelect);
    /* Set output polarity */
    ACMP_SetOutputPolarity(base, config->outputPolarity);
    /* Set power mode */
    ACMP_SetPowerMode(base, (config)->powerMode);
    
    if (config->filterEnable)
    {
        /* Set filter clock source */
        ACMP_SetFilterClockSrc(base, (uint8_t)config->filterClkSrc);
        /* Set filter sample period */
        ACMP_SetFilterSamplePeriod(base, config->filterSamplePeriod);
        /* Set filter sample count */
        ACMP_SetFilterSampleCount(base, config->filterSampleCount);
    }
    else
    {
#if FEATURE_ACMP_HAS_FILTER_BYPASS
        ACMP_SetFilterEnState(base, false);
#else
        /* Set filter sample period */
        ACMP_SetFilterSamplePeriod(base, 0U);
        /* Set filter sample count */
        ACMP_SetFilterSampleCount(base, 0U);
#endif
    }
#if FEATURE_ACMP_HAS_AUTODIS
    /* Set auto disable hard block */
    ACMP_SetAutoDisableHardBlock(base, config->autoDisableHardBlock);
#endif
    /* Set interrupt enable state */
    ACMP_SetInterruptEnState(base, config->interruptEnable);
    /* Set DMA enable state */
    ACMP_SetDmaEnState(base, config->dmaTriggerEnable);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetComparator
 * Description   : Return configuration for comparator components from ACMP module.
 * Implements : ACMP_DRV_GetComparatorConfig_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetComparatorConfig(const uint32_t instance, acmp_comparator_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];

#if FEATURE_ACMP_HAS_CLK_SRC_SEL
    config->acmpClkSrc = (acmp_clock_source_t)ACMP_GetClockSrc(base);
#endif
    config->sampleMode = (acmp_sample_mode_t)ACMP_GetSampleMode(base);
    config->edgeSelection = (acmp_edge_select_t)ACMP_GetEdgeSelection(base);
#if FEATURE_ACMP_HAS_OFFSET_CONTROL
    config->hardBlockOffset = ACMP_GetHardBlockOffset(base);
#endif
    config->hysteresisLevel = (acmp_hysteresis_t)ACMP_GetHysteresis(base);
    config->outputSelect = (acmp_output_select_t)ACMP_GetOutputFilterSelection(base);
    config->outputPolarity = (acmp_output_polarity_t)ACMP_GetOutputPolarity(base);
    config->powerMode = (acmp_power_mode_t)ACMP_GetPowerMode(base);
#if FEATURE_ACMP_HAS_AUTODIS
    config->autoDisableHardBlock = ACMP_GetAutoDisableHardBlock(base);
#endif
    config->filterEnable = false;
    config->filterClkSrc = (acmp_filter_clk_src_t)ACMP_GetFilterClockSrc(base);
    config->filterSampleCount = ACMP_GetFilterSampleCount(base);
    config->filterSamplePeriod = ACMP_GetFilterSamplePeriod(base);
    config->interruptEnable = ACMP_GetInterruptEnState(base);
    config->dmaTriggerEnable = ACMP_GetDmaTriggerEnable(base);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_ConfigDac
 * Description   : Configure only DAC component from comparator module.
 * Implements : ACMP_DRV_ConfigDac_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_ConfigDac(const uint32_t instance, const acmp_dac_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
#if ACMP_DAC_RESOLUTION != 255U
    DEV_ASSERT(config->voltage <= ACMP_DAC_RESOLUTION);
#endif
    ACMP_Type *const base = g_acmpBase[instance];

    ACMP_SetDacEnState(base, config->enable);
#if FEATURE_ACMP_HAS_DAC_OUTPUT
    ACMP_SetDacOutEnState(base, config->outputEnable);
#endif
    ACMP_SetVoltage(base, config->voltage);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetDacConfig
 * Description   : Return configuration for DAC component from comparator module.
 * Implements : ACMP_DRV_GetDacConfig_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetDacConfig(const uint32_t instance, acmp_dac_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];

    config->enable = ACMP_GetDacEnState(base);
#if FEATURE_ACMP_HAS_DAC_OUTPUT
    config->outputEnable = ACMP_GetDacOutEnState(base);
#endif
    config->voltage = ACMP_GetVoltage(base);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_ConfigMux
 * Description   : Configure only MUX component from comparator module to select
 * source signals for comparator ports.
 * Implements : ACMP_DRV_ConfigMux_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_ConfigMux(const uint32_t instance, const acmp_mux_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    /* Set positive port input source */
    ACMP_SetPositivePortInputSrc(base, config->positiveInputSrc);
    /* Set negative port input source */
    ACMP_SetNegativePortInputSrc(base, config->negativeInputSrc);
    /* Set positive port input channel */
    ACMP_SetPositivePortInputChannel(base, config->positiveInputChnSel);
    /* Set negative port input channel */
    ACMP_SetNegativePortInputChannel(base, config->negativeInputChnSel);

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetMuxConfig
 * Description   : Return configuration for the MUX component which select
 * source signals for comparator ports.
 * Implements : ACMP_DRV_GetMuxConfig_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetMuxConfig(const uint32_t instance, acmp_mux_config_t *config)
{
    status_t status = STATUS_SUCCESS;
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    /* Get positive port input source */
    config->positiveInputSrc = (acmp_input_source_t)ACMP_GetPositivePortInputSrc(base);
    /* Get negative port input source */
    config->negativeInputSrc = (acmp_input_source_t)ACMP_GetNegativePortInputSrc(base);
    /* Get positive port input channel */
    config->positiveInputChnSel = ACMP_GetPositivePortInputChannel(base);
    /* Get negative port input channel */
    config->negativeInputChnSel = ACMP_GetNegativePortInputChannel(base);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_ConfigContinuous
 * Description   : Configure comparator in trigger mode.
 * Implements : ACMP_DRV_ConfigContinuous_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_ConfigContinuous(const uint32_t instance, const acmp_continuous_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];
    uint8_t enTemp = 0;
    uint8_t expectTemp = 0;

    /* Set continuous one-shot or loop mode */
    ACMP_SetContinuousMode(base, config->continuousMode);

#if FEATURE_ACMP_HAS_TRIG_MODE_GATE
    if ((config->continuousMode == ACMP_CONTINUOUS_ONE_SHOT_MODE) &&
        (config->oneshotTriggerEnable == true))
    {
        ACMP_SetOneshotTriggerMode(base, true);
    }
#endif

    /* Set continuous interrupt enable state */
    ACMP_SetContinuousModeInterruptEnState(base, config->continuousInterruptEnable);
    /* Set fixed port */
    ACMP_SetFixedPort(base, config->fixedPort);
    /* Set continuous mode sample period */
    ACMP_SetContinuousModeSamplePeriod(base, config->samplePeriod);
    /* Set continuous mode sample position */
    ACMP_SetContinuousModeSamplePosition(base, config->samplePosition);
    for (uint8_t i = 0; i < 8U; i++)
    {
        enTemp |= (config->channelConfig[i].enable ? 1U : 0U) << i;
        expectTemp |= (uint8_t)config->channelConfig[i].expectation << i;
    }
    /* Set continuous mode input channels */
    ACMP_SetContinuousModeChannels(base, enTemp);
    /* Set expectation for each channel comparison */
    ACMP_SetExpectation(base, expectTemp);
    /* Set acmp continuous mode */
#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
    if (config->continuousEnable)
    {
        ACMP_SetSampleMode(base, ACMP_CONTINUOUS_MODE);
    }
#else
    if (config->continuousEnable)
    {
        ACMP_SetContinuousModeEnState(base, true);
    }
    else
    {
        ACMP_SetContinuousModeEnState(base, false);
    }
#endif

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetContinuousConfig
 * Description   : Return configuration for the continuous mode.
 * Implements : ACMP_DRV_GetContinuousConfig_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetContinuousConfig(const uint32_t instance, acmp_continuous_config_t *config)
{
    DEV_ASSERT(config != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const base = g_acmpBase[instance];

#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
    config->continuousEnable = (ACMP_GetSampleMode(base) == (uint8_t)ACMP_CONTINUOUS_MODE) ? true : false;
#else
    config->continuousEnable = ACMP_GetContinuousModeEnState(base);
#endif
    config->continuousInterruptEnable = ACMP_GetContinuousModeInterruptEnState(base);
    config->continuousMode = (acmp_continuous_mode_t)ACMP_GetContinuousMode(base);
    config->fixedPort = (acmp_fixed_port_t)ACMP_GetFixedPort(base);
    config->samplePeriod = ACMP_GetContinuousModeSamplePeriod(base);
    config->samplePosition = ACMP_GetContinuousModeSamplePosition(base);
    for (uint8_t i = 0; i < 8U; i++)
    {
        config->channelConfig[i].enable = ACMP_GetContinuousChannelEnState(base, i);
    }

    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetOutputFlags
 * Description   : Return in <flags> comparator output flags(rising and falling edge on output).
 * Implements : ACMP_DRV_GetOutputFlags_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_GetOutputFlags(const uint32_t instance, uint8_t *flags)
{
    DEV_ASSERT(flags != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    *flags = ACMP_GetOutputFlags(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_ClearOutputFlags
 * Description   : Clear comparator output flags(rising and falling edge on output).
 * Implements : ACMP_DRV_ClearOutputFlags_Activity
 *
 *END**************************************************************************/
status_t ACMP_DRV_ClearOutputFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    ACMP_ClearOutputFlags(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
*
* Function Name : ACMP_DRV_GetChannelFlags
* Description   : Return all input change flags in <flags>.
* <flags> format : Flag_Ch7 Flag_Ch6 ... Flag_Ch0
* Implements : ACMP_DRV_GetInputFlags_Activity
*
*END**************************************************************************/
status_t ACMP_DRV_GetChannelFlags(const uint32_t instance, acmp_ch_list_t *flags)
{
    DEV_ASSERT(flags != NULL);
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    *flags = ACMP_GetChannelFlags(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
*
* Function Name : ACMP_DRV_ClearChannelFlags
* Description   : Clear all input channels flags .
* Implements : ACMP_DRV_ClearInputFlags_Activity
*
*END**************************************************************************/
status_t ACMP_DRV_ClearChannelFlags(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    ACMP_ClearChannelFlags(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_Enable
 * Description   : This function enable the ACMP.
 *
 * Implements : ACMP_DRV_Enable_Activity
 *END**************************************************************************/
status_t ACMP_DRV_Enable(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    ACMP_Enable(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_Disable
 * Description   : This function disable the ACMP.
 *
 * Implements : ACMP_DRV_Disable_Activity
 *END**************************************************************************/
status_t ACMP_DRV_Disable(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    ACMP_Disable(baseAddr);
    return STATUS_SUCCESS;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetChannelId
 * Description   : This function is to get channel ID in the continuous mode.
 *
 * Implements : ACMP_DRV_GetChannelId_Activity
 *END**************************************************************************/
uint8_t ACMP_DRV_GetChannelId(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);
    uint8_t channelId = 0U;
    ACMP_Type *const baseAddr = g_acmpBase[instance];
    /* Get channel ID when sampling */
    channelId = ACMP_GetChannelId(baseAddr);
    return channelId;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetOutput
 * Description   : This function is to get module output value.
 *
 * Implements : ACMP_DRV_GetOutput_Activity
 *END**************************************************************************/
bool ACMP_DRV_GetOutput(const uint32_t instance)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);

    ACMP_Type *baseAddr = g_acmpBase[instance];

    return ACMP_GetOutput(baseAddr);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : ACMP_DRV_GetChannelOutput
 * Description   : This function is to get channel output value.
 *
 * Implements : ACMP_DRV_GetOutput_Activity
 *END**************************************************************************/
bool ACMP_DRV_GetChannelOutput(const uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < ACMP_INSTANCE_COUNT);

    ACMP_Type *baseAddr = g_acmpBase[instance];
    
    return (ACMP_GetContinuousChannelOutput(baseAddr, channel)) != 0U;
}

/******************************************************************************
 * EOF
 *****************************************************************************/
