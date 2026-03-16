/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file ptmr_driver.c
 * @version 1.4.0
 */

#include <stddef.h>
#include "ptmr_driver.h"
#include "ptmr_hw_access.h"
#include "interrupt_manager.h"
#include "clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/* Table of base addresses for pTMR instances */
static pTMR_Type * const s_ptmrBase[] = pTMR_BASE_PTRS;
/* Table to save pTMR indexes for clock configuration */
static const clock_names_t s_ptmrClkNames[pTMR_INSTANCE_COUNT] = pTMR_CLOCK_NAMES;
#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
/* Table to save pTMR IPC clock configuration */
static const clock_names_t s_ptmrIpcClkNames[pTMR_INSTANCE_COUNT] = pTMR_IPC_CLOCK_NAMES;
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */
/* pTMR functional clock variable which will be updated in some driver functions */
static uint32_t s_ptmrSourceClockFrequency[pTMR_INSTANCE_COUNT] = {0};
static const IRQn_Type ptmrIrqId[pTMR_INSTANCE_COUNT][pTMR_CH_COUNT] = pTMR_IRQS;

/*******************************************************************************
 * INTERNAL FUNCTIONS
 ******************************************************************************/
static void pTMR_Update_ClockFreq(uint32_t instance);
/******************************************************************************
 * Code
 *****************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetDefaultConfig
 * Description   : This function gets default pTMR module configuration structure.
 *
 * Implements    : pTMR_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void pTMR_DRV_GetDefaultConfig(ptmr_user_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->enableRunInDebug = false;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetDefaultChanConfig
 * Description   : This function gets default timer channel configuration structure.
 *
 * Implements    : pTMR_DRV_GetDefaultChanConfig_Activity
 *END**************************************************************************/
void pTMR_DRV_GetDefaultChanConfig(ptmr_user_channel_config_t * const config)
{
    DEV_ASSERT(config != NULL);

    config->periodUnits = pTMR_PERIOD_UNITS_MICROSECONDS;
    config->period = 1000000U;
    config->chainChannel = false;
    config->isInterruptEnabled = true;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_Init
 * Description   : Initializes pTMR module.
 * This function resets pTMR module, enables the pTMR module, configures pTMR
 * module operation in Debug and DOZE mode. The pTMR configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other pTMR driver function.
 *
 * Implements    : pTMR_DRV_Init_Activity
 *END**************************************************************************/
void pTMR_DRV_Init(uint32_t instance,
                   const ptmr_user_config_t *userConfig)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(userConfig != NULL);

    /* Gets current functional clock frequency of pTMR instance */
    pTMR_Update_ClockFreq(instance);
    /* When resetting the pTMR module, a delay of 4 peripheral clock cycles
        must be ensured. This peripheral clock and the core clock running the
        code could be very different, two distinct cases are identified:
         - core_clk > peripheral_clk. This requires a delay loop to be implemented,
            and the delay value based on the ratio between the two frequencies.
         - core_clk <= peripheral_clk. This requires a short delay, which is usually
            below the delay caused naturally by the read-modify-write operation.
     */
    base = s_ptmrBase[instance];
    /* Resets pTMR module */
    pTMR_Reset(base, 5);
    /* Enables functional clock of pTMR module*/
#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
    pTMR_SetTimerClockSource(base, userConfig->useFunctionClockSource);
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */
    pTMR_Enable(base, 5);
    /* Sets pTMR operation in Debug and DOZE mode*/
    pTMR_SetTimerRunInDebugCmd(base, userConfig->enableRunInDebug);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_Deinit
 * Description   : De-initializes pTMR module.
 * This function disables pTMR module.
 * In order to use the pTMR module again, pTMR_DRV_Init must be called.
 *
 * Implements    : pTMR_DRV_Deinit_Activity
 *END**************************************************************************/
void pTMR_DRV_Deinit(uint32_t instance)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);

    base = s_ptmrBase[instance];
    for (uint8_t i = 0U; i < 4U; i++)
    {
        /* Stops timer channel from counting */
        pTMR_StopTimerChannels(base, i);
        INT_SYS_DisableIRQ(ptmrIrqId[instance][i]);
    }
    /* Disables pTMR module functional clock*/
    pTMR_Disable(base);
    /* Resets pTMR module */
    pTMR_Reset(base, 5);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_InitChannel
 * Description   : Initializes pTMR channel.
 * This function initializes the pTMR timers by using a channel, this function
 * configures timer channel chaining, timer channel mode, timer channel period,
 * interrupt generation, trigger source, trigger select, reload on trigger,
 * stop on interrupt and start on trigger.
 * The timer channel number and its configuration structure shall be passed as arguments.
 * Timer channels do not start counting by default after calling this function.
 * The function pTMR_DRV_StartTimerChannels must be called to start the timer channel counting.
 * In order to re-configures the period, call the pTMR_DRV_SetTimerPeriodByUs or
 * pTMR_DRV_SetTimerPeriodByCount.
 *
 * Implements    : pTMR_DRV_InitChannel_Activity
 *END**************************************************************************/
status_t pTMR_DRV_InitChannel(uint32_t instance,
                              uint32_t channel,
                              const ptmr_user_channel_config_t * userChannelConfig)
{
    pTMR_Type * base;
    status_t reVal = STATUS_SUCCESS;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(userChannelConfig != NULL);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];

    /* Setups the timer channel chaining  */
    pTMR_SetTimerChannelChainCmd(base, channel, userChannelConfig->chainChannel);

    if (userChannelConfig->periodUnits == pTMR_PERIOD_UNITS_MICROSECONDS)
    {
        /* Setups timer channel period in microsecond unit */
        reVal = pTMR_DRV_SetTimerPeriodByUs(instance, channel, userChannelConfig->period);
    }
    else
    {
        /* Setups timer channel period in count unit */
        pTMR_DRV_SetTimerPeriodByCount(instance, channel, userChannelConfig->period);
    }

    if (reVal == STATUS_SUCCESS)
    {
        /* Setups interrupt generation for timer channel */
        if (userChannelConfig->isInterruptEnabled)
        {
            /* Enables interrupt generation */
            pTMR_EnableInterruptTimerChannels(base, channel);
            INT_SYS_EnableIRQ(ptmrIrqId[instance][channel]);
        }
        else
        {
            /* Disables interrupt generation */
            pTMR_DisableInterruptTimerChannels(base, channel);
            /* Only disable channel interrupt globally if each channel has a separate interrupt line */
#if defined(FEATURE_pTMR_HAS_NUM_IRQS_CHANS) && (FEATURE_pTMR_HAS_NUM_IRQS_CHANS == pTMR_CH_COUNT)
            INT_SYS_DisableIRQ(ptmrIrqId[instance][channel]);
#endif
        }
    }

    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_StartTimerChannels
 * Description   : Starts timer channel counting.
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * Implements    : pTMR_DRV_StartTimerChannels_Activity
 *END**************************************************************************/
void pTMR_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t channel)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Starts timer channel counting */
    pTMR_StartTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_StopTimerChannels
 * Description   : Stop timer channel from counting.
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the pTMR_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * Implements    : pTMR_DRV_StopTimerChannels_Activity
 *END**************************************************************************/
void pTMR_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t channel)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Stops timer channel from counting */
    pTMR_StopTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_SetTimerPeriodByUs
 * Description   : Sets timer channel period in microseconds unit.
 * This function sets the timer channel period in microseconds
 * when timer channel mode is 32 bit periodic or dual 16 bit counter mode.
 * The period range depends on the frequency of the pTMR functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * Implements    : pTMR_DRV_SetTimerPeriodByUs_Activity
 *END**************************************************************************/
status_t pTMR_DRV_SetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel,
                                     uint32_t periodUs)
{
    pTMR_Type * base;
    status_t reVal = STATUS_SUCCESS;
    uint64_t count;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    /* Gets current functional clock frequency of pTMR instance */
    pTMR_Update_ClockFreq(instance);

    base = s_ptmrBase[instance];
    /* Calculates the count value, assign it to timer channel counter register.*/
    count = ((uint64_t)periodUs) * s_ptmrSourceClockFrequency[instance];
    count = (count / 1000000U) - 1U;
    /* Checks whether the count is valid with timer channel operation mode */
    if (count > MAX_PERIOD_COUNT)
    {
        reVal = STATUS_ERROR;
    }
    if (reVal == STATUS_SUCCESS)
    {
        /* Sets the timer channel period in count unit */
        pTMR_SetTimerPeriodByCount(base, channel, (uint32_t)count);
    }
    return reVal;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_SetTimerPeriodByCount
 * Description   : Sets the timer channel period in count unit.
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * Implements    : pTMR_DRV_SetTimerPeriodByCount_Activity
 *END**************************************************************************/
void pTMR_DRV_SetTimerPeriodByCount(uint32_t instance,
                                    uint32_t channel,
                                    uint32_t count)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Sets the timer channel period in count unit */
    pTMR_SetTimerPeriodByCount(base, channel, count);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetTimerPeriodByUs
 * Description   : Gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * Implements    : pTMR_DRV_GetTimerPeriodByUs_Activity
 *END**************************************************************************/
uint64_t pTMR_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel)
{
    const pTMR_Type * base;
    uint64_t currentPeriod;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    /* Gets current functional clock frequency of pTMR instance */
    pTMR_Update_ClockFreq(instance);

    base = s_ptmrBase[instance];
    /* Gets current timer channel period in count.*/
    currentPeriod = pTMR_GetTimerPeriodByCount(base, channel);

    /* Converts period from count unit to microseconds unit for other modes */
    currentPeriod = ((currentPeriod + 1U) * 1000000U) / s_ptmrSourceClockFrequency[instance];

    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetTimerPeriodByCount
 * Description   : Gets the current timer channel period in count unit.
 *
 * Implements    : pTMR_DRV_GetTimerPeriodByCount_Activity
 *END**************************************************************************/
uint32_t pTMR_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel)
{
    const pTMR_Type * base;
    uint32_t currentPeriod;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Gets current timer channel period by count.*/
    currentPeriod = pTMR_GetTimerPeriodByCount(base, channel);

    return currentPeriod;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetCurrentTimerUs
 * Description   : Gets current timer channel counting value in microseconds unit.
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * Implements    : pTMR_DRV_GetCurrentTimerUs_Activity
 *END**************************************************************************/
uint64_t pTMR_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel)
{
    const pTMR_Type * base;
    uint64_t currentTime = 0U;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    /* Gets current functional clock frequency of pTMR instance */
    pTMR_Update_ClockFreq(instance);

    base = s_ptmrBase[instance];
    /* Gets current timer channel counting value */
    currentTime = pTMR_GetCurrentTimerCount(base, channel);

    /* Converts counting value to microseconds unit for other modes */
    currentTime = (currentTime * 1000000U) / s_ptmrSourceClockFrequency[instance];

    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetCurrentTimerCount
 * Description   : Gets the current timer channel counting value in count.
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * Implements    : pTMR_DRV_GetCurrentTimerCount_Activity
 *END**************************************************************************/
uint32_t pTMR_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel)
{
    const pTMR_Type * base;
    uint32_t currentTime;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Gets current timer channel counting value */
    currentTime = pTMR_GetCurrentTimerCount(base, channel);

    return currentTime;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_EnableTimerChannelInterrupt
 * Description   : This function allows enabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * Implements    : pTMR_DRV_EnableTimerChannelInterrupt_Activity
 *END**************************************************************************/
void pTMR_DRV_EnableTimerChannelInterrupt(uint32_t instance,
                                          uint32_t channel)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Enable interrupt of timer channels */
    pTMR_EnableInterruptTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_DisableTimerChannelInterrupt
 * Description   : This function allows disabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * Implements    : pTMR_DRV_DisableTimerChannelInterrupt_Activity
 *END**************************************************************************/
void pTMR_DRV_DisableTimerChannelInterrupt(uint32_t instance,
                                           uint32_t channel)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Disable interrupt of timer channels */
    pTMR_DisableInterruptTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_GetInterruptFlagTimerChannels
 * Description   : Gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * Implements    : pTMR_DRV_GetInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
uint32_t pTMR_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t channel)
{
    const pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Gets the interrupt flag for timer channels */
    return pTMR_GetInterruptFlagTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : pTMR_DRV_ClearInterruptFlagTimerChannels
 * Description   : Clears the interrupt flag of timer channels.
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 * Implements    : pTMR_DRV_ClearInterruptFlagTimerChannels_Activity
 *END**************************************************************************/
void pTMR_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t channel)
{
    pTMR_Type * base;

    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < pTMR_CH_COUNT);

    base = s_ptmrBase[instance];
    /* Clears the interrupt flag for timer channels */
    pTMR_ClearInterruptFlagTimerChannels(base, channel);
}

/*FUNCTION**********************************************************************
 * Function Name : pTMR_Get_ClockFreq
 * Description   : Get pTMR clock frequency based on pTMR clock setting
 * END**************************************************************************/

static void pTMR_Update_ClockFreq(uint32_t instance)
{
    DEV_ASSERT(instance < pTMR_INSTANCE_COUNT);
    status_t clkErr;
#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
    pTMR_Type *base;
    uint32_t function_clk_freq;
    base = s_ptmrBase[instance];
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */
    /* Gets current functional clock frequency of pTMR instance */
    clkErr = CLOCK_SYS_GetFreq(s_ptmrClkNames[instance], &s_ptmrSourceClockFrequency[instance]);
    /* Checks the functional clock of pTMR module */
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    (void)clkErr;
#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
    /* Get function clock frequency */
    clkErr = CLOCK_SYS_GetFreq(s_ptmrIpcClkNames[instance], &function_clk_freq);
    DEV_ASSERT(clkErr == STATUS_SUCCESS);
    (void)clkErr;
    /* Check if pTMR using function clock */
    if (pTMR_GetTimerClockSource(base))
    {
        /* Function clock should be less than pclk freq / 4 */
        if (function_clk_freq > (s_ptmrSourceClockFrequency[instance] / 4U))
        {
            s_ptmrSourceClockFrequency[instance] = 0;
        }
        else
        {
            s_ptmrSourceClockFrequency[instance] = function_clk_freq;
        }
    }
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */
    DEV_ASSERT(s_ptmrSourceClockFrequency[instance] > 0U);
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
