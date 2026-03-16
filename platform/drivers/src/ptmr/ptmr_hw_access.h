/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file ptmr_hw_access.h
 * @version 1.4.0
 */

#ifndef PTMR_HW_ACCESS_H
#define PTMR_HW_ACCESS_H

#include <stdbool.h>
#include "device_registers.h"
#include "ptmr_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Enables the pTMR module.
 *
 * This function enables the functional clock of pTMR module (Note: this function
 * does not un-gate the system clock gating control). It should be called before
 * setup any timer channel.
 *
 * @param[in] base pTMR peripheral base address
 */
static inline void pTMR_Enable(pTMR_Type * const base, volatile uint32_t delay)
{
    volatile uint32_t tempDelay = delay;
    base->MCR |= pTMR_MCR_EN_MASK;
    /* Run this counter down to zero
        If the delay is 0, the four clock delay between setting and clearing
        the SW_RST bit is ensured by the read-modify-write operation.
    */
    while(tempDelay != 0u)
    {
        /* Since we need a four cycle delay, we assume the decrement is one cycle
            and insert three NOP instructions. The actual delay will be larger because
            of the loop overhead and the compiler optimization.
        */
        tempDelay = tempDelay - 1U; 
        __NOP();
        __NOP();
        __NOP();
    }
}

/*!
 * @brief Disables the pTMR module.
 *
 * This function disables functional clock of pTMR module (Note: it does not
 * affect the system clock gating control).
 *
 * @param[in] base pTMR peripheral base address
 */
static inline void pTMR_Disable(pTMR_Type * const base)
{
    base->MCR &= ~pTMR_MCR_EN_MASK;
}

/*!
 * @brief Resets the pTMR module.
 *
 * This function sets all pTMR registers to reset value,
 * except the Module Control Register.
 *
 * @param[in] base pTMR peripheral base address
 */
static inline void pTMR_Reset(pTMR_Type * const base, volatile uint32_t delay)
{
    volatile uint32_t tempDelay = delay;
    base->MCR = 0;
    for (int i = 0; i < 4; i++)
    {
        base->CH[i].TCR = 0;
        base->CH[i].TCV = 0;
        base->CH[i].TFR = 0;
        base->CH[i].TSV = 0;
    }
    /* Run this counter down to zero
        If the delay is 0, the four clock delay between setting and clearing
        the SW_RST bit is ensured by the read-modify-write operation.
    */
    while(tempDelay != 0u)
    {
        /* Since we need a four cycle delay, we assume the decrement is one cycle
            and insert three NOP instructions. The actual delay will be larger because
            of the loop overhead and the compiler optimization.
        */
        tempDelay = tempDelay - 1U;
        __NOP();
        __NOP();
        __NOP();
    }
}

/*!
  @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * will be started
 * - For example:
 *      - with channel = 0x00U then channel 0 will be started
 *      - with channel = 0x01U then channel 1 will be started
 *      - with channel = 0x02U then channel 2 will be started
 *      - with channel = 0x03U then channel 3 will be started
 */
static inline void pTMR_StartTimerChannels(pTMR_Type * const base,
                                           uint32_t channel)
{
    base->CH[channel].TCR |= pTMR_CH_TCR_TEN_MASK;
}

/*!
 * @brief Stops the timer channel from counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the pTMR_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * will be stopped
 * - For example:
 *      - with channel = 0x00U then channel 0 will be stopped
 *      - with channel = 0x01U then channel 1 will be stopped
 *      - with channel = 0x02U then channel 2 will be stopped
 *      - with channel = 0x03U then channel 3 will be stopped
 */
static inline void pTMR_StopTimerChannels(pTMR_Type * const base,
                                          uint32_t channel)
{
    base->CH[channel].TCR &= ~pTMR_CH_TCR_TEN_MASK;
}

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit.
 * The period range depends on the frequency of the pTMR functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * Timer channel begins counting from the value that is set by this function.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
static inline void pTMR_SetTimerPeriodByCount(pTMR_Type * const base,
                                              uint32_t channel,
                                              uint32_t count)
{
    base->CH[channel].TSV = count;
}

/*!
 * @brief Gets the timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 */
static inline uint32_t pTMR_GetTimerPeriodByCount(const pTMR_Type * base,
                                                  uint32_t channel)
{
    return (base->CH[channel].TSV);
}

/*!
 * @brief Gets the current timer channel counting value.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value
 */
static inline uint32_t pTMR_GetCurrentTimerCount(const pTMR_Type * base,
                                                 uint32_t channel)
{
    return (base->CH[channel].TCV);
}

/*!
 * @brief Enables the interrupt generation for timer channels.
 *
 * This function allows enabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel The interrupt enabling channel that decides which channels will
 * be enabled interrupt.
 * - For example:
 *      - with channel = 0x00u then will enable interrupt for channel 0 only
 *      - with channel = 0x01u then will enable interrupt for channel 1 only
 *      - with channel = 0x02u then will enable interrupt for channel 2 only
 *      - with channel = 0x03u then will enable interrupt for channel 3 only
 */
static inline void pTMR_EnableInterruptTimerChannels(pTMR_Type * const base,
                                                     uint32_t channel)
{
    base->CH[channel].TCR |= pTMR_CH_TCR_TIE_MASK;
}

/*!
 * @brief Disables the interrupt generation for timer channels.
 *
 * This function allows disabling interrupt generation for timer channels simultaneously.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel The interrupt disabling channel that decides which channels will
 * be disabled interrupt.
 * - For example:
 *      - with channel = 0x00u then will disable interrupt for channel 0 only
 *      - with channel = 0x01u then will disable interrupt for channel 1 only
 *      - with channel = 0x02u then will disable interrupt for channel 2 only
 *      - with channel = 0x03u then will disable interrupt for channel 3 only
 */
static inline void pTMR_DisableInterruptTimerChannels(pTMR_Type * const base,
                                                      uint32_t channel)
{
    base->CH[channel].TCR &= ~pTMR_CH_TCR_TIE_MASK;
}

/*!
 * @brief Gets the interrupt flag of timer channels.
 *
 * This function gets current interrupt flag of timer channels.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel The interrupt flag getting channel that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with channel = 0x00u then the interrupt flag of channel 0 only will be got
 *      - with channel = 0x01u then the interrupt flag of channel 1 only will be got
 *      - with channel = 0x02u then the interrupt flag of channel 2 only will be got
 *      - with channel = 0x03u then the interrupt flag of channel 3 only will be got
 * @return The interrupt flag of timer channels.
 */
static inline uint32_t pTMR_GetInterruptFlagTimerChannels(const pTMR_Type * base,
                                                          uint32_t channel)
{
    return (base->CH[channel].TFR) & pTMR_CH_TFR_TIF_MASK;
}

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears current interrupt flag of timer channels.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel The interrupt flag clearing channel that decides which channels will
 * be cleared interrupt flag.
 * - For example:
 *      - with channel = 0x00u then the interrupt flag of channel 0 only will be cleared
 *      - with channel = 0x01u then the interrupt flag of channel 1 only will be cleared
 *      - with channel = 0x02u then the interrupt flag of channel 2 only will be cleared
 *      - with channel = 0x03u then the interrupt flag of channel 3 only will be cleared
 */
static inline void pTMR_ClearInterruptFlagTimerChannels(pTMR_Type * const base,
                                                        uint32_t channel)
{
    /* Write 1 to clear the interrupt flag. */
    base->CH[channel].TFR |= pTMR_CH_TFR_TIF_MASK;
    base->CH[channel].TFR &= ~pTMR_CH_TFR_TIF_MASK;
}

/*!
 * @brief Sets timer channel chaining.
 *
 * This function sets the timer channel to be chained or not chained.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] channel Timer channel number
 * @param[in] isChannelChained Timer channel chaining
 *        - True : Timer channel is chained. Timer channel decrements on previous channel's timeout
 *        - False : Timer channel is not chained. Timer channel runs independently
 */
static inline void pTMR_SetTimerChannelChainCmd(pTMR_Type * const base,
                                                uint32_t channel,
                                                bool isChannelChained)
{
    base->CH[channel].TCR &= ~pTMR_CH_TCR_CHAIN_MASK;
    base->CH[channel].TCR |=  pTMR_CH_TCR_CHAIN(isChannelChained ? 1UL : 0UL);
}

/*!
 * @brief Sets operation of pTMR in debug mode.
 *
 * When the device enters debug mode, the timer channels may or may not be frozen,
 * based on the configuration of this function. This is intended to aid software development,
 * allowing the developer to halt the processor, investigate the current state of
 * the system (for example, the timer channel values), and continue the operation.
 *
 * @param[in] base pTMR peripheral base address
 * @param[in] isRunInDebug pTMR run in debug mode
 *        - True: pTMR continue to run when the device enters debug mode
 *        - False: pTMR stop when the device enters debug mode
 */
static inline void pTMR_SetTimerRunInDebugCmd(pTMR_Type * const base,
                                              bool isRunInDebug)
{
    base->MCR &= ~pTMR_MCR_FRZ_MASK;
    base->MCR |= pTMR_MCR_FRZ(isRunInDebug ? 0UL: 1UL);
}

#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
/*!
 * @brief Sets pTMR clock source.
 *
 * pTMR support pclk and function clock, when using function clock from IPC module, make sure
 * to set the clock source to function clock, otherwise, the pTMR will be stopped.
 * @note
 * This function is only available when the pTMR support IPC clock source.
 * @param[in] base pTMR peripheral base address
 * @param[in] useFuncClk pTMR clock source
 *        - True: pTMR clock source is function clock
 *        - False: pTMR clock source is pclk
 *
 *
 */
static inline void pTMR_SetTimerClockSource(pTMR_Type * const base,
                                              bool useFuncClk)
{
    base->MCR &= ~pTMR_MCR_CLK_SEL_MASK;
    base->MCR |= pTMR_MCR_CLK_SEL(useFuncClk ? 1UL: 0UL);
}

/*!
 * @brief Gets pTMR clock source.
 *
 * Returns the pTMR clock source.
 *
 * @return pTMR clock source
 *        - True: pTMR clock source is function clock
 *        - False: pTMR clock source is pclk
 *
 */
 static inline bool pTMR_GetTimerClockSource(const pTMR_Type * const base)
 {
     return ((base->MCR & pTMR_MCR_CLK_SEL_MASK) >> pTMR_MCR_CLK_SEL_SHIFT) != 0U;
 }
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */

#if defined(__cplusplus)
}
#endif

#endif /* PTMR_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
