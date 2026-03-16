/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file ptmr_driver.h
 * @version 1.4.0
 */

#ifndef PTMR_DRIVER_H
#define PTMR_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @addtogroup ptmr_drv
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Max period in count of all operation mode */
#define MAX_PERIOD_COUNT                    (0xFFFFFFFFU)

/*!
 * @brief Unit options for pTMR period.
 *
 * This is used to determine unit of timer period
 * Implements : ptmr_period_units_t_Class
 */
typedef enum
{
    pTMR_PERIOD_UNITS_COUNTS = 0x00U, /*!< Period value unit is count */
    pTMR_PERIOD_UNITS_MICROSECONDS = 0x01U  /*!< Period value unit is microsecond */
} ptmr_period_units_t;

/*!
 * @brief pTMR configuration structure
 *
 * This structure holds the configuration settings for the pTMR peripheral to
 * enable or disable pTMR module in DEBUG and DOZE mode
 * Implements : ptmr_user_config_t_Class
 */
typedef struct
{
    bool enableRunInDebug; /*!< True: Timer channels continue to run in debug mode
                                False: Timer channels stop in debug mode            */
#if (defined(FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE) && (FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE == 1))
    bool useFunctionClockSource;          /*!< pTMR use function clock from IPC module */
#endif /* FEATURE_pTMR_HAS_IPC_CLOCK_SOURCE */
} ptmr_user_config_t;

/*! @brief Structure to configure the channel timer
 *
 * This structure holds the configuration settings for the pTMR timer channel
 * Implements : ptmr_user_channel_config_t_Class
 */
typedef struct
{
    ptmr_period_units_t periodUnits;     /*!< Timer period value units                                      */
    uint32_t period;                     /*!< Period of timer channel                                       */
    bool chainChannel;                   /*!< Channel chaining enable                                       */
    bool isInterruptEnabled;             /*!< Timer channel interrupt generation enable                     */
} ptmr_user_channel_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name Initialization and De-initialization
 * @{
 */

/*!
 * @brief Gets the default pTMR configuration
 *
 * This function gets default pTMR module configuration structure, with the following settings:
 * - pTMR runs in debug mode: Disable
 *
 * @param[out] config The configuration structure
 */
void pTMR_DRV_GetDefaultConfig(ptmr_user_config_t *const config);

/*!
 * @brief Gets the default timer channel configuration
 *
 * This function gets the default timer channel configuration structure, with the following settings:
 * - Period unit: Period value unit is microsecond
 * - Period: 1000000 microseconds(1 second)
 * - Channel chaining: Disable
 * - Interrupt generating: Enable
 *
 * @param[out] config The channel configuration structure
 */
void pTMR_DRV_GetDefaultChanConfig(ptmr_user_channel_config_t *const config);

/*!
 * @brief Initializes the pTMR module.
 *
 * This function resets pTMR module, enables the pTMR module, configures pTMR
 * module operation in Debug mode. The pTMR configuration structure shall
 * be passed as arguments.
 * This configuration structure affects all timer channels.
 * This function should be called before calling any other pTMR driver function.
 *
 * This is an example demonstrating how to define a pTMR configuration structure:
   @code
   ptmr_user_config_t ptmrInit =
   {
        .enableRunInDebug = false,
   };
   @endcode
 *
 * @param[in] instance pTMR module instance number.
 * @param[in] userConfig Pointer to pTMR configuration structure.
 */
void pTMR_DRV_Init(uint32_t instance,
                   const ptmr_user_config_t *userConfig);

/*!
 * @brief De-Initializes the pTMR module.
 *
 * This function disables pTMR module.
 * In order to use the pTMR module again, pTMR_DRV_Init must be called.
 *
 * @param[in] instance pTMR module instance number
 */
void pTMR_DRV_Deinit(uint32_t instance);

/*!
 * @brief Initializes the pTMR channel.
 *
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
 * This is an example demonstrating how to define a pTMR channel configuration structure:
   @code
   ptmr_user_channel_config_t ptmrTestInit =
   {
    .periodUnits = LPTT_PERIOD_UNITS_MICROSECONDS,
    .period = 1000000U,
    .chainChannel = false,
    .isInterruptEnabled = true
   };
   @endcode
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @param[in] userChannelConfig Pointer to pTMR channel configuration structure
 * @return Operation status
 *         - STATUS_SUCCESS: Operation was successful.
 *         - STATUS_ERROR: The input period is invalid.
 */
status_t pTMR_DRV_InitChannel(uint32_t instance,
                              uint32_t channel,
                              const ptmr_user_channel_config_t *userChannelConfig);

/* @} */

/*!
 * @name Timer Start and Stop
 * @{
 */

/*!
 * @brief Starts the timer channel counting.
 *
 * This function allows starting timer channels simultaneously .
 * After calling this function, timer channels are going operate depend on mode and
 * control bits which controls timer channel start, reload and restart.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channels starting channel that decides which channels
 * will be started
 * - For example:
 *      - with channel = 0x00U then channel 0 will be started
 *      - with channel = 0x01U then channel 1 will be started
 *      - with channel = 0x02U then channel 2 will be started
 *      - with channel = 0x03U then channel 3 will be started
 */
void pTMR_DRV_StartTimerChannels(uint32_t instance,
                                 uint32_t channel);

/*!
 * @brief Stops the timer channel counting.
 *
 * This function allows stop timer channels simultaneously from counting.
 * Timer channels reload their periods respectively after the next time
 * they call the pTMR_DRV_StartTimerChannels. Note that: In 32-bit Trigger Accumulator
 * mode, the counter will load on the first trigger rising edge.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channels stopping channel that decides which channels
 * will be stopped
 * - For example:
 *      - with channel = 0x00U then channel 0 will be stopped
 *      - with channel = 0x01U then channel 1 will be stopped
 *      - with channel = 0x02U then channel 2 will be stopped
 *      - with channel = 0x03U then channel 3 will be stopped
 */
void pTMR_DRV_StopTimerChannels(uint32_t instance,
                                uint32_t channel);

/* @} */

/*!
 * @name Timer Period
 * @{
 */

/*!
 * @brief Sets the timer channel period in microseconds.
 *
 * This function sets the timer channel period in microseconds
 * when timer channel mode is 32 bit periodic or dual 16 bit counter mode.
 * The period range depends on the frequency of the pTMR functional clock and
 * operation mode of timer channel.
 * If the required period is out of range, use the suitable mode if applicable.
 * This function is only valid for one single channel.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @param[in] periodUs Timer channel period in microseconds
 * @return Operation status
 *         - STATUS_SUCCESS: Input period of timer channel is valid.
 *         - STATUS_ERROR: Input period of timer channel is invalid.
 */
status_t pTMR_DRV_SetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel,
                                     uint32_t periodUs);

/*!
 * @brief Sets the timer channel period in count unit.
 *
 * This function sets the timer channel period in count unit.
 * The counter period of a running timer channel can be modified by first setting
 * a new load value, the value will be loaded after the timer channel expires.
 * To abort the current cycle and start a timer channel period with the new value,
 * the timer channel must be disabled and enabled again.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @param[in] count Timer channel period in count unit
 */
void pTMR_DRV_SetTimerPeriodByCount(uint32_t instance,
                                    uint32_t channel,
                                    uint32_t count);

/*!
 * @brief Gets the timer channel period in microseconds.
 *
 * This function gets the timer channel period in microseconds.
 * The returned period here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in microseconds
 */
uint64_t pTMR_DRV_GetTimerPeriodByUs(uint32_t instance,
                                     uint32_t channel);

/*!
 * @brief Gets the current timer channel period in count unit.
 *
 * This function returns current period of timer channel given as argument.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @return Timer channel period in count unit
 */
uint32_t pTMR_DRV_GetTimerPeriodByCount(uint32_t instance,
                                        uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in microseconds.
 *
 * This function returns an absolute time stamp in microseconds.
 * One common use of this function is to measure the running time of a part of
 * code. Call this function at both the beginning and end of code. The time
 * difference between these two time stamps is the running time.
 * The return counting value here makes sense if the operation mode of timer channel
 * is 32 bit periodic counter or dual 16 bit periodic counter or 32-bit trigger input capture.
 * Need to make sure the running time will not exceed the timer channel period.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in microseconds
 */
uint64_t pTMR_DRV_GetCurrentTimerUs(uint32_t instance,
                                    uint32_t channel);

/*!
 * @brief Gets the current timer channel counting value in count.
 *
 * This function returns the real-time timer channel counting value, the value in
 * a range from 0 to timer channel period.
 * Need to make sure the running time does not exceed the timer channel period.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel Timer channel number
 * @return Current timer channel counting value in count
 */
uint32_t pTMR_DRV_GetCurrentTimerCount(uint32_t instance,
                                       uint32_t channel);
/* @} */

/*!
 * @name Interrupt
 * @{
 */

/*!
 * @brief Enables the interrupt generation of timer channel.
 *
 * This function allows enabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * @param[in] instance pTMR module instance number.
 * @param[in] channel The channel that decides which channels will be enabled interrupt.
 * - For example:
 *      - with channel = 0x00u then the interrupt of channel 0 will be enabled
 *      - with channel = 0x01u then the interrupt of channel 1 will be enabled
 *      - with channel = 0x02u then the interrupt of channel 2 will be enabled
 *      - with channel = 0x03u then the interrupt of channel 3 will be enabled
 */
void pTMR_DRV_EnableTimerChannelInterrupt(uint32_t instance,
                                          uint32_t channel);

/*!
 * @brief Disables the interrupt generation of timer channel.
 *
 * This function allows disabling interrupt generation of timer channel
 * when timeout occurs or input trigger occurs.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel The channel that decides which channels will be disable interrupt.
 * - For example:
 *      - with channel = 0x00u then the interrupt of channel 0 will be disabled
 *      - with channel = 0x01u then the interrupt of channel 1 will be disabled
 *      - with channel = 0x02u then the interrupt of channel 2 will be disabled
 *      - with channel = 0x03u then the interrupt of channel 3 will be disabled
 */
void pTMR_DRV_DisableTimerChannelInterrupt(uint32_t instance,
                                           uint32_t channel);

/*!
 * @brief Gets the current interrupt flag of timer channels.
 *
 * This function gets the current interrupt flag of timer channels.
 * In compare modes, the flag sets to 1 at the end of the timer period.
 * In capture modes, the flag sets to 1 when the trigger asserts.
 *
 * @param[in] instance pTMR module instance number.
 * @param[in] channel The interrupt flag getting channel that decides which channels will
 * be got interrupt flag.
 * - For example:
 *      - with channel = 0x00u then the interrupt flag of channel 0 only will be got
 *      - with channel = 0x01u then the interrupt flag of channel 1 only will be got
 *      - with channel = 0x02u then the interrupt flag of channel 2 only will be got
 *      - with channel = 0x03u then the interrupt flag of channel 3 only will be got
 * @return Current the interrupt flag of timer channels
 */
uint32_t pTMR_DRV_GetInterruptFlagTimerChannels(uint32_t instance,
                                                uint32_t channel);

/*!
 * @brief Clears the interrupt flag of timer channels.
 *
 * This function clears the interrupt flag of timer channels after
 * their interrupt event occurred.
 *
 * @param[in] instance pTMR module instance number
 * @param[in] channel The interrupt flag clearing channel that decides which channels will
 * be cleared interrupt flag
 * - For example:
 *      - with channel = 0x00u then the interrupt flag of channel 0 only will be cleared
 *      - with channel = 0x01u then the interrupt flag of channel 1 only will be cleared
 *      - with channel = 0x02u then the interrupt flag of channel 2 only will be cleared
 *      - with channel = 0x03u then the interrupt flag of channel 3 only will be cleared
 */
void pTMR_DRV_ClearInterruptFlagTimerChannels(uint32_t instance,
                                              uint32_t channel);

/* @} */

#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* PTMR_DRIVER_H*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
