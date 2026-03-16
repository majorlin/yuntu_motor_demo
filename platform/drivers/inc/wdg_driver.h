/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file wdg_driver.h
 * @version 1.4.0
 */

#ifndef WDG_DRIVER_H
#define WDG_DRIVER_H

#include "status.h"
#include "interrupt_manager.h"

/*!
 * @addtogroup WDG_DRIVER
 * @brief Watchdog Timer Peripheral Driver.
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*!
 * @brief set modes for the WDG.
 * Implements : wdg_set_mode_t_Class
 */
typedef enum
{
    WDG_DEBUG_MODE = 0x00U,     /*!< Debug mode */
    WDG_DEEPSLEEP_MODE = 0x01U, /*!< DeepSleep mode */
} wdg_set_mode_t;

/*!
 * @brief WDG option mode configuration structure
 * Implements : wdg_op_mode_t_Class
 */
typedef struct
{
    bool deepsleep;  /*!< Deepsleep mode */
    bool debug;      /*!< Debug mode */
} wdg_op_mode_t;

/*!
 * @brief WDG user configuration structure
 * Implements : wdg_user_config_t_Class
 */
typedef struct
{
#if (defined(WDG_CR_CLKSRC_MASK))
    wdg_clock_source_t clockSource;           /*!< Clock source */
#endif
    wdg_op_mode_t opMode;                     /*!< The modes in which the WDG is functional */
    bool updateEnable;                        /*!< If true, further updates of the WDG are enabled */
    bool intEnable;                           /*!< If true, an interrupt request is generated before reset */
    bool winEnable;                           /*!< If true, window mode is enabled */
    uint32_t windowValue;                     /*!< The window value */
    uint32_t timeoutValue;                    /*!< The timeout value */
    bool apbErrorResetEnable;                 /*!< If true, reset will happen if APB access error happens */
} wdg_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Initializes the WDG driver.
 *
 * @param[in] instance WDG peripheral instance number
 * @param[in] userConfigPtr pointer to the WDG user configuration structure
 * @return operation status
 *        - STATUS_SUCCESS: Operation was successful.
 *        - STATUS_ERROR: Operation failed. Possible causes: previous
 *        clock source or the one specified in the configuration structure is
 *        disabled; WDG configuration updates are not allowed; WDG instance has been initialized before;
 *        If window mode enabled and window value greater than or equal to the timeout value.
 */
status_t WDG_DRV_Init(uint32_t instance,
                      const wdg_user_config_t *userConfigPtr);

/*!
 * @brief De-initializes the WDG driver
 *
 * @param[in] instance  WDG peripheral instance number
 * @return operation status
 *        - STATUS_SUCCESS: if allowed reconfigures WDG module and de-initializes successful.
 *        - STATUS_ERROR: Operation failed. Possible causes: failed to
 *          WDG configuration updates not allowed.
 */
status_t WDG_DRV_Deinit(uint32_t instance);

/*!
 * @brief Gets the current configuration of the WDG.
 *
 * @param[in] instance  WDG peripheral instance number
 * @param[out] config   A pointer to the current user configuration
 */
void WDG_DRV_GetConfig(uint32_t instance, wdg_user_config_t *const config);
/*!
 * @brief Gets default configuration of the WDG.
 *
 * @param[out] config A pointer to the default configuration
 */
void WDG_DRV_GetDefaultConfig(wdg_user_config_t *const config);

/*!
 * @brief  Enables/Disables the WDG timeout interrupt and sets a function to be
 * called when a timeout interrupt is received, before reset.
 *
 * @param[in] instance WDG peripheral instance number
 * @param[in] enable enable/disable interrupt
 * @return operation status
 *        - STATUS_SUCCESS: if allowed reconfigures WDG timeout interrupt.
 *        - STATUS_ERROR: Operation failed. Possible causes: failed to
 *          WDG configuration updates not allowed.
 */
status_t WDG_DRV_SetInt(uint32_t instance, bool enable);

/*!
 * @brief Clear interrupt flag of the WDG.
 *
 * @param[in] instance WDG peripheral instance number
 */
void WDG_DRV_ClearIntFlag(uint32_t instance);

/*!
 * @brief Refreshes the WDG counter.
 *
 * @param[in] instance WDG peripheral instance number
 */
void WDG_DRV_Trigger(uint32_t instance);

/*!
 * @brief Gets the value of the WDG  counter.
 *
 * @param[in] instance WDG peripheral instance number.
 * @return the value of the WDG counter.
 */
uint32_t WDG_DRV_GetCounter(uint32_t instance);

/*!
 * @brief Set window mode and window value of the WDG.
 *
 * This function set window mode, window value is set when window mode enabled.
 *
 * @param[in] instance WDG peripheral instance number.
 * @param[in] enable enable/disable window mode and window value.
 * @param[in] windowValue the value of the WDG window.
 * @return operation status
 *        - STATUS_SUCCESS: if allowed reconfigures window value success.
 *        - STATUS_ERROR: Operation failed. Possible causes: failed to
 *          WDG configuration updates not allowed.
 */
status_t WDG_DRV_SetWindow(uint32_t instance,
                           bool enable,
                           uint32_t windowValue);

/*!
 * @brief Sets the mode operation of the WDG.
 *
 * This function changes the mode operation of the WDG.
 *
 * @param[in] instance WDG peripheral instance number.
 * @param[in] enable enable/disable mode of the WDG.
 * @param[in] setMode select mode of the WDG.
 * @return operation status
 *        - STATUS_SUCCESS: if allowed reconfigures mode operation of the WDG.
 *        - STATUS_ERROR: Operation failed. Possible causes: failed to
 *          WDG configuration updates not allowed.
 */
status_t WDG_DRV_SetMode(uint32_t instance,
                         bool enable,
                         wdg_set_mode_t setMode);

/*!
 * @brief Sets the value of the WDG timeout.
 *
 * This function sets the value of the WDG timeout.
 *
 * @param[in] instance WDG peripheral instance number.
 * @param[in] timeout the value of the WDG timeout.
 * @return operation status
 *        - STATUS_SUCCESS: if allowed reconfigures WDG timeout.
 *        - STATUS_ERROR: Operation failed. Possible causes: failed to
 *          WDG configuration updates not allowed.
 */
status_t WDG_DRV_SetTimeout(uint32_t instance, uint32_t timeout);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* WDG_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
