/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file wdg_hw_access.h
 * @version 1.4.0
 */

#ifndef WDG_HW_ACCESS_H
#define WDG_HW_ACCESS_H

#include <stddef.h>
#include "wdg_driver.h"
#include "interrupt_manager.h"

/*!
 * @brief Watchdog Timer Hardware Access.
 *
 * This hardware access provides low-level access to all hardware features of the WDG.
 * @{
 */

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The dummy read is used in order to make sure that any write to the
 * WDG registers will be started only after the write of the unlock value was
 * completed.
 */
#define WDG_UNLOCK(base)  do\
{ \
    (base)->SVCR = FEATURE_WDG_UNLOCK_VALUE_1; \
    (base)->SVCR = FEATURE_WDG_UNLOCK_VALUE_2; \
} while(0)

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name WDG Common Configurations
 * @{
 */

/*!
 * @brief Disable the WDG.
 * This function disable the WDG by user
 *
 * @param[in] base WDG base pointer.
 */
void WDG_Deinit(WDG_Type *const base);

/*!
 * @brief Set the WDG timeout interrupt.
 * This function enable/disable WDG timeout interrupt
 *
 * @param[in] base WDG base pointer.
 * @param[in] enable enable/disable WDG timeout interrupt.
 */
void WDG_SetInt(WDG_Type *const base,
                bool enable);

/*!
 * @brief Configures the WDG.
 * This function configures the WDG by user configuration
 *
 * @param[in] base WDG base pointer.
 * @param[in] wdgUserConfig pointer to user configuration structure
 * @return the state of the WDG.
 */
status_t WDG_Config(WDG_Type *const base,
                    const wdg_user_config_t *wdgUserConfig);

/*!
 * @brief Gets the current WDG configuration.
 *
 * This function gets the current WDG configuration
 *
 *
 * @param[in] base WDG base pointer.
 * @param[in] config WDG config pointer.
 */
void WDG_GetConfig(const WDG_Type *base, wdg_user_config_t *const config);

/*!
 * @brief Verifies if the WDG is enabled.
 *
 * This function verifies the state of the WDG.
 *
 * @param[in] base WDG base pointer.
 * @return the state of the WDG.
 */
static inline bool WDG_IsEnabled(const WDG_Type *base)
{
    return (((base->CR & WDG_CR_EN_MASK) >> WDG_CR_EN_SHIFT) != 0U);
}

/*!
 * @brief Refreshes the WDG counter
 *
 * @param[in] base WDG base pointer.
 */
static inline void WDG_Trigger(WDG_Type *const base)
{
    SDK_ENTER_CRITICAL();
    base->SVCR = FEATURE_WDG_TRIGGER_VALUE_1;
    base->SVCR = FEATURE_WDG_TRIGGER_VALUE_2;
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Enables/Disables window mode.
 *
 * This function enables/disables window mode for the WDG.
 *
 * @param[in] base WDG base pointer.
 * @param[in] enable enable/disable window mode
 */
static inline void WDG_SetWindowMode(WDG_Type *const base,
                                     bool enable)
{
    SDK_ENTER_CRITICAL();
    WDG_UNLOCK(base);
    base->CR = (base->CR & ~WDG_CR_WIN_MASK) | WDG_CR_WIN(enable ? 1UL : 0UL);
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Sets the value of the WDG window.
 *
 * This sets the value of the WDG window.
 *
 * @param[in] base WDG base pointer.
 * @param[in] window the value of the WDG window.
 */
static inline void WDG_SetWindowValue(WDG_Type *const base,
                                      uint32_t window)
{
    SDK_ENTER_CRITICAL();
    WDG_UNLOCK(base);
    base->WVR = window;
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Clears the Interrupt Flag.
 *
 * This function clears the Interrupt Flag (FLG).
 *
 * @param[in] base WDG base pointer.
 */
static inline void WDG_ClearIntFlag(WDG_Type *const base)
{
    SDK_ENTER_CRITICAL();
    WDG_UNLOCK(base);
    base->INTF = WDG_INTF_IF_MASK;
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Verifies if the WDG updates are allowed.
 *
 * This function verifies if software is allowed to reconfigure the WDG without
 * a reset.
 *
 * @param[in] base WDG base pointer.
 * @return the state of the WDG updates:
 *         - false: updates are not allowed
 *         - true: updates are allowed
 */
static inline bool WDG_IsUpdateEnabled(const WDG_Type *base)
{
    return ((base->LR & WDG_LR_HL_MASK) >> WDG_LR_HL_SHIFT) == 0U;
}

/*!
 * @brief Enables/Disables WDG in debug mode.
 *
 * This function enables/disables the WDG in debug mode.
 *
 * @param[in] base WDG base pointer.
 * @param[in] enable enable/disable WDG in debug mode
 */
static inline void WDG_SetDebug(WDG_Type *const base,
                                bool enable)
{
    SDK_ENTER_CRITICAL();
    WDG_UNLOCK(base);
    /*LDRA_NOANALYSIS*/
    /* The comment LDRA_NOANALYSIS only use to run code coverage */
    base->CR = (base->CR & ~WDG_CR_DBGDIS_MASK) | WDG_CR_DBGDIS(enable ? 0UL : 1UL);
    /*LDRA_ANALYSIS*/
    /* The comment LDRA_ANALYSIS only use to run code coverage */
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Enables/Disables WDG in wait mode.
 *
 * This function enables/disables the WDG in wait mode.
 *
 * @param[in] base WDG base pointer.
 * @param[in] enable enable/disable WDG in wait mode.
 */
static inline void WDG_SetDeepSleep(WDG_Type *const base,
                                    bool enable)
{
    SDK_ENTER_CRITICAL();
    WDG_UNLOCK(base);
    /*LDRA_NOANALYSIS*/
    /* The comment LDRA_NOANALYSIS only use to run code coverage */
    base->CR = (base->CR & ~WDG_CR_DSDIS_MASK) | WDG_CR_DSDIS(enable ? 0UL : 1UL);
    /*LDRA_ANALYSIS*/
    /* The comment LDRA_ANALYSIS only use to run code coverage */
    SDK_EXIT_CRITICAL();
}

/*!
 * @brief Checks if the WDG is unlocked.
 *
 * This function checks if the WDG is unlocked. If the module is unlocked,
 * reconfiguration of the registers is allowed.
 *
 * @param[in] base WDG base pointer.
 * @return true if the module is unlocked, false if the module is locked.
 */
static inline bool WDG_IsUnlocked(const WDG_Type *base)
{
    return ((((base->LR & WDG_LR_SL_MASK) >> WDG_LR_SL_SHIFT) == 0U)
           && (((base->LR & WDG_LR_HL_MASK) >> WDG_LR_HL_SHIFT) == 0U));
}

/*! @} */

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* WDG_HW_ACCESS_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
