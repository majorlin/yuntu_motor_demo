/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file wdg_hw_access.c
 * @version 1.4.0
 */

#include "wdg_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_Deinit
 * Description   : De-init WDG module.
 *
 *END**************************************************************************/
void WDG_Deinit(WDG_Type *const base)
{
    /* Unlock WDG register */
    WDG_UNLOCK(base);
    /* Disable WDG, enables support for 32-bit refresh/unlock command, LPO clock source,
       allow updates and disable watchdog interrupts, window mode, wait/debug/stop mode */
    base->CR = FEATURE_WDG_CR_RESET_VALUE;
    /* Default timeout value */
    base->TOVR = FEATURE_WDG_TOVR_RESET_VALUE;
    /* Clear window value */
    base->WVR = FEATURE_WDG_WVR_RESET_VALUE;

    /* Refresh counter value */
    WDG_Trigger(base);

}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_SetInt
 * Description   : enable/disable the WDG timeout interrupt
 *
 *END**************************************************************************/
void WDG_SetInt(WDG_Type *const base,
                bool enable)
{
    /* Unlock WDG register */
    WDG_UNLOCK(base);
    /* Enable/disable WDG interrupt */
    if (enable)
    {
        base->CR = (base->CR & ~WDG_CR_IBR_MASK) | WDG_CR_IBR(1U);
    }
    else
    {
        base->CR &= ~WDG_CR_IBR_MASK;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_Config
 * Description   : Configures all WDG registers.
 *
 *END**************************************************************************/
status_t WDG_Config(WDG_Type *const base,
                    const wdg_user_config_t *wdgUserConfig)
{
    status_t status = STATUS_SUCCESS;
    uint32_t cr = 0;
    bool tmp1 = WDG_IsEnabled(base);
    bool tmp2 = WDG_IsUpdateEnabled(base);

    INT_SYS_DisableIRQGlobal();

    if ((tmp1 == false) && (tmp2 == true))
    {
#if (defined(WDG_CR_CLKSRC_MASK))
        /* Update clock source */
        cr = WDG_CR_CLKSRC(wdgUserConfig->clockSource);
#endif
        /* Construct CS register new value */
        if (wdgUserConfig->winEnable)
        {
            cr |= WDG_CR_WIN_MASK;
        }
        if (wdgUserConfig->intEnable)
        {
            cr |= WDG_CR_IBR_MASK;
        }
        if (wdgUserConfig->apbErrorResetEnable)
        {
            cr |= WDG_CR_RIA_MASK;
        }
        if (!wdgUserConfig->opMode.debug)
        {
            cr |= WDG_CR_DBGDIS_MASK;
        }
        if (!wdgUserConfig->opMode.deepsleep)
        {
            cr |= WDG_CR_DSDIS_MASK;
        }
        /* Enable WDG in 32-bit mode */
        cr |= WDG_CR_EN_MASK;

        while (!WDG_IsUnlocked(base))
        {
            /* Wait until registers are unlocked */
            WDG_UNLOCK(base);
        }

        base->TOVR = wdgUserConfig->timeoutValue;
        if (wdgUserConfig->winEnable)
        {
            base->WVR = wdgUserConfig->windowValue;
        }
        /* Reset interrupt flags */
        base->INTF |= WDG_INTF_IF_MASK;
        base->CR = cr;
        /* Lock configuration */
        if (wdgUserConfig->updateEnable == false)
        {
            base->LR = WDG_LR_HL(1U) | WDG_LR_SL(1U);
        }
        else
        {
            base->LR = WDG_LR_SL(1U);
        }
    } else
    {
        status = STATUS_ERROR;
    }

    INT_SYS_EnableIRQGlobal();

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_GetConfig
 * Description   : Gets the current WDG configuration.
 *
 *END**************************************************************************/
void WDG_GetConfig(const WDG_Type *base, wdg_user_config_t *const config)
{
    uint32_t cr = base->CR;

    /* Construct CS register new value */
#ifdef FEATURE_WDG_HAS_CLOCK_SOURCE_SELECT
    config->clockSource = (cr & WDG_CR_CLKSRC_MASK) >> WDG_CR_CLKSRC_SHIFT;
#endif
    config->winEnable = ((cr & WDG_CR_WIN_MASK) != 0U);
    config->intEnable = ((cr & WDG_CR_IBR_MASK) != 0U);
    config->apbErrorResetEnable = ((cr & WDG_CR_RIA_MASK) != 0U);
    config->opMode.debug = ((cr & WDG_CR_DBGDIS_MASK) == 0U);
    config->opMode.deepsleep = ((cr & WDG_CR_DSDIS_MASK) == 0U);
    config->timeoutValue = (uint32_t) base->TOVR;
    config->windowValue = (uint32_t) base->WVR;
    config->updateEnable = ((base->LR & WDG_LR_HL_MASK) == 0U);

}

/*******************************************************************************
 * EOF
 ******************************************************************************/

