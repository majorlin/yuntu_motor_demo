/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file wdg_driver.c
 * @version 1.4.0
 */

#include "wdg_hw_access.h"
#include "clock_manager.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for WDG instances. */
static WDG_Type *const s_wdgBase[] = WDG_BASE_PTRS;

/*! @brief Table to save WDG IRQ enum numbers defined in CMSIS header file. */
static const IRQn_Type s_wdgIrqId[] = WDG_IRQS;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_Init
 * Description   : initialize the WDG driver
 *
 * Implements    : WDG_DRV_Init_Activity
 *END**************************************************************************/
status_t WDG_DRV_Init(uint32_t instance, const wdg_user_config_t *userConfigPtr)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    DEV_ASSERT(userConfigPtr != NULL);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    /* If window mode enabled and window value greater than or equal to the timeout value. Or timeout value is 0 */
    if (((userConfigPtr->winEnable) && (userConfigPtr->windowValue >= userConfigPtr->timeoutValue)) ||
        (userConfigPtr->timeoutValue <= FEATURE_WDG_MINIMUM_TIMEOUT_VALUE))
    {
        status = STATUS_ERROR;
    }
    else
    {
        /* Configure the WDG module */
        status = WDG_Config(base, userConfigPtr);
    }

    if (status == STATUS_SUCCESS)
    {
        /* enable WDG timeout interrupt */
        INT_SYS_EnableIRQ(s_wdgIrqId[instance]);
    }
    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_Deinit
 * Description   : De-initialize the WDG driver
 *
 * Implements    : WDG_DRV_Deinit_Activity
 *END**************************************************************************/
status_t WDG_DRV_Deinit(uint32_t instance)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    INT_SYS_DisableIRQGlobal();

    /* If allowed reconfigures WDG */
    if (WDG_IsUpdateEnabled(base))
    {
        /* Disable WDG timeout interrupt */
        INT_SYS_DisableIRQ(s_wdgIrqId[instance]);

        /* Disable WDG */
        WDG_Deinit(base);
    }
    else
    {
        status = STATUS_ERROR;
    }

    /* Enable global interrupt */
    INT_SYS_EnableIRQGlobal();

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_GetConfig
 * Description   : get the current configuration of the WDG driver
 *
 * Implements    : WDG_DRV_GetConfig_Activity
 *END**************************************************************************/
void WDG_DRV_GetConfig(uint32_t instance, wdg_user_config_t *const config)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    DEV_ASSERT(config != NULL);
    const WDG_Type *baseAddr = s_wdgBase[instance];

    WDG_GetConfig(baseAddr, config);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_GetDefaultConfig
 * Description   : get default configuration of the WDG driver
 *
 * Implements    : WDG_DRV_GetDefaultConfig_Activity
 *END**************************************************************************/
void WDG_DRV_GetDefaultConfig(wdg_user_config_t *const config)
{
    DEV_ASSERT(config != NULL);

    /* Construct CS register new value */
    config->winEnable = false;
    config->intEnable = false;
    config->updateEnable = true;
    config->apbErrorResetEnable = false;
    config->opMode.debug = false;
    config->opMode.deepsleep = false;
    /* Construct TOVAL register new value */
    config->timeoutValue = FEATURE_WDG_TOVR_RESET_VALUE;
    /* Construct WIN register new value */
    config->windowValue = FEATURE_WDG_WVR_RESET_VALUE;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_SetInt
 * Description   : enable/disable the WDG timeout interrupt
 *
 * Implements    : WDG_DRV_SetInt_Activity
 *END**************************************************************************/
status_t WDG_DRV_SetInt(uint32_t instance, bool enable)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    /* If allowed reconfigures WDG */
    if (WDG_IsUpdateEnabled(base))
    {
        /* Disable global interrupt */
        INT_SYS_DisableIRQGlobal();
        /* Enable/disable WDG timeout interrupt */
        WDG_SetInt(base, enable);
        /* Enable global interrupt */
        INT_SYS_EnableIRQGlobal();
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_ClearIntFlag
 * Description   : Clear interrupt flag of the WDG
 *
 * Implements    : WDG_DRV_ClearIntFlag_Activity
 *END**************************************************************************/
void WDG_DRV_ClearIntFlag(uint32_t instance)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];

    /* Clear interrupt flag of the WDG */
    WDG_ClearIntFlag(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_Trigger
 * Description   : Refreshes the WDG counter
 *
 * Implements    : WDG_DRV_Trigger_Activity
 *END**************************************************************************/
void WDG_DRV_Trigger(uint32_t instance)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];

    WDG_Trigger(base);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_GetCounter
 * Description   : Get the value of the WDG counter.
 *
 * Implements    : WDG_DRV_GetCounter_Activity
 *END**************************************************************************/
uint32_t WDG_DRV_GetCounter(uint32_t instance)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    const WDG_Type *base = s_wdgBase[instance];

    return (uint32_t)base->CNTCVR;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_SetWindow
 * Description   : Set window mode and window value of the WDG.
 *
 * Implements    : WDG_DRV_SetWindow_Activity
 *END**************************************************************************/
status_t WDG_DRV_SetWindow(uint32_t instance, bool enable, uint32_t windowValue)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    /* If allowed reconfigures WDG */
    if (WDG_IsUpdateEnabled(base))
    {
        /* Set WDG window mode */
        WDG_SetWindowMode(base, enable);

        /* If enable window mode */
        if (enable)
        {
            /* Set window value for the WDG */
            WDG_SetWindowValue(base, windowValue);
        }
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_SetMode
 * Description   : Set mode operation of the WDG.
 *
 * Implements    : WDG_DRV_SetMode_Activity
 *END**************************************************************************/
status_t WDG_DRV_SetMode(uint32_t instance, bool enable, wdg_set_mode_t setMode)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    /* If allowed reconfigures WDG */
    if (WDG_IsUpdateEnabled(base))
    {
        switch (setMode)
        {
            case WDG_DEBUG_MODE:
                /* Set WDG debug mode */
                WDG_SetDebug(base, enable);
                break;
            case WDG_DEEPSLEEP_MODE:
                /* Set WDG Deep Sleep mode */
                WDG_SetDeepSleep(base, enable);
                break;
            default:
                /* Do nothings */
                break;
        }
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : WDG_DRV_SetTimeout
 * Description   : Set time value of the WDG timeout.
 *
 * Implements    : WDG_DRV_SetTimeout_Activity
 *END**************************************************************************/
status_t WDG_DRV_SetTimeout(uint32_t instance, uint32_t timeout)
{
    DEV_ASSERT(instance < WDG_INSTANCE_COUNT);
    WDG_Type *base = s_wdgBase[instance];
    status_t status = STATUS_SUCCESS;

    /* If allowed reconfigures WDG */
    if (WDG_IsUpdateEnabled(base))
    {
        WDG_UNLOCK(base);

        base->TOVR = timeout;
    }
    else
    {
        status = STATUS_ERROR;
    }

    return status;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
