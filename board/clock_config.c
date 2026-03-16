/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file clock_config.c
 * @brief 
 * 
 */

#include "stddef.h"
#include "clock_config.h"

/*! @brief peripheral clock PeripheralClockConfig */

peripheral_clock_config_t clock_config0PeripheralClockConfig[12] = {
    {
        .clkName = DMA_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = EFM_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = GPIO_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = LINFlexD1_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = FlexCAN0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = ADC0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = ADC1_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = ACMP0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = eTMR0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = eTMR1_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = pTMR0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_DISABLED,
    },
    {
        .clkName = lpTMR0_CLK,
        .clkGate = true,
        .divider = DIV_BY_1,
        .clkSrc = CLK_SRC_SIRC,
    },
};

const scu_config_t clock_config0ScuConfig = {
    .fircEnable = true,
    .fircDeepSleepEnable = false,
    .sircDeepSleepEnable = false,
    .sircStandbyEnable = false,
    .sircPowerDownEnable = false,
    .sysClkSrc = SCU_SYSTEM_CLOCK_SRC_PLL,
    .fxoscConfig =
        {
            .enable = true,
            .bypassMode = false,
            .gainSelection = 6,
            .frequency = 24000000U,
        },
    .sxoscConfig = 
        {
            .enable = false,
            .gainSelection = 0,
            .bypassMode = false,
            .deepsleepEnable = false,
            .standbyEnable = false,
            .powerdownEnable = false,
            .frequency = 32768U,
        },
    .pllConfig =
        {
            .enable = true,
            .pllRefClock = SCU_PLL_REF_FXOSC_CLK,
            .pllFeedBackDiv = 10,
            .pllRefClkDiv = 1,
        },
    .sysDiv = SCU_SYS_CLK_DIV_BY_1,
    .fastBusDiv = SCU_SYS_CLK_DIV_BY_1,
    .slowBusDiv = SCU_SYS_CLK_DIV_BY_3,
    .flashDiv = SCU_SYS_CLK_DIV_BY_4,
    .clockOutConfig =
        {
            .enable = false,
            .source = SCU_CLKOUT_SEL_PLL_CLK,
            .divider = 1
        },
};

const cmu_config_t clock_config0CmuConfig = {
    .slowBusMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,              
        .compareHigh = (50 * 128 / 12),        
        .compareLow = (30 * 128 / 12),       
    },
    .fircClockMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,              
        .compareHigh = (120 * 128 / 12),        
        .compareLow = (72 * 128 / 12),         
    },
    .pllClockMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,              
        .compareHigh = (150 * 128 / 12),        
        .compareLow = (90 * 128 / 12),         
    },
    .fxoscClockMonitor={
        .enable = true,                 
        .resetEnable = true,           
        .refClock = CMU_REF_SIRC_CLOCK,              
        .compareHigh = (30 * 128 / 12),        
        .compareLow = (18 * 128 / 12),         
    },
};

/*! @brief User Configuration structure clock_config0ClockManager */
clock_manager_user_config_t clock_config0ClockManager = {
    .scuConfigPtr = &clock_config0ScuConfig,
    .cmuConfigPtr = &clock_config0CmuConfig,
    .ipcConfig =
        {
            .peripheralClocks = clock_config0PeripheralClockConfig,
            .count = 12,
        },
};

/*! @brief Array of pointers to User configuration structures */
clock_manager_user_config_t const *g_clockManConfigsArr[] = {
    &clock_config0ClockManager,
};

/*! @brief Array of pointers to User defined Callbacks configuration structures */
/* The tool do not support generate Callbacks configuration. It's always empty. */
clock_manager_callback_user_config_t *g_clockManCallbacksArr[] = {(void *)0};
