/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file ltmr_config.c
 * @brief 
 * 
 */


#include "lptmr_config.h"

const lptmr_config_t LPTMR_Config={
    .dmaRequest=false,
    .interruptEnable=false,
    .freeRun=false,
    .workMode=lpTMR_WORKMODE_TIMER,
    .prescaler=lpTMR_PRESCALE_2,
    .bypassPrescaler=false,
    .compareValue=65535,
    .counterUnits=lpTMR_COUNTER_UNITS_TICKS,
    .pinSelect=lpTMR_PINSELECT_TMU,
    .pinPolarity=lpTMR_PINPOLARITY_RISING,
    .clockSource=lpTMR_CLOCK_SOURCE_IPC,
};

