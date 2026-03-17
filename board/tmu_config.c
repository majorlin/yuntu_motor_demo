/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file tmu_config.c
 * @brief 
 * 
 */


#include "tmu_config.h"

const tmu_inout_mapping_config_t inOutMappingConfig[1]={
    {
        .triggerSource = TMU_TRIG_SOURCE_eTMR0_EXT_TRIG,
        .targetModule = TMU_TARGET_MODULE_ADC0_EXT_TRIG,
    },
};

const tmu_user_config_t tmu_config0={
    .numInOutMappingConfigs=1,
    .inOutMappingConfig=inOutMappingConfig,
};
