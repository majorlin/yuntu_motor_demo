/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file tmu_driver.c
 * @version 1.4.0
 */

#include <stddef.h>
#include "tmu_driver.h"
#include "tmu_hw_access.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*! @brief Table of base addresses for TMU instances. */
static TMU_Type *const s_tmuBase[TMU_INSTANCE_COUNT] = TMU_BASE_PTRS;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name :     TMU_DRV_Init
 * Description   :     This function first resets the source triggers of all TMU target modules
 * to their default values, then configures the TMU with all the user defined in-out mappings.
 * If at least one of the target modules is locked, the function will not change any of the
 * TMU target modules and return an error code.
 * This example shows how to set up the tmu_user_config_t parameters and how to call the
 * TMU_DRV_Init() function with the required parameters:
 *   tmu_user_config_t             tmuConfig;
 *   tmu_inout_mapping_config_t    tmuInoutMappingConfig[] =
 *   {
 *      {TMU_TRIG_SOURCE_TMU_IN9,     TMU_TARGET_MODULE_DMA_CH0,     false},
 *      {TMU_TRIG_SOURCE_FTM1_EXT_TRIG,  TMU_TARGET_MODULE_TMU_OUT4, true}
 *   };
 *   tmuConfig.numInOutMappingConfigs = 2;
 *   tmuConfig.inOutMappingConfig     = tmuInoutMappingConfig;
 *   TMU_DRV_Init(instance, &tmuConfig);
 *
 * Implements    :     TMU_DRV_Init_Activity
 *END**************************************************************************/
status_t TMU_DRV_Init(const uint32_t instance,
                      const tmu_user_config_t *const tmuUserConfig)
{
    DEV_ASSERT(instance < TMU_INSTANCE_COUNT);
    DEV_ASSERT(tmuUserConfig != NULL);

    status_t status;
    TMU_Type *base = s_tmuBase[instance];
    uint8_t count;

    /* Reset source triggers of all TMU target modules to default. */
    status = TMU_Init(base);

    if (status == STATUS_SUCCESS)
    {
        /* Loop through all in-out mappings in the configuration and apply them in TMU */
        for (count = 0U; count < tmuUserConfig->numInOutMappingConfigs; count++)
        {
            TMU_SetTrigSourceForTargetModule(base, tmuUserConfig->inOutMappingConfig[count].triggerSource,
                                             tmuUserConfig->inOutMappingConfig[count].targetModule);
        }
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_DRV_Deinit
 * Description   : Reset to default values the source triggers corresponding to all target modules,
 * if none of the target modules is locked.
 *
 * Implements    : TMU_DRV_Deinit_Activity
 *END**************************************************************************/
status_t TMU_DRV_Deinit(const uint32_t instance)
{
    DEV_ASSERT(instance < TMU_INSTANCE_COUNT);

    TMU_Type *base = s_tmuBase[instance];
    status_t status;

    /* Reset source triggers of all TMU target modules to default. */
    status = TMU_Init(base);

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_DRV_SetTrigSourceForTargetModule
 * Description   : This function configures a TMU link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * Implements    : TMU_DRV_SetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
status_t TMU_DRV_SetTrigSourceForTargetModule(const uint32_t instance,
                                              const tmu_trigger_source_t triggerSource,
                                              const tmu_target_module_t targetModule)
{
    DEV_ASSERT(instance < TMU_INSTANCE_COUNT);

    TMU_Type *base = s_tmuBase[instance];
    status_t status;

    /* Configure link between trigger source and target module. */
    TMU_SetTrigSourceForTargetModule(base, triggerSource, targetModule);

    status = STATUS_SUCCESS;

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_DRV_GetTrigSourceForTargetModule
 * Description   : This function returns the TMU source trigger linked to a selected target module.
 *
 * Implements    : TMU_DRV_GetTrigSourceForTargetModule_Activity
 *END**************************************************************************/
tmu_trigger_source_t TMU_DRV_GetTrigSourceForTargetModule(const uint32_t instance,
                                                          const tmu_target_module_t targetModule)
{
    DEV_ASSERT(instance < TMU_INSTANCE_COUNT);

    const TMU_Type *base = s_tmuBase[instance];

    return TMU_GetTrigSourceForTargetModule(base, targetModule);
}


/*******************************************************************************
 * EOF
 ******************************************************************************/
