/*
 * Copyright 2020-2022 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
 
/*! 
 * @file tmu_driver.h
 * @version 1.4.0
 */

#ifndef TMU_DRIVER_H
#define TMU_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "status.h"
#include "device_registers.h"

/*!
 * @defgroup tmu_driver Tmu Driver
 * @ingroup tmu
 * @{
 */

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* TMU module features */
/*!
 * @brief Enumeration for trigger source module of TMU
 *
 * Describes all possible inputs (trigger sources) of the TMU IP
 * This enumeration depends on the supported instances in device
 *
 * Implements : tmu_trigger_source_t_Class
 */
typedef enum tmu_trigger_source_e tmu_trigger_source_t;

/*!
 * @brief Enumeration for target module of TMU
 *
 * Describes all possible outputs (target modules) of the TMU IP
 * This enumeration depends on the supported instances in device
 *
 * Implements : tmu_target_module_t_Class
 */
typedef enum tmu_target_module_e tmu_target_module_t;

/*!
 * @brief Configuration structure for pairing source triggers with target modules.
 *
 * Use an instance of this structure to define a TMU link between a trigger source and a target module.
 * This structure is used by the user configuration structure.
 *
 * Implements : tmu_inout_mapping_config_t_Class
 */
typedef struct
{
    tmu_trigger_source_t triggerSource; /*!< selects one of the TMU trigger sources */
    tmu_target_module_t targetModule;   /*!< selects one of the TMU target modules  */
} tmu_inout_mapping_config_t;

/*!
 * @brief User configuration structure for the TMU driver.
 *
 * Use an instance of this structure with the TMU_DRV_Init() function. This enables configuration of TMU with the user
 * defined mappings between inputs (source triggers) and outputs (target modules), via a single function call.
 *
 * Implements : tmu_user_config_t_Class
 */
typedef struct
{
    uint8_t numInOutMappingConfigs;                           /*!< number of in-out mappings defined in TMU configuration */
    const tmu_inout_mapping_config_t *inOutMappingConfig; /*!< pointer to array of in-out mapping structures */
} tmu_user_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif


/*!
 * @brief Initialize a TMU instance for operation.
 *
 * This function first resets the source triggers of all TMU target modules to their default values,
 * then configures the TMU with all the user defined in-out mappings.
 * If at least one of the target modules is locked, the function will not change any of the TMU target modules
 * and return error code.
 * This example shows how to set up the tmu_user_config_t parameters and how to call the
 * TMU_DRV_Init() function with the required parameters:
 *  @code
 *   tmu_user_config_t             tmuConfig;
 *   tmu_inout_mapping_config_t    tmuInoutMappingConfig[] =
 *   {
 *       {TMU_TRIG_SOURCE_TMU_IN9,     TMU_TARGET_MODULE_DMA_CH0,     false},
 *       {TMU_TRIG_SOURCE_FTM1_EXT_TRIG,  TMU_TARGET_MODULE_TMU_OUT4, true}
 *   };
 *
 *   tmuConfig.numInOutMappingConfigs = 2;
 *   tmuConfig.inOutMappingConfig     = tmuInoutMappingConfig;
 *
 *   TMU_DRV_Init(instance, &tmuConfig);
 *   @endcode
 *
 * @param[in] instance          The TMU instance number.
 * @param[in] tmuUserConfig  Pointer to the user configuration structure.
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if at least one of the target module register is locked.  */
status_t TMU_DRV_Init(const uint32_t instance,  const tmu_user_config_t *const tmuUserConfig);

/*!
 * @brief Reset to default values the source triggers corresponding to all target modules,
 * if none of the target modules is locked.
 *
 * @param[in] instance          The TMU instance number.
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if at least one of the target module register is locked.
 */
status_t TMU_DRV_Deinit(const uint32_t instance);

/*!
 * @brief Configure a source trigger for a selected target module.
 *
 * This function configures a TMU link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * @param[in] instance          The TMU instance number.
 * @param[in] triggerSource     One of the values in the tmu_trigger_source_t enumeration
 * @param[in] targetModule      One of the values in the tmu_target_module_t enumeration
 * @return                      Execution status: \n
 *   STATUS_SUCCESS \n
 *   STATUS_ERROR    - if requested target module is locked  */
status_t TMU_DRV_SetTrigSourceForTargetModule(const uint32_t instance,
                                              const tmu_trigger_source_t triggerSource,
                                              const tmu_target_module_t targetModule);

/*!
 * @brief Get the source trigger configured for a target module.
 *
 * This function returns the TMU source trigger linked to a selected target module.
 *
 * @param[in] instance      The TMU instance number.
 * @param[in] targetModule  One of the values in the tmu_target_module_t enumeration.
 * @return                  Enum value corresponding to the trigger source configured
 *                          for the selected target module.
 */
tmu_trigger_source_t TMU_DRV_GetTrigSourceForTargetModule(const uint32_t instance,
                                                          const tmu_target_module_t targetModule);


#if defined(__cplusplus)
}
#endif

/*! @}*/

#endif /* TMU_DRIVER_H */
/*******************************************************************************
 * EOF
 ******************************************************************************/
