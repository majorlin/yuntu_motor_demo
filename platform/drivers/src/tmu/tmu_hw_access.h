/*
 * Copyright (c) 2021-2025 Yuntu Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*! 
 * @file tmu_hw_access.h
 * @version 1.4.0
 */

#ifndef TMU_HW_ACCESS_H
#define TMU_HW_ACCESS_H

#include "tmu_driver.h"

/*!
 * tmu_hw_access TMU Hardware Access
 * @details This section describes the programming interface of the TMU Hardware Access.
 * @{
 */

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
* API
*******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Restore the TMU module to reset value.
 *
 * This function restores the TMU module to reset value.
 *
 * @param[in] base   The TMU peripheral base address
 * @return           Execution status:
 *   STATUS_SUCCESS
 *   STATUS_ERROR    If at least one of the target module register is locked.
 */
status_t TMU_Init(TMU_Type *const base);

/*!
 * @brief Configures a source trigger for a target module.
 *
 * This function configures a TMU link between a source trigger and a target module,
 * if the requested target module is not locked.
 *
 * @param[in] base           The TMU peripheral base address
 * @param[in] triggerSource  One of the values in the tmu_trigger_source_t enumeration
 * @param[in] targetModule   One of the values in the tmu_target_module_t enumeration
 */
void TMU_SetTrigSourceForTargetModule(TMU_Type *const base,
                                      const tmu_trigger_source_t triggerSource,
                                      const tmu_target_module_t targetModule);

/*!
 * @brief Get the source trigger configured for a target module.
 *
 * This function returns the TMU source trigger linked to a selected target module.
 *
 * @param[in] base         The TMU peripheral base address
 * @param[in] targetModule One of the values in the tmu_target_module_t enumeration
 * @return                 Enum value corresponding to the trigger source configured for the given target module
 */
tmu_trigger_source_t TMU_GetTrigSourceForTargetModule(const TMU_Type *const base,
                                                      const tmu_target_module_t targetModule);

/*!
 * @brief Set lock bit for target module
 *
 * This function set the lock bit for target module
 * 
 * @param[in] base              The TMU peripheral base address
 * @param[in] targetModule      One of the values in the tmu_target_module_t enumeration
 */
void TMU_SetLockForTargetModule(TMU_Type *const base,
                                const tmu_target_module_t targetModule);

/*!
 * @brief Get the lock bit status of the TMU register of a target module.
 *
 * This function get the lock bit status of the TMU register of a target module.
 *
 * @param[in] base         The TMU peripheral base address
 * @param[in] targetModule One of the values in the tmu_target_module_t enumeration
 * @return                 true or false depending on the state of the lock bit
 */
bool TMU_GetLockForTargetModule(const TMU_Type *const base,
                                const tmu_target_module_t targetModule);

#if defined(__cplusplus)
}
#endif

/*! @} */

#endif /* TMU_HW_ACCESS_H */
/*******************************************************************************
* EOF
*******************************************************************************/
