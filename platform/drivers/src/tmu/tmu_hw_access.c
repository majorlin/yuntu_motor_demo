/*
 * Copyright (c) 2021-2025 Yuntu Microelectronics Co., Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */
 
/*!
 * @file tmu_hw_access.c
 * @version 1.4.0
 */

/*!
 * @page misra_violations MISRA-C:2012 violations list
 *
 * PRQA S 2985 Rule 2.2: This operation is redundant. The value of the result is 
 *                       always that of the left-hand operand.
 */

#include <stddef.h>
#include "tmu_hw_access.h"

/*******************************************************************************
* Definitions
*******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_Init
 * Description   : This function restores the TMU module to reset value.
 *
 *END**************************************************************************/
status_t TMU_Init(TMU_Type *const base)
{
    DEV_ASSERT(base != NULL);

    status_t status = STATUS_SUCCESS;

    /* unlock all registers */
    for (uint8_t i = 0; i < TMU_MUX_COUNT; i++)
    {
        base->MUX[i] = 0;
    }

    return status;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_SetTrigSourceForTargetModule
 * Description   : This function configures a TMU link between a source trigger
 * and a target module, if the requested target module is not locked.
 *
 *END**************************************************************************/
void TMU_SetTrigSourceForTargetModule(TMU_Type *const base,
                                      const tmu_trigger_source_t triggerSource,
                                      const tmu_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(triggerSource < (1 << TMU_MUX_SEL0_WIDTH));
    DEV_ASSERT(targetModule < (TMU_MUX_COUNT * 4));

    uint8_t muxCnt = 0U;
    uint8_t selCnt = 0U;
    uint32_t tmpReg;

    muxCnt = (uint8_t)targetModule >> 2U;
    selCnt = (uint8_t)targetModule & 0x3U;

    /* Read value of entire TMU register in a temp variable */
    tmpReg = base->MUX[muxCnt];
    switch (selCnt)
    {
        case (0x0U):
            tmpReg &= ~TMU_MUX_SEL0_MASK;
            tmpReg |= TMU_MUX_SEL0(triggerSource); /*PRQA S 2985*/
            break;
        case (0x1U):
            tmpReg &= ~TMU_MUX_SEL1_MASK;
            tmpReg |= TMU_MUX_SEL1(triggerSource);
            break;
        case (0x2U):
            tmpReg &= ~TMU_MUX_SEL2_MASK;
            tmpReg |= TMU_MUX_SEL2(triggerSource);
            break;
        case (0x3U):
            tmpReg &= ~TMU_MUX_SEL3_MASK;
            tmpReg |= TMU_MUX_SEL3(triggerSource);
            break;
        default:
            /* Nothing to do */
            break;
    }
    /* Write back the TMU MUX register */
    base->MUX[muxCnt] = tmpReg;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_GetTrigSourceForTargetModule
 * Description   : This function returns the TMU source trigger linked to
 * a selected target module.
 *
 *END**************************************************************************/
tmu_trigger_source_t TMU_GetTrigSourceForTargetModule(const TMU_Type *const base,
                                                      const tmu_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(targetModule < (TMU_MUX_COUNT * 4));

    uint8_t muxCnt = 0U;
    uint8_t selCnt = 0U;
    uint8_t trigSource;

    muxCnt = (uint8_t)targetModule / 4U;
    selCnt = (uint8_t)targetModule % 4U;

    if (selCnt == 0U)
    {
        /* Perform the update operation */
        trigSource = (uint8_t)((base->MUX[muxCnt] >> TMU_MUX_SEL0_SHIFT) & TMU_MUX_SEL0_MASK);
    } else if (selCnt == 1U)
    {
        /* Perform the update operation */
        trigSource = (uint8_t)((base->MUX[muxCnt] >> TMU_MUX_SEL1_SHIFT) & TMU_MUX_SEL1_MASK);
    } else if (selCnt == 2U)
    {
        /* Perform the update operation */
        trigSource = (uint8_t)((base->MUX[muxCnt] >> TMU_MUX_SEL2_SHIFT) & TMU_MUX_SEL2_MASK);
    } else
    {
        /* Perform the update operation */
        trigSource = (uint8_t)((base->MUX[muxCnt] >> TMU_MUX_SEL3_SHIFT) & TMU_MUX_SEL3_MASK);
    }

    return (tmu_trigger_source_t) (trigSource);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_SetLockForTargetModule
 * Description   : This function sets the Lock bit of the TMU register corresponding
 * to the selected target module.
 *
 *END**************************************************************************/
void TMU_SetLockForTargetModule(TMU_Type *const base,
                                const tmu_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(targetModule < (TMU_MUX_COUNT * 4));

    uint8_t muxCnt = 0;

    muxCnt = (uint8_t)targetModule / 4U;

    /* Perform the update operation */
    base->MUX[muxCnt] |= (((uint32_t) 1U) << TMU_MUX_LOCK_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : TMU_GetLockForTargetModule
 * Description   : Get the lock bit status of the TMU register of a target module.
 *
 *END**************************************************************************/
bool TMU_GetLockForTargetModule(const TMU_Type *const base,
                                const tmu_target_module_t targetModule)
{
    DEV_ASSERT(base != NULL);
    DEV_ASSERT(targetModule < (TMU_MUX_COUNT * 4));

    uint8_t muxCnt;
    uint32_t lockVal;
    bool isLock;

    muxCnt = (uint8_t)targetModule / 4U;

    /* Get the lock bit value */
    lockVal = ((base->MUX[muxCnt] & TMU_MUX_LOCK_MASK) >> TMU_MUX_LOCK_SHIFT);

    isLock = (lockVal == 0U) ? false : true;

    return isLock;
}

/*******************************************************************************
* EOF
*******************************************************************************/
