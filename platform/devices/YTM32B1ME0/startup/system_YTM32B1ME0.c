/*
 * Copyright (c) 2021-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file system_YTM32B1ME0.c
 * @version 1.4.0
 */

/*!
 * @page misra_violations MISRA-C:2012 violations list
 *
 * PRQA S 2016 Rule 16.4: This 'switch' statement 'default' clause is empty.
 *
 * PRQA S 1533 Rule 8.9: The object '%1s' is only referenced by function '%2s'. 
 *
 * PRQA S 3493 Rule 14.3: The first operand of this conditional operator is always constant 'true'.
 */

#include "device_registers.h"
#include "system_YTM32B1ME0.h"
#include "stdbool.h"

/* ----------------------------------------------------------------------------
   -- Core clock
   ---------------------------------------------------------------------------- */

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK; /*PRQA S 1533*/

#ifndef EFM_CTRL_WE_MASK
/*FUNCTION**********************************************************************
 *
 * Function Name : EfmInitMpu
 * Description   : This function initializes the MPU to protect the 
 * all flash memory regions. Only internal use.
 *
 * Implements    : EfmInitMpu_Activity
 *END**************************************************************************/
static void EfmInitMpu(void)
{
    ARM_MPU_Region_t mpuTable[1] = {
#if defined(FEATURE_EFM_HAS_DATA_FLASH) && (FEATURE_EFM_HAS_DATA_FLASH == 1)
        //                     BASE          SH              RO   NP   XN                         LIMIT                               ATTR 
        { .RBAR = ARM_MPU_RBAR(0x00000000UL, ARM_MPU_SH_NON, 1UL, 1UL, 0UL), .RLAR = ARM_MPU_RLAR(FEATURE_EFM_DATA_ARRAY_END_ADDRESS, 0UL) },
#else
        //                     BASE          SH              RO   NP   XN                         LIMIT                               ATTR 
        { .RBAR = ARM_MPU_RBAR(0x00000000UL, ARM_MPU_SH_NON, 1UL, 1UL, 0UL), .RLAR = ARM_MPU_RLAR(FEATURE_EFM_MAIN_ARRAY_END_ADDRESS, 0UL) },
#endif
    };

    ARM_MPU_SetMemAttr(0, ARM_MPU_ATTR(           /* Normal memory */ /*PRQA S 3493*/
        ARM_MPU_ATTR_MEMORY_(1U, 0U, 1U, 0U),   /* Outer Write-Through non-transient with read allocate and write not allocate */
        ARM_MPU_ATTR_MEMORY_(1U, 0U, 1U, 0U)    /* Inner Write-Through non-transient with read allocate and write not allocate */
    ));
    ARM_MPU_Load(0U, mpuTable, 1);
    ARM_MPU_Enable(MPU_CTRL_PRIVDEFENA_Msk);
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemInit
 * Description   : This function disables the watchdog, enables FPU
 * and the power mode protection if the corresponding feature macro
 * is enabled. SystemInit is called from startup_device file.
 *
 * Implements    : SystemInit_Activity
 *END**************************************************************************/
void SystemInit(void)
{
/**************************************************************************/
                      /* FPU ENABLE*/
/**************************************************************************/
#ifdef ENABLE_FPU
  /* Enable CP10 and CP11 coprocessors */
  SCB->CPACR |= (3UL << 20 | 3UL << 22);
#endif /* ENABLE_FPU */
/**************************************************************************/
/* EFM CONTRROL */
/**************************************************************************/
    EFM->CTRL |= EFM_CTRL_DPD_EN_MASK | EFM_CTRL_PREFETCH_EN_MASK;
#ifndef EFM_CTRL_WE_MASK
    EfmInitMpu();
#endif
/**************************************************************************/
/* RCU CONTROL */
/**************************************************************************/
    CIM->CTRL |= CIM_CTRL_LOCKUPEN_MASK;
/**************************************************************************/
/* WDOG DISABLE*/
/**************************************************************************/
#if (DISABLE_WDOG)
    WDG0->SVCR = 0xB631;
    WDG0->SVCR = 0xC278;
    WDG0->CR &= ~WDG_CR_EN_MASK;
#endif /* (DISABLE_WDOG) */
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemCoreClockUpdate
 * Description   : This function must be called whenever the core clock is changed
 * during program execution. It evaluates the clock register settings and calculates
 * the current core clock.
 *
 * Implements    : SystemCoreClockUpdate_Activity
 *END**************************************************************************/
void SystemCoreClockUpdate(void)
{
    uint32_t refClkFreq;
    uint32_t feedbackDiv;
    uint32_t referenceDiv;
    switch ((SCU->STS & SCU_STS_CLKST_MASK) >> SCU_STS_CLKST_SHIFT)
    {
        case 0x0:
            /* FIRC */
            SystemCoreClock = FEATURE_SCU_FIRC_FREQ;
            break;
#if defined(FEATURE_SCU_SUPPORT_PLL) && (FEATURE_SCU_SUPPORT_PLL)
        case 0x1:
            /* PLL */
            if (SCU_PLL_CTRL_REFCLKSRCSEL_MASK == (SCU->PLL_CTRL & SCU_PLL_CTRL_REFCLKSRCSEL_MASK))
            {
                /* Select FIRC as PLL reference clock */
                refClkFreq = FEATURE_SCU_FIRC_FREQ;
            }
            else
            {
                /* Select FXOSC as PLL reference clock */
                refClkFreq = FEATURE_SCU_FXOSC_FREQ;
            }
            /* Fout = (Fref * Ndiv) / ( 2 * Npre) */
            feedbackDiv = ((SCU->PLL_CTRL & SCU_PLL_CTRL_FBDIV_MASK) >> SCU_PLL_CTRL_FBDIV_SHIFT) + 1U;
            referenceDiv = ((SCU->PLL_CTRL & SCU_PLL_CTRL_REFDIV_MASK) >> SCU_PLL_CTRL_REFDIV_SHIFT) + 1U;
            SystemCoreClock = ((refClkFreq * feedbackDiv) / referenceDiv) >> 1;
            break;
#endif /* FEATURE_SCU_SUPPORT_PLL */
        case 0x2:
            /* FXOSC */
            SystemCoreClock = FEATURE_SCU_FXOSC_FREQ;
            break;
        case 0x3:
            /* SIRC */
            SystemCoreClock = FEATURE_SCU_SIRC_FREQ;
            break;
        default: /*PRQA S 2016*/
            break;
    }
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemSoftwareReset
 * Description   : This function is used to initiate a system reset
 *
 * Implements    : SystemSoftwareReset_Activity
 *END**************************************************************************/
void SystemSoftwareReset(void)
{
    NVIC_SystemReset();
}

/*FUNCTION**********************************************************************
 *
 * Function Name : SystemGetUid
 * Description   : This function is only used to get chip unit ID from reserved flash area.
 *
 * Implements    : SystemGetUid_Activity
 *END**************************************************************************/
void SystemGetUid(uint32_t *uidh, uint32_t *uidl)
{
    /* UID store in CIM */
    *uidh  = ((CIM->UID0 >>  0U) & 0xFU) <<  0U;
    *uidh |= ((CIM->UID0 >>  8U) & 0xFU) <<  4U;
    *uidh |= ((CIM->UID0 >> 16U) & 0xFU) <<  8U;
    *uidh |= ((CIM->UID0 >> 24U) & 0xFU) << 12U;
    *uidh |= ((CIM->UID1 >>  0U) & 0xFU) << 16U;
    *uidh |= ((CIM->UID1 >>  8U) & 0xFFU) << 20U;
    
    *uidl  = ((CIM->UID2 >>  0U) & 0xFFFFU) <<  0U;
    *uidl |= ((CIM->UID3 >>  0U) & 0xFFFFU) << 16U;
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
