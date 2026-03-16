/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file etmr_hw_access.c
 * @version 1.4.0
 */

/*!
 * @page misra_violations MISRA-C:2012 violations list
 *
 * PRQA S 2985 Rule 2.2: This operation is redundant. The value of the result is
 *                       always that of the left-hand operand.
 * PRQA S 2986 Rule 2.2: This operation is redundant. The value of the result is
 *                       always that of the right-hand operand.
 *
 */

#include "etmr_hw_access.h"


#if defined(FEATURE_eTMR_HAS_HARDWARE_CAPTURE) && (FEATURE_eTMR_HAS_HARDWARE_CAPTURE == 1U)
/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetChnNegPulseRdyFlag
 * Description   : Get channel negative pulse ready flag
 *
 * Implements    : eTMR_DRV_GetChnNegPulseRdyFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetChnNegPulseRdyFlag(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return ((etmrBase->STS & (eTMR_STS_CH0NPF_MASK << channel)) >> (eTMR_STS_CH0NPF_SHIFT + channel)) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetChnPosPulseRdyFlag
 * Description   : Get the channel positive pulse ready flag
 *
 * Implements    : eTMR_DRV_GetChnPosPulseRdyFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetChnPosPulseRdyFlag(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return ((etmrBase->STS & (eTMR_STS_CH0PPF_MASK << channel)) >> (eTMR_STS_CH0PPF_SHIFT + channel)) != 0U;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetQuadDecodeDir
 * Description   : Get quadrature decoder direction
 *
 * Implements    : eTMR_DRV_GetQuadDecodeDir_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetQuadDecodeDir(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (((etmrBase->STS) & eTMR_STS_QDDIR_MASK) >> eTMR_STS_QDDIR_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetQuadDecodeTofFlag
 * Description   : Get quadrature decoder counter overflow flag
 *
 * Implements    : eTMR_DRV_GetQuadDecodeTofFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetQuadDecodeTofFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (((etmrBase->STS) & eTMR_STS_QDTOF_MASK) >> eTMR_STS_QDTOF_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetTofFlag
 * Description   : Get eTMR counter overflow flag
 *
 * Implements    : eTMR_DRV_GetTofFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetTofFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (((etmrBase->STS) & eTMR_STS_TOF_MASK) >> eTMR_STS_TOF_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetReloadFlag
 * Description   : Get eTMR reload flag
 *
 * Implements    : eTMR_DRV_GetReloadFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetReloadFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (((etmrBase->STS) & eTMR_STS_RF_MASK) >> eTMR_STS_RF_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetFaultFlag
 * Description   : Get eTMR fault channel flag
 *
 * Implements    : eTMR_DRV_GetFaultFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetFaultFlag(uint32_t instance, uint8_t fltChannel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(fltChannel < (FEATURE_eTMR_CHANNEL_MAX_COUNT >> 1));
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return ((etmrBase->STS & ((uint32_t)eTMR_STS_F0F_MASK << fltChannel)) >> (eTMR_STS_F0F_SHIFT + fltChannel)) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetChnFlag
 * Description   : Get eTMR channel interrupt flag
 *
 * Implements    : eTMR_DRV_GetChnFlag_Activity
 *END**************************************************************************/
bool eTMR_DRV_GetChnFlag(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (((etmrBase->STS) & ((uint32_t)0x1U << channel)) >> channel) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearChnNegPulseRdyFlag
 * Description   : Clear channel negative pulse ready flag
 *
 * Implements    : eTMR_DRV_ClearChnNegPulseRdyFlag_Activity
 *END**************************************************************************/
#if defined(FEATURE_eTMR_HAS_HARDWARE_CAPTURE) && (FEATURE_eTMR_HAS_HARDWARE_CAPTURE == 1U)
void eTMR_DRV_ClearChnNegPulseRdyFlag(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = eTMR_STS_CH0NPF_MASK << channel;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearChnPosPulseRdyFlag
 * Description   : Clear channel positive pulse ready flag
 *
 * Implements    : eTMR_DRV_ClearChnPosPulseRdyFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearChnPosPulseRdyFlag(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = eTMR_STS_CH0PPF_MASK << channel;
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearQuadDecodeTofFlag
 * Description   : Clear quadrature decoder counter overflow flag
 *
 * Implements    : eTMR_DRV_ClearQuadDecodeTofFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearQuadDecodeTofFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = eTMR_STS_QDTOF_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearTofFlag
 * Description   : Clear eTMR counter overflow flag
 *
 * Implements    : eTMR_DRV_ClearTofFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearTofFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = eTMR_STS_TOF_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearReloadFlag
 * Description   : Clear eTMR reload flag
 *
 * Implements    : eTMR_DRV_ClearReloadFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearReloadFlag(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = eTMR_STS_RF_MASK;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearFaultFlag
 * Description   : Clear fault channel flag
 *
 * Implements    : eTMR_DRV_ClearFaultFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearFaultFlag(uint32_t instance, uint8_t fltChannel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(fltChannel < FEATURE_eTMR_FAULT_CHANNELS);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = (uint32_t)eTMR_STS_F0F_MASK << fltChannel;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_ClearChnFlag
 * Description   : Clear channel interrupt flag
 *
 * Implements    : eTMR_DRV_ClearChnFlag_Activity
 *END**************************************************************************/
void eTMR_DRV_ClearChnFlag(uint32_t instance, uint8_t channelSet)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    etmrBase->STS = ((uint32_t)channelSet << eTMR_STS_CH0F_SHIFT); /*PRQA S 2985*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetPhaseBStatus
 * Description   : Get status of phase B in quadrature decode mode
 *
 * Implements    : eTMR_DRV_GetPhaseBStatus_Activity
 *END**************************************************************************/
uint8_t eTMR_DRV_GetPhaseBStatus(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (uint8_t)(((etmrBase->IOSTS) & eTMR_IOSTS_PHB_MASK) >> eTMR_IOSTS_PHB_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetPhaseAStatus
 * Description   : Get status of phase A in quadrature decode mode
 *
 * Implements    : eTMR_DRV_GetPhaseAStatus_Activity
 *END**************************************************************************/
uint8_t eTMR_DRV_GetPhaseAStatus(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (uint8_t)(((etmrBase->IOSTS) & eTMR_IOSTS_PHA_MASK) >> eTMR_IOSTS_PHA_SHIFT);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetFaultInputStatus
 * Description   : Get the fault input status(polarity)
 *
 * Implements    : eTMR_DRV_GetFaultInputStatus_Activity
 *END**************************************************************************/
uint8_t eTMR_DRV_GetFaultInputStatus(uint32_t instance, uint8_t fltChannel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(fltChannel < FEATURE_eTMR_FAULT_CHANNELS);
    eTMR_Type *const etmrBase = g_etmrBase[instance];
    return (uint8_t)((etmrBase->IOSTS & ((uint32_t)eTMR_IOSTS_F0_MASK << fltChannel)) >> (eTMR_IOSTS_F0_SHIFT + fltChannel));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetChnIoStatus
 * Description   : Get eTMR channel IO status in pwm, output compare and input
 *                 capture mode.
 *
 * Implements    : eTMR_DRV_GetChnIoStatus_Activity
 *END**************************************************************************/
uint8_t eTMR_DRV_GetChnIoStatus(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (uint8_t)((etmrBase->IOSTS >> channel) & eTMR_IOSTS_CH0IO_MASK);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnCompMode
 * Description   : Set channel as complementary mode
 *
 * Implements    : eTMR_DRV_SetChnCompMode_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnCompMode(uint32_t instance, uint8_t channelPair, etmr_pwm_mode_t mode)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channelPair < (FEATURE_eTMR_CHANNEL_MAX_COUNT >> 1));
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->CTRL &= ~(eTMR_CTRL_COMPL01_MASK << channelPair);
    etmrBase->CTRL |= ((uint32_t)mode << (eTMR_CTRL_COMPL01_SHIFT + channelPair));
}

#if FEATURE_eTMR_HAS_DOUBLE_SWITCH
/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnDoubleSwitch
 * Description   : Set channel as double switch mode
 *
 * Implements    : eTMR_DRV_SetChnDoubleSwitch_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnDoubleSwitch(uint32_t instance, uint8_t channelPair, bool en)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channelPair < (FEATURE_eTMR_CHANNEL_MAX_COUNT >> 1));
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->CTRL &= ~(eTMR_CTRL_DBSW01_MASK << channelPair);
    etmrBase->CTRL |= ((uint32_t)(en ? 1U : 0U) << (eTMR_CTRL_DBSW01_SHIFT + channelPair));
}
#endif

#if FEATURE_eTMR_HAS_COMBINATION_CAPTURE
/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnCombMode
 * Description   : Set channel as combination capture mode
 *
 * Implements    : eTMR_DRV_SetChnCombMode_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnCombMode(uint32_t instance, uint8_t channelPair, bool en)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channelPair < (FEATURE_eTMR_CHANNEL_MAX_COUNT >> 1));
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->CTRL &= ~(eTMR_CTRL_COMB01_MASK << channelPair);
    etmrBase->CTRL |= ((uint32_t)(en ? 1U : 0U) << (eTMR_CTRL_COMB01_SHIFT + channelPair));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnCombSrc
 * Description   : Set channel as combination capture source
 *
 * Implements    : eTMR_DRV_SetChnCombSrc_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnCombSrc(uint32_t instance, uint8_t channelPair, uint8_t src)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channelPair < (FEATURE_eTMR_CHANNEL_MAX_COUNT >> 1));
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->CTRL &= ~(eTMR_CTRL_COMBSRC01_MASK << channelPair);
    etmrBase->CTRL |= ((uint32_t)src << (eTMR_CTRL_COMBSRC01_SHIFT + channelPair));
}
#endif

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetFaultChnPol
 * Description   : Set fault channel input polarity
 *
 * Implements    : eTMR_DRV_SetFaultChnPol_Activity
 *END**************************************************************************/
void eTMR_DRV_SetFaultChnPol(uint32_t instance, uint8_t fltChannel, uint32_t pol)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(fltChannel < FEATURE_eTMR_FAULT_CHANNELS);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->FAULT &= ~((uint32_t)eTMR_FAULT_F0POL_MASK << fltChannel);
    etmrBase->FAULT |= (pol << (eTMR_FAULT_F0POL_SHIFT + fltChannel));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetFaultChnEnable
 * Description   : Set fault channel interrupt enable or not
 *
 * Implements    : eTMR_DRV_SetFaultChnEnable_Activity
 *END**************************************************************************/
void eTMR_DRV_SetFaultChnEnable(uint32_t instance, uint8_t fltChannel, bool enable)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(fltChannel < FEATURE_eTMR_FAULT_CHANNELS);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->FAULT &= ~((uint32_t)eTMR_FAULT_F0EN_MASK << fltChannel);
    etmrBase->FAULT |= ((uint32_t)(enable ? 1U : 0U) << (eTMR_FAULT_F0EN_SHIFT + fltChannel)); /*PRQA S 2986*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_EnableQuadDecodeTofInt
 * Description   : Enable quadrature decoder counter overflow interrupt.
 *
 * Implements    : eTMR_DRV_EnableQdTofInt_Activity
 *END**************************************************************************/
void eTMR_DRV_EnableQuadDecodeTofInt(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    eTMR_SetQdTofIntEnable(etmrBase, true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_DisableQuadDecodeTofInt
 * Description   : Disable quadrature decoder counter overflow interrupt
 *
 * Implements    : eTMR_DRV_DisableQuadDecodeTofInt_Activity
 *END**************************************************************************/
void eTMR_DRV_DisableQuadDecodeTofInt(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    eTMR_SetQdTofIntEnable(etmrBase, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_EnableTofInt
 * Description   : Enable eTMR overflow interrupt
 *
 * Implements    : eTMR_DRV_EnableTofInt_Activity
 *END**************************************************************************/
void eTMR_DRV_EnableTofInt(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    eTMR_SetTofIntEnable(etmrBase, true);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_DisableTofInt
 * Description   : Disable eTMR overflow interrupt
 *
 * Implements    : eTMR_DRV_DisableTofInt_Activity
 *END**************************************************************************/
void eTMR_DRV_DisableTofInt(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    eTMR_SetTofIntEnable(etmrBase, false);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_EnableChnInt
 * Description   : Enable specified channel interrupt.
 *
 * Implements    : eTMR_DRV_EnableChnInt_Activity
 *END**************************************************************************/
void eTMR_DRV_EnableChnInt(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->INTE &= ~((uint32_t)eTMR_INTE_CH0IE_MASK << channel);
    etmrBase->INTE |= ((uint32_t)0x1U << (eTMR_INTE_CH0IE_SHIFT + channel)); /*PRQA S 2986*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnsetIntEnable
 * Description   : Enable channel set interrupt.
 *
 * Implements    : eTMR_DRV_SetChnsetIntEnable_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnsetIntEnable(uint32_t instance, uint8_t channelSet)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->INTE |= ((uint32_t)channelSet << eTMR_INTE_CH0IE_SHIFT); /*PRQA S 2985*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_DisableChnInt
 * Description   : Disable specified channel interrupt.
 *
 * Implements    : eTMR_DRV_DisableChnInt_Activity
 *END**************************************************************************/
void eTMR_DRV_DisableChnInt(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->INTE &= ~((uint32_t)eTMR_INTE_CH0IE_MASK << channel);
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_SetChnsetIntDisable
 * Description   : Disable channel set interrupt.
 *
 * Implements    : eTMR_DRV_SetChnsetIntDisable_Activity
 *END**************************************************************************/
void eTMR_DRV_SetChnsetIntDisable(uint32_t instance, uint8_t channelSet)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    etmrBase->INTE &= ~((uint32_t)channelSet << eTMR_INTE_CH0IE_SHIFT); /*PRQA S 2985*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_IsTofIntEnabled
 * Description   : Get if counter overflow interrupt is enabled or not.
 *
 * Implements    : eTMR_DRV_IsTofIntEnabled_Activity
 *END**************************************************************************/
bool eTMR_DRV_IsTofIntEnabled(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (((etmrBase->INTE) & eTMR_INTE_TOIE_MASK) >> eTMR_INTE_TOIE_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_IsQuadDecodeTofIntEnabled
 * Description   : Get if quadrature decoder counter overflow interrupt is enabled or not.
 *
 * Implements    : eTMR_DRV_IsQdTofIntEnabled_Activity
 *END**************************************************************************/
bool eTMR_DRV_IsQuadDecodeTofIntEnabled(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (((etmrBase->INTE) & eTMR_INTE_QDTOIE_MASK) >> eTMR_INTE_QDTOIE_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_IsFaultIntEnabled
 * Description   : Get if fault interrupt is enabled or not.
 *
 * Implements    : eTMR_DRV_IsFaultIntEnabled_Activity
 *END**************************************************************************/
bool eTMR_DRV_IsFaultIntEnabled(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return (((etmrBase->INTE) & eTMR_INTE_FIE_MASK) >> eTMR_INTE_FIE_SHIFT) != 0U;
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_IsChnIntEnabled
 * Description   : Get if the channel interrupt is enabled or not.
 *
 * Implements    : eTMR_DRV_IsChnIntEnabled_Activity
 *END**************************************************************************/
bool eTMR_DRV_IsChnIntEnabled(uint32_t instance, uint8_t channel)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    DEV_ASSERT(channel < g_etmrChannelNum[instance]);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return ((etmrBase->INTE & ((uint32_t)eTMR_INTE_CH0IE_MASK << channel)) >> (eTMR_INTE_CH0IE_SHIFT + channel)) != 0U; /*PRQA S 2986*/
}

/*FUNCTION**********************************************************************
 *
 * Function Name : eTMR_DRV_GetCntVal
 * Description   : Get eTMR current counter value.
 *
 * Implements    : eTMR_DRV_GetCntVal_Activity
 *END**************************************************************************/
uint32_t eTMR_DRV_GetCntVal(uint32_t instance)
{
    DEV_ASSERT(instance < eTMR_INSTANCE_COUNT);
    eTMR_Type *etmrBase = g_etmrBase[instance];
    return eTMR_GetCntVal(etmrBase);
}

/*! @}*/
/*******************************************************************************
 * EOF
 ******************************************************************************/
