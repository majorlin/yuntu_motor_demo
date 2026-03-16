/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file acmp_hw_access.c
 * @version 1.4.0
 */

#include "acmp_hw_access.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief This function set all ACMP registers to reset values.
 * @param baseAddr acmp base pointer
 * @param val - state
 * @return void
 */
void ACMP_Reset(ACMP_Type *const baseAddr)
{
    baseAddr->CTRL = 0;
    baseAddr->DAC = 0;
    baseAddr->MUX = 0;
    baseAddr->FILT = 0;
    baseAddr->DMACR = 0;
    baseAddr->STS = FEATURE_ACMP_STS_CLEAR_MASK;
    baseAddr->INTE = 0;
    baseAddr->CONT = 0;
#if FEATURE_ACMP_HAS_EN_REG
    baseAddr->EN = 0;
#endif
#if FEATURE_ACMP_HAS_EXP_REG
    baseAddr->EXP = 0;
#endif
}

/*!
 * @brief This function enable ACMP.
 * @param baseAddr acmp base pointer
 * @return void
 */
void ACMP_Enable(ACMP_Type *const baseAddr)
{
#if FEATURE_ACMP_HAS_EN_REG
    baseAddr->EN |= ACMP_EN_EN_MASK;
#else
    baseAddr->CTRL |= ACMP_CTRL_EN_MASK;
#endif
}

/*!
 * @brief This function disable ACMP.
 * @param baseAddr acmp base pointer
 * @param val - state
 * @return void
 */
void ACMP_Disable(ACMP_Type *const baseAddr)
{
#if FEATURE_ACMP_HAS_EN_REG
    baseAddr->EN &= ~ACMP_EN_EN_MASK;
#else
    baseAddr->CTRL &= ~ACMP_CTRL_EN_MASK;
#endif
}

/*!
 * @brief This function Clear all channel flags.
 * @param baseAddr acmp base pointer
 * @param val - state
 * @return void
 */
void ACMP_ClearChannelFlags(ACMP_Type *const baseAddr)
{
    baseAddr->STS = (uint32_t)ACMP_STS_CH_FLAG_MASK;
}

/*!
 * @brief This function Return all channel change flags in <flags>.
 * @param baseAddr acmp base pointer
 * @return acmp_ch_list_t
 */
acmp_ch_list_t ACMP_GetChannelFlags(const ACMP_Type *const baseAddr)
{
    return (acmp_ch_list_t)((baseAddr->STS & (uint32_t)ACMP_STS_CH_FLAG_MASK) >> ACMP_STS_CH_FLAG_SHIFT);
}

#if defined(FEATURE_ACMP_SUPPORT_LEVEL_DETECTION)
/*!
 * @brief This function Clear comparator output positive/negative edge and high/low level flags.
 * @param baseAddr acmp base pointer
 * @return void
 */
void ACMP_ClearOutputFlags(ACMP_Type *const baseAddr)
{
    baseAddr->STS = (ACMP_STS_OUTPF_MASK | ACMP_STS_OUTNF_MASK |
                     ACMP_STS_OUTHF_MASK | ACMP_STS_OUTLF_MASK);
}

/*!
 * @brief This function Return comparator output positive/negative edge, high/low level flags.
 * @param baseAddr acmp base pointer
 * @return uint8_t
 */
uint8_t ACMP_GetOutputFlags(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->STS & (ACMP_STS_OUTPF_MASK | ACMP_STS_OUTNF_MASK |
                                      ACMP_STS_OUTHF_MASK | ACMP_STS_OUTLF_MASK)) >> ACMP_STS_OUTPF_SHIFT);
}
#else
/*!
 * @brief This function Clear comparator output positive and negative flags.
 * @param baseAddr acmp base pointer
 * @return void
 */
void ACMP_ClearOutputFlags(ACMP_Type *const baseAddr)
{
    baseAddr->STS = (ACMP_STS_OUTPF_MASK | ACMP_STS_OUTNF_MASK);
}

/*!
 * @brief This function Return comparator output positive and negative flags.
 * @param baseAddr acmp base pointer
 * @return uint8_t
 */
uint8_t ACMP_GetOutputFlags(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->STS & (ACMP_STS_OUTPF_MASK |
                                       ACMP_STS_OUTNF_MASK)) >> ACMP_STS_OUTPF_SHIFT);
}
#endif

/*!
 * @brief Get channel enable state for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - true or not
 */
bool ACMP_GetContinuousChannelEnState(const ACMP_Type *const baseAddr, uint8_t ch)
{
    return ((baseAddr->CONT & ((uint32_t)ACMP_CONT_CH0EN_MASK << ch)) >> ch) != 0U;
}

/*!
 * @brief Get channel expectation for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - channel expectation
 */
uint8_t ACMP_GetContinuousChannelExpectation(const ACMP_Type *const baseAddr, uint8_t ch)
{
#if FEATURE_ACMP_HAS_EXP_REG
    return (uint8_t)((baseAddr->EXP & ((uint32_t)ACMP_EXP_CH0EXP_MASK << ch)) >> ch);
#else
    return (uint8_t)((baseAddr->STS & (ACMP_STS_CH0OUT_MASK << ch)) >> ch);
#endif
}

/*!
 * @brief Get channel output for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - channel output
 */
uint8_t ACMP_GetContinuousChannelOutput(const ACMP_Type *const baseAddr, uint8_t ch)
{
    return (uint8_t)((baseAddr->STS & (ACMP_STS_CH0OUT_MASK << ch)) >> (ch + ACMP_STS_CH0OUT_SHIFT));
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
