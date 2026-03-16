/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file acmp_hw_access.h
 * @version 1.4.0
 */

#ifndef ACMP_HW_ACCESS_H
#define ACMP_HW_ACCESS_H

#include "acmp_driver.h"
#include "device_registers.h"
#include <stdbool.h>
#include <stdint.h>

#if defined(__cplusplus)
extern "C" {
#endif

static inline bool ACMP_GetOutput(const ACMP_Type *const baseAddr)
{
    return ((baseAddr->STS >> ACMP_STS_OUT_SHIFT) & 0x1U) != 0U;
}

/*!
 * @brief Set the comparator edge selection for interrupt(none, rising edge, falling edge or both edges)
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - comparator output interrupts configuration
 *  ACMP_NO_EDGE
 *  ACMP_RISING_EDGE
 *  ACMP_FALLING_EDGE
 *  ACMP_BOTH_EDGES
 */
static inline void ACMP_SetEdgeSelection(ACMP_Type *const baseAddr, acmp_edge_select_t val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_EDGESEL_MASK;
    baseAddr->CTRL |= ACMP_CTRL_EDGESEL(val);
}

/*!
 * @brief Return the comparator edge selection for interrupt(none, rising edge, falling edge or both edges)
 * @param[in] baseAddr - acmp base pointer
 * @return - comparator output interrupts configuration
 */
static inline uint8_t ACMP_GetEdgeSelection(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_EDGESEL_MASK) >> ACMP_CTRL_EDGESEL_SHIFT);
}

/*!
 * @brief Return the current comparator output polarity
 * @param[in] baseAddr acmp base pointer
 * @return - output polarity state
 */
static inline uint8_t ACMP_GetOutputPolarity(const ACMP_Type *const baseAddr)
{
    return (uint8_t)(((baseAddr->CTRL) & ACMP_CTRL_POL_MASK) >> ACMP_CTRL_POL_SHIFT);
}

/*!
 * @brief Configure the comparator output polarity mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - comparator output polarity mode
 *           ACMP_OUT_POL_NOT_INVERT
 *           ACMP_OUT_POL_INVERT
 */
static inline void ACMP_SetOutputPolarity(ACMP_Type *const baseAddr, acmp_output_polarity_t val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_POL_MASK;
    baseAddr->CTRL |= ACMP_CTRL_POL(val);
}

/*!
 * @brief Return the current comparator output selection
 * @param[in] baseAddr - acmp base pointer
 * @return - comparator output signal source
 *            ACMP_OUTPUT_FILTERED
 *            ACMP_OUTPUT_DIRECTLY
 */
static inline uint8_t ACMP_GetOutputFilterSelection(const ACMP_Type *const baseAddr)
{
    return (uint8_t)(((baseAddr->CTRL) & ACMP_CTRL_OUTSEL_MASK) >> ACMP_CTRL_OUTSEL_SHIFT);
}

/*!
 * @brief Select the comparator output signal source
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - comparator output signal source
 *            ACMP_OUTPUT_FILTERED
 *            ACMP_OUTPUT_DIRECTLY
 * @return void
 */
static inline void ACMP_SetOutputFilterSelection(ACMP_Type *const baseAddr, acmp_output_select_t val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_OUTSEL_MASK;
    baseAddr->CTRL |= ACMP_CTRL_OUTSEL(val);
}

/*!
 * @brief Return the current hysteresis level
 * @param[in] baseAddr - acmp base pointer
 * @return - current hysteresis level
 */
static inline uint8_t ACMP_GetHysteresis(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_HYSTVAL_MASK) >> ACMP_CTRL_HYSTVAL_SHIFT);
}

/*!
 * @brief Set the hysteresis level
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - hysteresis level
 *  ACMP_LEVEL_HYS_0
 *  ACMP_LEVEL_HYS_1
 *  ACMP_LEVEL_HYS_2
 *  ACMP_LEVEL_HYS_3
 * @return - void
 */
static inline void ACMP_SetHysteresis(ACMP_Type *const baseAddr, acmp_hysteresis_t val)
{
    baseAddr->CTRL &= ~(ACMP_CTRL_HYSTVAL_MASK);
    baseAddr->CTRL |= ACMP_CTRL_HYSTVAL(val);
}

/*!
 * @brief Return the current source for positive port of the comparator
 * @param[in] baseAddr - acmp base pointer
 * @return - signal source
 */
static inline uint8_t ACMP_GetPositivePortInputSrc(const ACMP_Type *const baseAddr)
{
#if defined(FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG) && (FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG == 1)
    return (uint8_t)((baseAddr->MUX & ACMP_MUX_INPSEL_MASK) >> ACMP_MUX_INPSEL_SHIFT);
#else
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_INPSEL_MASK) >> ACMP_CTRL_INPSEL_SHIFT);
#endif
}

/*!
 * @brief Set the source for positive port of the comparator
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - signal source
 *                  ACMP_INPUT_SRC_DAC
 *                  ACMP_INPUT_SRC_MUX
 * @return - void
 */
static inline void ACMP_SetPositivePortInputSrc(ACMP_Type *const baseAddr, acmp_input_source_t val)
{
#if defined(FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG) && (FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG == 1)
    baseAddr->MUX &= ~ACMP_MUX_INPSEL_MASK;
    baseAddr->MUX |= ACMP_MUX_INPSEL(val);
#else
    baseAddr->CTRL &= ~(ACMP_CTRL_INPSEL_MASK);
    baseAddr->CTRL |= ACMP_CTRL_INPSEL(val);
#endif
}

/*!
 * @brief Return the current source for negative port of the comparator
 * @param[in] baseAddr - acmp base pointer
 * @return - signal source
 */
static inline uint8_t ACMP_GetNegativePortInputSrc(const ACMP_Type *const baseAddr)
{
#if defined(FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG) && (FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG == 1)
    return (uint8_t)((baseAddr->MUX & ACMP_MUX_INNSEL_MASK) >> ACMP_MUX_INNSEL_SHIFT);
#else
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_INNSEL_MASK) >> ACMP_CTRL_INNSEL_SHIFT);
#endif
}

/*!
 * @brief Set the source for negative port of the comparator
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - signal source
 *            ACMP_INPUT_SRC_DAC
 *            ACMP_INPUT_SRC_MUX
 * @return - void
 */
static inline void ACMP_SetNegativePortInputSrc(ACMP_Type *const baseAddr, acmp_input_source_t val)
{
#if defined(FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG) && (FEATURE_ACMP_HAS_INPUT_SRC_SEL_IN_MUX_REG == 1)
    baseAddr->MUX &= ~ACMP_MUX_INNSEL_MASK;
    baseAddr->MUX |= ACMP_MUX_INNSEL(val);
#else
    baseAddr->CTRL &= ~(ACMP_CTRL_INNSEL_MASK);
    baseAddr->CTRL |= ACMP_CTRL_INNSEL(val);
#endif
}

#if FEATURE_ACMP_HAS_AUTODIS
/*!
 * @brief Whether disable hard block automatically in one-shot mode or not
 *
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] val   true  -- disable hard block automatically in one-shot mode
 *                  false -- not disable hard block automatically
 */
static inline void ACMP_SetAutoDisableHardBlock(ACMP_Type *const baseAddr, bool val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_AUTODIS_MASK;
    baseAddr->CTRL |= ACMP_CTRL_AUTODIS(val ? 1U : 0U);
}

/*!
 * @brief Return if automatically disable hard block in one-shot mode
 *
 * @param[in] baseAddr - acmp base pointer
 * @return - bool
 */
static inline bool ACMP_GetAutoDisableHardBlock(const ACMP_Type *const baseAddr)
{
    return ((baseAddr->CTRL & ACMP_CTRL_AUTODIS_MASK) >> ACMP_CTRL_AUTODIS_SHIFT) != 0U;
}
#endif

#if FEATURE_ACMP_HAS_OFFSET_CONTROL
/*!
 * @brief Control comparator hard block offset for hysteresis
 *
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] val
 *            0 -- The hysteresis selected by HYSTVAL exists for INP crossing INN in the falling direction
 *            1 -- The hysteresis selected by HYSTVAL exists for both directions
 */
static inline void ACMP_SetHardBlockOffset(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_OFFSET_MASK;
    baseAddr->CTRL |= ACMP_CTRL_OFFSET(val);
}

/*!
 * @brief Return if set hard block offset for hysteresis
 * @param[in] baseAddr - acmp base pointer
 * @return - bool
 */
static inline bool ACMP_GetHardBlockOffset(const ACMP_Type *const baseAddr)
{
    return ((baseAddr->CTRL & ACMP_CTRL_OFFSET_MASK) >> ACMP_CTRL_OFFSET_SHIFT) != 0U;
}
#endif

#if FEATURE_ACMP_HAS_CLK_SRC_SEL
/*!
 * @brief Select the source of ACMP module clock
 *
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] sel Clock source
 */
static inline void ACMP_SetClockSrc(ACMP_Type *const baseAddr, uint8_t sel)
{
    baseAddr->CTRL &= ~ACMP_CTRL_CLKSEL_MASK;
    baseAddr->CTRL |= ACMP_CTRL_CLKSEL(sel);
}

/*!
 * @brief Gets analog comparator clock source
 * @param[in] baseAddr - acmp base pointer
 * @return - clock source
 */
static inline uint8_t ACMP_GetClockSrc(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_CLKSEL_MASK) >> ACMP_CTRL_CLKSEL_SHIFT);
}
#endif

/*!
 * @brief Return the current power mode
 * @param[in] baseAddr - acmp base pointer
 * @return - current power mode
 */
static inline uint8_t ACMP_GetPowerMode(const ACMP_Type *const baseAddr)
{
    return (uint8_t)(((baseAddr->CTRL) & ACMP_CTRL_PWRMD_MASK) >> ACMP_CTRL_PWRMD_SHIFT);
}

/*!
 * @brief Set the power mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - power mode
 *            ACMP_HIGH_SPEED
 *            ACMP_LOW_POWER
 */
static inline void ACMP_SetPowerMode(ACMP_Type *const baseAddr, acmp_power_mode_t val)
{
    baseAddr->CTRL &= ~ACMP_CTRL_PWRMD_MASK;
    baseAddr->CTRL |= ACMP_CTRL_PWRMD(val);
}

#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
/*!
 * @brief Sets ACMP sample mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] mode - sample mode
 */
static inline void ACMP_SetSampleMode(ACMP_Type *const baseAddr, acmp_sample_mode_t mode)
{
    baseAddr->CTRL &= ~ACMP_CTRL_MODE_MASK;
    baseAddr->CTRL |= ACMP_CTRL_MODE(mode);
}

/*!
 * @brief Gets ACMP sample mode
 * @param[in] baseAddr - acmp base pointer
 */
static inline uint8_t ACMP_GetSampleMode(ACMP_Type *const baseAddr)
{
    return (uint8_t)(baseAddr->CTRL & ~ACMP_CTRL_MODE_MASK) >> ACMP_CTRL_MODE_SHIFT;
}
#else
/*!
 * @brief Sets the comparator sample mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] mode - sample mode
 */
static inline void ACMP_SetSampleMode(ACMP_Type *const baseAddr, acmp_sample_mode_t val)
{
    DEV_ASSERT(val < 4);
    baseAddr->CTRL &= ~ACMP_CTRL_SMPMD_MASK;
    baseAddr->CTRL |= ACMP_CTRL_SMPMD(val);
}

/*!
 * @brief Gets the comparator sample mode
 * @param[in] baseAddr - acmp base pointer
 */
static inline uint8_t ACMP_GetSampleMode(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_SMPMD_MASK) >> ACMP_CTRL_SMPMD_SHIFT);
}
#endif

/*!
 * @brief Verify if the DMA transfer trigger is enabled
 * @param[in] baseAddr - acmp base pointer
 * @return - DMA transfer trigger state
 *           true - DMA trigger is enabled
 *           false - DMA trigger is disabled
 */
static inline bool ACMP_GetDmaTriggerEnable(const ACMP_Type *const baseAddr)
{
    return (((baseAddr->DMACR) & ACMP_DMACR_EN_MASK) >> ACMP_DMACR_EN_SHIFT) != 0U;
}

/*!
 * @brief Configure the DMA transfer trigger
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - DMA transfer trigger state
 *  true - DMA trigger is enabled
 *  false - DMA trigger is disabled
 * @return - void
 */
static inline void ACMP_SetDmaEnState(ACMP_Type *const baseAddr, bool val)
{
    baseAddr->DMACR &= ~ACMP_DMACR_EN_MASK;
    baseAddr->DMACR |= ACMP_DMACR_EN(val ? 1U : 0U);
}

/*!
 * @brief Verify if the interrupt is enabled
 * @param[in] baseAddr - acmp base pointer
 * @return - interrupt trigger state
 *  true - interrupt is enabled
 *  false - interrupt is disabled
 */
static inline bool ACMP_GetInterruptEnState(const ACMP_Type *const baseAddr)
{
    return (((baseAddr->INTE) & ACMP_INTE_IE_MASK) >> ACMP_INTE_IE_SHIFT) != 0U;
}

/*!
 * @brief Configure the interrupt
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - interrupt state
 *  true - interrupt is enabled
 *  false - interrupt is disabled
 * @return - void
 */
static inline void ACMP_SetInterruptEnState(ACMP_Type *const baseAddr, bool val)
{
    baseAddr->INTE &= ~ACMP_INTE_IE_MASK;
    baseAddr->INTE |= ACMP_INTE_IE(val ? 1U : 0U);
}

/*!
 * @brief Verify if the continuous mode interrupt is enabled
 * @param[in] baseAddr - acmp base pointer
 * @return - interrupt state
 *  true - continuous mode interrupt is enabled
 *  false - continuous mode interrupt is disabled
 */
static inline bool ACMP_GetContinuousModeInterruptEnState(const ACMP_Type *const baseAddr)
{
    return (((baseAddr->INTE) & ACMP_INTE_CONTIE_MASK) >> ACMP_INTE_CONTIE_SHIFT) != 0U;
}

/*!
 * @brief Set the continuous mode interrupt state
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - continuous mode interrupt state
 *  true - continuous mode interrupt is enabled
 *  false - continuous mode interrupt is disabled
 * @return - void
 */
static inline void ACMP_SetContinuousModeInterruptEnState(ACMP_Type *const baseAddr, bool val)
{
    baseAddr->INTE &= ~ACMP_INTE_CONTIE_MASK;
    baseAddr->INTE |= ACMP_INTE_CONTIE(val ? 1U : 0U);
}

/*!
 * @brief Verify if the DAC is enabled
 * @param[in] baseAddr - acmp base pointer
 * @return - dac state
 *  true - DAC is enabled
 *  false - DAC is disabled
 */
static inline bool ACMP_GetDacEnState(const ACMP_Type *const baseAddr)
{
    return (((baseAddr->DAC) >> ACMP_DAC_EN_SHIFT) & ACMP_DAC_EN_MASK) != 0U;
}

/*!
 * @brief Set the DAC state (enabled/disabled)
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - DAC state
 *  true - DAC is enabled
 *  false - DAC is disabled
 * @return - void
 */
static inline void ACMP_SetDacEnState(ACMP_Type *const baseAddr, bool val)
{
    (baseAddr->DAC) = ((baseAddr->DAC) & (~ACMP_DAC_EN_MASK)) | ACMP_DAC_EN(val ? 1U : 0U);
}

#if FEATURE_ACMP_HAS_DAC_OUTPUT
/*!
 * @brief Return the current DAC output state
 * @param[in] baseAddr - acmp base pointer
 * @return - DAC output state
 *  true - DAC output is enabled
 *  false - DAC output is disabled
 */
static inline bool ACMP_GetDacOutEnState(const ACMP_Type *const baseAddr)
{
    return (((baseAddr->DAC) >> ACMP_DAC_OUTEN_SHIFT) & ACMP_DAC_OUTEN_MASK) != 0U;
}

/*!
 * @brief Set the DAC output
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - DAC output enable
 *  true - DAC output is enabled
 *  false - DAC output is disabled
 * @return - void
 */
static inline void ACMP_SetDacOutEnState(ACMP_Type *const baseAddr, bool val)
{
    (baseAddr->DAC) = ((baseAddr->DAC) & (~ACMP_DAC_OUTEN_MASK)) | ACMP_DAC_OUTEN(val ? 1U : 0U);
}
#endif

#if FEATURE_ACMP_HAS_DAC_VOLTAGE_REF_SRC
/*!
 * @brief Return the current voltage reference
 * @param[in] baseAddr - acmp base pointer
 * @return - voltage reference
 *  ACMP_EXT_REF
 *  ACMP_INT_REF
 */
static inline acmp_voltage_reference_t ACMP_GetVoltageReference(const ACMP_Type *const baseAddr)
{
    uint32_t tmp = ((baseAddr->DAC) >> ACMP_DAC_VREF_SHIFT) & ACMP_DAC_VREF_MASK;
    return (acmp_voltage_reference_t)tmp;
}

/*!
 * @brief Set the voltage reference
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - voltage reference
 *  ACMP_EXT_REF
 *  ACMP_INT_REF
 * @return - void
 */
static inline void ACMP_SetVoltageReference(ACMP_Type *const baseAddr, acmp_voltage_reference_t val)
{
    (baseAddr->DAC) = (((baseAddr->DAC) & (~ACMP_DAC_VREF_MASK)) | ACMP_DAC_VREF(val));
}
#endif

/*!
 * @brief Return the current output voltage(0-255)
 * @param[in] baseAddr - acmp base pointer
 * @return - voltage level
 */
static inline uint8_t ACMP_GetVoltage(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->DAC & ACMP_DAC_VAL_MASK) >> ACMP_DAC_VAL_SHIFT);
}

/*!
 * @brief Set the output voltage
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - voltage level
 * @return - void
 */
static inline void ACMP_SetVoltage(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->DAC &= ~(ACMP_DAC_VAL_MASK);
    baseAddr->DAC |= ACMP_DAC_VAL(val);
}

/*!
 * @brief Get which input is selected for the positive port
 * @param[in] baseAddr - acmp base pointer
 * @return - channel for the positive port
 */
static inline acmp_ch_number_t ACMP_GetPositivePortInputChannel(const ACMP_Type *const baseAddr)
{
    return (acmp_ch_number_t)((baseAddr->MUX & ACMP_MUX_CHPSEL_MASK) >> ACMP_MUX_CHPSEL_SHIFT);
}

/*!
 * @brief Select input for the positive port
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - channel for the positive port
 * @return - void
 */
static inline void ACMP_SetPositivePortInputChannel(ACMP_Type *const baseAddr, acmp_ch_number_t val)
{
    baseAddr->MUX &= ~(ACMP_MUX_CHPSEL_MASK);
    baseAddr->MUX |= ACMP_MUX_CHPSEL(val);
}

/*!
 * @brief Determine which input is selected for the negative port
 * @param[in] baseAddr - acmp base pointer
 * @return - channel for the negative port
 */
static inline acmp_ch_number_t ACMP_GetNegativePortInputChannel(const ACMP_Type *const baseAddr)
{
    return (acmp_ch_number_t)((baseAddr->MUX & ACMP_MUX_CHNSEL_MASK) >> ACMP_MUX_CHNSEL_SHIFT);
}

/*!
 * @brief Select input for the negative port
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - channel for the negative port
 * @return - void
 */
static inline void ACMP_SetNegativePortInputChannel(ACMP_Type *const baseAddr, acmp_ch_number_t val)
{
    baseAddr->MUX &= ~(ACMP_MUX_CHNSEL_MASK);
    baseAddr->MUX |= ACMP_MUX_CHNSEL(val);
}

#if FEATURE_ACMP_HAS_FILTER_BYPASS
static inline void ACMP_SetFilterEnState(ACMP_Type *const baseAddr, bool enable)
{
    baseAddr->FILT |= ACMP_FILT_BP(enable ? 0U : 1U);
}
#endif

/*!
 * @brief Return the sample period for filter(clock cycles)
 * @param[in] baseAddr - acmp base pointer
 * @return - sampling period(in bus cycles)
 */
static inline uint8_t ACMP_GetFilterSamplePeriod(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->FILT & ACMP_FILT_PER_MASK) >> ACMP_FILT_PER_SHIFT);
}

/*!
 * @brief Set the filter sample period(clock cycles)
 * @param[in] baseAddr -acmp base pointer
 * @param[in] val - number of bus cycles
 *
 * @return - void
 */
static inline void ACMP_SetFilterSamplePeriod(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->FILT &= ~(ACMP_FILT_PER_MASK);
    baseAddr->FILT |= ACMP_FILT_PER(val);
}

/*!
 * @brief Return the number of consecutive samples that must agree prior to the comparator output filter
 *        accepting a new output state
 * @param[in] baseAddr - acmp base pointer
 * @return - filter sample count
 */
static inline uint8_t ACMP_GetFilterSampleCount(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->FILT & ACMP_FILT_CNT_MASK) >> ACMP_FILT_CNT_SHIFT);
}

/*!
 * @brief Set the number of consecutive samples that must agree prior to the comparator output filter
 *        accepting a new output state
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - filter sample count(min value 0, max value 7)
 * @return - void
 */
static inline void ACMP_SetFilterSampleCount(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->FILT &= ~(ACMP_FILT_CNT_MASK);
    baseAddr->FILT |= ACMP_FILT_CNT(val);
}

/*!
 * @brief Select filter clock source
 *
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] sel filter clock source
 */
static inline void ACMP_SetFilterClockSrc(ACMP_Type *const baseAddr, uint8_t sel)
{
#if FEATURE_ACMP_HAS_FILTER_CLK_SRC_IN_CTRL_REG
    baseAddr->CTRL &= ~ACMP_CTRL_FILTCLK_MASK;
    baseAddr->CTRL |= (uint32_t)sel << ACMP_CTRL_FILTCLK_SHIFT;
#else
    baseAddr->FILT &= ~(ACMP_FILT_CLKSRC_MASK);
    baseAddr->FILT |= ACMP_FILT_CLKSRC(sel);
#endif
}

/*!
 * @brief Get filter clock source
 *
 * @param[in] baseAddr - acmp base pointer
 * @return - filter clock source
 */
static inline uint8_t ACMP_GetFilterClockSrc(const ACMP_Type *const baseAddr)
{
#if FEATURE_ACMP_HAS_FILTER_CLK_SRC_IN_CTRL_REG
    return (uint8_t)((baseAddr->CTRL & ACMP_CTRL_FILTCLK_MASK) >> ACMP_CTRL_FILTCLK_SHIFT);
#else
    return (uint8_t)((baseAddr->FILT & ACMP_FILT_CLKSRC_MASK) >> ACMP_FILT_CLKSRC_SHIFT);
#endif
}

/*!
 * @brief Return which channels are used in continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @return - channels states, one bite for each channel state
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 */
static inline acmp_ch_list_t ACMP_GetContinuousModeChannels(const ACMP_Type *const baseAddr)
{
    uint32_t tmp = baseAddr->CONT;
    tmp = tmp & (uint32_t)ACMP_CONTINUOUS_MODE_CHANNELS_MASK;
    return (acmp_ch_list_t)(tmp >> ACMP_CONTINUOUS_MODE_CHANNELS_SHIFT);
}

/*!
 * @brief Set which channels are used in continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - channels states, one bit for each channel state
 * |---------|---------|-----|---------|---------|
 * |CH7_state|CH6_state|.....|CH1_state|CH0_state|
 * |---------|---------|-----|---------|---------|
 * @return - void
 */
static inline void ACMP_SetContinuousModeChannels(ACMP_Type *const baseAddr, acmp_ch_list_t val)
{
    baseAddr->CONT &= ~(uint32_t)(ACMP_CONTINUOUS_MODE_CHANNELS_MASK);
    baseAddr->CONT |= (uint32_t)val << (uint32_t)ACMP_CONTINUOUS_MODE_CHANNELS_SHIFT;
}

/*!
 * @brief Verify if the continuous mode is enabled
 * @param[in] baseAddr acmp base pointer
 * @return - continuous mode state
 *  true - continuous mode is enabled
 *  false - continuous mode is disabled
 */
static inline bool ACMP_GetContinuousModeEnState(const ACMP_Type *const baseAddr)
{
#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
    return ((((baseAddr->CTRL) >> ACMP_CTRL_MODE_SHIFT) & ACMP_CTRL_MODE_MASK) == 0x3U) ? true : false;
#else
    return (((baseAddr->CONT) >> ACMP_CONT_EN_SHIFT) & ACMP_CONT_EN_MASK) != 0U;
#endif
}

/*!
 * @brief Set the continuous mode state
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - continuous mode state
 *  true - continuous mode is enabled
 *  false - continuous mode is disabled
 * @return - void
 */
static inline void ACMP_SetContinuousModeEnState(ACMP_Type *const baseAddr, bool val)
{
#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
    baseAddr->CTRL &= ~ACMP_CTRL_MODE_MASK;
    baseAddr->CTRL |= ACMP_CTRL_MODE(0x3);
    (void)val;
#else
    baseAddr->CONT &= ~ACMP_CONT_EN_MASK;
    baseAddr->CONT |= ACMP_CONT_EN(val ? 1U : 0U);
#endif
}


/*!
 * @brief Return the port fixed for continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @return - fixed port
 */
static inline uint8_t ACMP_GetFixedPort(const ACMP_Type *const baseAddr)
{
    return (uint8_t)(((baseAddr->CONT) >> ACMP_CONT_CHFIX_SHIFT) & ACMP_CONT_CHFIX_MASK);
}

/*!
 * @brief Set the fixed port for continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - fixed port
 * ACMP_FIXED_POS_PORT
 * ACMP_FIXED_NEG_PORT
 * @return - void
 */
static inline void ACMP_SetFixedPort(ACMP_Type *const baseAddr, acmp_fixed_port_t val)
{
    baseAddr->CONT &= ~(ACMP_CONT_CHFIX_MASK);
    baseAddr->CONT |= ACMP_CONT_CHFIX(val);
}

/*!
 * @brief Return continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @return - continuous mode
 */
static inline uint8_t ACMP_GetContinuousMode(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CONT & ACMP_CONT_MODE_MASK) >> ACMP_CONT_MODE_SHIFT);
}

/*!
 * @brief Set continuous mode
 * @param[in] baseAddr - acmp base pointer
 * @param[in] mode - acmp continuous mode
 *            loop mode
 *            one-shot mode
 * @return - void
 */
static inline void ACMP_SetContinuousMode(ACMP_Type *const baseAddr, acmp_continuous_mode_t mode)
{
    baseAddr->CONT &= ~(ACMP_CONT_MODE_MASK);
    baseAddr->CONT |= ACMP_CONT_MODE(mode);
}

#if FEATURE_ACMP_HAS_TRIG_MODE_GATE
/*!
 * @brief Set trigger mode enable or disable for one-shot mode
 * @param[in] baseAddr - acmp base pointer
 */
static inline void ACMP_SetOneshotTriggerMode(ACMP_Type *const baseAddr, bool enable)
{
    baseAddr->CONT |= ACMP_CONT_TRIGMD(enable ? 1U : 0U);
}
#endif

/*!
 * @brief Return how many clock cycles switch channel
 * @param[in] baseAddr - acmp base pointer
 * @return - channel switch period
 */
static inline uint8_t ACMP_GetContinuousModeSamplePeriod(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CONT & ACMP_CONT_PER_MASK) >> ACMP_CONT_PER_SHIFT);
}

/*!
 * @brief Set channel switch period
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - number of sample clocks
 * @return - void
 */
static inline void ACMP_SetContinuousModeSamplePeriod(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->CONT &= ~(ACMP_CONT_PER_MASK);
    baseAddr->CONT |= ACMP_CONT_PER(val);
}

/*!
 * @brief Return how many clock cycles before sampling(switch stabilization time)
 * @param[in] baseAddr - acmp base pointer
 * @return - number of sample clocks
 */
static inline uint8_t ACMP_GetContinuousModeSamplePosition(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->CONT & ACMP_CONT_SMP_MASK) >> ACMP_CONT_SMP_SHIFT);
}

/*!
 * @brief Set how many clock cycles before sampling(switch stabilization)
 * @param[in] baseAddr - acmp base pointer
 * @param[in] val - number of sample clocks
 * @return - void
 */
static inline void ACMP_SetContinuousModeSamplePosition(ACMP_Type *const baseAddr, uint8_t val)
{
    baseAddr->CONT &= ~(ACMP_CONT_SMP_MASK);
    baseAddr->CONT |= ACMP_CONT_SMP(val);
}

/*!
 * @brief Return last input comparison results for all channels
 * @param[in] baseAddr - acmp base pointer
 * @return - comparison results
 */
static inline acmp_ch_list_t ACMP_GetLastComparisonResult(const ACMP_Type *const baseAddr)
{
    return (acmp_ch_list_t)((baseAddr->STS & ACMP_STS_CH_OUT_MASK) >> ACMP_STS_CH_OUT_SHIFT);
}

/*!
 * @brief Sets the expectation of input channels.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
static inline void ACMP_SetExpectation(ACMP_Type *const baseAddr, acmp_ch_list_t val)
{
#if FEATURE_ACMP_HAS_EXP_REG
    baseAddr->EXP |= val;
#else
    baseAddr->STS |= ((uint32_t)val << ACMP_STS_CH_OUT_SHIFT) & ACMP_STS_CH_OUT_MASK;
#endif
}

/*!
 * @brief Get Channel ID to indicate current channel in continuous mode
 *
 * @param[in] baseAddr The ACMP base address pointer
 */
static inline uint8_t ACMP_GetChannelId(const ACMP_Type *const baseAddr)
{
    return (uint8_t)((baseAddr->STS & ACMP_STS_CHID_MASK) >> ACMP_STS_CHID_SHIFT);
}

/*!
 * @brief This function set all ACMP registers to reset values.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
void ACMP_Reset(ACMP_Type *const baseAddr);

/*!
 * @brief This function enable ACMP.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
void ACMP_Enable(ACMP_Type *const baseAddr);

/*!
 * @brief This function disable ACMP.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
void ACMP_Disable(ACMP_Type *const baseAddr);

/*!
 * @brief This function Clear all input change flags.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
void ACMP_ClearChannelFlags(ACMP_Type *const baseAddr);

/*!
 * @brief This function Return all input change flags in <flags>.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
acmp_ch_list_t ACMP_GetChannelFlags(const ACMP_Type *const baseAddr);

/*!
 * @brief This function Clear comparator output flags.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
void ACMP_ClearOutputFlags(ACMP_Type *const baseAddr);

/*!
 * @brief This function Return comparator output flags in <flags>.
 * @param[in] baseAddr acmp base pointer
 * @param[in] val - state
 * @return void
 */
uint8_t ACMP_GetOutputFlags(const ACMP_Type *const baseAddr);

/*!
 * @brief Get channel enable state for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - enable or not
 */
bool ACMP_GetContinuousChannelEnState(const ACMP_Type *const baseAddr, uint8_t ch);

/*!
 * @brief Get channel expectation for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - channel expectation
 */
uint8_t ACMP_GetContinuousChannelExpectation(const ACMP_Type *const baseAddr, uint8_t ch);

/*!
 * @brief Get channel output for continuous mode
 * @param[in] baseAddr The ACMP base address pointer
 * @param[in] ch The ACMP input channel
 * @return - channel output
 */
uint8_t ACMP_GetContinuousChannelOutput(const ACMP_Type *const baseAddr, uint8_t ch);

#if defined(__cplusplus)
}
#endif

#endif /* ACMP_HW_ACCESS_H */

/*******************************************************************************
 * EOF
 ******************************************************************************/
