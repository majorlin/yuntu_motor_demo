/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*!
 * @file acmp_driver.h
 * @version 1.4.0
 */

#ifndef ACMP_DRIVER_H
#define ACMP_DRIVER_H

#include "device_registers.h"
#include "status.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ACMP_CONTINUOUS_MODE_CHANNELS_MASK  0xFF00
#define ACMP_CONTINUOUS_MODE_CHANNELS_SHIFT 8U
#define ACMP_STS_CH_OUT_MASK                0xFF000000U
#define ACMP_STS_CH_OUT_SHIFT               24U
#define ACMP_STS_CH_FLAG_MASK               0x00FF0000U
#define ACMP_STS_CH_FLAG_SHIFT              16U

/*******************************************************************************
 * Enumerations.
 ******************************************************************************/

/*!
 * @brief Power Modes selection
 * Implements : acmp_power_mode_t_Class
 */
typedef enum
{
    ACMP_LOW_POWER = 0U, /*!< Module in low power mode. */
    ACMP_HIGH_SPEED = 1U, /*!< Module in high speed mode. */
} acmp_power_mode_t;

#if FEATURE_ACMP_HAS_DAC_VOLTAGE_REF_SRC
/*!
 * @brief Voltage Reference selection
 * Implements : acmp_voltage_reference_t_Class
 */
typedef enum
{
    ACMP_EXT_REF = 0U, /*!< Use VREFH/VREFL as supply reference source for DAC. */
    ACMP_INT_REF = 1U  /*!< Use internal supply reference source(VDDA) for DAC. */
} acmp_voltage_reference_t;
#endif /* FEATURE_ACMP_HAS_DAC */

/*!
 * @brief Port input source selection
 * Implements : acmp_input_source_t_Class
 */
typedef enum
{
    ACMP_INPUT_SRC_DAC = 0U, /*!< Select DAC as source for the comparator port. */
    ACMP_INPUT_SRC_MUX = 1U, /*!< Select MUX8 as source for the comparator port. */
} acmp_input_source_t;

/*!
 * @brief Output polarity selection
 * Implements : acmp_output_polarity_t_Class
 */
typedef enum
{
    ACMP_OUT_POL_NOT_INVERT = 0U, /*!< ACMP output polarity isn't inverted. */
    ACMP_OUT_POL_INVERT = 1U,     /*!< ACMP output polarity is inverted. */
} acmp_output_polarity_t;

/*! @brief Output selection
 * Implements : acmp_output_select_t_Class
 */
typedef enum
{
    ACMP_OUTPUT_FILTERED = 0U, /*!< ACMP output filtered to pin */
    ACMP_OUTPUT_DIRECTLY = 1U  /*!< ACMP output to pin directly */
} acmp_output_select_t;

/*! @brief Hysteresis level selection
 * Implements : acmp_hysteresis_t_Class
 */
typedef enum
{
    ACMP_HYS_LEVEL_0 = 0U, /*!< Comparator has no hysteresis internally*/
    ACMP_HYS_LEVEL_1 = 1U, /*!< Comparator has 20mV hysteresis internally*/
    ACMP_HYS_LEVEL_2 = 2U, /*!< Comparator has 40mV hysteresis internally*/
    ACMP_HYS_LEVEL_3 = 3U  /*!< Reserved*/
} acmp_hysteresis_t;

/*! @brief Fixed port selection
 * Implements : acmp_fixed_port_t_Class
 */
typedef enum
{
    ACMP_FIXED_NEG_PORT = 0U, /*!< The negative port is fixed. */
    ACMP_FIXED_POS_PORT = 1U  /*!< The positive port is fixed. */
} acmp_fixed_port_t;

/*! @brief Edge selection for entering interrupt/dma request
 * Implements : acmp_edge_select_t_Class
 */
typedef enum
{
    ACMP_NO_EDGE = 0U,      /*!< Comparator interrupt not generated */
    ACMP_RISING_EDGE = 1U,  /*!< Comparator interrupt generated on rising edge  */
    ACMP_FALLING_EDGE = 2U, /*!< Comparator interrupt generated on falling edge  */
    ACMP_BOTH_EDGES = 3U,   /*!< Comparator interrupt generated on both edges */
#if defined(FEATURE_ACMP_SUPPORT_LEVEL_DETECTION)
    ACMP_HIGH_LEVEL = 4U,   /*!< Comparator interrupt generated on high level */
    ACMP_LOW_LEVEL = 5U,    /*!< Comparator interrupt generated on low level */
#endif
} acmp_edge_select_t;

/*! 
 * @brief Filter clock source selection
 * Implements : acmp_filter_clk_src_t_Class
 */
typedef enum
{
    ACMP_FILTER_SEL_FUNC_CLK = 0U, /*!< Select functional clock which is selected in IPC module */
    ACMP_FILTER_SEL_BUS_CLK = 1U,  /*!< Select bus clock(register-used clock) */
} acmp_filter_clk_src_t;

/*! @brief Analog Comparator sample modes
 * Implements : acmp_sample_mode_t_Class
 */
typedef enum
{
    ACMP_COMMON_MODE = 0U,
    ACMP_TRIGGER_MODE = 1U,
    ACMP_WINDOW_MODE = 2U,
#if FEATURE_ACMP_HAS_CONTINUOUS_MODE_IN_CTRL_REG
    ACMP_CONTINUOUS_MODE = 3U,
#else
    ACMP_RESERVED = 3U,
#endif
} acmp_sample_mode_t;

/*!
 * @brief Analog comparator clock source selection
 *
 * Implements : acmp_clock_source_t_Class
 */
typedef enum
{
    ACMP_CLK_SRC_IPC = 0U,
    ACMP_CLK_SRC_SIRC = 1U,
#if FEATURE_ACMP_SUPPORT_SXOSC_CLK
    ACMP_CLK_SRC_SXOSC = 2U,
#endif
} acmp_clock_source_t;

/*! 
 * @brief Analog comparator continuous mode
 * Implements : acmp_continuous_mode_t_Class
 */
typedef enum
{
    ACMP_CONTINUOUS_LOOP_MODE = 0U,     /*!< ACMP continuous mode - loop mode */
    ACMP_CONTINUOUS_ONE_SHOT_MODE = 1U, /*!< ACMP continuous mode - one-shot mode */
} acmp_continuous_mode_t;

/*!
 * @brief Channel expectation configuration
 * Implements : acmp_ch_expectation_t_Class
 */
typedef enum
{
    ACMP_EXPECT_POS_LESS_THAN_NEG = 0U,  /*!< Expect that positive port input < negative port input */
    ACMP_EXPECT_POS_GREAT_THAN_NEG = 1U, /*!< Expect that positive port input > negative port input */
} acmp_ch_expectation_config_t;

/*!
 * @brief Channel output result
 * Implements : acmp_ch_output_result_t_Class
 */
typedef enum
{
    ACMP_POS_LESS_THAN_NEG = 0U,  /*!< Expect that positive port input < negative port input */
    ACMP_POS_GREAT_THAN_NEG = 1U, /*!< Expect that positive port input > negative port input */
} acmp_ch_output_result_t;

/*!
 * @brief ACMP channel configuration
 * Implements : acmp_ch_config_t_Class
 */
typedef struct
{
    bool enable;
    acmp_ch_expectation_config_t expectation;
} acmp_ch_config_t;

/*! 
 * @brief Channels list (1bit/channel)
 * 
 * Implements : acmp_ch_list_t_Class
 */
typedef uint8_t acmp_ch_list_t;

/*! 
 * @brief Number of channel
 *
 * Implements : acmp_ch_number_t_Class
 */
typedef uint8_t acmp_ch_number_t;

/*!
 * @brief Comparator configuration
 *
 * This structure is used to configure only comparator block module(filtering, sampling, power_mode etc.)
 * Implements : acmp_comparator_config_t_Class
 */
typedef struct
{
    acmp_sample_mode_t sampleMode;         /*!< ACMP sample mode */
    acmp_edge_select_t edgeSelection;      /*!< ACMP edge selection for compare interrupt */
    acmp_hysteresis_t hysteresisLevel;     /*!< ACMP hysteresis level */
#if FEATURE_ACMP_HAS_OFFSET_CONTROL
    bool hardBlockOffset;                  /*!< ACMP hard block offset control */
#endif
    acmp_output_select_t outputSelect;     /*!< ACMP output if filtered */
    acmp_output_polarity_t outputPolarity; /*!< ACMP output polarity */
    acmp_power_mode_t powerMode;           /*!< ACMP power mode */
    bool filterEnable;                     /*!< True if filter is not bypassed */
    acmp_filter_clk_src_t filterClkSrc;    /*!< Select filter clock source */
    uint8_t filterSamplePeriod;            /*!< Filter sample period */
    uint8_t filterSampleCount;             /*!< Number of sample count for filter */
#if FEATURE_ACMP_HAS_AUTODIS
    bool autoDisableHardBlock;             /*!< ACMP disable hard block automatically in one-shot mode */
#endif
#if FEATURE_ACMP_HAS_CLK_SRC_SEL
    acmp_clock_source_t acmpClkSrc;        /*!< ACMP clock source */
#endif
    bool interruptEnable;                  /*!< true if interrupt from comparator is enable */
    bool dmaTriggerEnable;                 /*!< true if DMA transfer trigger from comparator is enable */
} acmp_comparator_config_t;

/*!
 * @brief DAC configuration
 *
 * This structure is used to configure the dac block integrated in analog comparator module
 * Implements : acmp_dac_config_t_Class
 */
typedef struct
{
#if FEATURE_ACMP_HAS_DAC_VOLTAGE_REF_SRC
    acmp_voltage_reference_t voltageReferenceSource; /*!< DAC voltage reference selection */
#endif
    bool enable;                                     /*!< True if DAC is enabled */
#if FEATURE_ACMP_HAS_DAC_OUTPUT
    bool outputEnable;                               /*!< True if DAC output is enabled */
#endif
    uint8_t voltage;                                 /*!< The digital value which is converted to analog signal */
} acmp_dac_config_t;

/*!
 * @brief Analog multiplexer configuration
 *
 * This structure is used to configure the analog multiplexer to select compared signals 
 * Implements : acmp_mux_config_t_Class
 */
typedef struct
{
    acmp_input_source_t positiveInputSrc; /*!< Select positive port input source */
    acmp_input_source_t negativeInputSrc; /*!< Select negative port input source */
    acmp_ch_number_t positiveInputChnSel; /*!< Select which channel is selected for positive port */
    acmp_ch_number_t negativeInputChnSel; /*!< Select which channel is selected for negative port */
} acmp_mux_config_t;

/*!
 * @brief Continuous mode configuration
 *
 * This structure is used to configure the continuous mode 
 * Implements : acmp_continuous_config_t_Class
 */
typedef struct
{
    bool continuousEnable;                 /*!< True if Continuous Mode is enabled.*/
    acmp_continuous_mode_t continuousMode; /*!< Configure continuous mode loop or one-shot */
#if FEATURE_ACMP_HAS_TRIG_MODE_GATE
    bool oneshotTriggerEnable;
#endif
    acmp_fixed_port_t fixedPort;           /*!< Configure fixed port positive or negative */
    uint8_t samplePeriod;                  /*!< Select sample period for a given channel */
    uint8_t samplePosition;                /*!< Select sample position for a given channel */
    bool continuousInterruptEnable;        /*!< True if Continuous Mode interrupt is enabled */
    acmp_ch_config_t channelConfig[8];     /*!< Channel enable and expectation configuration */
} acmp_continuous_config_t;

/*!
 * @brief Defines the acmp module configuration
 *
 * This structure is used to configure all components of acmp module
 * Implements : acmp_config_t_Class
 */
typedef struct
{
    acmp_comparator_config_t *comparatorConfig;
    acmp_dac_config_t *dacConfig;
    acmp_mux_config_t *muxConfig;
    acmp_continuous_config_t *continuousConfig;
} acmp_config_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Reset all registers
 *
 * This function set all ACMP registers to reset values.
 *
 * @param[in] instance - instance number
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_Reset(const uint32_t instance);

/*!
 * @brief Gets a default comparator configuration
 *
 * This function returns a default configuration for the comparator as a configuration structure.
 *
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetDefaultConfig(const acmp_config_t *config);

/*!
 * @brief Configure all comparator features with the given configuration structure
 *
 * This function configures the comparator module with the parameters
 * provided in the config structure.
 *
 * @param[in] instance - instance number
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_Init(const uint32_t instance, const acmp_config_t *config);

/*!
 * @brief Set ACMP channel expected state
 *
 * @param[in] instance - instance number
 * @param[in] state - channel expected state
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_SetExpectation(const uint32_t instance, uint8_t state);

/*!
 * @brief Enable continuous mode
 *
 * @param[in] instance - instance number
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_EnableContinuous(const uint32_t instance);

/*!
 * @brief Disable continuous mode
 *
 * @param[in] instance - instance number
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_DisableContinuous(const uint32_t instance);

/*!
 * @brief Gets the current comparator configuration
 *
 * This function returns the current configuration for comparator as a configuration structure.
 *
 * @param[in] instance - instance number
 * @param[out] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetConfigAll(const uint32_t instance, const acmp_config_t *config);

/*!
 * @brief Configure the DAC component
 *
 * This function configures the DAC with the parameters provided in the config structure.
 *
 * @param[in] instance - instance number
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ConfigDac(const uint32_t instance, const acmp_dac_config_t *config);

/*!
 * @brief Return current configuration for DAC
 *
 * This function returns current configuration only for DAC.
 *
 * @param[in] instance - instance number
 * @param[out] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetDacConfig(const uint32_t instance, acmp_dac_config_t *config);

/*!
 * @brief Configure the ACMP MUX component
 *
 * This function configures the MUX with the parameters provided in the config structure.
 *
 * @param[in] instance - instance number
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ConfigMux(const uint32_t instance, const acmp_mux_config_t *config);

/*!
 * @brief Return configuration only for the MUX component
 *
 * This function returns current configuration to determine which signals go to comparator ports.
 *
 * @param[in] instance - instance number
 * @param[out] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetMuxConfig(const uint32_t instance, acmp_mux_config_t *config);

/*!
 * @brief Configure continuous mode
 *
 * This function configures the continuous mode with the parameters provided in the config structure.
 *
 * @param[in] instance - instance number
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ConfigContinuous(const uint32_t instance, const acmp_continuous_config_t *config);

/*!
 * @brief Get current continuous mode configuration
 *
 * This function returns the current continuous mode configuration.
 *
 * @param[in] instance - instance number
 * @param[out] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetContinuousConfig(const uint32_t instance, acmp_continuous_config_t *config);

/*!
 * @brief Get comparator output flags
 *
 * This function returns in <flags> comparator output flags(rising and falling edge).
 *
 * @param[in] instance - instance number
 * @param[out] flags - pointer to output flags
 *      - NO_EDGE
 *      - RISING_EDGE
 *      - FALLING_EDGE
 *      - BOTH_EDGE
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetOutputFlags(const uint32_t instance, uint8_t *flags);

/*!
 * @brief Clear comparator output flags
 *
 * This function clear comparator output flags(rising and falling edge).
 *
 * @param[in] instance - instance number
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ClearOutputFlags(const uint32_t instance);

/*!
 * @brief Gets input channels change flags
 *
 * This function return in <flags> all input channels flags as uint8_t(1 bite
 *for each channel flag).
 *
 * @param[in] instance - instance number
 * @param[out] flags - pointer to input flags
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetChannelFlags(const uint32_t instance, acmp_ch_list_t *flags);

/*!
 * @brief Clear comparator input channels flags
 *
 * This function clear comparator input channels flags.
 *
 * @param[in] instance - instance number
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ClearChannelFlags(const uint32_t instance);

/*!
 * @brief Configure comparator
 *
 * This function configure only features related with comparator:
 * DMA request, power mode, output select, interrupts enable, polarity,
 * offset, hysteresis, auto disable hard block, internal current control
 * sample mode, filter clock source, filter sample period and position.
 *
 * @param[in] instance - instance number
 * @param[in] config - the configuration structure
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_ConfigComparator(const uint32_t instance, const acmp_comparator_config_t *config);

/*!
 * @brief Return configuration for comparator from ACMP module
 *
 * This function return configuration for features related with comparator:
 * DMA request, power mode, output select, interrupts enable, polarity,
 * offset, hysteresis, auto disable hard block, internal current control
 * sample modes, filter clock source, filter sample period and position.
 *
 * @param[in] instance - instance number
 * @param[out] config - the configuration structure returned
 * @return Operation status
 * @retval STATUS_SUCCESS : Completed successfully.
 */
status_t ACMP_DRV_GetComparatorConfig(const uint32_t instance, acmp_comparator_config_t *config);

/*!
 * @brief Enable ACMP module
 *
 * This function enable the ACMP.
 * @param[in] instance - instance number
 * @return 
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t ACMP_DRV_Enable(const uint32_t instance);

/*!
 * @brief Disable ACMP module
 *
 * This function disable the ACMP.
 * @param[in] instance - instance number
 * @return 
 *        - STATUS_SUCCESS : Completed successfully.
 *        - STATUS_ERROR : Error occurred.
 */
status_t ACMP_DRV_Disable(const uint32_t instance);

/*!
 * @brief This function is to get channel ID in the continuous mode.
 *
 * @param[in] instance - instance number
 * @return channel ID
 */
uint8_t ACMP_DRV_GetChannelId(const uint32_t instance);

/*!
 * @brief This function is to get module output value.
 *
 * @param[in] instance - instance number
 * @return bool output value
 */
bool ACMP_DRV_GetOutput(const uint32_t instance);

/*!
 * @brief This function is to get module output value.
 *
 * @param[in] instance - instance number
 * @param[in] channel - channel number
 * @return bool output value
 */
bool ACMP_DRV_GetChannelOutput(const uint32_t instance, uint8_t channel);

#if defined(__cplusplus)
}
#endif

#endif /* __ACMP_DRIVER_H__*/

/*******************************************************************************
 * EOF
 ******************************************************************************/
