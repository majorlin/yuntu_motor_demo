/**
 * @file motor_hw_ytm32.h
 * @brief Hardware Abstraction Layer for YTM32 features used in FOC.
 *
 * Implements hardware-specific initialisation, timer synchronization,
 * ADC triggering, and PWM configurations for the YTM32 microcontroller.
 */

#ifndef MOTOR_HW_YTM32_H
#define MOTOR_HW_YTM32_H

#include <stdbool.h>
#include <stdint.h>

#include "device_registers.h"

#define MOTOR_HW_ADC_IRQ_DEBUG_GPIO              (GPIOC)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN               (7U)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK          (1UL << MOTOR_HW_ADC_IRQ_DEBUG_PIN)

/**
 * @brief Hardware-independent ADC raw reading frame.
 */
typedef struct
{
    uint16_t current_a_raw;  /**< Phase A raw 12-bit ADC count. */
    uint16_t current_b_raw;  /**< Phase B raw 12-bit ADC count. */
    uint16_t current_c_raw;  /**< Phase C raw 12-bit ADC count. */
    uint16_t vbus_raw;       /**< Bus voltage raw 12-bit ADC count. */
    bool overrun;            /**< Flag set if hardware FIFO overran. */
} motor_adc_raw_frame_t;

/**
 * @brief Initializes module clocks, IO, routing, ADC, eTMR, and pTMR peripherals.
 */
void MotorHwYtm32_Init(void);

/**
 * @brief Start the periodic pTMR timer for the slow speed loop.
 */
void MotorHwYtm32_StartSpeedLoopTimer(void);

/**
 * @brief Clear the interrupt flag for the slow speed loop timer.
 */
void MotorHwYtm32_ClearSpeedLoopIrq(void);

/**
 * @brief Trigger a software ADC conversion and wait for it to finish.
 * @param[out] frame Pointer to store the acquired raw data.
 * @return True if read succeeded, false if timed out.
 */
bool MotorHwYtm32_ReadSoftwareFrame(motor_adc_raw_frame_t *frame);

/**
 * @brief Configure ADC for eTMR hardware triggers and enable interrupts.
 */
void MotorHwYtm32_EnableFastLoopSampling(void);

/**
 * @brief Disable ADC interrupts and switch back to software trigger mode.
 */
void MotorHwYtm32_DisableFastLoopSampling(void);

/**
 * @brief Retrieve the most recent ADC frame fetched via hardware trigger.
 * @param[out] frame Pointer to store the acquired raw data.
 * @return True on success.
 */
bool MotorHwYtm32_ReadTriggeredFrame(motor_adc_raw_frame_t *frame);

/**
 * @brief Enable the main eTMR timebase used for PWM.
 */
void MotorHwYtm32_StartPwmTimeBase(void);

/**
 * @brief Halt the main eTMR timebase used for PWM.
 */
void MotorHwYtm32_StopPwmTimeBase(void);

/**
 * @brief Mask or unmask the physical PWM output pins.
 * @param masked True to mask (high-Z/safe), False to connect outputs.
 */
void MotorHwYtm32_SetOutputsMasked(bool masked);

/**
 * @brief Program the phase modulation duty cycle into the eTMR shadow registers.
 * @param dutyU Duty cycle for phase U [0.0 - 1.0].
 * @param dutyV Duty cycle for phase V [0.0 - 1.0].
 * @param dutyW Duty cycle for phase W [0.0 - 1.0].
 */
void MotorHwYtm32_ApplyPhaseDuty(float dutyU, float dutyV, float dutyW);

/**
 * @brief Assert the debug scope probe pin used to measure ADC IRQ duration.
 */
static inline void MotorHwYtm32_SetAdcIrqDebugPinHigh(void)
{
    MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PSOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

/**
 * @brief De-assert the debug scope probe pin.
 */
static inline void MotorHwYtm32_SetAdcIrqDebugPinLow(void)
{
    MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PCOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

/**
 * @brief Hardware-independent BEMF voltage raw reading frame.
 *
 * Used during WIND_DETECT to sample phase voltages through the
 * 1/5 resistor divider network with all MOSFETs off.
 */
typedef struct
{
    uint16_t bemf_u_raw;   /**< BEMF phase U raw 12-bit ADC count. */
    uint16_t bemf_v_raw;   /**< BEMF phase V raw 12-bit ADC count. */
    uint16_t bemf_w_raw;   /**< BEMF phase W raw 12-bit ADC count. */
    uint16_t bemf_com_raw; /**< BEMF virtual neutral raw 12-bit ADC count. */
} motor_bemf_raw_frame_t;

/**
 * @brief Switch ADC sequence from current sensing to BEMF voltage sensing.
 *
 * Reconfigures the ADC CHSEL registers to sample BEMF_U, BEMF_V, BEMF_W,
 * BEMF_COM on the next hardware trigger.  Must be called when no ADC
 * conversion is in progress (between EOSEQ and next trigger).
 */
void MotorHwYtm32_SwitchAdcToBemfSensing(void);

/**
 * @brief Switch ADC sequence back to current sensing (normal FOC mode).
 */
void MotorHwYtm32_SwitchAdcToCurrentSensing(void);

/**
 * @brief Read a BEMF voltage frame from the ADC FIFO.
 * @param[out] frame  Pointer to store the acquired BEMF raw data.
 * @return True on success.
 */
bool MotorHwYtm32_ReadBemfFrame(motor_bemf_raw_frame_t *frame);

#endif /* MOTOR_HW_YTM32_H */
