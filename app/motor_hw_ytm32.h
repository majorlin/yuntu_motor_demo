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

#define MOTOR_HW_ADC_IRQ_DEBUG_GPIO (GPIOC)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN (7U)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK (1UL << MOTOR_HW_ADC_IRQ_DEBUG_PIN)

#define MOTOR_HW_ADC_CURRENT_A_CH (ADC_INPUTCHAN_EXT2)
#define MOTOR_HW_ADC_CURRENT_B_CH (ADC_INPUTCHAN_EXT3)
#define MOTOR_HW_ADC_CURRENT_C_CH (ADC_INPUTCHAN_EXT4)
#define MOTOR_HW_ADC_VBUS_CH (ADC_INPUTCHAN_EXT5)

/* --- BEMF voltage sensing ADC channels (for passive wind detection) ---
 * From pin_mux.c:
 *   PTA_1  → ADC0_SE1  → BEMF_U
 *   PTC_3  → ADC0_SE11 → BEMF_V
 *   PTC_2  → ADC0_SE10 → BEMF_W
 *   PTA_0  → ADC0_SE0  → BEMF_COM (virtual neutral)
 * All use a 1/5 resistor divider (4k/1k). */
#define MOTOR_HW_ADC_BEMF_U_CH ADC_INPUTCHAN_EXT1
#define MOTOR_HW_ADC_BEMF_V_CH ADC_INPUTCHAN_EXT11
#define MOTOR_HW_ADC_BEMF_W_CH ADC_INPUTCHAN_EXT10
#define MOTOR_HW_ADC_BEMF_COM_CH ADC_INPUTCHAN_EXT0

/** Number of channels in the normal current-sensing sequence. */
#define MOTOR_HW_ADC_CURRENT_SEQ_LEN (4U)
/** Number of channels in the passive BEMF sensing sequence. */
#define MOTOR_HW_ADC_BEMF_SEQ_LEN (4U)
/** Scatter-buffer size — must be >= highest channel index + 1 (EXT11 = 0x0B).
 */
#define MOTOR_HW_ADC_BUFFER_SIZE (32U)

/**
 * @brief Initializes module clocks, IO, routing, ADC, eTMR, and pTMR
 * peripherals.
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
 * @brief Configure ADC for eTMR hardware triggers and enable interrupts.
 */
void MotorHwYtm32_EnableFastLoopSampling(void);

/**
 * @brief Disable ADC interrupts and switch back to software trigger mode.
 */
void MotorHwYtm32_DisableFastLoopSampling(void);

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
 * @brief Program the phase modulation duty cycle into the eTMR shadow
 * registers.
 * @param dutyU Duty cycle for phase U [0.0 - 1.0].
 * @param dutyV Duty cycle for phase V [0.0 - 1.0].
 * @param dutyW Duty cycle for phase W [0.0 - 1.0].
 */
void MotorHwYtm32_ApplyPhaseDuty(float dutyU, float dutyV, float dutyW);

/**
 * @brief Assert the debug scope probe pin used to measure ADC IRQ duration.
 */
static inline void MotorHwYtm32_SetAdcIrqDebugPinHigh(void) {
  MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PSOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

/**
 * @brief De-assert the debug scope probe pin.
 */
static inline void MotorHwYtm32_SetAdcIrqDebugPinLow(void) {
  MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PCOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

/**
 * @brief Hardware-independent ADC raw reading frame.
 */
typedef struct {
  uint16_t current_a_raw; /**< Phase A raw 12-bit ADC count. */
  uint16_t current_b_raw; /**< Phase B raw 12-bit ADC count. */
  uint16_t current_c_raw; /**< Phase C raw 12-bit ADC count. */
  uint16_t vbus_raw;      /**< Bus voltage raw 12-bit ADC count. */
  bool overrun;           /**< Flag set if hardware FIFO overran. */
} motor_adc_raw_frame_t;

/**
 * @brief Hardware-independent BEMF voltage raw reading frame.
 */
typedef struct {
  uint16_t bemf_u_raw;   /**< BEMF phase U raw 12-bit ADC count. */
  uint16_t bemf_v_raw;   /**< BEMF phase V raw 12-bit ADC count. */
  uint16_t bemf_w_raw;   /**< BEMF phase W raw 12-bit ADC count. */
  uint16_t bemf_com_raw; /**< BEMF virtual neutral raw 12-bit ADC count. */
} motor_bemf_raw_frame_t;

/**
 * @brief Trigger a software ADC conversion and wait for it to finish.
 * @param[out] frame Pointer to store the acquired raw data.
 * @return True if read succeeded, false if timed out.
 */
bool MotorHwYtm32_ReadSoftwareFrame(motor_adc_raw_frame_t *frame);

/**
 * @brief Retrieve the most recent ADC frame fetched via hardware trigger.
 * @param[out] frame Pointer to store the acquired raw data.
 * @return True on success.
 */
bool MotorHwYtm32_ReadTriggeredFrame(motor_adc_raw_frame_t *frame);

/**
 * @brief Return the number of ADC FIFO words produced by the active sequence.
 */
uint8_t MotorHwYtm32_GetActiveAdcSequenceLength(void);

/**
 * @brief Switch ADC sequence from current sensing to BEMF voltage sensing.
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
