#ifndef MOTOR_HW_YTM32_H
#define MOTOR_HW_YTM32_H

#include <stdbool.h>
#include <stdint.h>

#include "device_registers.h"

#define MOTOR_HW_ADC_IRQ_DEBUG_GPIO              (GPIOC)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN               (7U)
#define MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK          (1UL << MOTOR_HW_ADC_IRQ_DEBUG_PIN)

typedef struct
{
    uint16_t current_a_raw;
    uint16_t current_b_raw;
    uint16_t current_c_raw;
    uint16_t vbus_raw;
    bool overrun;
} motor_adc_raw_frame_t;

void MotorHwYtm32_Init(void);
void MotorHwYtm32_StartSpeedLoopTimer(void);
void MotorHwYtm32_ClearSpeedLoopIrq(void);
bool MotorHwYtm32_ReadSoftwareFrame(motor_adc_raw_frame_t *frame);
void MotorHwYtm32_EnableFastLoopSampling(void);
void MotorHwYtm32_DisableFastLoopSampling(void);
bool MotorHwYtm32_ReadTriggeredFrame(motor_adc_raw_frame_t *frame);
void MotorHwYtm32_StartPwmTimeBase(void);
void MotorHwYtm32_StopPwmTimeBase(void);
void MotorHwYtm32_SetOutputsMasked(bool masked);
void MotorHwYtm32_ApplyPhaseDuty(float dutyU, float dutyV, float dutyW);

static inline void MotorHwYtm32_SetAdcIrqDebugPinHigh(void)
{
    MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PSOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

static inline void MotorHwYtm32_SetAdcIrqDebugPinLow(void)
{
    MOTOR_HW_ADC_IRQ_DEBUG_GPIO->PCOR = MOTOR_HW_ADC_IRQ_DEBUG_PIN_MASK;
}

#endif /* MOTOR_HW_YTM32_H */
