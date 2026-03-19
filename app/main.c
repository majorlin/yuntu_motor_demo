/* USER CODE BEGIN Header */
/* you can remove the copyright */
/*
 * Copyright 2020-2025 Yuntu Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * @file main.c
 * @brief 
 * 
 */

/* USER CODE END Header */
#include "sdk_project_config.h"
/* Includes ------------------------------------------------------------------*/

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor_control.h"
#include "motor_user_config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_BUTTON_START_STOP_PIN                (11U)
#define APP_BUTTON_DIRECTION_PIN                 (3U)
#define APP_BUTTON_ACTIVE_LEVEL                  (0U)
#define APP_BUTTON_DEBOUNCE_MS                   (30U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
typedef struct
{
    volatile bool startStopPending;
    volatile bool directionPending;
    volatile uint32_t startStopIrqTickMs;
    volatile uint32_t directionIrqTickMs;
    float appliedSpeedCmdRpm;
} app_control_ctx_t;

volatile bool g_motorRunCmd = (MOTOR_APP_AUTO_START != 0U);
volatile int32_t g_motorDirectionCmd = MOTOR_CFG_DEFAULT_DIRECTION;
volatile float g_motorSpeedCmdRpm = MOTOR_CFG_DEFAULT_TARGET_RPM;

static app_control_ctx_t s_appCtrl;
/* USER CODE END PV */

/* Private function declare --------------------------------------------------*/
/* USER CODE BEGIN PFDC */
static void App_InitControlInputs(void);
static void App_ProcessButtons(void);
static void App_ProcessSpeedCommand(void);
/* USER CODE END PFDC */
static void Board_Init(void);

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void App_InitControlInputs(void)
{
    s_appCtrl.startStopPending = false;
    s_appCtrl.directionPending = false;
    s_appCtrl.startStopIrqTickMs = 0U;
    s_appCtrl.directionIrqTickMs = 0U;
    s_appCtrl.appliedSpeedCmdRpm = g_motorSpeedCmdRpm;

    PINS_DRV_ClearPinIntFlagCmd(GPIOE, APP_BUTTON_START_STOP_PIN);
    PINS_DRV_ClearPinIntFlagCmd(GPIOE, APP_BUTTON_DIRECTION_PIN);
    INT_SYS_SetPriority(GPIOE_IRQn, 3U);
    INT_SYS_EnableIRQ(GPIOE_IRQn);
}

static void App_ProcessButtons(void)
{
    const uint32_t nowMs = MotorControl_GetTickMs();

    if (s_appCtrl.startStopPending &&
        ((uint32_t)(nowMs - s_appCtrl.startStopIrqTickMs) >= APP_BUTTON_DEBOUNCE_MS))
    {
        s_appCtrl.startStopPending = false;
        if (PINS_DRV_ReadPin(GPIOE, APP_BUTTON_START_STOP_PIN) == APP_BUTTON_ACTIVE_LEVEL)
        {
            g_motorRunCmd = !g_motorRunCmd;
            MotorControl_Enable(g_motorRunCmd);
        }
    }

    if (s_appCtrl.directionPending &&
        ((uint32_t)(nowMs - s_appCtrl.directionIrqTickMs) >= APP_BUTTON_DEBOUNCE_MS))
    {
        s_appCtrl.directionPending = false;
        if (PINS_DRV_ReadPin(GPIOE, APP_BUTTON_DIRECTION_PIN) == APP_BUTTON_ACTIVE_LEVEL)
        {
            const motor_status_t *status = MotorControl_GetStatus();
            const int8_t nextDirection = (g_motorDirectionCmd >= 0) ? -1 : 1;

            if ((status->state == MOTOR_STATE_STOP) && MotorControl_SetDirection(nextDirection))
            {
                g_motorDirectionCmd = nextDirection;
            }
        }
    }
}

static void App_ProcessSpeedCommand(void)
{
    if (g_motorSpeedCmdRpm != s_appCtrl.appliedSpeedCmdRpm)
    {
        MotorControl_SetTargetRpm(g_motorSpeedCmdRpm);
        s_appCtrl.appliedSpeedCmdRpm = g_motorSpeedCmdRpm;
    }
}

void GPIOE_IRQHandler(void)
{
    const uint32_t nowMs = MotorControl_GetTickMs();
    const uint32_t irqFlags = GPIOE->PIFR;

    if ((irqFlags & (1UL << APP_BUTTON_START_STOP_PIN)) != 0U)
    {
        PINS_DRV_ClearPinIntFlagCmd(GPIOE, APP_BUTTON_START_STOP_PIN);
        s_appCtrl.startStopPending = true;
        s_appCtrl.startStopIrqTickMs = nowMs;
    }

    if ((irqFlags & (1UL << APP_BUTTON_DIRECTION_PIN)) != 0U)
    {
        PINS_DRV_ClearPinIntFlagCmd(GPIOE, APP_BUTTON_DIRECTION_PIN);
        s_appCtrl.directionPending = true;
        s_appCtrl.directionIrqTickMs = nowMs;
    }
}
/* USER CODE END 0 */


/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
    /* USER CODE BEGIN 1 */
    /* USER CODE END 1 */ 
    Board_Init();
    /* USER CODE BEGIN 2 */
    INT_SYS_ConfigInit();
    MotorControl_Init();
    MotorControl_SetTargetRpm(g_motorSpeedCmdRpm);
    App_InitControlInputs();
    MotorControl_Enable(g_motorRunCmd);
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        App_ProcessButtons();
        App_ProcessSpeedCommand();
    }
    /* USER CODE END 3 */
}

static void Board_Init(void)
{
    CLOCK_SYS_Init(g_clockManConfigsArr,CLOCK_MANAGER_CONFIG_CNT,g_clockManCallbacksArr,CLOCK_MANAGER_CALLBACK_CNT);
    if(STATUS_SUCCESS != CLOCK_SYS_UpdateConfiguration(CLOCK_MANAGER_ACTIVE_INDEX,CLOCK_MANAGER_POLICY_AGREEMENT))
    {
        /* USER CODE BEGIN ERROR_HANDLER 1 */
        /* USER CODE END ERROR_HANDLER 1 */
    }
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS0,g_pin_mux_InitConfigArr0);
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */
