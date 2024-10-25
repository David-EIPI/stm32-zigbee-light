/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32wbxx_hal.h"
#include "app_conf.h"
#include "app_entry.h"
#include "app_common.h"

#include "stm32wbxx_ll_tim.h"
#include "stm32wbxx_ll_system.h"
#include "stm32wbxx_ll_gpio.h"
#include "stm32wbxx_ll_exti.h"
#include "stm32wbxx_ll_bus.h"
#include "stm32wbxx_ll_cortex.h"
#include "stm32wbxx_ll_rcc.h"
#include "stm32wbxx_ll_utils.h"
#include "stm32wbxx_ll_pwr.h"
#include "stm32wbxx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
enum operationModes {
		OP_AUTO = 1,
		OP_FULL = 2,
		OP_DIMMED = 3,
		OP_OFF = 4
};
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
extern int32_t dimLevel; // light brightness when in manual mode
extern int32_t ambientThreshold; // ambient light threshold
extern int32_t motionSensitivity; // motion sensor sensitivity
extern int32_t motionLight; // time to keep the light on after motion detection
extern int32_t nightLight; // time to keep dimmed light after dusk
extern uint16_t operationMode; // 1-4 = Auto, On, Dim, Off

extern int led_blink_count;

extern int32_t phase_correction;
extern uint8_t trailing_edge;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

void delay_us(uint16_t au16_us);
void delay_ms(uint16_t au16_ms);

void update_motion_sensitivity(int32_t *value);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DLINK_Pin LL_GPIO_PIN_8
#define DLINK_GPIO_Port GPIOB
#define SERIN_Pin LL_GPIO_PIN_9
#define SERIN_GPIO_Port GPIOB
#define TRIAC_CTRL_Pin LL_GPIO_PIN_0
#define TRIAC_CTRL_GPIO_Port GPIOA
#define ZERO_CROSS_Pin LL_GPIO_PIN_8
#define ZERO_CROSS_GPIO_Port GPIOA
#define SW1_P1_Pin LL_GPIO_PIN_0
#define SW1_P1_GPIO_Port GPIOB
#define SW1_P2_Pin LL_GPIO_PIN_1
#define SW1_P2_GPIO_Port GPIOB
#define BOARDLED_Pin LL_GPIO_PIN_4
#define BOARDLED_GPIO_Port GPIOE
#define SW1_P4_Pin LL_GPIO_PIN_15
#define SW1_P4_GPIO_Port GPIOA
#define SW1_P5_Pin LL_GPIO_PIN_3
#define SW1_P5_GPIO_Port GPIOB
#define SW2_P1_Pin LL_GPIO_PIN_4
#define SW2_P1_GPIO_Port GPIOB
#define SW2_P2_Pin LL_GPIO_PIN_5
#define SW2_P2_GPIO_Port GPIOB
#define SW2_P4_Pin LL_GPIO_PIN_6
#define SW2_P4_GPIO_Port GPIOB
#define SW2_P5_Pin LL_GPIO_PIN_7
#define SW2_P5_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define lengthof(arr) (sizeof(arr)/sizeof((arr)[0]))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
