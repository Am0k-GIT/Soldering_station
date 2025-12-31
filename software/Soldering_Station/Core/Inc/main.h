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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PS_ON_Pin GPIO_PIN_13
#define PS_ON_GPIO_Port GPIOC
#define EXT_SENSOR_Pin GPIO_PIN_0
#define EXT_SENSOR_GPIO_Port GPIOC
#define EXT_SENSOR_EXTI_IRQn EXTI0_IRQn
#define ONBOARD_NTC_Pin GPIO_PIN_1
#define ONBOARD_NTC_GPIO_Port GPIOC
#define HOTAIR_NTC_Pin GPIO_PIN_3
#define HOTAIR_NTC_GPIO_Port GPIOC
#define HOTAIR_TC_Pin GPIO_PIN_0
#define HOTAIR_TC_GPIO_Port GPIOA
#define IRON_TC_Pin GPIO_PIN_1
#define IRON_TC_GPIO_Port GPIOA
#define IRON_NTC_Pin GPIO_PIN_2
#define IRON_NTC_GPIO_Port GPIOA
#define IRON_OUTPUT_Pin GPIO_PIN_3
#define IRON_OUTPUT_GPIO_Port GPIOA
#define IRON_POWER_Pin GPIO_PIN_4
#define IRON_POWER_GPIO_Port GPIOA
#define EN1_B_Pin GPIO_PIN_6
#define EN1_B_GPIO_Port GPIOA
#define EN1_A_Pin GPIO_PIN_7
#define EN1_A_GPIO_Port GPIOA
#define EN1_button_Pin GPIO_PIN_4
#define EN1_button_GPIO_Port GPIOC
#define EN1_button_EXTI_IRQn EXTI4_IRQn
#define IRON_SENSOR_Pin GPIO_PIN_5
#define IRON_SENSOR_GPIO_Port GPIOC
#define IRON_SENSOR_EXTI_IRQn EXTI9_5_IRQn
#define VOLTAGE_24V_Pin GPIO_PIN_0
#define VOLTAGE_24V_GPIO_Port GPIOB
#define K_4_Pin GPIO_PIN_12
#define K_4_GPIO_Port GPIOB
#define K_4_EXTI_IRQn EXTI15_10_IRQn
#define K_3_Pin GPIO_PIN_13
#define K_3_GPIO_Port GPIOB
#define K_3_EXTI_IRQn EXTI15_10_IRQn
#define K_2_Pin GPIO_PIN_14
#define K_2_GPIO_Port GPIOB
#define K_2_EXTI_IRQn EXTI15_10_IRQn
#define K_1_Pin GPIO_PIN_15
#define K_1_GPIO_Port GPIOB
#define K_1_EXTI_IRQn EXTI15_10_IRQn
#define EXT_FAN_Pin GPIO_PIN_8
#define EXT_FAN_GPIO_Port GPIOA
#define HOTAIR_FAN_Pin GPIO_PIN_9
#define HOTAIR_FAN_GPIO_Port GPIOA
#define SSR_ON_Pin GPIO_PIN_10
#define SSR_ON_GPIO_Port GPIOC
#define EN2_button_Pin GPIO_PIN_11
#define EN2_button_GPIO_Port GPIOC
#define EN2_button_EXTI_IRQn EXTI15_10_IRQn
#define TFT_CS_Pin GPIO_PIN_12
#define TFT_CS_GPIO_Port GPIOC
#define TFT_RS_Pin GPIO_PIN_2
#define TFT_RS_GPIO_Port GPIOD
#define TFT_SCK_Pin GPIO_PIN_3
#define TFT_SCK_GPIO_Port GPIOB
#define TFT_RES_Pin GPIO_PIN_4
#define TFT_RES_GPIO_Port GPIOB
#define TFT_SDA_Pin GPIO_PIN_5
#define TFT_SDA_GPIO_Port GPIOB
#define EN2_B_Pin GPIO_PIN_6
#define EN2_B_GPIO_Port GPIOB
#define EN2_A_Pin GPIO_PIN_7
#define EN2_A_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
