/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PUMP_VOLTAGE_Pin GPIO_PIN_0
#define PUMP_VOLTAGE_GPIO_Port GPIOA
#define WEJSCIE_2_Pin GPIO_PIN_2
#define WEJSCIE_2_GPIO_Port GPIOA
#define WEJSCIE_1_Pin GPIO_PIN_3
#define WEJSCIE_1_GPIO_Port GPIOA
#define PUMP_CURRENT_Pin GPIO_PIN_4
#define PUMP_CURRENT_GPIO_Port GPIOA
#define PUMP_POWER_Pin GPIO_PIN_5
#define PUMP_POWER_GPIO_Port GPIOA
#define PUMP_ON_Pin GPIO_PIN_0
#define PUMP_ON_GPIO_Port GPIOB
#define LED_ON_Pin GPIO_PIN_15
#define LED_ON_GPIO_Port GPIOA
#define SEC_IN_Pin GPIO_PIN_6
#define SEC_IN_GPIO_Port GPIOB
#define SEC_IN_EXTI_IRQn EXTI9_5_IRQn
#define SEC_OUT_Pin GPIO_PIN_7
#define SEC_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
