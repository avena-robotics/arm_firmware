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
#include "motorcontrol.h"

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
#define SEC_IN_Pin GPIO_PIN_14
#define SEC_IN_GPIO_Port GPIOC
#define SEC_OUT_Pin GPIO_PIN_15
#define SEC_OUT_GPIO_Port GPIOC
#define ABSOLUTE_ENCODER_CS_Pin GPIO_PIN_3
#define ABSOLUTE_ENCODER_CS_GPIO_Port GPIOC
#define M1_BUS_VOLTAGE_Pin GPIO_PIN_0
#define M1_BUS_VOLTAGE_GPIO_Port GPIOA
#define M1_CURR_SHUNT_U_Pin GPIO_PIN_1
#define M1_CURR_SHUNT_U_GPIO_Port GPIOA
#define M1_OPAMP1_OUT_Pin GPIO_PIN_2
#define M1_OPAMP1_OUT_GPIO_Port GPIOA
#define M1_OPAMP1_EXT_GAIN_Pin GPIO_PIN_3
#define M1_OPAMP1_EXT_GAIN_GPIO_Port GPIOA
#define BEARING_TEMPERATURE_Pin GPIO_PIN_4
#define BEARING_TEMPERATURE_GPIO_Port GPIOA
#define M1_OPAMP2_OUT_Pin GPIO_PIN_6
#define M1_OPAMP2_OUT_GPIO_Port GPIOA
#define M1_CURR_SHUNT_V_Pin GPIO_PIN_7
#define M1_CURR_SHUNT_V_GPIO_Port GPIOA
#define M1_TEMPERATURE_Pin GPIO_PIN_4
#define M1_TEMPERATURE_GPIO_Port GPIOC
#define M1_OPAMP2_EXT_GAIN_Pin GPIO_PIN_5
#define M1_OPAMP2_EXT_GAIN_GPIO_Port GPIOC
#define M1_CURR_SHUNT_W_Pin GPIO_PIN_0
#define M1_CURR_SHUNT_W_GPIO_Port GPIOB
#define GD_WAKE_Pin GPIO_PIN_7
#define GD_WAKE_GPIO_Port GPIOE
#define M1_PWM_UL_Pin GPIO_PIN_8
#define M1_PWM_UL_GPIO_Port GPIOE
#define M1_PWM_UH_Pin GPIO_PIN_9
#define M1_PWM_UH_GPIO_Port GPIOE
#define M1_PWM_VL_Pin GPIO_PIN_10
#define M1_PWM_VL_GPIO_Port GPIOE
#define M1_PWM_VH_Pin GPIO_PIN_11
#define M1_PWM_VH_GPIO_Port GPIOE
#define M1_PWM_WL_Pin GPIO_PIN_12
#define M1_PWM_WL_GPIO_Port GPIOE
#define M1_PWM_WH_Pin GPIO_PIN_13
#define M1_PWM_WH_GPIO_Port GPIOE
#define GD_READY_Pin GPIO_PIN_14
#define GD_READY_GPIO_Port GPIOE
#define GD_NFAULT_Pin GPIO_PIN_15
#define GD_NFAULT_GPIO_Port GPIOE
#define GD_SCL_Pin GPIO_PIN_8
#define GD_SCL_GPIO_Port GPIOC
#define GD_SDA_Pin GPIO_PIN_9
#define GD_SDA_GPIO_Port GPIOC
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define M1_ENCODER_A_Pin GPIO_PIN_6
#define M1_ENCODER_A_GPIO_Port GPIOB
#define M1_ENCODER_B_Pin GPIO_PIN_7
#define M1_ENCODER_B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

#define KT									(float) 0.1118
#define POLE_PAIRS					(uint16_t) 14
#define SECTOR_SIZE					(uint16_t) 14
#define MOTOR_TYPE					RI70
#define PCB_VERSION 				0x030200
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
