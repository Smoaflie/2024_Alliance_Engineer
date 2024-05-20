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
#include "stm32h7xx_hal.h"

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
#define Power_5V_EN_Pin GPIO_PIN_15
#define Power_5V_EN_GPIO_Port GPIOC
#define Z_limit_detect_Pin GPIO_PIN_2
#define Z_limit_detect_GPIO_Port GPIOA
#define Z_limit_detect_EXTI_IRQn EXTI2_IRQn
#define YAW_limit_detect_Pin GPIO_PIN_9
#define YAW_limit_detect_GPIO_Port GPIOE
#define redLight_detect_Pin GPIO_PIN_13
#define redLight_detect_GPIO_Port GPIOE
#define KEY_Pin GPIO_PIN_15
#define KEY_GPIO_Port GPIOA
#define airpump_linear_Pin GPIO_PIN_0
#define airpump_linear_GPIO_Port GPIOE
#define airpump_arm_Pin GPIO_PIN_1
#define airpump_arm_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
