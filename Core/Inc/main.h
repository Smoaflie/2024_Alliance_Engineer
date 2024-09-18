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
#include "stm32f1xx_hal.h"

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
#define X_11_Pin GPIO_PIN_1
#define X_11_GPIO_Port GPIOA
#define X_10_Pin GPIO_PIN_2
#define X_10_GPIO_Port GPIOA
#define X_9_Pin GPIO_PIN_3
#define X_9_GPIO_Port GPIOA
#define X_8_Pin GPIO_PIN_4
#define X_8_GPIO_Port GPIOA
#define X_7_Pin GPIO_PIN_5
#define X_7_GPIO_Port GPIOA
#define X_6_Pin GPIO_PIN_6
#define X_6_GPIO_Port GPIOA
#define X_5_Pin GPIO_PIN_7
#define X_5_GPIO_Port GPIOA
#define X_4_Pin GPIO_PIN_0
#define X_4_GPIO_Port GPIOB
#define X_3_Pin GPIO_PIN_1
#define X_3_GPIO_Port GPIOB
#define X_2_Pin GPIO_PIN_10
#define X_2_GPIO_Port GPIOB
#define X_1_Pin GPIO_PIN_11
#define X_1_GPIO_Port GPIOB
#define X_12_Pin GPIO_PIN_14
#define X_12_GPIO_Port GPIOB
#define X_13_Pin GPIO_PIN_15
#define X_13_GPIO_Port GPIOB
#define X_14_Pin GPIO_PIN_8
#define X_14_GPIO_Port GPIOA
#define X_15_Pin GPIO_PIN_9
#define X_15_GPIO_Port GPIOA
#define X_16_Pin GPIO_PIN_10
#define X_16_GPIO_Port GPIOA
#define X_17_Pin GPIO_PIN_15
#define X_17_GPIO_Port GPIOA
#define X_18_Pin GPIO_PIN_3
#define X_18_GPIO_Port GPIOB
#define X_19_Pin GPIO_PIN_4
#define X_19_GPIO_Port GPIOB
#define X_20_Pin GPIO_PIN_5
#define X_20_GPIO_Port GPIOB
#define X_21_Pin GPIO_PIN_6
#define X_21_GPIO_Port GPIOB
#define X_22_Pin GPIO_PIN_7
#define X_22_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
