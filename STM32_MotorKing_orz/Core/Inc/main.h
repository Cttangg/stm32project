/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define EN_2_Pin GPIO_PIN_15
#define EN_2_GPIO_Port GPIOB
#define EN_1_Pin GPIO_PIN_8
#define EN_1_GPIO_Port GPIOD
#define MS1_2_Pin GPIO_PIN_9
#define MS1_2_GPIO_Port GPIOD
#define MS1_1_Pin GPIO_PIN_10
#define MS1_1_GPIO_Port GPIOD
#define MS2_2_Pin GPIO_PIN_11
#define MS2_2_GPIO_Port GPIOD
#define MS2_1_Pin GPIO_PIN_12
#define MS2_1_GPIO_Port GPIOD
#define STEP_2_Pin GPIO_PIN_13
#define STEP_2_GPIO_Port GPIOD
#define STEP_1_Pin GPIO_PIN_14
#define STEP_1_GPIO_Port GPIOD
#define DIR_2_Pin GPIO_PIN_15
#define DIR_2_GPIO_Port GPIOD
#define DIR_1_Pin GPIO_PIN_6
#define DIR_1_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
