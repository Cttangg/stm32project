/**
  ******************************************************************************
  * @file    platform.h
  * @brief   Target platform header for STM32 Open Bootloader
  ******************************************************************************
  * This header selects the HAL device header used by the whole Open Bootloader.
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef PLATFORM_H
#define PLATFORM_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* PLATFORM_H */
