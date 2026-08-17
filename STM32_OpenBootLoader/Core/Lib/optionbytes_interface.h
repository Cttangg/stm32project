/**
  ******************************************************************************
  * @file    optionbytes_interface.h
  * @brief   Header for optionbytes_interface.c module (STM32F4)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPTIONBYTES_INTERFACE_H
#define OPTIONBYTES_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern OPENBL_MemoryTypeDef OB_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t OPENBL_OB_Read(uint32_t Address);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPTIONBYTES_INTERFACE_H */
