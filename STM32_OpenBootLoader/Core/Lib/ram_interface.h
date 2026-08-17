/**
  ******************************************************************************
  * @file    ram_interface.h
  * @brief   Header for ram_interface.c module
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAM_INTERFACE_H
#define RAM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern OPENBL_MemoryTypeDef RAM_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
uint8_t OPENBL_RAM_Read(uint32_t Address);
void OPENBL_RAM_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength);
void OPENBL_RAM_JumpToAddress(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* RAM_INTERFACE_H */
