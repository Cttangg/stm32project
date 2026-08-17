/**
  ******************************************************************************
  * @file    flash_interface.h
  * @brief   Header for flash_interface.c module (STM32F4 FLASH)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "common_interface.h"
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_BUSY_STATE_ENABLED       ((uint32_t)0xAAAA0000)
#define FLASH_BUSY_STATE_DISABLED      ((uint32_t)0x0000DDDD)

/* F407 (512 KB, single bank): 12 sectors (0..11)
   Bootloader occupies sectors 0..1 (32 KB), App starts at sector 2 */
#define FLASH_BL_SECTOR_COUNT          2U
#define FLASH_SECTOR_COUNT             12U

/* Exported variables --------------------------------------------------------*/
extern OPENBL_MemoryTypeDef FLASH_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void OPENBL_FLASH_JumpToAddress(uint32_t Address);
void OPENBL_FLASH_Lock(void);
void OPENBL_FLASH_OB_Unlock(void);
uint8_t OPENBL_FLASH_Read(uint32_t Address);
void OPENBL_FLASH_SetReadOutProtectionLevel(uint32_t Level);
void OPENBL_FLASH_Write(uint32_t Address, uint8_t *Data, uint32_t DataLength);
void OPENBL_FLASH_Unlock(void);
ErrorStatus OPENBL_FLASH_MassErase(uint8_t *p_Data, uint32_t DataLength);
ErrorStatus OPENBL_FLASH_Erase(uint8_t *p_Data, uint32_t DataLength);
ErrorStatus OPENBL_FLASH_SetWriteProtection(FunctionalState State, uint8_t *ListOfPages, uint32_t Length);
uint32_t OPENBL_FLASH_GetReadOutProtectionLevel(void);
void OPENBL_Enable_BusyState_Flag(void);
void OPENBL_Disable_BusyState_Flag(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* FLASH_INTERFACE_H */
