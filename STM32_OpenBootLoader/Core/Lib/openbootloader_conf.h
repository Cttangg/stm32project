/**
  ******************************************************************************
  * @file    openbootloader_conf.h
  * @brief   Open Bootloader configuration (STM32F407VET6)
  ******************************************************************************
  * Target : STM32F407VET6 - 512 KByte Flash, 128 KByte SRAM (+64 KByte CCM)
  * Memory map:
  *   0x08000000 .. 0x08007FFF  Bootloader  (32 KByte, sectors 0..1)
  *   0x08008000 .. 0x0807FFFF  Application (480 KByte, sectors 2..11)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPENBOOTLOADER_CONF_H
#define OPENBOOTLOADER_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/* -------------------------------- Device ID ------------------------------- */
/* STM32F407: DEV_ID = 0x413 (reads as MSB 0x04, LSB 0x13) */
#define DEVICE_ID_MSB                     0x04U
#define DEVICE_ID_LSB                     0x13U

/* -------------------------- Definitions for Memories ---------------------- */
#define FLASH_MEM_SIZE                    (512U * 1024U)              /* Size of Flash 512 KByte */
#define FLASH_START_ADDRESS               0x08000000U                /* Flash start address */
#define FLASH_END_ADDRESS                 (FLASH_START_ADDRESS + FLASH_MEM_SIZE)
#define FLASH_BL_SIZE                     (32U * 1024U)              /* Bootloader size 32 KByte (sectors 0..1) */
#define FLASH_APP_START_ADDRESS           (FLASH_START_ADDRESS + FLASH_BL_SIZE)  /* 0x08008000 */

#define RAM_SIZE                          (128U * 1024U)             /* Size of SRAM 128 KByte */
#define RAM_START_ADDRESS                 0x20000000U                /* SRAM start address */
#define RAM_END_ADDRESS                   (RAM_START_ADDRESS + RAM_SIZE)

/* RAM used by the Open Bootloader (its own variables/stack must not be
   touched by the host. The RAM descriptor starts after this area.) */
#define OPENBL_RAM_SIZE                   0x2000U                    /* 8 KByte */

/* Option Bytes (readable area on STM32F4, read by CubeProgrammer on connect) */
#define OB_SIZE                          16U                        /* Option bytes 16 Bytes */
#define OB_START_ADDRESS                 0x1FFFC000U                /* Option bytes read address */
#define OB_END_ADDRESS                   (OB_START_ADDRESS + OB_SIZE)

/* The host always uses this address for Erase/Write-protect commands */
#define OPENBL_DEFAULT_MEM                FLASH_START_ADDRESS

#define RDP_LEVEL_0                       OB_RDP_LEVEL_0
#define RDP_LEVEL_1                       OB_RDP_LEVEL_1
#define RDP_LEVEL_2                       OB_RDP_LEVEL_2

#define AREA_ERROR                        0x0U                       /* Error Address Area */
#define FLASH_AREA                        0x1U                       /* Flash Address Area */
#define RAM_AREA                          0x2U                       /* RAM Address area */
#define OB_AREA                           0x3U                       /* Option bytes Address area */

#define FLASH_MASS_ERASE                  0xFFFFU
#define FLASH_BANK1_ERASE                 0xFFFEU
#define FLASH_BANK2_ERASE                 0xFFFDU

#define INTERFACES_SUPPORTED              1U                         /* USART only */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPENBOOTLOADER_CONF_H */
