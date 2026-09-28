/**
  ******************************************************************************
  * @file    app_boot_interface.h
  * @brief   Reserved interface between the Bootloader and the upper-layer APP.
  *
  *          To build an APP that runs under the ST Open Bootloader:
  *            1. Link the APP at 0x08008000 (FLASH origin), length 480 KByte.
  *            2. Call APP_SetVectorTable() as the FIRST instruction in main()
  *               (before HAL_Init / any interrupt can fire).
  *            3. Call APP_JumpToBootloader() whenever the APP wants to return
  *               to the bootloader for a firmware update.
  *
  *          These values MUST match the ones used by the bootloader
  *          (openbootloader_conf.h + openbl/main.c).
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_BOOT_INTERFACE_H
#define APP_BOOT_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "stm32f4xx.h"

/* Exported constants --------------------------------------------------------*/
#define APP_BASE_ADDRESS      0x08008000U  /* App flash start (32 KB bootloader) */
#define APP_BOOT_FLAG_ADDR    0x2001FFFCU  /* Last 4 bytes of 128 KB SRAM */
#define APP_BOOT_MAGIC        0xDEADBEEFU  /* "stay in bootloader" request token */

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  Point the vector table to the APP (must be called first thing in main).
  * @retval None.
  */
static inline void APP_SetVectorTable(void)
{
  SCB->VTOR = APP_BASE_ADDRESS;
}

/**
  * @brief  Request the bootloader to stay and wait for the programmer.
  *         Writes a magic token in RAM, then resets the MCU.
  * @retval None.
  */
static inline void APP_JumpToBootloader(void)
{
  *((volatile uint32_t *)APP_BOOT_FLAG_ADDR) = APP_BOOT_MAGIC;
  NVIC_SystemReset();
}

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* APP_BOOT_INTERFACE_H */
