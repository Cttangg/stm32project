/**
  ******************************************************************************
  * @file    app_openbootloader.h
  * @brief   Application level glue for the Open Bootloader (project specific)
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef APP_OPENBOOTLOADER_H
#define APP_OPENBOOTLOADER_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "openbootloader_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* FLASH_BL_SIZE and FLASH_APP_START_ADDRESS are defined in openbootloader_conf.h */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void OpenBootloader_DeInit(void);
void OPENBL_OB_Launch(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* APP_OPENBOOTLOADER_H */
