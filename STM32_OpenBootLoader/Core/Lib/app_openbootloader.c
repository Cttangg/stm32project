/**
  ******************************************************************************
  * @file    app_openbootloader.c
  * @brief   Application level glue for the Open Bootloader (project specific)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "openbl_core.h"
#include "app_openbootloader.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  De-initialize all the Open Bootloader resources before jumping
  *         to the user application (called by OPENBL_DeInit()).
  * @retval None.
  */
void OpenBootloader_DeInit(void)
{
  /* De-initialize the registered interfaces (e.g. USART) */
  OPENBL_InterfacesDeInit();

  /* Stop the SysTick interrupt to avoid it firing in the application
     before the application re-configures its own vector table */
  HAL_SuspendTick();
}

/**
  * @brief  Reload option bytes and trigger a system reset.
  *         Used as a post-processing callback after OB changes.
  * @retval None.
  */
void OPENBL_OB_Launch(void)
{
  HAL_FLASH_OB_Launch();
  NVIC_SystemReset();
}
