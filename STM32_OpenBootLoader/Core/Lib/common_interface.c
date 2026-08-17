/**
  ******************************************************************************
  * @file    common_interface.c
  * @brief   Common functions used by the Open Bootloader interfaces
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "flash_interface.h"
#include "openbootloader_conf.h"
#include "common_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static Function_Pointer ResetCallback;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Assign the given value to the Main Stack Pointer (MSP).
  * @param  TopOfMainStack  Main Stack Pointer value to set.
  * @retval None.
  */
void Common_SetMsp(uint32_t TopOfMainStack)
{
  __set_MSP(TopOfMainStack);
}

/**
  * @brief  Enable IRQ interrupts.
  * @retval None.
  */
void Common_EnableIrq(void)
{
  __enable_irq();
}

/**
  * @brief  Disable IRQ interrupts.
  * @retval None.
  */
void Common_DisableIrq(void)
{
  __disable_irq();
}

/**
  * @brief  Check whether the target Protection Status is set or not.
  * @retval Returns SET if protection is enabled else RESET.
  */
FlagStatus Common_GetProtectionStatus(void)
{
  FlagStatus status;

  if (OPENBL_FLASH_GetReadOutProtectionLevel() != RDP_LEVEL_0)
  {
    status = SET;
  }
  else
  {
    status = RESET;
  }

  return status;
}

/**
  * @brief  Register a callback to be called at the end of commands processing.
  * @retval None.
  */
void Common_SetPostProcessingCallback(Function_Pointer Callback)
{
  ResetCallback = Callback;
}

/**
  * @brief  Start post processing task.
  * @retval None.
  */
void Common_StartPostProcessing(void)
{
  if (ResetCallback != NULL)
  {
    ResetCallback();

    /* In case there is no system reset, we must reset the callback */
    ResetCallback = NULL;
  }
}
