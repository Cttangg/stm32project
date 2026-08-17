/**
  ******************************************************************************
  * @file    ram_interface.c
  * @brief   RAM access functions (used for RAM download / Go from RAM)
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "openbl_mem.h"
#include "common_interface.h"
#include "openbl_core.h"
#include "ram_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
OPENBL_MemoryTypeDef RAM_Descriptor =
{
  RAM_START_ADDRESS + OPENBL_RAM_SIZE,  /* protect the OpenBootloader RAM area */
  RAM_END_ADDRESS,
  RAM_SIZE,
  RAM_AREA,
  OPENBL_RAM_Read,
  OPENBL_RAM_Write,
  NULL,
  NULL,
  OPENBL_RAM_JumpToAddress,
  NULL,
  NULL
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Read a byte from a given address.
  * @param  Address The address to be read.
  * @retval Returns the read value.
  */
uint8_t OPENBL_RAM_Read(uint32_t Address)
{
  return (*(uint8_t *)(Address));
}

/**
  * @brief  Write data in RAM memory.
  * @param  Address The address where the data will be written.
  * @param  pData The data to be written.
  * @param  DataLength The length of the data to be written.
  * @retval None.
  */
void OPENBL_RAM_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength)
{
  uint32_t index;
  uint32_t aligned_length = DataLength;

  if (aligned_length & 0x3U)
  {
    aligned_length = (aligned_length & 0xFFFFFFFCU) + 4U;
  }

  for (index = 0U; index < aligned_length; index += 4U)
  {
    *(__IO uint32_t *)(Address + index) = *(__IO uint32_t *)(pData + index);
  }
}

/**
  * @brief  Jump to the application at the given address.
  * @param  Address The address where the function will jump.
  * @retval None.
  */
void OPENBL_RAM_JumpToAddress(uint32_t Address)
{
  Function_Pointer jump_to_address;

  /* De-initialize all HW resources used by the Open Bootloader */
  OPENBL_DeInit();

  /* Enable IRQ */
  Common_EnableIrq();

  jump_to_address = (Function_Pointer)(*(__IO uint32_t *)(Address + 4U));

  /* Initialize user application's stack pointer */
  Common_SetMsp(*(__IO uint32_t *)Address);

  jump_to_address();
}
