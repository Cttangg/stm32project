/**
  ******************************************************************************
  * @file    optionbytes_interface.c
  * @brief   Option Bytes access functions for STM32F4.
  *
  *          On STM32F4 the option bytes are readable at 0x1FFFC000 (16 bytes).
  *          CubeProgrammer reads this area on connect to display the option
  *          bytes, so it must be registered in the memory map.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "openbl_mem.h"
#include "openbootloader_conf.h"
#include "optionbytes_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
OPENBL_MemoryTypeDef OB_Descriptor =
{
  OB_START_ADDRESS,
  OB_END_ADDRESS,
  OB_SIZE,
  OB_AREA,
  OPENBL_OB_Read,
  NULL,   /* write not supported (programmed via FLASH registers) */
  NULL,
  NULL,
  NULL,
  NULL,
  NULL
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Read a byte from the option bytes area.
  * @param  Address The address to be read (0x1FFFC000..0x1FFFC00F).
  * @retval Returns the read value.
  */
uint8_t OPENBL_OB_Read(uint32_t Address)
{
  return (*(uint8_t *)(Address));
}
