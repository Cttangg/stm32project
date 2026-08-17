/**
  ******************************************************************************
  * @file    flash_interface.c
  * @brief   FLASH access functions for STM32F4 (HAL based).
  *
  *          Self-protection: the bootloader sectors (0..1) can never be
  *          erased, mass-erased or written by the host.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "openbl_mem.h"
#include "openbl_core.h"
#include "app_openbootloader.h"
#include "common_interface.h"
#include "flash_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* This F4 HAL package version has no FLASH_FLAG_ALL_ERRORS macro */
#define FLASH_ERROR_FLAGS                (FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | \
                                          FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static ErrorStatus OPENBL_FLASH_EnableWriteProtection(uint8_t *ListOfPages, uint32_t Length);
static ErrorStatus OPENBL_FLASH_DisableWriteProtection(void);

/* Exported variables --------------------------------------------------------*/
OPENBL_MemoryTypeDef FLASH_Descriptor =
{
  FLASH_START_ADDRESS,
  FLASH_END_ADDRESS,
  FLASH_MEM_SIZE,
  FLASH_AREA,
  OPENBL_FLASH_Read,
  OPENBL_FLASH_Write,
  OPENBL_FLASH_SetReadOutProtectionLevel,
  OPENBL_FLASH_SetWriteProtection,
  OPENBL_FLASH_JumpToAddress,
  OPENBL_FLASH_MassErase,
  OPENBL_FLASH_Erase
};

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Unlock the FLASH control register access.
  * @retval None.
  */
void OPENBL_FLASH_Unlock(void)
{
  HAL_FLASH_Unlock();
}

/**
  * @brief  Lock the FLASH control register access.
  * @retval None.
  */
void OPENBL_FLASH_Lock(void)
{
  HAL_FLASH_Lock();
}

/**
  * @brief  Unlock the FLASH Option Bytes registers access.
  * @retval None.
  */
void OPENBL_FLASH_OB_Unlock(void)
{
  HAL_FLASH_Unlock();
  HAL_FLASH_OB_Unlock();
}

/**
  * @brief  Read a byte from a given address.
  * @param  Address The address to be read.
  * @retval Returns the read value.
  */
uint8_t OPENBL_FLASH_Read(uint32_t Address)
{
  return (*(uint8_t *)(Address));
}

/**
  * @brief  Write data to FLASH memory (byte/word mixed, F4 supports byte access).
  * @param  Address The address where the data will be written.
  * @param  Data The data to be written.
  * @param  DataLength The length of the data to be written.
  * @retval None.
  */
void OPENBL_FLASH_Write(uint32_t Address, uint8_t *Data, uint32_t DataLength)
{
  uint32_t index = 0U;

  /* Never allow the host to overwrite the bootloader itself */
  if (Address < FLASH_APP_START_ADDRESS)
  {
    return;
  }

  OPENBL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_ERROR_FLAGS);

  /* Program the head bytes until the address is word-aligned */
  while ((((Address + index) & 0x3U) != 0U) && (index < DataLength))
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Address + index), Data[index]);
    index++;
  }

  /* Program full words */
  while ((DataLength - index) >= 4U)
  {
    uint32_t word = ((uint32_t)Data[index]) |
                    ((uint32_t)Data[index + 1U] << 8) |
                    ((uint32_t)Data[index + 2U] << 16) |
                    ((uint32_t)Data[index + 3U] << 24);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, (Address + index), word);
    index += 4U;
  }

  /* Program the tail bytes */
  while (index < DataLength)
  {
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, (Address + index), Data[index]);
    index++;
  }

  OPENBL_FLASH_Lock();
}

/**
  * @brief  Jump to the application at the given address.
  * @param  Address User application address (0x08008000).
  * @retval None.
  */
void OPENBL_FLASH_JumpToAddress(uint32_t Address)
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

/**
  * @brief  Return the FLASH Read Protection level.
  * @retval OB_RDP_LEVEL_0 / OB_RDP_LEVEL_1 / OB_RDP_LEVEL_2.
  */
uint32_t OPENBL_FLASH_GetReadOutProtectionLevel(void)
{
  FLASH_OBProgramInitTypeDef flash_ob;

  HAL_FLASHEx_OBGetConfig(&flash_ob);

  return (uint32_t)flash_ob.RDPLevel;
}

/**
  * @brief  Set the FLASH Read Protection level.
  * @param  Level OB_RDP_LEVEL_0 / OB_RDP_LEVEL_1.
  * @retval None.
  */
void OPENBL_FLASH_SetReadOutProtectionLevel(uint32_t Level)
{
  FLASH_OBProgramInitTypeDef flash_ob;

  if (Level != OB_RDP_LEVEL_2)
  {
    flash_ob.OptionType = OPTIONBYTE_RDP;
    flash_ob.RDPLevel   = Level;

    /* Unlock the FLASH & Option Bytes registers access */
    OPENBL_FLASH_OB_Unlock();

    /* Change the RDP level */
    HAL_FLASHEx_OBProgram(&flash_ob);
  }

  /* Register system reset callback (needed to apply option bytes) */
  Common_SetPostProcessingCallback(OPENBL_OB_Launch);
}

/**
  * @brief  Enable or disable write protection of the specified FLASH sectors.
  * @param  State ENABLE / DISABLE.
  * @param  ListOfPages List of sector numbers.
  * @param  Length Length of the list in bytes.
  * @retval ErrorStatus.
  */
ErrorStatus OPENBL_FLASH_SetWriteProtection(FunctionalState State, uint8_t *ListOfPages, uint32_t Length)
{
  ErrorStatus status = SUCCESS;

  if (State == ENABLE)
  {
    OPENBL_FLASH_EnableWriteProtection(ListOfPages, Length);

    /* Register system reset callback */
    Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  }
  else if (State == DISABLE)
  {
    OPENBL_FLASH_DisableWriteProtection();

    /* Register system reset callback */
    Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  }
  else
  {
    status = ERROR;
  }

  return status;
}

/**
  * @brief  Erase the whole user Flash area (sectors 2..11).
  *         The bootloader sectors are always preserved.
  * @param  p_Data Pointer to the buffer that contains mass erase options (ignored).
  * @param  DataLength Size of the Data buffer.
  * @retval ErrorStatus.
  */
ErrorStatus OPENBL_FLASH_MassErase(uint8_t *p_Data, uint32_t DataLength)
{
  uint32_t sector;
  uint32_t sector_error;
  ErrorStatus status = SUCCESS;
  FLASH_EraseInitTypeDef erase_init_struct;

  (void)p_Data;
  (void)DataLength;

  /* Unlock the flash memory for erase operation */
  OPENBL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_ERROR_FLAGS);

  erase_init_struct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Banks        = FLASH_BANK_1;
  erase_init_struct.NbSectors    = 1U;
  erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  for (sector = FLASH_BL_SECTOR_COUNT; sector < FLASH_SECTOR_COUNT; sector++)
  {
    erase_init_struct.Sector = sector;

    if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
    {
      status = ERROR;
      break;
    }
  }

  /* Lock the Flash to disable the flash control register access */
  OPENBL_FLASH_Lock();

  return status;
}

/**
  * @brief  Erase the specified FLASH sectors.
  *         The bootloader sectors (0..1) are always skipped.
  * @param  p_Data Pointer to the buffer that contains the sector numbers.
  * @param  DataLength Size of the Data buffer.
  * @retval ErrorStatus.
  */
ErrorStatus OPENBL_FLASH_Erase(uint8_t *p_Data, uint32_t DataLength)
{
  uint32_t counter;
  uint32_t pages_number;
  uint32_t sector;
  uint32_t sector_error;
  ErrorStatus status = SUCCESS;
  FLASH_EraseInitTypeDef erase_init_struct;

  /* Unlock the flash memory for erase operation */
  OPENBL_FLASH_Unlock();
  __HAL_FLASH_CLEAR_FLAG(FLASH_ERROR_FLAGS);

  pages_number = (uint32_t)(*(uint16_t *)(p_Data));
  p_Data += 2;

  erase_init_struct.TypeErase    = FLASH_TYPEERASE_SECTORS;
  erase_init_struct.Banks        = FLASH_BANK_1;
  erase_init_struct.NbSectors    = 1U;
  erase_init_struct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

  for (counter = 0U; (counter < pages_number) && (counter < (DataLength / 2U)); counter++)
  {
    sector = (uint32_t)(*(uint16_t *)(p_Data));
    p_Data += 2;

    /* Skip the bootloader sectors */
    if (sector >= FLASH_BL_SECTOR_COUNT)
    {
      erase_init_struct.Sector = sector;

      if (HAL_FLASHEx_Erase(&erase_init_struct, &sector_error) != HAL_OK)
      {
        status = ERROR;
      }
    }
  }

  /* Lock the Flash to disable the flash control register access */
  OPENBL_FLASH_Lock();

  return status;
}

/**
  * @brief  Enable busy state sending (I2C non-stretch only, no-op here).
  * @retval None.
  */
void OPENBL_Enable_BusyState_Flag(void)
{
}

/**
  * @brief  Disable busy state sending (no-op here).
  * @retval None.
  */
void OPENBL_Disable_BusyState_Flag(void)
{
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Enable write protection of the specified sectors (bitmask, bank 1).
  * @param  ListOfPages List of sector numbers.
  * @param  Length Length of the list in bytes.
  * @retval ErrorStatus.
  */
static ErrorStatus OPENBL_FLASH_EnableWriteProtection(uint8_t *ListOfPages, uint32_t Length)
{
  uint32_t counter;
  uint32_t mask = 0U;
  FLASH_OBProgramInitTypeDef flash_ob;

  /* Unlock the FLASH & Option Bytes registers access */
  OPENBL_FLASH_OB_Unlock();

  for (counter = 0U; counter < (Length / 2U); counter++)
  {
    uint16_t page = (uint16_t)(*(uint16_t *)(ListOfPages + (counter * 2U)));

    /* Never protect the bootloader sectors */
    if (page < FLASH_SECTOR_COUNT)
    {
      if (page >= FLASH_BL_SECTOR_COUNT)
      {
        mask |= (1UL << page);
      }
    }
  }

  flash_ob.OptionType = OPTIONBYTE_WRP;
  flash_ob.WRPState   = OB_WRPSTATE_ENABLE;
  flash_ob.WRPSector  = mask;
  flash_ob.Banks      = FLASH_BANK_1;
  flash_ob.RDPLevel   = 0U;
  flash_ob.BORLevel   = 0U;
  flash_ob.USERConfig = 0U;

  HAL_FLASHEx_OBProgram(&flash_ob);

  return SUCCESS;
}

/**
  * @brief  Disable write protection of all sectors.
  * @retval ErrorStatus.
  */
static ErrorStatus OPENBL_FLASH_DisableWriteProtection(void)
{
  FLASH_OBProgramInitTypeDef flash_ob;

  /* Unlock the FLASH & Option Bytes registers access */
  OPENBL_FLASH_OB_Unlock();

  flash_ob.OptionType = OPTIONBYTE_WRP;
  flash_ob.WRPState   = OB_WRPSTATE_DISABLE;
  flash_ob.WRPSector  = 0x0FFFU;  /* all 12 sectors */
  flash_ob.Banks      = FLASH_BANK_1;
  flash_ob.RDPLevel   = 0U;
  flash_ob.BORLevel   = 0U;
  flash_ob.USERConfig = 0U;

  HAL_FLASHEx_OBProgram(&flash_ob);

  return SUCCESS;
}
