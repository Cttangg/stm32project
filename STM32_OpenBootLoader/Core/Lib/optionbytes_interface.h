/**
  ******************************************************************************
  * @file    optionbytes_interface.h
  * @brief   Header for optionbytes_interface.c module (STM32F4)
  * @details 提供选项字节 (Option Bytes) 只读访问接口, 供 Open Bootloader
  *          核心通过 OB_Descriptor 注册到存储器映射表, 使 CubeProgrammer
  *          等上位机连接时可读取 0x1FFFC000 处的选项字节内容。
  *          注意: 选项字节的"修改"不在此模块, 而是通过
  *          flash_interface 的 OPENBL_FLASH_SetReadOutProtectionLevel /
  *          OPENBL_FLASH_SetWriteProtection 编程实现。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef OPTIONBYTES_INTERFACE_H
#define OPTIONBYTES_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/**
  * @brief 选项字节存储器描述符, 由 openbl_mem 注册到存储器映射表。
  * @note  只支持 Read, 写操作为 NULL (不可写)。
  */
extern OPENBL_MemoryTypeDef OB_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  从选项字节区域读取 1 个字节 (直接总线读取)。
  * @param  Address 要读取的地址, 有效范围 0x1FFFC000U .. 0x1FFFC00FU
  *                 (OB_START_ADDRESS ~ OB_END_ADDRESS)。
  * @retval 返回该地址处的字节值。
  * @note   使用示例:
  *           uint8_t opt = OPENBL_OB_Read(0x1FFFC000U);
  */
uint8_t OPENBL_OB_Read(uint32_t Address);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* OPTIONBYTES_INTERFACE_H */
