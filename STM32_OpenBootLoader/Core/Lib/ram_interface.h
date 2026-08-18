/**
  ******************************************************************************
  * @file    ram_interface.h
  * @brief   Header for ram_interface.c module
  * @details 提供 RAM 读/写/跳转接口, 供 Open Bootloader 核心通过
  *          RAM_Descriptor 调用, 用于"RAM 下载"和"从 RAM 启动"。
  *          RAM 描述符起始地址已跳过 OPENBL_RAM_SIZE (8 KB), 防止上位机
  *          覆盖 Bootloader 自身的变量与栈。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef RAM_INTERFACE_H
#define RAM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/**
  * @brief RAM 存储器描述符, 由 openbl_mem 注册到存储器映射表。
  * @note  只支持 Read / Write / JumpToAddress, 不支持擦除与保护。
  */
extern OPENBL_MemoryTypeDef RAM_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  从指定地址读取 1 个字节 (直接总线读取)。
  * @param  Address 要读取的 RAM 地址, 如 0x20000000U。
  * @retval 返回该地址处的字节值。
  * @note   使用示例:
  *           uint8_t val = OPENBL_RAM_Read(0x20000000U);
  */
uint8_t OPENBL_RAM_Read(uint32_t Address);

/**
  * @brief  向 RAM 写入数据 (按 32 位字传输, 长度自动向上取整到 4 字节)。
  * @param  Address    写入起始地址, 必须 4 字节对齐, 且位于 RAM 描述符
  *                    允许的范围内 (>= RAM_START_ADDRESS + OPENBL_RAM_SIZE)。
  * @param  pData      待写入数据的源缓冲区指针 (亦需 4 字节对齐)。
  * @param  DataLength 待写入数据的长度 (字节数)。
  * @retval 无。
  * @note   使用示例:
  *           uint8_t buf[16] = {0};
  *           OPENBL_RAM_Write(0x20001000U, buf, sizeof(buf));
  *         注意: 内部按 uint32 整字搬运, 若长度不是 4 的倍数, 会多写
  *         最多 3 个字节, 请确保缓冲区边界安全。
  */
void OPENBL_RAM_Write(uint32_t Address, uint8_t *pData, uint32_t DataLength);

/**
  * @brief  跳转到 RAM 中的用户程序执行 (读取向量表并执行 Reset_Handler)。
  * @param  Address RAM 中用户程序向量表首地址。
  * @retval 无返回值 (跳转成功后将不再返回)。
  * @note   使用示例:
  *           OPENBL_RAM_JumpToAddress(0x20001000U);
  *         函数内部会自动调用 OPENBL_DeInit() 释放外设、使能中断、设置
  *         MSP 后跳转; 常用于"从 RAM 运行测试程序"调试场景。
  */
void OPENBL_RAM_JumpToAddress(uint32_t Address);

#ifdef __cplusplus
}
#endif

#endif /* RAM_INTERFACE_H */
