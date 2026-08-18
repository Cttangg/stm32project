/**
  ******************************************************************************
  * @file    flash_interface.h
  * @brief   Header for flash_interface.c module (STM32F4 FLASH)
  * @details 提供 FLASH 读/写/擦除/跳转/读写保护等接口, 供 Open Bootloader
  *          核心 (openbl_mem / openbl_cmd) 通过 FLASH_Descriptor 调用。
  *          本模块自带自保护: 引导区扇区 (0..1) 不允许被上位机擦除、整体
  *          擦除或写入, 防止引导程序本身被破坏。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef FLASH_INTERFACE_H
#define FLASH_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "common_interface.h"
#include "openbl_mem.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
#define FLASH_BUSY_STATE_ENABLED       ((uint32_t)0xAAAA0000)
#define FLASH_BUSY_STATE_DISABLED      ((uint32_t)0x0000DDDD)

/* F407 (512 KB, single bank): 12 sectors (0..11)
   Bootloader occupies sectors 0..1 (32 KB), App starts at sector 2 */
#define FLASH_BL_SECTOR_COUNT          2U
#define FLASH_SECTOR_COUNT             12U

/* Exported variables --------------------------------------------------------*/
/**
  * @brief FLASH 存储器描述符, 由 openbl_mem 注册到存储器映射表。
  * @note  定义了 FLASH 的地址范围、读写/擦除/跳转回调, 上位机对 FLASH
  *        区域的访问最终都会走到本文件中的 OPENBL_FLASH_* 函数。
  */
extern OPENBL_MemoryTypeDef FLASH_Descriptor;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  跳转到指定地址的用户应用程序 (读取向量表并执行 Reset_Handler)。
  * @param  Address 用户程序向量表首地址, 通常为 FLASH_APP_START_ADDRESS
  *                 (0x08008000U)。
  * @retval 无返回值 (跳转成功后将不再返回)。
  * @note   使用示例:
  *           OPENBL_FLASH_JumpToAddress(FLASH_APP_START_ADDRESS);
  *         函数内部会自动调用 OPENBL_DeInit() 释放外设、使能中断、用
  *         (Address) 处的前 4 字节初始化 MSP, 再用 (Address+4) 处的
  *         复位向量跳转, 调用者无需再做任何准备工作。
  */
void OPENBL_FLASH_JumpToAddress(uint32_t Address);

/**
  * @brief  锁定 FLASH 控制寄存器访问, 禁止编程/擦除。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_FLASH_Lock();
  *         与 OPENBL_FLASH_Unlock() 配对使用; 每次编程/擦除完成后建议
  *         立即调用, 防止误操作改写 FLASH。
  */
void OPENBL_FLASH_Lock(void);

/**
  * @brief  解锁 FLASH 及 Option Bytes (选项字节) 寄存器访问。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_FLASH_OB_Unlock();
  *         仅在对选项字节 (读写保护、写保护) 编程前调用, 修改完成后需
  *         通过 OPENBL_OB_Launch() 或系统复位使新配置生效。
  */
void OPENBL_FLASH_OB_Unlock(void);

/**
  * @brief  从指定地址读取 1 个字节 (直接总线读取, 不经过 HAL)。
  * @param  Address 要读取的地址, 如 0x08008000U。
  * @retval 返回该地址处的字节值。
  * @note   使用示例:
  *           uint8_t val = OPENBL_FLASH_Read(0x08008000U);
  */
uint8_t OPENBL_FLASH_Read(uint32_t Address);

/**
  * @brief  设置 FLASH 读保护 (RDP) 级别。
  * @param  Level 目标保护级别: OB_RDP_LEVEL_0 (不保护) /
  *               OB_RDP_LEVEL_1 (禁止调试读取) / OB_RDP_LEVEL_2 (永久保护)。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_FLASH_SetReadOutProtectionLevel(OB_RDP_LEVEL_1);
  *         注意:
  *           - Level 2 为不可逆保护, 本函数会忽略该值不执行;
  *           - 函数内部会自动注册系统复位回调, 在命令处理结束后通过
  *             Common_StartPostProcessing() 触发复位使新配置生效。
  */
void OPENBL_FLASH_SetReadOutProtectionLevel(uint32_t Level);

/**
  * @brief  向 FLASH 写入数据 (自动按 字节对齐头尾 + 字 编程)。
  * @param  Address     写入起始地址, 必须 >= FLASH_APP_START_ADDRESS
  *                     (引导区受保护, 小于该地址的写入会被忽略)。
  * @param  Data        待写入数据的字节缓冲区指针。
  * @param  DataLength  待写入数据的长度 (字节数)。
  * @retval 无。
  * @note   使用示例:
  *           uint8_t buf[8] = {1,2,3,4,5,6,7,8};
  *           OPENBL_FLASH_Write(0x08008000U, buf, sizeof(buf));
  *         前提条件:
  *           - 目标扇区必须先擦除 (调用 OPENBL_FLASH_Erase / MassErase);
  *           - 长度可为任意字节数, 内部自动处理非 4 字节对齐;
  *           - 函数内部自动完成解锁/加锁, 无需外部调用 Unlock/Lock。
  */
void OPENBL_FLASH_Write(uint32_t Address, uint8_t *Data, uint32_t DataLength);

/**
  * @brief  解锁 FLASH 控制寄存器访问, 允许编程/擦除。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_FLASH_Unlock();
  *         OPENBL_FLASH_Write/Erase 内部已调用, 仅在自行操作 HAL
  *         FLASH 接口时才需要手动调用。
  */
void OPENBL_FLASH_Unlock(void);

/**
  * @brief  整体擦除用户 FLASH 区域 (扇区 2..11)。
  * @param  p_Data     指向擦除参数缓冲区的指针 (本实现未使用, 可传 NULL)。
  * @param  DataLength 参数缓冲区长度 (本实现未使用, 可传 0)。
  * @retval SUCCESS 擦除成功; ERROR 某扇区擦除失败。
  * @note   使用示例:
  *           if (OPENBL_FLASH_MassErase(NULL, 0U) == SUCCESS) { ... }
  *         引导区 (扇区 0..1) 始终保留不会被擦除。用于上位机"全片擦除"
  *         命令 (FLASH_MASS_ERASE)。
  */
ErrorStatus OPENBL_FLASH_MassErase(uint8_t *p_Data, uint32_t DataLength);

/**
  * @brief  按扇区号列表擦除指定的 FLASH 扇区。
  * @param  p_Data     指向扇区列表缓冲区的指针, 格式:
  *                    前 2 字节 = 扇区个数 (uint16), 其后每 2 字节 = 一个
  *                    扇区号 (uint16)。
  * @param  DataLength 扇区列表缓冲区的总字节数。
  * @retval SUCCESS 全部擦除成功; ERROR 存在擦除失败的扇区。
  * @note   使用示例 (擦除扇区 2 和 3):
  *           uint8_t list[6] = {2U, 0U, 3U, 0U, 0U, 0U};  (前 2 字节 = 个数, 后为扇区号)
  *           OPENBL_FLASH_Erase(list, sizeof(list));
  *         引导区扇区 (0..1) 会自动跳过; 扇区号越界 (>= 12) 时直接忽略。
  */
ErrorStatus OPENBL_FLASH_Erase(uint8_t *p_Data, uint32_t DataLength);

/**
  * @brief  设置/取消 FLASH 扇区写保护。
  * @param  State       ENABLE 使能写保护 / DISABLE 取消写保护。
  * @param  ListOfPages 扇区号列表 (每个扇区号为 uint16, 本实现按 F4 扇区
  *                     处理), State == DISABLE 时可传 NULL。
  * @param  Length      列表的字节数。
  * @retval SUCCESS 设置成功; ERROR State 参数非法。
  * @note   使用示例:
  *           uint8_t sectors[2] = {2U, 0U};  (保护扇区 2)
  *           OPENBL_FLASH_SetWriteProtection(ENABLE, sectors, sizeof(sectors));
  *         修改后需系统复位生效 (函数内部已注册复位回调)。
  */
ErrorStatus OPENBL_FLASH_SetWriteProtection(FunctionalState State, uint8_t *ListOfPages, uint32_t Length);

/**
  * @brief  获取当前 FLASH 读保护级别。
  * @retval 返回 OB_RDP_LEVEL_0 / OB_RDP_LEVEL_1 / OB_RDP_LEVEL_2。
  * @note   使用示例:
  *           uint32_t level = OPENBL_FLASH_GetReadOutProtectionLevel();
  *         供 Common_GetProtectionStatus() 判断系统是否处于保护状态。
  */
uint32_t OPENBL_FLASH_GetReadOutProtectionLevel(void);

/**
  * @brief  使能"忙状态"上报 (I2C 非时钟拉伸模式专用, USART 下为空操作)。
  * @retval 无。
  * @note   仅当移植到 I2C 非时钟拉伸模式时需要实现; 当前 USART 工程
  *         下调用无任何效果。
  */
void OPENBL_Enable_BusyState_Flag(void);

/**
  * @brief  禁止"忙状态"上报 (空操作)。
  * @retval 无。
  * @note   与 OPENBL_Enable_BusyState_Flag() 对应, 当前工程下为空操作。
  */
void OPENBL_Disable_BusyState_Flag(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* FLASH_INTERFACE_H */
