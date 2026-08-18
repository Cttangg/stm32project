/**
  ******************************************************************************
  * @file    usart_interface.h
  * @brief   Header for usart_interface.c module (STM32F4 USART)
  * @details 提供 USART 串口通信接口 (USART1, PA9/PA10, 115200-8E1),
  *          供 Open Bootloader 核心 (openbl_core / openbl_usart_cmd)
  *          实现 ST 官方 USART 协议: 0x7F 同步、命令+取反校验、ACK 应答。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef USART_INTERFACE_H
#define USART_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "openbl_core.h"

/* Special command lists (none supported: size 1 dummy entry is never matched
   because the corresponding MAX_NUMBER counters are 0) */
#define SPECIAL_CMD_MAX_NUMBER            0U
#define EXTENDED_SPECIAL_CMD_MAX_NUMBER   0U

extern const uint16_t SpecialCmdList[1];
extern const uint16_t ExtendedSpecialCmdList[1];

/* Exported functions ------------------------------------------------------- */

/**
  * @brief  配置 USART 引脚 (GPIO 复用) 并初始化 USART 外设。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_USART_Configuration();
  *         必须在其他 USART 收发函数之前调用 (通常在 Bootloader 主循环
  *         启动时调用一次)。通信参数 (引脚/波特率) 由 interfaces_conf.h
  *         配置, 帧格式固定为 8 数据位 + 偶校验 + 1 停止位 (8E1)。
  */
void OPENBL_USART_Configuration(void);

/**
  * @brief  关闭 USART 外设并释放引脚 (跳转用户程序前调用)。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_USART_DeInit();
  *         由 OPENBL_DeInit() 流程调用, 确保应用代码可重新配置串口引脚。
  */
void OPENBL_USART_DeInit(void);

/**
  * @brief  非阻塞检测上位机是否发起 USART 协议 (同步字节 0x7F)。
  * @retval 1 = 已检测到协议 (收到 0x7F 并已回 ACK); 0 = 尚未检测到。
  * @note   使用示例:
  *           if (OPENBL_USART_ProtocolDetection() == 1U) { ... }
  *         一旦检测成功, 状态会保持, 后续命令会话直接进入命令处理;
  *         该函数为轮询式非阻塞调用, 无数据时立即返回 0。
  */
uint8_t OPENBL_USART_ProtocolDetection(void);

/**
  * @brief  读取上位机发来的命令操作码 (含取反字节校验)。
  * @retval 命令操作码 (如 CMD_ERASE); 校验失败时返回 ERROR_COMMAND。
  * @note   使用示例:
  *           uint8_t opc = OPENBL_USART_GetCommandOpcode();
  *         协议要求上位机发送 命令码 + 命令码取反 两个字节,
  *         两者异或 != 0xFF 即判定为通信错误。
  */
uint8_t OPENBL_USART_GetCommandOpcode(void);

/**
  * @brief  从 USART 接收 1 个字节 (阻塞等待)。
  * @retval 返回接收到的字节。
  * @note   使用示例:
  *           uint8_t byte = OPENBL_USART_ReadByte();
  *         在没有数据时该函数会一直等待, 请勿在中断/超时要求严格的
  *         上下文中调用。
  */
uint8_t OPENBL_USART_ReadByte(void);

/**
  * @brief  通过 USART 发送 1 个字节 (阻塞等待发送完成)。
  * @param  Byte 要发送的字节。
  * @retval 无。
  * @note   使用示例:
  *           OPENBL_USART_SendByte(ACK_BYTE);
  *         内部等待 TC (发送完成) 标志后才返回。
  */
void OPENBL_USART_SendByte(uint8_t Byte);

/**
  * @brief  处理特殊命令 (当前版本无特殊命令, 仅按协议回送空应答)。
  * @param  SpecialCmd 指向 OPENBL_SpecialCmdTypeDef 结构, 含命令码与类型。
  * @retval 无。
  * @note   使用示例 (由 openbl_core 内部调用):
  *           OPENBL_USART_SpecialCommandProcess(&cmd);
  *         本工程 SPECIAL_CMD_MAX_NUMBER / EXTENDED_SPECIAL_CMD_MAX_NUMBER
  *         均为 0, 该函数不会被真正匹配调用。
  */
void OPENBL_USART_SpecialCommandProcess(OPENBL_SpecialCmdTypeDef *SpecialCmd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* USART_INTERFACE_H */
