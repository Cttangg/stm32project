/**
  ******************************************************************************
  * @file    app_openbootloader.h
  * @brief   Application level glue for the Open Bootloader (project specific)
  * @details 工程级应用层接口: 跳转前的外设释放 (DeInit) 与选项字节
  *          生效复位 (OB Launch), 由 Open Bootloader 核心流程调用。
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

/**
  * @brief  释放 Open Bootloader 占用的全部硬件资源。
  * @retval 无。
  * @note   使用示例 (由 OPENBL_DeInit() 内部调用):
  *           OpenBootloader_DeInit();
  *         功能包括: 反初始化已注册的通信接口 (USART DeInit 并释放引脚)、
  *         挂起 SysTick 定时中断, 确保跳转到用户程序后外设状态干净、
  *         不会因旧向量表中断而跑飞。
  */
void OpenBootloader_DeInit(void);

/**
  * @brief  重载选项字节并触发系统复位, 使 RDP/WRP 等新配置生效。
  * @retval 无 (触发复位后不再返回)。
  * @note   使用示例:
  *           Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  *         flash_interface 在修改读保护/写保护后会自动注册本函数为
  *         后处理回调, 命令处理结束时由 Common_StartPostProcessing()
  *         调用, 完成选项字节加载并复位。
  */
void OPENBL_OB_Launch(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* APP_OPENBOOTLOADER_H */
