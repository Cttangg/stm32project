/**
  ******************************************************************************
  * @file    common_interface.h
  * @brief   Header for common_interface.c module
  * @details 提供 Open Bootloader 各接口共用的基础函数: MSP/中断控制、
  *          保护状态查询、后处理 (复位) 回调注册与触发。
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef COMMON_INTERFACE_H
#define COMMON_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/**
  * @brief 无参数函数指针类型, 用于跳转地址与后处理回调。
  */
typedef void (*Function_Pointer)(void);

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/**
  * @brief  设置主堆栈指针 (MSP)。
  * @param  TopOfMainStack 要设置的 MSP 值 (通常为应用程序向量表首地址处
  *                        存放的栈顶地址)。
  * @retval 无。
  * @note   使用示例:
  *           Common_SetMsp(*(__IO uint32_t *)0x08008000U);
  *         跳转到用户程序前调用, 与 OPENBL_FLASH_JumpToAddress() 内部
  *         用法一致; 设置后应立即跳转, 期间勿执行复杂代码。
  */
void Common_SetMsp(uint32_t TopOfMainStack);

/**
  * @brief  全局使能中断 (PRIMASK 清零)。
  * @retval 无。
  * @note   使用示例:
  *           Common_EnableIrq();
  *         跳转用户程序前调用, 恢复应用所需的中断使能状态。
  */
void Common_EnableIrq(void);

/**
  * @brief  全局关闭中断 (PRIMASK 置 1)。
  * @retval 无。
  * @note   使用示例:
  *           Common_DisableIrq();
  *         用于临界区保护, 与 Common_EnableIrq() 配对使用。
  */
void Common_DisableIrq(void);

/**
  * @brief  查询目标芯片的读保护 (RDP) 是否开启。
  * @retval SET  = 保护已开启 (RDP > Level 0);
  *         RESET = 未保护 (RDP Level 0)。
  * @note   使用示例:
  *           if (Common_GetProtectionStatus() == SET) { ... }
  *         内部通过 OPENBL_FLASH_GetReadOutProtectionLevel() 判断,
  *         用于协议层决定是否允许访问受保护的区域。
  */
FlagStatus Common_GetProtectionStatus(void);

/**
  * @brief  注册后处理回调 (命令处理结束后自动调用)。
  * @param  Callback 要注册的回调函数指针 (如 OPENBL_OB_Launch 触发复位),
  *                  传 NULL 可清除已注册的回调。
  * @retval 无。
  * @note   使用示例:
  *           Common_SetPostProcessingCallback(OPENBL_OB_Launch);
  *         该回调由 Common_StartPostProcessing() 在每条命令处理完成后
  *         执行, 用于"修改选项字节后需要复位生效"等场景。
  */
void Common_SetPostProcessingCallback(Function_Pointer Callback);

/**
  * @brief  触发后处理任务 (执行已注册的回调并清除)。
  * @retval 无。
  * @note   使用示例:
  *           Common_StartPostProcessing();
  *         若回调内部触发了系统复位则不会返回; 否则回调会被清空,
  *         保证只执行一次。
  */
void Common_StartPostProcessing(void);

#ifdef __cplusplus
}
#endif

#endif /* COMMON_INTERFACE_H */
