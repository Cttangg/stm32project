/**
  ******************************************************************************
  * @file    usart_interface.h
  * @brief   Header for usart_interface.c module (STM32F4 USART)
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
void OPENBL_USART_Configuration(void);
void OPENBL_USART_DeInit(void);
uint8_t OPENBL_USART_ProtocolDetection(void);

uint8_t OPENBL_USART_GetCommandOpcode(void);
uint8_t OPENBL_USART_ReadByte(void);
void OPENBL_USART_SendByte(uint8_t Byte);
void OPENBL_USART_SpecialCommandProcess(OPENBL_SpecialCmdTypeDef *SpecialCmd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* USART_INTERFACE_H */
