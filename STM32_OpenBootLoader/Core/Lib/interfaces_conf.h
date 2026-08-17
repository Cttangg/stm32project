/**
  ******************************************************************************
  * @file    interfaces_conf.h
  * @brief   Interfaces configuration (USART1 PA9/PA10, 115200-8E1)
  *
  *          Frame format is fixed by the ST Open Bootloader protocol:
  *          8 data bits + EVEN parity, 1 stop bit (see usart_interface.c).
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef INTERFACES_CONF_H
#define INTERFACES_CONF_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#define MEMORIES_SUPPORTED                3U                         /* Flash + RAM + Option Bytes */

/*-------------------------- Definitions for USART ---------------------------*/
#define USARTx                            USART1
#define USARTx_CLK_ENABLE()               __HAL_RCC_USART1_CLK_ENABLE()
#define USARTx_CLK_DISABLE()              __HAL_RCC_USART1_CLK_DISABLE()
#define USARTx_GPIO_CLK_ENABLE()          __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_GPIO_CLK_DISABLE()         __HAL_RCC_GPIOA_CLK_DISABLE()

#define USARTx_TX_PIN                     GPIO_PIN_9
#define USARTx_TX_GPIO_PORT               GPIOA
#define USARTx_RX_PIN                     GPIO_PIN_10
#define USARTx_RX_GPIO_PORT               GPIOA
#define USARTx_ALTERNATE                  GPIO_AF7_USART1

#define DEFAULT_USART_BAUDRATE            115200U

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* INTERFACES_CONF_H */
