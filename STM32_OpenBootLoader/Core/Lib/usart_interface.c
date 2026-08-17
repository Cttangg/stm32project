/**
  ******************************************************************************
  * @file    usart_interface.c
  * @brief   USART interface for STM32F4 Open Bootloader.
  *
  *          Written against the F4 register map only (no LL driver needed).
  *          USART1, PA9 (TX) / PA10 (RX), 115200-8-N-1.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "platform.h"
#include "interfaces_conf.h"
#include "openbl_core.h"
#include "openbl_usart_cmd.h"
#include "usart_interface.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* USART sync byte is 0x7F (same as the ST system bootloader / CubeProgrammer).
   NOTE: the MW's generic SYNC_BYTE macro (0xA5) is for other interfaces
   (SPI/I3C), NOT for USART. The official USART pattern detects the
   auto-baud "0x7F frame". */
#define USART_SYNC_BYTE                  0x7FU

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t UsartDetected = 0U;

/* Special command opcode lists: empty (no special command implemented) */
const uint16_t SpecialCmdList[1] = {0x0000U};
const uint16_t ExtendedSpecialCmdList[1] = {0x0000U};

/* Private function prototypes -----------------------------------------------*/
static void OPENBL_USART_Init(void);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initialize the used USART instance (register level).
  *         ST Open Bootloader protocol requires: 8 data bits + EVEN parity
  *         (8E1, M=1 + PCE=1 on STM32F4), 1 stop bit — same as CubeProgrammer.
  * @retval None.
  */
static void OPENBL_USART_Init(void)
{
  uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();

  /* USART1 is on APB2 (PCLK2). For 16x oversampling:
     BRR = PCLK2 / baudrate (mantissa:fraction packing) */
  if (pclk2 != 0U)
  {
    uint16_t brr = (uint16_t)((pclk2 + (DEFAULT_USART_BAUDRATE / 2U)) / DEFAULT_USART_BAUDRATE);
    USARTx->BRR = brr;
  }

  /* 8 data bits + even parity (8E1), 1 stop bit, TX+RX enabled.
     On STM32F4: to get 8 data + 1 parity on the wire, word length must be
     9-bit (M=1) with parity enabled (PCE=1). M=0 + PCE=1 would be 7 data
     + parity, which does NOT match the protocol. */
  USARTx->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_M | USART_CR1_PCE;
  USARTx->CR2 = 0U;
  USARTx->CR3 = 0U;
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  Configure USART pins and initialize the USART instance.
  * @retval None.
  */
void OPENBL_USART_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStruct;

  USARTx_GPIO_CLK_ENABLE();
  USARTx_CLK_ENABLE();

  GPIO_InitStruct.Pin       = USARTx_TX_PIN | USARTx_RX_PIN;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_PULLUP;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = USARTx_ALTERNATE;
  HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

  OPENBL_USART_Init();
}

/**
  * @brief  De-initialize the USART instance and pins (called before jumping to the app).
  * @retval None.
  */
void OPENBL_USART_DeInit(void)
{
  USARTx->CR1 = 0U;              /* disable USART */

  USARTx_CLK_DISABLE();
  USARTx_GPIO_CLK_DISABLE();

  /* Release the pins so the application can re-use them */
  HAL_GPIO_DeInit(USARTx_TX_GPIO_PORT, USARTx_TX_PIN);
  HAL_GPIO_DeInit(USARTx_RX_GPIO_PORT, USARTx_RX_PIN);

  UsartDetected = 0U;
}

/**
  * @brief  Non-blocking detection of the USART protocol.
  *         The host starts every session with the 0x7F synchronization byte.
  *         Once detected, the state is kept so the command session continues.
  * @retval Returns 1 if interface is detected else 0.
  */
uint8_t OPENBL_USART_ProtocolDetection(void)
{
  if (UsartDetected == 0U)
  {
    if ((USARTx->SR & USART_SR_RXNE) != 0U)
    {
      uint8_t byte = (uint8_t)(USARTx->DR & 0xFFU);

      if (byte == USART_SYNC_BYTE)
      {
        /* Acknowledge the host */
        OPENBL_USART_SendByte(ACK_BYTE);
        UsartDetected = 1U;
      }
    }
  }

  return UsartDetected;
}

/**
  * @brief  Get the command opcode from the host.
  * @retval Returns the command (ERROR_COMMAND if checksum mismatch).
  */
uint8_t OPENBL_USART_GetCommandOpcode(void)
{
  uint8_t command_opc = 0x0;

  command_opc = OPENBL_USART_ReadByte();

  if ((command_opc ^ OPENBL_USART_ReadByte()) != 0xFFU)
  {
    command_opc = ERROR_COMMAND;
  }

  return command_opc;
}

/**
  * @brief  Read one byte from the USART pipe (blocking).
  * @retval Returns the read byte.
  */
uint8_t OPENBL_USART_ReadByte(void)
{
  while ((USARTx->SR & USART_SR_RXNE) == 0U)
  {
  }

  return (uint8_t)(USARTx->DR & 0xFFU);
}

/**
  * @brief  Send one byte through the USART pipe (blocking).
  * @param  Byte The byte to be sent.
  * @retval None.
  */
void OPENBL_USART_SendByte(uint8_t Byte)
{
  USARTx->DR = (uint8_t)Byte;

  while ((USARTx->SR & USART_SR_TC) == 0U)
  {
  }
}

/**
  * @brief  Process and execute the special commands (none supported).
  * @param  SpecialCmd Pointer to the OPENBL_SpecialCmdTypeDef structure.
  * @retval None.
  */
void OPENBL_USART_SpecialCommandProcess(OPENBL_SpecialCmdTypeDef *SpecialCmd)
{
  switch (SpecialCmd->OpCode)
  {
    /* Unknown command opcode */
    default:
      if (SpecialCmd->CmdType == OPENBL_SPECIAL_CMD)
      {
        /* Send NULL data size */
        OPENBL_USART_SendByte(0x00U);
        OPENBL_USART_SendByte(0x00U);

        /* Send NULL status size */
        OPENBL_USART_SendByte(0x00U);
        OPENBL_USART_SendByte(0x00U);
      }
      else if (SpecialCmd->CmdType == OPENBL_EXTENDED_SPECIAL_CMD)
      {
        /* Send NULL status size */
        OPENBL_USART_SendByte(0x00U);
        OPENBL_USART_SendByte(0x00U);
      }
      break;
  }
}
