/**
 ******************************************************************************
 * @file    uart.h
 * @brief   STM32 通用串口库 — 基于 HAL_UART + DMA 环形接收
 *
 * 设计目标:
 *   - Linux tty 式抽象: 应用只调 UART_Init/Open/Send/Read/Task/RegisterFrame
 *   - 非阻塞收发: TX 环形缓冲 + DMA 链式; RX DMA CIRCULAR + IDLE → 软件环形
 *   - 帧协议可插拔: get_length / check / callback, 多协议链表共存
 *   - 多串口: 端口表预留 UART1~UART6, 当前注册 USART1
 *   - 零 malloc / ISR 安全 / 裸机 RTOS 双栖 / 统一错误码 + 统计
 *
 * 硬件: STM32F407VETx, 底层 DMA 由 CubeMX 配置 (RX CIRCULAR), TX 在 Open 时
 *       由库重配为 NORMAL (一次性发送).
 ******************************************************************************
 */
#ifndef __UART_H
#define __UART_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  一、端口枚举 (预留多串口, 命名带 P 前缀避开 CMSIS 外设宏 UART4/UART5)       */
/* ========================================================================= */
typedef enum {
    UART_P1 = 0,   /* USART1  PA9(TX) PA10(RX)  APB2 AF7  DMA2-S7(TX) DMA2-S2(RX) */
    UART_P2 = 1,   /* USART2  PA2(TX) PA3(RX)   APB1 AF7  DMA1-S6(TX) DMA1-S5(RX) */
    UART_P3 = 2,   /* USART3  PB10(TX) PB11(RX) APB1 AF7                         */
    UART_P4 = 3,   /* UART4                                                      */
    UART_P5 = 4,   /* UART5                                                      */
    UART_P6 = 5,   /* USART6  PC6(TX) PC7(RX)   APB2 AF8                         */
    UART_PMAX
} UART_Port;

/* ========================================================================= */
/*  二、错误码                                                                */
/* ========================================================================= */
typedef enum {
    UART_OK       =  0,
    UART_ERR_PARAM = -1,
    UART_ERR_BUSY  = -2,
    UART_ERR_FULL  = -3,
    UART_ERR_NOTOPEN = -4,
} UART_Status;

/* ========================================================================= */
/*  三、帧协议                                                                */
/* ========================================================================= */

/** 帧长度提取回调: 输入已收数据, 返回帧总长度 (0=错误) */
typedef uint16_t (*UART_FrameGetLength)(const uint8_t *buf);
/** 帧校验回调: 返回 1=通过 0=失败 */
typedef uint8_t  (*UART_FrameCheck)(const uint8_t *buf, uint16_t len);
/** 帧完成回调 (在 UART_Task 上下文中执行) */
typedef void     (*UART_FrameCallback)(UART_Port port, uint8_t *data, uint16_t len);

/**
 * 帧格式描述符 (链表节点). 用户只需配置:
 *   header_len  — 帧头长度 (0=逐字节回调)
 *   header[]    — 帧头字节
 *   max_len     — 单帧最大长度
 *   get_length  — 长度回调 (NULL = 帧头长度即帧长)
 *   check       — 校验回调 (NULL = 不校验)
 *   callback    — 帧完成回调
 *   timeout_ms  — 帧超时
 */
typedef struct UART_Frame {
    /* ── 用户配置 ── */
    uint8_t            header[4];
    uint8_t            header_len;
    uint16_t           max_len;
    UART_FrameGetLength get_length;
    UART_FrameCheck     check;
    UART_FrameCallback  callback;
    uint32_t           timeout_ms;

    /* ── 内部状态 (勿改) ── */
    uint8_t            state;
    uint16_t           expected_len;
    uint16_t           recv_count;
    uint32_t           last_tick;
    struct UART_Frame *next;
} UART_Frame;

/* ========================================================================= */
/*  四、统计                                                                  */
/* ========================================================================= */
typedef struct {
    uint32_t rx_bytes;        /* 累计接收字节数 */
    uint32_t tx_bytes;        /* 累计发送字节数 */
    uint32_t rx_overflow;     /* RX 环形缓冲溢出次数 */
    uint32_t tx_overflow;     /* TX 环形缓冲溢出次数 */
    uint32_t uart_errors;     /* UART 错误 (ORE/FE/NE) 累计 */
    uint32_t rx_events;       /* 接收事件次数 (IDLE/HT/TC) */
} UART_Stats;

/* ========================================================================= */
/*  五、公开 API                                                              */
/* ========================================================================= */

/** @brief 初始化端口表 (HAL_Init + 外设 Init 之后调用一次) */
void UART_Init(void);

/** @brief 打开串口 (使用 CubeMX 已初始化的 huart 句柄) */
UART_Status UART_Open(UART_Port port);

/** @brief 关闭串口, 停止 DMA */
UART_Status UART_Close(UART_Port port);

/** @brief 非阻塞发送, 实际入队字节数写入 *written */
UART_Status UART_Send(UART_Port port, const uint8_t *data, uint16_t len,
                      uint16_t *written);

/** @brief 是否正在发送 (1=忙 0=空闲) */
uint8_t UART_IsSending(UART_Port port);

/** @brief 查询 RX 环形缓冲可读字节数 */
uint16_t UART_Available(UART_Port port);

/** @brief 读取原始字节 */
uint16_t UART_Read(UART_Port port, uint8_t *buf, uint16_t max_len);

/** @brief 注册帧协议和回调 */
UART_Status UART_RegisterFrame(UART_Port port, UART_Frame *frame);

/** @brief 主循环周期调用: 环形→帧解析→回调 + 帧超时处理 */
void UART_Task(void);

/** @brief 调试统计 */
const UART_Stats *UART_GetStats(UART_Port port);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
