/**
 ******************************************************************************
 * @file    uart.h
 * @brief   STM32 通用串口库 — 基于 HAL_UART + DMA 环形接收 (句柄化, 多实例)
 *
 * 设计目标:
 *   - Linux tty 式抽象: 应用只调 UART_Init/Open/Send/Read/Task/RegisterFrame
 *   - 非阻塞收发: TX 环形缓冲 + DMA 链式; RX DMA CIRCULAR + IDLE → 软件环形
 *   - 帧协议可插拔: get_length / check / callback, 多协议链表共存
 *   - 句柄化: 每个串口一个 UART_Device, 绑定各自的 HAL huart, 互不干扰
 *   - 零 malloc: 缓冲内嵌于 UART_Device (应用层 static 定义)
 *   - ISR 安全 / 裸机 RTOS 双栖 / 统一错误码 + 统计
 *
 * 硬件: STM32F407VETx, 底层 DMA 由 CubeMX 配置 (RX CIRCULAR), TX 在 Open 时
 *       由库重配为 NORMAL (一次性发送).
 *
 * 使用示例:
 *   static UART_Device dbg, cam;
 *
 *   UART_Init(&dbg, &huart1);   // USART1 调试口
 *   UART_Init(&cam, &huart2);   // USART2 摄像头
 *   UART_Open(&dbg);
 *   UART_Open(&cam);
 *   ...
 *   while (1) {
 *       UART_Task(&dbg);
 *       UART_Task(&cam);
 *   }
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
/*  一、错误码                                                                */
/* ========================================================================= */
typedef enum {
    UART_OK       =  0,
    UART_ERR_PARAM = -1,
    UART_ERR_BUSY  = -2,
    UART_ERR_FULL  = -3,
    UART_ERR_NOTOPEN = -4,
} UART_Status;

/* ========================================================================= */
/*  二、帧协议                                                                */
/* ========================================================================= */

/* 前置声明 (UART_Device 在下方定义, 回调签名需要它) */
struct UART_Device;

/** 帧长度提取回调: 输入已收数据, 返回帧总长度 (0=错误) */
typedef uint16_t (*UART_FrameGetLength)(const uint8_t *buf);
/** 帧校验回调: 返回 1=通过 0=失败 */
typedef uint8_t  (*UART_FrameCheck)(const uint8_t *buf, uint16_t len);
/** 帧完成回调 (在 UART_Task 上下文中执行) */
typedef void     (*UART_FrameCallback)(struct UART_Device *dev, uint8_t *data, uint16_t len);

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
/*  三、软件环形缓冲 (单生产者-单消费者, ISR 安全)                             */
/* ========================================================================= */
typedef struct {
    uint8_t           *buf;
    uint16_t           size;    /* 2^n */
    volatile uint16_t  write;
    volatile uint16_t  read;
    volatile uint32_t  overflow;
} UART_RingBuf;

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
/*  五、串口句柄                                                              */
/* ========================================================================= */
typedef struct UART_Device {
    UART_HandleTypeDef *huart;     /* 绑定的 HAL UART 句柄 (NULL=未绑定) */
    uint8_t             opened;

    /* RX */
    uint8_t             rx_dma_buf[256];
    volatile uint16_t   rx_last_pos;
    UART_RingBuf        rx_rb;
    uint8_t             rx_rb_buf[1024];

    /* TX */
    UART_RingBuf        tx_rb;
    uint8_t             tx_rb_buf[512];
    uint8_t             tx_temp[128];
    volatile uint8_t    tx_busy;

    /* Frame */
    uint8_t             frame_buf[256];
    uint16_t            frame_idx;
    UART_Frame         *frame_list;
    UART_Frame          frame_pool[4];
    uint8_t             frame_used;

    UART_Stats          stats;
} UART_Device;

/* ========================================================================= */
/*  六、公开 API (全部接收句柄指针, 多串口独立控制)                            */
/* ========================================================================= */

/** @brief 绑定 HAL huart 并初始化句柄 (外设 Init 之后调用; 每设备调用一次) */
void UART_Init(UART_Device *dev, UART_HandleTypeDef *huart);

/** @brief 打开串口, 启动 DMA RX (IDLE 接收) */
UART_Status UART_Open(UART_Device *dev);

/** @brief 关闭串口, 停止 DMA */
UART_Status UART_Close(UART_Device *dev);

/** @brief 非阻塞发送, 实际入队字节数写入 *written */
UART_Status UART_Send(UART_Device *dev, const uint8_t *data, uint16_t len,
                      uint16_t *written);

/** @brief 是否正在发送 (1=忙 0=空闲) */
uint8_t UART_IsSending(const UART_Device *dev);

/** @brief 查询 RX 环形缓冲可读字节数 */
uint16_t UART_Available(const UART_Device *dev);

/** @brief 读取原始字节 */
uint16_t UART_Read(UART_Device *dev, uint8_t *buf, uint16_t max_len);

/** @brief 注册帧协议和回调 (链表, 最多 4 个) */
UART_Status UART_RegisterFrame(UART_Device *dev, UART_Frame *frame);

/** @brief 主循环周期调用: 环形→帧解析→回调 + 帧超时处理 */
void UART_Task(UART_Device *dev);

/** @brief 调试统计 */
const UART_Stats *UART_GetStats(const UART_Device *dev);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */