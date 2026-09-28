/**
 ******************************************************************************
 * @file    communication.h
 * @brief   通信管理器 — 在 UART 库之上提供帧校验与 VisionTarget
 *
 * 职责 (requirements.md §12-15):
 *   - 绑定摄像头串口 (uart.c UART_Device)
 *   - 注册视觉帧协议 (帧头/长度/校验)
 *   - 解析出 VisionTarget, 供 Tracking 层消费
 *   - 维护视觉超时
 *
 * Tracking 只能看到 VisionTarget, 不解析 UART 原始字节.
 ******************************************************************************
 */
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include <stdbool.h>
#include <stdint.h>
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  VisionTarget 数据结构                                                     */
/* ========================================================================= */
typedef struct
{
    float    x;          /* 目标图像坐标 X (px) */
    float    y;          /* 目标图像坐标 Y (px) */
    bool     valid;      /* 当前观测是否有效 */
    uint32_t timestamp;  /* 观测时间戳 ms */
    uint32_t sequence;   /* 帧序号 */
} VisionTarget;

typedef struct
{
    VisionTarget target;
    uint32_t     receive_time;     /* 最近接收时刻 ms */
    uint32_t     last_valid_time;  /* 最近有效帧时刻 ms */
    bool         timeout;          /* 视觉超时标志 */
} VisionState;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定摄像头串口设备, 注册帧协议) */
void Communication_Init(UART_Device *cam);

/** @brief 主循环周期调用 (底层 UART_Task + 超时更新) */
void Communication_Update(uint32_t now_ms);

/** @brief 弹出最新视觉目标 (消费后返回 true) */
bool Communication_PopVisionTarget(VisionTarget *target);

/** @brief 查询视觉状态 (含超时) */
const VisionState *Communication_GetVisionState(void);

/** @brief 视觉数据是否超时 */
bool Communication_IsVisionTimeout(uint32_t now_ms, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __COMMUNICATION_H */