/**
 ******************************************************************************
 * @file    communication.c
 * @brief   通信管理器实现 — 视觉帧解析 + VisionTarget + 超时
 *
 * 视觉帧格式 (小端, 与视觉主机约定):
 *   [0]  0xAA       帧头
 *   [1]  0x55       帧头
 *   [2]  TYPE       0x01 = VisionTarget
 *   [3]  LEN        payload 长度 (字节)
 *   [4..] PAYLOAD   x(float) y(float) valid(u8) seq(u32) ts(u32)
 *   [N]  CRC        XOR 校验 (帧头到 payload 末字节)
 *
 * 接收链路: UART DMA → RingBuffer → 帧解析 (uart.c) → 本模块校验 → VisionTarget
 ******************************************************************************
 */
#include "communication.h"
#include <string.h>

/* ========================================================================= */
/*  视觉帧协议常量                                                             */
/* ========================================================================= */
#define VFRAME_HEAD0      0xAA
#define VFRAME_HEAD1      0x55
#define VFRAME_TYPE_TARGET 0x01
#define VFRAME_PAYLOAD    17   /* x+y+valid+seq+ts */
#define VFRAME_LEN        22   /* 2 head + 1 type + 1 len + 17 + 1 crc */

/* ========================================================================= */
/*  内部状态                                                                    */
/* ========================================================================= */
static UART_Device  *s_cam;
static VisionState   s_vision;
static uint8_t       s_vframe;   /* 注册的帧描述符 (拷贝进 UART 池, 需保持存活) */
static UART_Frame    s_frame;

/* 长度回调: 等 LEN 字节到达后计算总帧长 */
static uint16_t vision_get_length(const uint8_t *buf)
{
    uint8_t len = buf[3];
    if (len != VFRAME_PAYLOAD) return 0;          /* 非法 payload 长度 */
    return VFRAME_LEN;
}

/* 校验回调: XOR 全帧 */
static uint8_t vision_check(const uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    for (i = 0; i < len - 1; i++) crc ^= buf[i];
    return (crc == buf[len - 1]) ? 1 : 0;
}

/* 帧完成回调 (UART_Task 上下文): 解析为 VisionTarget */
static void vision_on_frame(UART_Device *dev, uint8_t *data, uint16_t len)
{
    uint32_t now = HAL_GetTick();
    (void)dev; (void)len;

    if (data[2] != VFRAME_TYPE_TARGET) return;

    s_vision.target.x = *((float *)(data + 4));
    s_vision.target.y = *((float *)(data + 8));
    s_vision.target.valid = (data[12] != 0);
    memcpy(&s_vision.target.sequence, data + 13, 4);
    memcpy(&s_vision.target.timestamp, data + 17, 4);

    s_vision.receive_time = now;
    if (s_vision.target.valid) s_vision.last_valid_time = now;
    s_vision.timeout = false;
}

void Communication_Init(UART_Device *cam)
{
    s_cam = cam;
    memset(&s_vision, 0, sizeof(s_vision));

    s_frame.header[0] = VFRAME_HEAD0;
    s_frame.header[1] = VFRAME_HEAD1;
    s_frame.header_len = 2;
    s_frame.max_len = VFRAME_LEN;
    s_frame.get_length = vision_get_length;
    s_frame.check = vision_check;
    s_frame.callback = vision_on_frame;
    s_frame.timeout_ms = 100;

    UART_RegisterFrame(cam, &s_frame);
    s_vframe = 1;
}

void Communication_Update(uint32_t now_ms)
{
    /* 驱动底层帧解析 (必须周期调用) */
    if (s_cam) UART_Task(s_cam);
    (void)now_ms;
}

bool Communication_PopVisionTarget(VisionTarget *target)
{
    if (!s_vision.receive_time) return false;   /* 尚未收到任何帧 */
    *target = s_vision.target;
    return true;
}

const VisionState *Communication_GetVisionState(void)
{
    return &s_vision;
}

bool Communication_IsVisionTimeout(uint32_t now_ms, uint32_t timeout_ms)
{
    if (s_vision.receive_time == 0) return true;          /* 从未收到 → 超时 */
    return ((now_ms - s_vision.receive_time) > timeout_ms);
}