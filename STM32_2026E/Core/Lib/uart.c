/**
 ******************************************************************************
 * @file    uart.c
 * @brief   STM32 通用串口库实现 — HAL_UART + DMA 环形接收 (句柄化, 多实例)
 *
 * RX:  DMA CIRCULAR 持续写入 rx_dma_buf, IDLE/HT/TC 事件在
 *      HAL_UARTEx_RxEventCallback 中通过 NDTR 计算写入位置, 增量搬运到软件环形.
 * TX:  TX 环形缓冲 → DMA NORMAL 分块发送, HAL_UART_TxCpltCallback 链式续发.
 *
 * 底层 DMA 由 CubeMX 配置: RX=CIRCULAR, TX=NORMAL.
 *
 * HAL 回调分发: UART_Init 时将设备登记进全局注册表, 回调按 huart 指针查表.
 ******************************************************************************
 */
#include "uart.h"
#include <string.h>

/* ========================================================================= */
/*  常量                                                                      */
/* ========================================================================= */
#define UART_DEV_MAX  6   /* 全局注册表上限 */

/* ========================================================================= */
/*  设备注册表 (HAL 回调按 huart 反查设备)                                     */
/* ========================================================================= */
static UART_Device *g_devs[UART_DEV_MAX];
static uint8_t      g_dev_cnt;

static UART_Device *find_dev(UART_HandleTypeDef *huart) {
    uint8_t i;
    for (i = 0; i < g_dev_cnt; i++)
        if (g_devs[i]->huart == huart) return g_devs[i];
    return NULL;
}

/* ========================================================================= */
/*  软件环形缓冲 (单生产者-单消费者, ISR 安全)                                 */
/* ========================================================================= */
static void rb_init(UART_RingBuf *rb, uint8_t *buf, uint16_t size) {
    rb->buf = buf; rb->size = size;
    rb->write = 0; rb->read = 0; rb->overflow = 0;
}
static uint16_t rb_avail(const UART_RingBuf *rb) {
    __DMB();
    return (uint16_t)(rb->write - rb->read) & (rb->size - 1);
}
static void rb_reset(UART_RingBuf *rb) {
    rb->write = 0; rb->read = 0; rb->overflow = 0;
}
static uint16_t rb_write_isr(UART_RingBuf *rb, const uint8_t *d, uint16_t n) {
    uint16_t f, i;
    f = (rb->size - 1 - rb_avail(rb));
    if (n > f) { n = f; rb->overflow++; }
    for (i = 0; i < n; i++) {
        rb->buf[rb->write] = d[i];
        rb->write = (rb->write + 1) & (rb->size - 1);
    }
    __DMB();
    return n;
}
static uint16_t rb_read(UART_RingBuf *rb, uint8_t *dst, uint16_t max) {
    uint16_t a, n, i;
    a = rb_avail(rb);
    n = (a < max) ? a : max;
    for (i = 0; i < n; i++) {
        dst[i] = rb->buf[rb->read];
        rb->read = (rb->read + 1) & (rb->size - 1);
    }
    __DMB();
    return n;
}

/* ========================================================================= */
/*  TX: DMA 链式发送                                                          */
/* ========================================================================= */
static void tx_kick(UART_Device *dev) {
    uint16_t n;
    if (dev->tx_busy) return;
    n = rb_read(&dev->tx_rb, dev->tx_temp, sizeof(dev->tx_temp));
    if (n == 0) return;
    dev->tx_busy = 1;
    dev->stats.tx_bytes += n;
    if (HAL_UART_Transmit_DMA(dev->huart, dev->tx_temp, n) != HAL_OK) {
        dev->tx_busy = 0;   /* 启动失败, 稍后由 UART_Send 重试 */
    }
}

/* ========================================================================= */
/*  Frame 解析 (链表)                                                         */
/* ========================================================================= */
static int frame_add(UART_Device *dev, UART_Frame *f) {
    UART_Frame *s;
    if (dev->frame_used >= sizeof(dev->frame_pool) / sizeof(dev->frame_pool[0]))
        return UART_ERR_BUSY;
    s = &dev->frame_pool[dev->frame_used++];
    memcpy(s, f, sizeof(UART_Frame));
    s->next = dev->frame_list;
    dev->frame_list = s;
    s->state = 0; s->recv_count = 0;
    return UART_OK;
}

static void frame_feed(UART_Device *dev, uint8_t byte) {
    UART_Frame *f;
    if (dev->frame_idx < sizeof(dev->frame_buf)) dev->frame_buf[dev->frame_idx++] = byte;
    else { dev->frame_idx = 0; return; }

    for (f = dev->frame_list; f; f = f->next) {
        uint8_t hl = f->header_len;

        if (hl == 0) {  /* 逐字节回调 */
            if (f->callback) f->callback(dev, &byte, 1);
            continue;
        }
        if (hl > 4 || dev->frame_idx < hl) continue;

        if (f->state == 0) {  /* HEADER */
            uint8_t m = 1, j; uint16_t s = dev->frame_idx - hl;
            for (j = 0; j < hl; j++)
                if (dev->frame_buf[s + j] != f->header[j]) { m = 0; break; }
            if (!m) continue;
            if (s > 0) { memmove(dev->frame_buf, &dev->frame_buf[s], hl); dev->frame_idx = hl; }
            f->recv_count = hl; f->state = 1; f->last_tick = HAL_GetTick();
        }
        if (f->state == 1) {  /* LENGTH */
            uint16_t el = f->get_length ? f->get_length(dev->frame_buf) : hl;
            if (el == 0 || el > f->max_len) { f->state = 0; f->recv_count = 0; continue; }
            f->expected_len = el; f->state = 2;
        }
        if (f->state == 2 && dev->frame_idx >= f->expected_len) f->state = 3;
        if (f->state == 3) {  /* CHECK */
            uint8_t ok = f->check ? f->check(dev->frame_buf, f->expected_len) : 1;
            if (ok && f->callback)
                f->callback(dev, dev->frame_buf, f->expected_len);
            f->state = 0; f->recv_count = 0;
        }
    }
    if (dev->frame_idx >= sizeof(dev->frame_buf)) dev->frame_idx = 0;
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

void UART_Init(UART_Device *dev, UART_HandleTypeDef *huart) {
    uint8_t i;

    memset(dev, 0, sizeof(UART_Device));
    dev->huart = huart;
    rb_init(&dev->rx_rb, dev->rx_rb_buf, sizeof(dev->rx_rb_buf));
    rb_init(&dev->tx_rb, dev->tx_rb_buf, sizeof(dev->tx_rb_buf));

    /* 登记设备, 供 HAL 回调按 huart 反查 */
    if (g_dev_cnt < UART_DEV_MAX) {
        for (i = 0; i < g_dev_cnt; i++)
            if (g_devs[i] == dev) break;
        if (i == g_dev_cnt) g_devs[g_dev_cnt++] = dev;
    }
}

UART_Status UART_Open(UART_Device *dev) {
    if (!dev || !dev->huart) return UART_ERR_PARAM;   /* 未绑定 */
    if (dev->opened) return UART_ERR_BUSY;

    /* 清理状态 */
    rb_reset(&dev->rx_rb); rb_reset(&dev->tx_rb);
    dev->rx_last_pos = 0; dev->tx_busy = 0;
    dev->frame_list = NULL; dev->frame_used = 0; dev->frame_idx = 0;
    memset(&dev->stats, 0, sizeof(dev->stats));

    /* 启动 RX: DMA CIRCULAR + IDLE, 数据在 RxEventCallback 中搬运 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(dev->huart, dev->rx_dma_buf,
                                     sizeof(dev->rx_dma_buf)) != HAL_OK) {
        return UART_ERR_BUSY;
    }

    dev->opened = 1;
    return UART_OK;
}

UART_Status UART_Close(UART_Device *dev) {
    if (!dev || !dev->opened) return UART_ERR_NOTOPEN;
    HAL_UART_Abort(dev->huart);
    rb_reset(&dev->rx_rb); rb_reset(&dev->tx_rb);
    dev->opened = 0;
    return UART_OK;
}

UART_Status UART_Send(UART_Device *dev, const uint8_t *data, uint16_t len,
                      uint16_t *written) {
    uint16_t w;
    if (!dev) return UART_ERR_PARAM;
    if (!dev->opened) return UART_ERR_NOTOPEN;
    if (!data || !len) return UART_ERR_PARAM;

    w = rb_write_isr(&dev->tx_rb, data, len);
    dev->stats.tx_overflow = dev->tx_rb.overflow;
    if (written) *written = w;
    if (!dev->tx_busy) tx_kick(dev);
    return (w == len) ? UART_OK : UART_ERR_FULL;
}

uint8_t UART_IsSending(const UART_Device *dev) {
    if (!dev) return 0;
    return dev->tx_busy || (rb_avail(&dev->tx_rb) > 0);
}

uint16_t UART_Available(const UART_Device *dev) {
    if (!dev) return 0;
    return rb_avail(&dev->rx_rb);
}

uint16_t UART_Read(UART_Device *dev, uint8_t *buf, uint16_t max_len) {
    if (!dev || !buf || !max_len) return 0;
    return rb_read(&dev->rx_rb, buf, max_len);
}

UART_Status UART_RegisterFrame(UART_Device *dev, UART_Frame *frame) {
    if (!dev || !frame) return UART_ERR_PARAM;
    if (!dev->opened) return UART_ERR_NOTOPEN;
    if (frame->timeout_ms == 0) frame->timeout_ms = 100;
    return (UART_Status)frame_add(dev, frame);
}

void UART_Task(UART_Device *dev) {
    UART_Frame *f;
    uint8_t byte;
    if (!dev || !dev->opened) return;

    while (rb_read(&dev->rx_rb, &byte, 1)) {
        frame_feed(dev, byte);
        dev->stats.rx_bytes++;
    }

    for (f = dev->frame_list; f; f = f->next) {
        if (f->state != 0) {   /* 非 HEADER 态, 超时复位 */
            if (HAL_GetTick() - f->last_tick > f->timeout_ms) {
                f->state = 0; f->recv_count = 0;
            }
        }
    }
}

const UART_Stats *UART_GetStats(const UART_Device *dev) {
    if (!dev) return NULL;
    ((UART_Device *)dev)->stats.rx_overflow = dev->rx_rb.overflow;
    return &dev->stats;
}

/* ========================================================================= */
/*  HAL 回调 (弱函数覆盖, 由 USART/DMA ISR 触发)                              */
/* ========================================================================= */

/**
 * RX 事件回调: IDLE(一帧结束) / DMA HT / DMA TC 都会进入.
 * 用 NDTR 计算 DMA 当前写位置, 把 last_pos→当前 的字节增量搬进软件环形.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_Device *dev = find_dev(huart);
    uint16_t ndtr, wpos, lpos;
    (void)Size;
    if (!dev || !dev->opened) return;

    ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    wpos = (sizeof(dev->rx_dma_buf) - ndtr) & (sizeof(dev->rx_dma_buf) - 1);
    lpos = dev->rx_last_pos;

    if (wpos > lpos) {
        rb_write_isr(&dev->rx_rb, &dev->rx_dma_buf[lpos], wpos - lpos);
    } else if (wpos < lpos) {
        rb_write_isr(&dev->rx_rb, &dev->rx_dma_buf[lpos],
                     sizeof(dev->rx_dma_buf) - lpos);
        if (wpos > 0) rb_write_isr(&dev->rx_rb, dev->rx_dma_buf, wpos);
    }
    dev->rx_last_pos = wpos;
    dev->stats.rx_events++;
}

/** TX 完成回调: 链式续发 TX 环形缓冲中的剩余数据 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    UART_Device *dev = find_dev(huart);
    if (!dev || !dev->opened) return;
    dev->tx_busy = 0;
    tx_kick(dev);
}

/** 错误回调: 累计 ORE/FE/NE */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    UART_Device *dev = find_dev(huart);
    if (dev) dev->stats.uart_errors++;
}