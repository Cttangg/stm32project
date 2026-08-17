/**
 ******************************************************************************
 * @file    uart.c
 * @brief   STM32 通用串口库实现 — HAL_UART + DMA 环形接收
 *
 * RX:  DMA CIRCULAR 持续写入 rx_dma_buf, IDLE/HT/TC 事件在
 *      HAL_UARTEx_RxEventCallback 中通过 NDTR 计算写入位置, 增量搬运到软件环形.
 * TX:  TX 环形缓冲 → DMA NORMAL 分块发送, HAL_UART_TxCpltCallback 链式续发.
 *
 * 底层 DMA 由 CubeMX 配置: RX=CIRCULAR, TX=NORMAL.
 ******************************************************************************
 */
#include "uart.h"
#include "usart.h"
#include <string.h>

/* ========================================================================= */
/*  常量                                                                      */
/* ========================================================================= */
#define UART_RX_DMA_SIZE  256
#define UART_RX_RING_SIZE 1024
#define UART_TX_RING_SIZE 512
#define UART_TX_CHUNK     128
#define UART_FRAME_BUF    256
#define UART_FRAME_MAX    4

/* ========================================================================= */
/*  软件环形缓冲 (单生产者-单消费者, ISR 安全)                                 */
/* ========================================================================= */
typedef struct {
    uint8_t           *buf;
    uint16_t           size;    /* 2^n */
    volatile uint16_t  write;
    volatile uint16_t  read;
    volatile uint32_t  overflow;
} UART_RingBuf;

static void rb_init(UART_RingBuf *rb, uint8_t *buf, uint16_t size) {
    rb->buf = buf; rb->size = size;
    rb->write = 0; rb->read = 0; rb->overflow = 0;
}
static uint16_t rb_avail(const UART_RingBuf *rb) {
    __DMB();
    return (uint16_t)(rb->write - rb->read) & (rb->size - 1);
}
static uint16_t rb_free(const UART_RingBuf *rb) {
    return (rb->size - 1 - rb_avail(rb));
}
static void rb_reset(UART_RingBuf *rb) {
    rb->write = 0; rb->read = 0; rb->overflow = 0;
}
static uint16_t rb_write_isr(UART_RingBuf *rb, const uint8_t *d, uint16_t n) {
    uint16_t f = rb_free(rb), i;
    if (n > f) { n = f; rb->overflow++; }
    for (i = 0; i < n; i++) {
        rb->buf[rb->write] = d[i];
        rb->write = (rb->write + 1) & (rb->size - 1);
    }
    __DMB();
    return n;
}
static uint16_t rb_read(UART_RingBuf *rb, uint8_t *dst, uint16_t max) {
    uint16_t a = rb_avail(rb), n = (a < max) ? a : max, i;
    for (i = 0; i < n; i++) {
        dst[i] = rb->buf[rb->read];
        rb->read = (rb->read + 1) & (rb->size - 1);
    }
    __DMB();
    return n;
}

/* ========================================================================= */
/*  端口实例                                                                  */
/* ========================================================================= */
typedef struct {
    UART_HandleTypeDef *huart;     /* NULL = 未注册端口 */
    uint8_t             opened;

    /* RX */
    uint8_t             rx_dma_buf[UART_RX_DMA_SIZE];
    volatile uint16_t   rx_last_pos;
    UART_RingBuf        rx_rb;
    uint8_t             rx_rb_buf[UART_RX_RING_SIZE];

    /* TX */
    UART_RingBuf        tx_rb;
    uint8_t             tx_rb_buf[UART_TX_RING_SIZE];
    uint8_t             tx_temp[UART_TX_CHUNK];
    volatile uint8_t    tx_busy;

    /* Frame */
    uint8_t             frame_buf[UART_FRAME_BUF];
    uint16_t            frame_idx;
    UART_Frame         *frame_list;
    UART_Frame          frame_pool[UART_FRAME_MAX];
    uint8_t             frame_used;

    UART_Stats          stats;
} UART_Instance;

static UART_Instance g_uart[UART_PMAX];

static UART_Instance *find_inst(UART_HandleTypeDef *huart) {
    uint8_t i;
    for (i = 0; i < UART_PMAX; i++)
        if (g_uart[i].huart == huart) return &g_uart[i];
    return NULL;
}

/* ========================================================================= */
/*  TX: DMA 链式发送                                                          */
/* ========================================================================= */
static void tx_kick(UART_Instance *u) {
    uint16_t n;
    if (u->tx_busy) return;
    n = rb_read(&u->tx_rb, u->tx_temp, UART_TX_CHUNK);
    if (n == 0) return;
    u->tx_busy = 1;
    u->stats.tx_bytes += n;
    if (HAL_UART_Transmit_DMA(u->huart, u->tx_temp, n) != HAL_OK) {
        u->tx_busy = 0;   /* 启动失败, 稍后由 UART_Send 重试 */
    }
}

/* ========================================================================= */
/*  Frame 解析 (链表)                                                         */
/* ========================================================================= */
static int frame_add(UART_Instance *u, UART_Frame *f) {
    UART_Frame *s;
    if (u->frame_used >= UART_FRAME_MAX) return UART_ERR_BUSY;
    s = &u->frame_pool[u->frame_used++];
    memcpy(s, f, sizeof(UART_Frame));
    s->next = u->frame_list;
    u->frame_list = s;
    s->state = 0; s->recv_count = 0;
    return UART_OK;
}

static void frame_feed(UART_Instance *u, uint8_t byte) {
    UART_Frame *f;
    if (u->frame_idx < UART_FRAME_BUF) u->frame_buf[u->frame_idx++] = byte;
    else { u->frame_idx = 0; return; }

    for (f = u->frame_list; f; f = f->next) {
        uint8_t hl = f->header_len;

        if (hl == 0) {  /* 逐字节回调 */
            if (f->callback) f->callback((UART_Port)(u - g_uart), &byte, 1);
            continue;
        }
        if (hl > 4 || u->frame_idx < hl) continue;

        if (f->state == 0) {  /* HEADER */
            uint8_t m = 1, j; uint16_t s = u->frame_idx - hl;
            for (j = 0; j < hl; j++)
                if (u->frame_buf[s + j] != f->header[j]) { m = 0; break; }
            if (!m) continue;
            if (s > 0) { memmove(u->frame_buf, &u->frame_buf[s], hl); u->frame_idx = hl; }
            f->recv_count = hl; f->state = 1; f->last_tick = HAL_GetTick();
        }
        if (f->state == 1) {  /* LENGTH */
            uint16_t el = f->get_length ? f->get_length(u->frame_buf) : hl;
            if (el == 0 || el > f->max_len) { f->state = 0; f->recv_count = 0; continue; }
            f->expected_len = el; f->state = 2;
        }
        if (f->state == 2 && u->frame_idx >= f->expected_len) f->state = 3;
        if (f->state == 3) {  /* CHECK */
            uint8_t ok = f->check ? f->check(u->frame_buf, f->expected_len) : 1;
            if (ok && f->callback)
                f->callback((UART_Port)(u - g_uart), u->frame_buf, f->expected_len);
            f->state = 0; f->recv_count = 0;
        }
    }
    if (u->frame_idx >= UART_FRAME_BUF) u->frame_idx = 0;
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

void UART_Init(void) {
    uint8_t i;
    for (i = 0; i < UART_PMAX; i++) {
        UART_Instance *u = &g_uart[i];
        memset(u, 0, sizeof(UART_Instance));
        rb_init(&u->rx_rb, u->rx_rb_buf, UART_RX_RING_SIZE);
        rb_init(&u->tx_rb, u->tx_rb_buf, UART_TX_RING_SIZE);
    }
    /* 端口注册: 当前仅 USART1, 其余槽位预留.
       启用 USART2: g_uart[UART_P2].huart = &huart2; */
    g_uart[UART_P1].huart = &huart1;
}

UART_Status UART_Open(UART_Port port) {
    UART_Instance *u;
    UART_HandleTypeDef *hu;
    if (port >= UART_PMAX) return UART_ERR_PARAM;
    u = &g_uart[port];
    if (!u->huart) return UART_ERR_PARAM;   /* 端口未注册 */
    if (u->opened) return UART_ERR_BUSY;
    hu = u->huart;

    /* 清理状态 */
    rb_reset(&u->rx_rb); rb_reset(&u->tx_rb);
    u->rx_last_pos = 0; u->tx_busy = 0;
    u->frame_list = NULL; u->frame_used = 0; u->frame_idx = 0;
    memset(&u->stats, 0, sizeof(u->stats));

    /* 启动 RX: DMA CIRCULAR + IDLE, 数据在 RxEventCallback 中搬运 */
    if (HAL_UARTEx_ReceiveToIdle_DMA(hu, u->rx_dma_buf, UART_RX_DMA_SIZE) != HAL_OK) {
        return UART_ERR_BUSY;
    }

    u->opened = 1;
    return UART_OK;
}

UART_Status UART_Close(UART_Port port) {
    UART_Instance *u;
    if (port >= UART_PMAX) return UART_ERR_PARAM;
    u = &g_uart[port];
    if (!u->opened) return UART_ERR_NOTOPEN;
    HAL_UART_Abort(u->huart);
    rb_reset(&u->rx_rb); rb_reset(&u->tx_rb);
    u->opened = 0;
    return UART_OK;
}

UART_Status UART_Send(UART_Port port, const uint8_t *data, uint16_t len,
                      uint16_t *written) {
    UART_Instance *u;
    uint16_t w;
    if (port >= UART_PMAX) return UART_ERR_PARAM;
    u = &g_uart[port];
    if (!u->opened) return UART_ERR_NOTOPEN;
    if (!data || !len) return UART_ERR_PARAM;

    w = rb_write_isr(&u->tx_rb, data, len);
    u->stats.tx_overflow = u->tx_rb.overflow;
    if (written) *written = w;
    if (!u->tx_busy) tx_kick(u);
    return (w == len) ? UART_OK : UART_ERR_FULL;
}

uint8_t UART_IsSending(UART_Port port) {
    if (port >= UART_PMAX) return 0;
    return g_uart[port].tx_busy || (rb_avail(&g_uart[port].tx_rb) > 0);
}

uint16_t UART_Available(UART_Port port) {
    if (port >= UART_PMAX) return 0;
    return rb_avail(&g_uart[port].rx_rb);
}

uint16_t UART_Read(UART_Port port, uint8_t *buf, uint16_t max_len) {
    if (port >= UART_PMAX || !buf || !max_len) return 0;
    return rb_read(&g_uart[port].rx_rb, buf, max_len);
}

UART_Status UART_RegisterFrame(UART_Port port, UART_Frame *frame) {
    UART_Instance *u;
    if (port >= UART_PMAX || !frame) return UART_ERR_PARAM;
    u = &g_uart[port];
    if (!u->opened) return UART_ERR_NOTOPEN;
    if (frame->timeout_ms == 0) frame->timeout_ms = 100;
    return (UART_Status)frame_add(u, frame);
}

void UART_Task(void) {
    uint8_t i;
    for (i = 0; i < UART_PMAX; i++) {
        UART_Instance *u = &g_uart[i];
        UART_Frame *f;
        uint8_t byte;
        if (!u->opened) continue;

        while (rb_read(&u->rx_rb, &byte, 1)) {
            frame_feed(u, byte);
            u->stats.rx_bytes++;
        }

        for (f = u->frame_list; f; f = f->next) {
            if (f->state != 0) {   /* 非 HEADER 态, 超时复位 */
                if (HAL_GetTick() - f->last_tick > f->timeout_ms) {
                    f->state = 0; f->recv_count = 0;
                }
            }
        }
    }
}

const UART_Stats *UART_GetStats(UART_Port port) {
    if (port >= UART_PMAX) return NULL;
    g_uart[port].stats.rx_overflow = g_uart[port].rx_rb.overflow;
    return &g_uart[port].stats;
}

/* ========================================================================= */
/*  HAL 回调 (弱函数覆盖, 由 USART/DMA ISR 触发)                              */
/* ========================================================================= */

/**
 * RX 事件回调: IDLE(一帧结束) / DMA HT / DMA TC 都会进入.
 * 用 NDTR 计算 DMA 当前写位置, 把 last_pos→当前 的字节增量搬进软件环形.
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    UART_Instance *u = find_inst(huart);
    uint16_t ndtr, wpos, lpos;
    (void)Size;
    if (!u || !u->opened) return;

    ndtr = (uint16_t)__HAL_DMA_GET_COUNTER(huart->hdmarx);
    wpos = (UART_RX_DMA_SIZE - ndtr) & (UART_RX_DMA_SIZE - 1);
    lpos = u->rx_last_pos;

    if (wpos > lpos) {
        rb_write_isr(&u->rx_rb, &u->rx_dma_buf[lpos], wpos - lpos);
    } else if (wpos < lpos) {
        rb_write_isr(&u->rx_rb, &u->rx_dma_buf[lpos], UART_RX_DMA_SIZE - lpos);
        if (wpos > 0) rb_write_isr(&u->rx_rb, u->rx_dma_buf, wpos);
    }
    u->rx_last_pos = wpos;
    u->stats.rx_events++;
}

/** TX 完成回调: 链式续发 TX 环形缓冲中的剩余数据 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    UART_Instance *u = find_inst(huart);
    if (!u || !u->opened) return;
    u->tx_busy = 0;
    tx_kick(u);
}

/** 错误回调: 累计 ORE/FE/NE */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    UART_Instance *u = find_inst(huart);
    if (u) u->stats.uart_errors++;
}
