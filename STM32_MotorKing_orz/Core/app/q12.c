/**
 ******************************************************************************
 * @file    q12.c
 * @brief   Q1/Q2 序列实现 — 按标定点依次运动, 到位推进, 结束保持
 ******************************************************************************
 */
#include "q12.h"
#include "gimbal_points.h"
#include "motion.h"
#include "uart.h"
#include "main.h"     /* KEY_RESET_* / KEY_BORDER_* */
#include <stdio.h>
#include <string.h>

/* ========================================================================= */
/*  动作序列 (点序)                                                            */
/* ========================================================================= */
/* RST: 回原点 */
static const GimbalPointId SEQ_RESET[] = { GP_ORIGIN };

/* BORDER: 左上 → 右上 → 右下 → 左下 → 左上 (闭合左边, 四周走满) → O */
static const GimbalPointId SEQ_BORDER[] = {
    GP_LEFT_UP, GP_RIGHT_UP, GP_RIGHT_DOWN, GP_LEFT_DOWN, GP_LEFT_UP, GP_ORIGIN
};

/* ========================================================================= */
/*  内部状态                                                                   */
/* ========================================================================= */
typedef enum { Q12_IDLE = 0, Q12_BUSY } Q12State;

static Q12Config s_cfg;
static Q12State  s_state;

static const GimbalPointId *s_list;
static uint8_t   s_n;
static uint8_t   s_idx;
static uint32_t  s_move_start;
static uint32_t  s_settle_start;

static uint8_t   s_prev_rst;
static uint8_t   s_prev_border;
static uint32_t  s_key_ms;
static bool      s_paused;

static void q12_print(const char *s)
{
    uint16_t w = 0;
    if (s_cfg.uart && s) {
        UART_Send(s_cfg.uart, (const uint8_t *)s, (uint16_t)strlen(s), &w);
    }
}

/* 移动到第 idx 个点 */
static void seq_goto(uint8_t idx, uint32_t now_ms)
{
    GimbalPoint gp;
    char msg[48];

    GimbalPoints_Get(s_list[idx], &gp);
    Motion_SetTarget(gp.pan_deg, gp.tilt_deg);
    s_move_start   = now_ms;
    s_settle_start = 0;
    snprintf(msg, sizeof(msg), "Q12 MOVE pt=%u -> %.1f,%.1f\r\n",
             (unsigned)s_list[idx], gp.pan_deg, gp.tilt_deg);
    q12_print(msg);
}

/* 启动一个序列 (先校验所有点已标定) */
static void seq_start(const GimbalPointId *list, uint8_t n, const char *tag)
{
    GimbalPoint gp;
    char msg[48];
    uint8_t i;

    for (i = 0; i < n; i++) {
        if (!GimbalPoints_Get(list[i], &gp) || !gp.valid) {
            snprintf(msg, sizeof(msg), "Q12 %s ABORT: pt=%u not calibrated\r\n",
                     tag, (unsigned)list[i]);
            q12_print(msg);
            return;
        }
    }

    s_list  = list;
    s_n     = n;
    s_idx   = 0;
    s_state = Q12_BUSY;
    snprintf(msg, sizeof(msg), "Q12 %s START\r\n", tag);
    q12_print(msg);
    seq_goto(0, HAL_GetTick());
}

void Q12_Reset(void)
{
    seq_start(SEQ_RESET, (uint8_t)(sizeof(SEQ_RESET) / sizeof(SEQ_RESET[0])), "RST");
}

void Q12_Border(void)
{
    seq_start(SEQ_BORDER, (uint8_t)(sizeof(SEQ_BORDER) / sizeof(SEQ_BORDER[0])), "BORDER");
}

bool Q12_IsBusy(void)
{
    return (s_state == Q12_BUSY);
}

void Q12_Init(const Q12Config *cfg)
{
    s_cfg          = *cfg;
    s_state        = Q12_IDLE;
    s_list         = NULL;
    s_n            = 0;
    s_idx          = 0;
    s_move_start   = 0;
    s_settle_start = 0;
    s_prev_rst     = 0;
    s_prev_border  = 0;
    s_key_ms       = 0;
    s_paused       = false;
}

void Q12_SetPaused(bool paused)
{
    s_paused = paused;
    if (!paused && s_state == Q12_BUSY) {
        /* 恢复: 重置计时并重发当前点目标 (停机期间目标已被 Motion_Stop 改写) */
        seq_goto(s_idx, HAL_GetTick());
    }
}

/* 序列推进: 到位稳定 / 超时 → 下一点; 走完 → 保持 idle */
static void seq_run(uint32_t now_ms)
{
    if (!Axis_IsInPosition(s_cfg.pan) || !Axis_IsInPosition(s_cfg.tilt)) {
        s_settle_start = 0;    /* 未到位, 重置稳定计时 */
    } else if (s_settle_start == 0) {
        s_settle_start = now_ms;
    }

    /* 推进条件: 到位稳定 settle_ms, 或单点超时 */
    if ((s_settle_start != 0 && (now_ms - s_settle_start) >= s_cfg.settle_ms) ||
        (now_ms - s_move_start) >= s_cfg.point_timeout_ms) {

        s_idx++;
        if (s_idx >= s_n) {
            Motion_Stop();               /* 走完, 保持当前位置 (= O) */
            s_state = Q12_IDLE;
            q12_print("Q12 DONE (idle)\r\n");
            return;
        }
        seq_goto(s_idx, now_ms);
    }
}

void Q12_Update(uint32_t now_ms)
{
    /* 按键边沿 + 50ms 防抖 (暂停期间忽略 RST/BORDER) */
    uint8_t rst = (HAL_GPIO_ReadPin(KEY_RESET_GPIO_Port, KEY_RESET_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t bor = (HAL_GPIO_ReadPin(KEY_BORDER_GPIO_Port, KEY_BORDER_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    if (!s_paused && (now_ms - s_key_ms) >= 50u) {
        if (rst && !s_prev_rst) {
            s_key_ms = now_ms;
            Q12_Reset();
        } else if (bor && !s_prev_border) {
            s_key_ms = now_ms;
            Q12_Border();
        }
    }
    s_prev_rst    = rst;
    s_prev_border = bor;

    if (s_state == Q12_BUSY && !s_paused) {
        seq_run(now_ms);
    }
}
