/**
 ******************************************************************************
 * @file    pause.c
 * @brief   暂停/继续实现 — 立即制动 + 目标保存/恢复
 ******************************************************************************
 */
#include "pause.h"
#include "motion.h"
#include "q12.h"
#include "uart.h"
#include "main.h"     /* KEY_PAUSE_GPIO_Port / KEY_PAUSE_Pin */
#include <string.h>

/* ========================================================================= */
/*  内部状态                                                                   */
/* ========================================================================= */
static PauseConfig    s_cfg;
static bool           s_active;
static GimbalSetpoint s_saved;      /* 暂停前的目标 (非 Q12 场景恢复用) */
static uint8_t        s_prev_key;
static uint32_t       s_last_ms;

static void pause_print(const char *s)
{
    uint16_t w = 0;
    if (s_cfg.uart && s) {
        UART_Send(s_cfg.uart, (const uint8_t *)s, (uint16_t)strlen(s), &w);
    }
}

void Pause_Init(const PauseConfig *cfg)
{
    s_cfg      = *cfg;
    s_active   = false;
    s_saved.pan  = 0.0f;
    s_saved.tilt = 0.0f;
    s_prev_key = 0;
    s_last_ms  = 0;
}

bool Pause_IsActive(void)
{
    return s_active;
}

void Pause_Update(uint32_t now_ms)
{
    /* 低有效按键 (内部上拉): 按下 = 0 */
    uint8_t key = (HAL_GPIO_ReadPin(KEY_PAUSE_GPIO_Port, KEY_PAUSE_Pin) == GPIO_PIN_RESET)
                  ? 1u : 0u;

    if (key && !s_prev_key && (now_ms - s_last_ms) >= 50u) {
        s_last_ms = now_ms;

        if (!s_active) {
            /* 进入暂停: 保存目标 → 冻结序列 → 立即制动 */
            s_saved  = Motion_GetSetpoint();
            s_active = true;
            Q12_SetPaused(true);
            Motion_Stop();
            pause_print("PAUSE (brake)\r\n");
        } else {
            /* 恢复: Q12 重发当前点, 否则恢复保存的目标 */
            s_active = false;
            if (Q12_IsBusy()) {
                Q12_SetPaused(false);
            } else {
                Motion_SetTarget(s_saved.pan, s_saved.tilt);
            }
            pause_print("RESUME\r\n");
        }
    }

    s_prev_key = key;
}
