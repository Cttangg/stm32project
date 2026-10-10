/**
 ******************************************************************************
 * @file    calib.c
 * @brief   KEY_CALIB 标定流程实现 — 释放电机 → 手动摆点 → 逐点记录
 ******************************************************************************
 */
#include "calib.h"
#include "gimbal_points.h"
#include "main.h"     /* KEY_CALIB_GPIO_Port / KEY_CALIB_Pin */
#include "uart.h"
#include <stdio.h>
#include <string.h>

/* ========================================================================= */
/*  内部状态                                                                   */
/* ========================================================================= */
static CalibConfig s_cfg;
static uint8_t     s_prev_key;
static uint32_t    s_last_ms;
static bool        s_active;
static uint8_t     s_index;

/* 打印到调试口 (COM) */
static void calib_print(const char *s)
{
    uint16_t w = 0;
    if (s_cfg.uart && s) {
        UART_Send(s_cfg.uart, (const uint8_t *)s, (uint16_t)strlen(s), &w);
    }
}

/* 释放两轴电机 (EN 断开, 手动可扳动) */
static void calib_release(void)
{
    if (s_cfg.pan)  Axis_Disable(s_cfg.pan);
    if (s_cfg.tilt) Axis_Disable(s_cfg.tilt);
}

/* 重新锁定两轴: 目标 = 当前角度, 避免使能瞬间跳变 */
static void calib_relock(void)
{
    if (s_cfg.pan && s_cfg.enc_pan) {
        Axis_SetTarget(s_cfg.pan, MT6701_ReadDegrees(s_cfg.enc_pan));
        Axis_Enable(s_cfg.pan);
    }
    if (s_cfg.tilt && s_cfg.enc_tilt) {
        Axis_SetTarget(s_cfg.tilt, MT6701_ReadDegrees(s_cfg.enc_tilt));
        Axis_Enable(s_cfg.tilt);
    }
}

void Calib_Init(const CalibConfig *cfg)
{
    s_cfg      = *cfg;
    s_prev_key = 0;
    s_last_ms  = 0;
    s_active   = false;
    s_index    = 0;
}

bool Calib_IsActive(void)
{
    return s_active;
}

uint8_t Calib_Index(void)
{
    return s_index;
}

void Calib_Update(uint32_t now_ms)
{
    /* 低有效按键 (内部上拉): 按下 = 0 */
    uint8_t key = (HAL_GPIO_ReadPin(KEY_CALIB_GPIO_Port, KEY_CALIB_Pin) == GPIO_PIN_RESET)
                  ? 1u : 0u;

    /* 上升沿 + 100ms 防抖 */
    if (key && !s_prev_key && (now_ms - s_last_ms) >= 100u) {
        char msg[64];
        s_last_ms = now_ms;

        if (!s_active) {
            /* 第 1 步: 释放电机, 进入标定 (不记录) */
            s_active = true;
            s_index  = 0;
            calib_release();
            calib_print("CAL-START: motors released, move to O then press\r\n");
        } else {
            /* 记录当前手动摆放的点 */
            float pan  = MT6701_ReadDegrees(s_cfg.enc_pan);
            float tilt = MT6701_ReadDegrees(s_cfg.enc_tilt);

            if (pan <= 360.0f && tilt <= 360.0f) {   /* 排除读失败哨兵 */
                GimbalPoints_Record((GimbalPointId)s_index, pan, tilt);
                snprintf(msg, sizeof(msg), "CAL pt=%u pan=%.2f tilt=%.2f\r\n",
                         (unsigned)s_index, pan, tilt);
                calib_print(msg);

                s_index++;
                if (s_index >= (uint8_t)GP_COUNT) {
                    s_active = false;
                    calib_relock();     /* 5 点记录完成, 重新锁定 */
                    calib_print("CAL-DONE: 5 points recorded, motors locked\r\n");
                }
            } else {
                calib_print("CAL read fail, retry\r\n");
            }
        }
    }

    s_prev_key = key;
}
