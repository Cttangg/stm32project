/**
 ******************************************************************************
 * @file    safety.c
 * @brief   安全管理器实现 — 轮询按键/轴故障, 阻止不安全运动
 *
 * 按键映射 (低有效, GPIO 内部上拉):
 *   KEY_PAUSE → 暂停运动 (SAFETY_STOP)
 *   KEY_RESET → 清除故障
 *
 * 轴故障 (编码器磁检/限位) 由 Axis 内部检测, Safety 轮询 Axis_IsFault.
 ******************************************************************************
 */
#include "safety.h"
#include "main.h"

/* ========================================================================= */
/*  内部状态                                                                    */
/* ========================================================================= */
static SafetyConfig  s_cfg;
static SafetyLevel   s_level;

static uint32_t s_pause_pressed_ms;  /* 暂停键按下起始时间 (防抖) */
static uint32_t s_pause_debounce;    /* 防抖窗口 ms */
static uint32_t s_prev_key_reset;    /* 复位键边沿检测 */

/* 按键读取 (低有效: 按下 = 0) */
static uint8_t key_pressed(GPIO_TypeDef *port, uint16_t pin)
{
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET) ? 1 : 0;
}

void Safety_Init(const SafetyConfig *cfg)
{
    s_cfg = *cfg;
    s_level = SAFETY_OK;
    s_pause_pressed_ms = 0;
    s_pause_debounce = 50;     /* 50ms 防抖 */
    s_prev_key_reset = 0;
}

/* 紧急停止: 立即禁止两轴输出 (通过 Axis_Disable 释放驱动) */
void Safety_EmergencyStop(void)
{
    s_level = SAFETY_STOP;
    if (s_cfg.pan)  Axis_Disable(s_cfg.pan);
    if (s_cfg.tilt) Axis_Disable(s_cfg.tilt);
}

bool Safety_IsMotionAllowed(void)
{
    return (s_level == SAFETY_OK || s_level == SAFETY_WARNING);
}

bool Safety_IsTrackingAllowed(void)
{
    return (s_level == SAFETY_OK);
}

void Safety_ClearFault(void)
{
    s_level = SAFETY_OK;
    if (s_cfg.pan)  Axis_ClearFault(s_cfg.pan);
    if (s_cfg.tilt) Axis_ClearFault(s_cfg.tilt);
    /* 重新使能 (需要时由上层调用 Axis_Enable) */
}

SafetyLevel Safety_GetLevel(void)
{
    return s_level;
}

void Safety_Update(uint32_t now_ms)
{
    uint8_t pause, reset;

    /* 1. 轴故障: 故障轴已自行停止输出 (Axis 内部 FAULT 停)。
          不冻结另一轴: 单轴故障只隔离该轴, 便于诊断与另一轴继续运动。
          急停 (按键) 才全局禁止运动。 */
    if (Axis_IsFault(s_cfg.pan) && Axis_IsFault(s_cfg.tilt)) {
        s_level = SAFETY_FAULT;
        return;
    }

    /* 2. 按键暂停 (防抖) */
    if (s_cfg.enable_key_pause) {
        pause = key_pressed(KEY_PAUSE_GPIO_Port, KEY_PAUSE_Pin);
        if (pause) {
            if (s_pause_pressed_ms == 0) {
                s_pause_pressed_ms = now_ms;      /* 按下起始 */
            } else if ((now_ms - s_pause_pressed_ms) >= s_pause_debounce) {
                s_level = SAFETY_STOP;            /* 确认暂停 */
            }
        } else {
            s_pause_pressed_ms = 0;
        }
    }

    /* 3. 按键复位 (边沿检测, 清故障) */
    reset = key_pressed(KEY_RESET_GPIO_Port, KEY_RESET_Pin);
    if (reset && !s_prev_key_reset) {
        Safety_ClearFault();
    }
    s_prev_key_reset = reset;

    /* 4. 无故障且非暂停 → OK */
    if (s_level != SAFETY_STOP && s_level != SAFETY_FAULT) {
        s_level = SAFETY_OK;
    }
}