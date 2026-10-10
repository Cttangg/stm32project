/**
 ******************************************************************************
 * @file    q12.h
 * @brief   Q1/Q2 序列 — RST 回原点 / BORDER 顺时针绕边一圈
 *
 * 行为 (按键, 低有效):
 *   RST   (KEY_RESET): 自动运动到 O 点 → 保持 idle
 *   BORDER(KEY_BORDER): 从当前点先到"左上", 再顺时针 左上→右上→右下→左下,
 *                       绕行一圈后回 O 点 → 保持 idle
 *
 * 点来自 gimbal_points (电机角度空间), 未标定的点拒绝启动.
 * 运动统一经 Motion_SetTarget(); 到位 (两轴 in_position) 并稳定 settle_ms
 * 后推进到下一点, 单点超过 point_timeout_ms 强制推进 (避免卡死).
 ******************************************************************************
 */
#ifndef __Q12_H
#define __Q12_H

#include <stdbool.h>
#include <stdint.h>
#include "axis.h"

#ifdef __cplusplus
extern "C" {
#endif

struct UART_Device;

/* ========================================================================= */
/*  配置 (依赖注入)                                                            */
/* ========================================================================= */
typedef struct
{
    Axis               *pan;
    Axis               *tilt;
    struct UART_Device *uart;             /* 调试口 (动作/推进打印, 可空) */
    uint32_t            settle_ms;        /* 到位后需保持的稳定时间 */
    uint32_t            point_timeout_ms; /* 单点最长耗时 (超时强制推进) */
} Q12Config;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定轴 + 调试口 + 时序) */
void Q12_Init(const Q12Config *cfg);

/** @brief 周期调用 (主循环): 轮询 RST/BORDER 按键 + 推进序列 */
void Q12_Update(uint32_t now_ms);

/** @brief RST 动作: 自动到 O 点 */
void Q12_Reset(void);

/** @brief BORDER 动作: 左上起顺时针绕一圈后回 O */
void Q12_Border(void);

/** @brief 序列是否正在运行 */
bool Q12_IsBusy(void);

/**
 * @brief 暂停/恢复序列 (由暂停键调用)
 *        true: 冻结推进 (外部负责停机); false: 重置计时并重发当前点目标
 */
void Q12_SetPaused(bool paused);

#ifdef __cplusplus
}
#endif

#endif /* __Q12_H */
