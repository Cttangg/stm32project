/**
 ******************************************************************************
 * @file    pause.h
 * @brief   暂停/继续 (KEY_PAUSE) — 立即制动, 再按恢复
 *
 * 行为:
 *   按下 KEY_PAUSE: 保存当前目标 → 立即停 (Motion_Stop) → 冻结 Q12 序列
 *   再按一次:       恢复 (Q12 重发当前点 / 手动恢复保存的目标)
 *
 * 用于"暂停键立即制动, 便于测量光斑位置", 再按继续.
 ******************************************************************************
 */
#ifndef __PAUSE_H
#define __PAUSE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct UART_Device;

/* ========================================================================= */
/*  配置                                                                      */
/* ========================================================================= */
typedef struct
{
    struct UART_Device *uart;   /* 调试口 (状态打印, 可空) */
} PauseConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 */
void Pause_Init(const PauseConfig *cfg);

/** @brief 周期调用 (主循环): 轮询 KEY_PAUSE 边沿, 切换暂停/恢复 */
void Pause_Update(uint32_t now_ms);

/** @brief 是否处于暂停状态 */
bool Pause_IsActive(void);

#ifdef __cplusplus
}
#endif

#endif /* __PAUSE_H */
