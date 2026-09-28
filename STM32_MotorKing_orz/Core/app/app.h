/**
 ******************************************************************************
 * @file    app.h
 * @brief   应用状态机 — 系统级模式与模块启停
 *
 * 职责 (requirements.md §22):
 *   - 管理系统模式 (INIT/IDLE/SEARCH/TRACK/LOST/FAULT)
 *   - 接收用户命令 (启动/停止/搜索)
 *   - 协调 Tracking / Motion / Safety 的启停
 *
 * Application 不允许直接操作 TMC2209 / MotorPID / MT6701 (由下层完成).
 ******************************************************************************
 */
#ifndef __APP_H
#define __APP_H

#include <stdbool.h>
#include <stdint.h>
#include "safety.h"
#include "motion.h"
#include "tracking_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  状态                                                                      */
/* ========================================================================= */
typedef enum
{
    APP_INIT = 0,
    APP_IDLE,
    APP_SEARCH,
    APP_TRACK,
    APP_LOST,
    APP_FAULT
} AppState;

/* ========================================================================= */
/*  配置                                                                      */
/* ========================================================================= */
typedef struct
{
    SafetyConfig *safety;      /* 安全 */
    MotionConfig *motion;      /* 运动 */
    float search_pan_deg;      /* SEARCH 模式下 Pan 扫描目标 (占位) */
} AppConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定下层模块, 状态 → IDLE) */
void App_Init(const AppConfig *cfg);

/** @brief 周期更新 (主循环调用, 由状态驱动下层) */
void App_Update(uint32_t now_ms);

/** @brief 查询状态 */
AppState App_GetState(void);

/** @brief 用户命令: 开始追踪 */
void App_RequestStart(void);

/** @brief 用户命令: 停止追踪 */
void App_RequestStop(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */