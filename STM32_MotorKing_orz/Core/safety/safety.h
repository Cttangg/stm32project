/**
 ******************************************************************************
 * @file    safety.h
 * @brief   安全管理器 — 横向模块, 统一监控按键/限位/编码器/故障
 *
 * 职责 (requirements.md §8-9):
 *   - 统一管理 按键 / 轴限位 / 编码器磁检 / 驱动故障 / 视觉超时
 *   - 出现紧急情况直接阻止运动 (优先级高于 Application/Tracking/Motion)
 *   - 提供 EmergencyStop / ClearFault 接口
 *
 * Safety 不依赖追踪算法, 不参与控制决策.
 ******************************************************************************
 */
#ifndef __SAFETY_H
#define __SAFETY_H

#include <stdbool.h>
#include <stdint.h>
#include "axis.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  安全等级                                                                    */
/* ========================================================================= */
typedef enum
{
    SAFETY_OK = 0,
    SAFETY_WARNING,
    SAFETY_STOP,
    SAFETY_FAULT
} SafetyLevel;

/* ========================================================================= */
/*  安全配置                                                                    */
/* ========================================================================= */
typedef struct
{
    Axis *pan;             /* Pan 轴 (用于限位/故障查询) */
    Axis *tilt;            /* Tilt 轴 */
    bool  enable_key_pause;/* 按键暂停使能 */
} SafetyConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化安全管理器 */
void Safety_Init(const SafetyConfig *cfg);

/** @brief 周期更新 (主循环调用): 轮询按键/限位/编码器/故障 */
void Safety_Update(uint32_t now_ms);

/** @brief 运动是否被允许 (等级为 OK/WARNING 时允许) */
bool Safety_IsMotionAllowed(void);

/** @brief 追踪是否被允许 (等级为 OK 时允许) */
bool Safety_IsTrackingAllowed(void);

/** @brief 紧急停止: 立即禁止两轴输出 */
void Safety_EmergencyStop(void);

/** @brief 清除故障 (仅按键复位流程可调用) */
void Safety_ClearFault(void);

/** @brief 查询当前安全等级 */
SafetyLevel Safety_GetLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* __SAFETY_H */