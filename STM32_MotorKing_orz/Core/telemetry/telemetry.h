/**
 ******************************************************************************
 * @file    telemetry.h
 * @brief   遥测 — 只读系统状态, 不参与控制
 *
 * 职责 (requirements.md §25):
 *   - 汇总 Pan/Tilt 位置/目标/速度
 *   - App / Tracking / Safety 状态
 *   - 通过调试串口周期发送 (或供上层读取)
 *
 * Telemetry 只读取状态, 不修改任何控制量.
 ******************************************************************************
 */
#ifndef __TELEMETRY_H
#define __TELEMETRY_H

#include <stdint.h>
#include "axis.h"
#include "safety.h"
#include "motion.h"
#include "app.h"
#include "communication.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  遥测数据结构                                                               */
/* ========================================================================= */
typedef struct
{
    float pan_position;
    float pan_target;
    float pan_velocity;

    float tilt_position;
    float tilt_target;
    float tilt_velocity;

    uint8_t app_state;
    uint8_t tracking_state;
    uint8_t safety_level;

    uint8_t fault_flags;

    uint32_t vision_age;    /* 距离上次有效帧 ms */
} Telemetry;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定各状态源) */
void Telemetry_Init(const Axis *pan, const Axis *tilt);

/** @brief 采样当前状态 (周期调用) */
void Telemetry_Sample(uint32_t now_ms);

/** @brief 读取最新遥测快照 */
const Telemetry *Telemetry_Get(void);

#ifdef __cplusplus
}
#endif

#endif /* __TELEMETRY_H */