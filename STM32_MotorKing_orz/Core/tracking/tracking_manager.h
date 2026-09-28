/**
 ******************************************************************************
 * @file    tracking_manager.h
 * @brief   追踪管理器 — 视觉闭环与运动控制的核心适配层
 *
 * 职责 (requirements.md §18-21):
 *   - 消费 VisionTarget (来自 Communication)
 *   - 数据有效性/视觉超时判断
 *   - EMA 滤波 + 死区
 *   - 坐标转换 (像素→角度增量)
 *   - 输出 GimbalSetpoint 给 Motion
 *
 * 输入: VisionTarget + 当前 Pan/Tilt
 * 输出: GimbalSetpoint
 ******************************************************************************
 */
#ifndef __TRACKING_MANAGER_H
#define __TRACKING_MANAGER_H

#include <stdbool.h>
#include <stdint.h>
#include "communication.h"
#include "coordinate_transform.h"
#include "motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  状态                                                                      */
/* ========================================================================= */
typedef enum
{
    TRACKING_DISABLED = 0,
    TRACKING_SEARCH,
    TRACKING_ACQUIRE,
    TRACKING_TRACK,
    TRACKING_LOST
} TrackingState;

/* ========================================================================= */
/*  配置                                                                      */
/* ========================================================================= */
typedef struct
{
    /* 坐标转换参数 */
    float center_x;
    float center_y;
    float pixel_to_angle_x;
    float pixel_to_angle_y;

    /* 死区 px */
    float deadband_x;
    float deadband_y;

    /* 滤波系数 (0~1, 越大越跟随快) */
    float filter_alpha;

    /* 视觉超时 ms */
    uint32_t vision_timeout_ms;

    /* ACQUIRE 稳定帧数 */
    uint32_t acquire_frames;
} TrackingConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定 Motion, 配置参数) */
void Tracking_Init(TrackingConfig *cfg, const MotionConfig *motion_cfg);

/** @brief 周期更新 (主循环调用, 10~20ms) */
void Tracking_Update(uint32_t now_ms);

/** @brief 送入视觉目标 */
void Tracking_SetVisionTarget(const VisionTarget *target);

/** @brief 查询追踪状态 */
TrackingState Tracking_GetState(void);

/** @brief 使能/禁止追踪 */
void Tracking_SetEnabled(bool enabled);

#ifdef __cplusplus
}
#endif

#endif /* __TRACKING_MANAGER_H */