/**
 ******************************************************************************
 * @file    motion.h
 * @brief   运动管理器 — 把目标姿态转换为 Pan/Tilt 轴目标, 双轴协调
 *
 * 职责 (Phase 3):
 *   - 接收 GimbalSetpoint (pan/tilt 目标角度)
 *   - 位置限幅 (Pan/Tilt 角度范围钳位)
 *   - 双轴同时启动 / 各自限速限加速度 (由 Axis→PID 内部承担, 不改控制参数)
 *   - Safety 联动: 不允许运动时立即停止
 *   - 生成各 Axis 实时目标
 *
 * 第一阶段不做复杂轨迹规划 (梯形/S曲线/同步到位), 后续再扩展.
 * Motion 不直接操作 GPIO/Timer/Step.
 ******************************************************************************
 */
#ifndef __MOTION_H
#define __MOTION_H

#include <stdbool.h>
#include "axis.h"
#include "safety.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  数据结构                                                                    */
/* ========================================================================= */
typedef struct
{
    float pan;
    float tilt;
} GimbalSetpoint;

typedef struct
{
    Axis *pan;             /* Pan 轴 */
    Axis *tilt;            /* Tilt 轴 */

    /* 角度限位 (deg) */
    float pan_min;
    float pan_max;
    float tilt_min;
    float tilt_max;
} MotionConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定 Pan/Tilt 轴 + 角度限位) */
void Motion_Init(const MotionConfig *cfg);

/** @brief 设置目标姿态 (deg); 内部位置限幅后双轴同时开始运动 */
void Motion_SetTarget(float pan, float tilt);

/** @brief 停止双轴 (回到当前位置保持) */
void Motion_Stop(void);

/** @brief 周期更新 (主循环调用): Safety 检查 + 目标下发 */
void Motion_Update(float dt);

/** @brief 查询当前 setpoint (已限幅) */
GimbalSetpoint Motion_GetSetpoint(void);

/** @brief 双轴是否都已到位 */
bool Motion_IsInPosition(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */