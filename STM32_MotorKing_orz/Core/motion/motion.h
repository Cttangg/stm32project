/**
 ******************************************************************************
 * @file    motion.h
 * @brief   运动管理器 — 把目标姿态转换为 Pan/Tilt 轴目标, 双轴协调
 *
 * 职责 (requirements.md §10-11):
 *   - 接收 GimbalSetpoint (pan/tilt 目标角度)
 *   - 位置限幅 (由 Axis 内部再次钳位)
 *   - 双轴同时启动 / 各自限速限加速度
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
} MotionConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定 Pan/Tilt 轴) */
void Motion_Init(const MotionConfig *cfg);

/** @brief 设置目标姿态 (deg); 两轴同时开始运动 */
void Motion_SetTarget(float pan, float tilt);

/** @brief 停止双轴 (回到当前位置保持) */
void Motion_Stop(void);

/** @brief 周期更新 (100~200Hz): 把 setpoint 下发到 Axis */
void Motion_Update(float dt);

/** @brief 查询当前 setpoint */
GimbalSetpoint Motion_GetSetpoint(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */