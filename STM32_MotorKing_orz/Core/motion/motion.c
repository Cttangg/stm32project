/**
 ******************************************************************************
 * @file    motion.c
 * @brief   运动管理器实现 — 双轴目标协调
 *
 * 双轴协调 (requirements.md §11):
 *   两轴同时启动 / 各自限速 / 各自限加速度 (由 Axis→PID 内部完成)
 *   Motion 不做跨轴轨迹规划, 仅统一目标入口.
 ******************************************************************************
 */
#include "motion.h"

static MotionConfig   s_cfg;
static GimbalSetpoint s_target;
static bool           s_inited;

void Motion_Init(const MotionConfig *cfg)
{
    s_cfg    = *cfg;
    s_target.pan  = 0.0f;
    s_target.tilt = 0.0f;
    s_inited = true;
}

void Motion_SetTarget(float pan, float tilt)
{
    if (!s_inited) return;

    s_target.pan  = pan;
    s_target.tilt = tilt;

    /* 双轴同时下发目标, 各自由 Axis 内部限幅/限速/限加速度 */
    if (s_cfg.pan)  Axis_SetTarget(s_cfg.pan, pan);
    if (s_cfg.tilt) Axis_SetTarget(s_cfg.tilt, tilt);
}

void Motion_Stop(void)
{
    if (!s_inited) return;

    /* 停到当前位置: 用当前角度作为目标, 保持不动 */
    if (s_cfg.pan) {
        s_target.pan = Axis_GetPosition(s_cfg.pan);
        Axis_SetTarget(s_cfg.pan, s_target.pan);
    }
    if (s_cfg.tilt) {
        s_target.tilt = Axis_GetPosition(s_cfg.tilt);
        Axis_SetTarget(s_cfg.tilt, s_target.tilt);
    }
}

void Motion_Update(float dt)
{
    (void)dt;   /* 目标已在 Motion_SetTarget 立即下发; 周期仅用于未来轨迹规划 */
}

GimbalSetpoint Motion_GetSetpoint(void)
{
    return s_target;
}