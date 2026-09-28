/**
 ******************************************************************************
 * @file    motion.c
 * @brief   运动管理器实现 — 位置限幅 + 双轴协调 + Safety 联动
 *
 * 链路 (Phase 3):
 *   target → 位置限幅 → Axis_SetTarget(Pan/Tilt 同时)
 *   速度/加速度限制由 Axis→PID 内部 max_speed/max_accel 承担 (参数未改)
 *   Safety 不允许运动时 → Motion_Stop()
 ******************************************************************************
 */
#include "motion.h"

static MotionConfig   s_cfg;
static GimbalSetpoint s_target;
static bool           s_inited;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

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

    /* ── 位置限幅 (Phase 3.2) ── */
    s_target.pan  = clampf(pan,  s_cfg.pan_min,  s_cfg.pan_max);
    s_target.tilt = clampf(tilt, s_cfg.tilt_min, s_cfg.tilt_max);

    /* ── 双轴同时下发目标, 各自由 Axis 内部限速/限加速度 ── */
    if (s_cfg.pan)  Axis_SetTarget(s_cfg.pan,  s_target.pan);
    if (s_cfg.tilt) Axis_SetTarget(s_cfg.tilt, s_target.tilt);
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
    (void)dt;

    if (!s_inited) return;

    /* ── Safety 联动 (Phase 3 验收 ⑦): 不允许运动 → 立即停止 ── */
    if (!Safety_IsMotionAllowed()) {
        Motion_Stop();
    }
}

GimbalSetpoint Motion_GetSetpoint(void)
{
    return s_target;
}

bool Motion_IsInPosition(void)
{
    if (!s_inited) return false;
    if (s_cfg.pan && !Axis_IsInPosition(s_cfg.pan)) return false;
    if (s_cfg.tilt && !Axis_IsInPosition(s_cfg.tilt)) return false;
    return true;
}