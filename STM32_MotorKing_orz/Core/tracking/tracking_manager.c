/**
 ******************************************************************************
 * @file    tracking_manager.c
 * @brief   追踪管理器实现 — 目标→姿态→运动
 *
 * 处理流程 (requirements.md §6.3):
 *   VisionTarget → Validate → Timestamp Check → Filter → Transform → Deadband → Setpoint
 *
 * 状态机: DISABLED → SEARCH → ACQUIRE → TRACK → LOST
 * 追踪过程只改 Motion setpoint, 不直接操作电机.
 ******************************************************************************
 */
#include "tracking_manager.h"

/* ========================================================================= */
/*  内部状态                                                                    */
/* ========================================================================= */
static TrackingConfig   s_cfg;
static TrackingState    s_state;
static bool             s_enabled;

static VisionTarget     s_target;      /* 最近有效目标 (滤波前) */
static bool             s_has_target;

static float            s_filt_x, s_filt_y;   /* EMA 滤波后目标 */
static uint32_t         s_acquire_count;
static uint32_t         s_last_rx_ms;
static bool             s_valid;

/* Motion 目标: 当前云台姿态 + 增量 */
static float            s_gimbal_pan;
static float            s_gimbal_tilt;

static MotionConfig     s_motion_cfg;

/* ========================================================================= */
/*  内部辅助                                                                    */
/* ========================================================================= */

/* 简单 EMA 滤波 */
static void filter_update(float *filt, float sample, float alpha)
{
    *filt = alpha * sample + (1.0f - alpha) * *filt;
}

void Tracking_Init(TrackingConfig *cfg, const MotionConfig *motion_cfg)
{
    s_cfg = *cfg;
    s_motion_cfg = *motion_cfg;

    s_state      = TRACKING_DISABLED;
    s_enabled    = false;
    s_has_target = false;
    s_valid      = false;
    s_filt_x = s_filt_y = 0.0f;
    s_acquire_count = 0;
    s_last_rx_ms    = 0;
    s_gimbal_pan    = 0.0f;
    s_gimbal_tilt   = 0.0f;

    /* 坐标转换配置 */
    CoordTransform_Init(&(CoordTransformConfig){
        .center_x         = s_cfg.center_x,
        .center_y         = s_cfg.center_y,
        .pixel_to_angle_x = s_cfg.pixel_to_angle_x,
        .pixel_to_angle_y = s_cfg.pixel_to_angle_y,
    });
}

void Tracking_SetEnabled(bool enabled)
{
    s_enabled = enabled;
    if (enabled) {
        s_state = TRACKING_SEARCH;
        s_acquire_count = 0;
        s_valid = false;
    } else {
        s_state = TRACKING_DISABLED;
    }
}

void Tracking_SetVisionTarget(const VisionTarget *target)
{
    uint32_t now = HAL_GetTick();

    if (!s_enabled) return;

    if (target->valid) {
        s_target      = *target;
        s_has_target  = true;
        s_last_rx_ms  = now;
        s_valid       = true;
    } else {
        /* 无效目标: 保留最近有效数据, 由超时判定 */
        s_valid = false;
    }
}

TrackingState Tracking_GetState(void)
{
    return s_state;
}

void Tracking_Update(uint32_t now_ms)
{
    TrackingOffset off;
    uint32_t timeout;

    if (!s_enabled) return;

    /* ── 视觉超时: 超过 vision_timeout_ms 未收到有效目标 → LOST ── */
    timeout = (s_last_rx_ms == 0) ? 0xFFFFFFFFu : (now_ms - s_last_rx_ms);
    if (s_valid && timeout > s_cfg.vision_timeout_ms) {
        s_valid = false;          /* 旧目标不可再用 */
    }

    if (!s_valid || !s_has_target) {
        s_state = TRACKING_LOST;
        return;
    }

    /* ── 滤波 (EMA) ── */
    if (s_filt_x == 0.0f && s_filt_y == 0.0f) {
        s_filt_x = s_target.x;
        s_filt_y = s_target.y;
    } else {
        filter_update(&s_filt_x, s_target.x, s_cfg.filter_alpha);
        filter_update(&s_filt_y, s_target.y, s_cfg.filter_alpha);
    }

    /* ── 状态推进: ACQUIRE 需要连续稳定帧 ── */
    if (s_state == TRACKING_SEARCH || s_state == TRACKING_LOST) {
        s_state = TRACKING_ACQUIRE;
        s_acquire_count = 0;
    }

    if (s_state == TRACKING_ACQUIRE) {
        if (++s_acquire_count >= s_cfg.acquire_frames) {
            s_state = TRACKING_TRACK;
        }
    }

    /* ── 坐标转换 + 死区 ── */
    off = CoordTransform_ComputeOffset(s_filt_x, s_filt_y);

    if (off.pan_delta  < -s_cfg.deadband_x || off.pan_delta  > s_cfg.deadband_x ||
        off.tilt_delta < -s_cfg.deadband_y || off.tilt_delta > s_cfg.deadband_y)
    {
        /* 死区外: 更新云台目标 (增量累加) */
        s_gimbal_pan  += off.pan_delta;
        s_gimbal_tilt += off.tilt_delta;
    }
    /* 死区内: 不调整, 保持当前姿态 */

    /* ── 下发 Motion setpoint ── */
    Motion_SetTarget(s_gimbal_pan, s_gimbal_tilt);
}