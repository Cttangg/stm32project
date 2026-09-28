/**
 ******************************************************************************
 * @file    telemetry.c
 * @brief   遥测实现 — 汇总各模块只读状态
 ******************************************************************************
 */
#include "telemetry.h"

static const Axis *s_pan;
static const Axis *s_tilt;
static Telemetry   s_t;

void Telemetry_Init(const Axis *pan, const Axis *tilt)
{
    s_pan  = pan;
    s_tilt = tilt;
}

void Telemetry_Sample(uint32_t now_ms)
{
    const VisionState *vs = Communication_GetVisionState();

    if (s_pan) {
        s_t.pan_position = s_pan->position;
        s_t.pan_target   = s_pan->target_position;
        s_t.pan_velocity = s_pan->velocity;
    }
    if (s_tilt) {
        s_t.tilt_position = s_tilt->position;
        s_t.tilt_target   = s_tilt->target_position;
        s_t.tilt_velocity = s_tilt->velocity;
    }

    s_t.app_state      = (uint8_t)App_GetState();
    s_t.tracking_state = (uint8_t)Tracking_GetState();
    s_t.safety_level   = (uint8_t)Safety_GetLevel();

    s_t.fault_flags = 0;
    if (s_pan  && Axis_IsFault(s_pan))  s_t.fault_flags |= 0x01;
    if (s_tilt && Axis_IsFault(s_tilt)) s_t.fault_flags |= 0x02;

    s_t.vision_age = (vs && vs->last_valid_time) ? (now_ms - vs->last_valid_time) : 0xFFFFFFFFu;
}

const Telemetry *Telemetry_Get(void)
{
    return &s_t;
}