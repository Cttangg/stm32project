/**
 ******************************************************************************
 * @file    target_source.c
 * @brief   目标数据源接口实现 — 注册表 + direct/vision 内置源
 *
 * direct 源 (Q1/Q2): 上层用 TargetSource_DirectSet() 写入绝对角度,
 *                    update 为空, get_target 返回该角度 → Motion 直闭环.
 *
 * vision 源 (Q3+):   enter 时 Communication_Init() 注册摄像头帧 + Tracking 使能,
 *                    update 消费 VisionTarget → Tracking → Motion.
 *                    未激活则完全不碰 UART, 摄像头帧保持未注册.
 ******************************************************************************
 */
#include "target_source.h"
#include "uart.h"
#include "communication.h"
#include "tracking_manager.h"

/* ========================================================================= */
/*  注册表                                                                     */
/* ========================================================================= */
static const TargetSource *s_sources[TARGET_SRC_ID_MAX];
static TargetSourceId      s_active = TARGET_SRC_ID_MAX;   /* 无激活 */

void TargetSource_Register(const TargetSource *src)
{
    if (src == NULL || src->id >= TARGET_SRC_ID_MAX) return;
    s_sources[src->id] = src;
}

const TargetSource *TargetSource_Find(TargetSourceId id)
{
    if (id >= TARGET_SRC_ID_MAX) return NULL;
    return s_sources[id];
}

bool TargetSource_Activate(TargetSourceId id)
{
    const TargetSource *src = TargetSource_Find(id);
    const TargetSource *old;

    if (src == NULL) return false;
    if (s_active == id) return true;

    old = (s_active < TARGET_SRC_ID_MAX) ? s_sources[s_active] : NULL;
    if (old && old->exit) old->exit();

    s_active = id;
    if (src->enter) src->enter();
    return true;
}

TargetSourceId TargetSource_GetActiveId(void)
{
    return s_active;
}

const TargetSource *TargetSource_GetActive(void)
{
    return (s_active < TARGET_SRC_ID_MAX) ? s_sources[s_active] : NULL;
}

/* ========================================================================= */
/*  direct 源 (Q1/Q2: 电机角度直接闭环)                                       */
/* ========================================================================= */
static GimbalSetpoint s_direct_target;
static bool           s_direct_valid;

static bool direct_get(GimbalSetpoint *out)
{
    if (out == NULL || !s_direct_valid) return false;
    *out = s_direct_target;
    return true;
}

static const TargetSource SRC_DIRECT = {
    TARGET_SRC_DIRECT, "direct",
    NULL,          /* enter */
    NULL,          /* exit  */
    NULL,          /* update */
    direct_get
};

void TargetSource_DirectSet(float pan_deg, float tilt_deg)
{
    s_direct_target.pan  = pan_deg;
    s_direct_target.tilt = tilt_deg;
    s_direct_valid = true;
}

/* ========================================================================= */
/*  vision 源 (Q3+: 视觉像素 → 外环 → 角度)                                   */
/* ========================================================================= */
static struct UART_Device *s_cam;
static bool                s_vision_on;

static void vision_enter(void)
{
    if (s_cam == NULL) return;
    Communication_Init(s_cam);        /* 至此才注册摄像头帧 (Q1/Q2 不注册) */
    Tracking_SetEnabled(true);
    s_vision_on = true;
}

static void vision_exit(void)
{
    Tracking_SetEnabled(false);
    /* 底层 UART 帧无注销接口: 用 s_vision_on 停止消费即可 */
    s_vision_on = false;
}

static void vision_update(uint32_t now_ms)
{
    VisionTarget vt;

    if (!s_vision_on) return;

    Communication_Update(now_ms);
    if (Communication_PopVisionTarget(&vt)) {
        Tracking_SetVisionTarget(&vt);
    }
    Tracking_Update(now_ms);
}

static bool vision_get(GimbalSetpoint *out)
{
    if (out == NULL || !s_vision_on) return false;
    *out = Motion_GetSetpoint();      /* Tracking 内部已写入 Motion */
    return true;
}

static const TargetSource SRC_VISION = {
    TARGET_SRC_VISION, "vision",
    vision_enter,
    vision_exit,
    vision_update,
    vision_get
};

void TargetSource_VisionBind(struct UART_Device *cam)
{
    s_cam = cam;
}

/* ========================================================================= */
/*  注册内置源 (不激活)                                                        */
/* ========================================================================= */
void TargetSource_Init(void)
{
    TargetSource_Register(&SRC_DIRECT);
    TargetSource_Register(&SRC_VISION);
}
