/**
 ******************************************************************************
 * @file    app.c
 * @brief   应用状态机实现
 *
 * 状态转移:
 *   INIT → IDLE
 *   IDLE --start--> SEARCH
 *   SEARCH --target--> TRACK
 *   TRACK --lost--> LOST
 *   LOST --timeout--> SEARCH
 *   any --safety fault--> FAULT
 *   FAULT --safety clear--> IDLE
 ******************************************************************************
 */
#include "app.h"

/* ========================================================================= */
/*  内部状态                                                                    */
/* ========================================================================= */
static AppConfig  s_cfg;
static AppState   s_state;
static uint32_t   s_lost_since_ms;
static bool       s_cmd_start;

static void set_state(AppState st)
{
    s_state = st;
}

void App_Init(const AppConfig *cfg)
{
    s_cfg   = *cfg;
    s_state = APP_INIT;
    s_cmd_start = false;
    s_lost_since_ms = 0;
}

AppState App_GetState(void)
{
    return s_state;
}

void App_RequestStart(void)
{
    s_cmd_start = true;
}

void App_RequestStop(void)
{
    s_cmd_start = false;
    Tracking_SetEnabled(false);
    Motion_Stop();
}

void App_Update(uint32_t now_ms)
{
    bool motion_ok, tracking_ok;
    TrackingState ts;
    SafetyLevel sl;

    /* ── Safety 具有最高优先级: 任何故障 → FAULT ── */
    sl = (s_cfg.safety) ? Safety_GetLevel() : SAFETY_OK;
    if (sl == SAFETY_FAULT || sl == SAFETY_STOP) {
        if (s_state != APP_FAULT) {
            Tracking_SetEnabled(false);
            Motion_Stop();
        }
        set_state(APP_FAULT);
        return;
    }

    if (s_state == APP_INIT) {
        set_state(APP_IDLE);
        return;
    }

    if (s_state == APP_FAULT) {
        /* 需要 Safety_ClearFault 后才能回到 IDLE */
        if (sl == SAFETY_OK) {
            set_state(APP_IDLE);
        }
        return;
    }

    motion_ok   = (s_cfg.motion)  ? true : false;
    tracking_ok = (s_cfg.safety)  ? Safety_IsTrackingAllowed() : true;

    if (s_state == APP_IDLE) {
        if (s_cmd_start && motion_ok && tracking_ok) {
            s_cmd_start = false;
            Motion_Stop();                    /* 从当前位置开始 */
            Tracking_SetEnabled(true);
            set_state(APP_SEARCH);
        }
        return;
    }

    /* 追踪相关状态 */
    ts = Tracking_GetState();

    if (s_state == APP_SEARCH) {
        if (ts == TRACKING_ACQUIRE || ts == TRACKING_TRACK) {
            set_state(APP_TRACK);
        } else if (ts == TRACKING_LOST) {
            s_lost_since_ms = now_ms;
            set_state(APP_LOST);
        }
        return;
    }

    if (s_state == APP_TRACK) {
        if (ts == TRACKING_LOST) {
            s_lost_since_ms = now_ms;
            set_state(APP_LOST);
        }
        return;
    }

    if (s_state == APP_LOST) {
        if (ts == TRACKING_ACQUIRE || ts == TRACKING_TRACK) {
            set_state(APP_TRACK);
        } else if ((now_ms - s_lost_since_ms) > 3000) {
            /* 超过 3s 未恢复 → 回到 SEARCH 主动搜索 */
            set_state(APP_SEARCH);
        }
        return;
    }
}