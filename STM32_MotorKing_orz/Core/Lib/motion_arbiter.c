/**
 ******************************************************************************
 * @file    motion_arbiter.c
 * @brief   最终运动仲裁库实现 (L3 仲裁层)
 ******************************************************************************
 */
#include "motion_arbiter.h"

/* 优先级 = 枚举值 (升序); 集中在此便于调整 */
static uint8_t arb_prio(int src) {
    return (uint8_t)src;
}

/* 把某源的命令下发给运动层 */
static void arb_dispatch(MotionArbiter *a, int src) {
    const Motion_ArbCmd *c = &a->slots[src].cmd;

    switch (c->type) {
    case MCMD_GOTO_XY:
        Motion_GotoXY(a->motion, c->x, c->y, c->speed);
        break;
    case MCMD_PATH_XY:
        Motion_FollowPathXY(a->motion, c->pts, c->npts,
                            c->closed, c->loops, c->speed);
        break;
    case MCMD_GOTO_ANGLE:
        Motion_GotoAngle(a->motion, c->pan, c->tilt, c->rate);
        break;
    case MCMD_PATH_ANGLE:
        Motion_FollowAnglePath(a->motion, c->apts, c->anpts,
                               c->closed, c->loops, c->rate);
        break;
    case MCMD_TRACK_XY:
        Motion_TrackXY(a->motion, c->x, c->y);
        break;
    case MCMD_STOP:
        Motion_Stop(a->motion);
        break;
    case MCMD_NONE:
    default:
        break;
    }
}

/* ========================================================================= */
/*  初始化                                                                     */
/* ========================================================================= */
void MotionArbiter_Init(MotionArbiter *a, Motion *motion) {
    int i;
    a->motion = motion;
    a->winner = MOTION_SRC_NONE;
    a->paused = 0;
    a->estop  = 0;
    for (i = 0; i < MOTION_SRC_COUNT; i++) {
        a->slots[i].cmd.type = MCMD_NONE;
        a->slots[i].active   = 0;
        a->slots[i].running  = 0;
    }
}

/* ========================================================================= */
/*  请求提交 / 取消                                                            */
/* ========================================================================= */
void MotionArbiter_Submit(MotionArbiter *a, Motion_Source src,
                          const Motion_ArbCmd *cmd, uint8_t preempt) {
    int i;

    if (!a || !cmd || src >= MOTION_SRC_COUNT) return;

    if (preempt) {
        for (i = 0; i < MOTION_SRC_COUNT; i++) {
            if (i == (int)src) continue;
            if (arb_prio(i) <= arb_prio((int)src)) {
                a->slots[i].active  = 0;
                a->slots[i].running = 0;
            }
        }
        if (a->winner != MOTION_SRC_NONE && a->winner != src &&
            arb_prio((int)a->winner) <= arb_prio((int)src)) {
            a->winner = MOTION_SRC_NONE;
        }
    }

    a->slots[src].cmd     = *cmd;
    a->slots[src].active  = 1;
    a->slots[src].running = 0;   /* 强制重新下发 */
}

void MotionArbiter_Goto(MotionArbiter *a, Motion_Source src,
                        float x, float y, float speed, uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type = MCMD_GOTO_XY;
    c.x = x; c.y = y; c.speed = speed;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_FollowPath(MotionArbiter *a, Motion_Source src,
                              const Motion_PointXY *pts, uint16_t npts,
                              uint8_t closed, uint32_t loops, float speed,
                              uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type   = MCMD_PATH_XY;
    c.pts    = pts;
    c.npts   = npts;
    c.closed = closed;
    c.loops  = loops;
    c.speed  = speed;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_GotoAngle(MotionArbiter *a, Motion_Source src,
                             float pan, float tilt, float rate, uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type = MCMD_GOTO_ANGLE;
    c.pan = pan; c.tilt = tilt; c.rate = rate;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_FollowAnglePath(MotionArbiter *a, Motion_Source src,
                                   const Motion_PointAngle *pts, uint16_t npts,
                                   uint8_t closed, uint32_t loops, float rate,
                                   uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type   = MCMD_PATH_ANGLE;
    c.apts   = pts;
    c.anpts  = npts;
    c.closed = closed;
    c.loops  = loops;
    c.rate   = rate;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_Track(MotionArbiter *a, Motion_Source src,
                         float x, float y, uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type = MCMD_TRACK_XY;
    c.x = x; c.y = y;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_StopCmd(MotionArbiter *a, Motion_Source src, uint8_t preempt) {
    Motion_ArbCmd c = {0};
    c.type = MCMD_STOP;
    MotionArbiter_Submit(a, src, &c, preempt);
}

void MotionArbiter_Cancel(MotionArbiter *a, Motion_Source src) {
    if (!a || src >= MOTION_SRC_COUNT) return;
    a->slots[src].active  = 0;
    a->slots[src].running = 0;
    if (a->winner == src) a->winner = MOTION_SRC_NONE;
}

void MotionArbiter_CancelAll(MotionArbiter *a) {
    int i;
    if (!a) return;
    for (i = 0; i < MOTION_SRC_COUNT; i++) {
        a->slots[i].active  = 0;
        a->slots[i].running = 0;
    }
    a->winner = MOTION_SRC_NONE;
}

/* ========================================================================= */
/*  暂停 / 急停                                                                */
/* ========================================================================= */
void MotionArbiter_Pause(MotionArbiter *a, uint8_t on) {
    if (a) a->paused = on ? 1 : 0;
}

void MotionArbiter_TogglePause(MotionArbiter *a) {
    if (a) a->paused = a->paused ? 0 : 1;
}

uint8_t MotionArbiter_IsPaused(const MotionArbiter *a) {
    return (a && a->paused) ? 1U : 0U;
}

void MotionArbiter_EStop(MotionArbiter *a) {
    if (a) a->estop = 1;
}

void MotionArbiter_ClearEStop(MotionArbiter *a) {
    if (a) a->estop = 0;
}

/* ========================================================================= */
/*  周期任务                                                                   */
/* ========================================================================= */
void MotionArbiter_Task(MotionArbiter *a, float dt_s) {
    int best, i;
    const Motion_ArbCmd *cmd;

    if (!a || !a->motion || dt_s <= 0.0f) return;

    /* ── ① 暂停/急停: 总覆盖, 立即制动 ── */
    if (a->estop || a->paused) {
        if (a->winner != MOTION_SRC_NONE) {
            Motion_Stop(a->motion);
            a->winner = MOTION_SRC_NONE;
        }
        for (i = 0; i < MOTION_SRC_COUNT; i++) a->slots[i].running = 0;
        return;
    }

    /* ── ② 选优先级最高的活动请求 ── */
    best = -1;
    for (i = 0; i < MOTION_SRC_COUNT; i++) {
        if (a->slots[i].active && a->slots[i].cmd.type != MCMD_NONE) {
            if (best < 0 || arb_prio(i) > arb_prio(best)) best = i;
        }
    }

    if (best < 0) {
        /* 无请求: 连续跟踪则制动, 有限运动完成则进入保持 */
        if (Motion_GetState(a->motion) == MOTION_STATE_TRACK)
            Motion_Stop(a->motion);
        a->winner = MOTION_SRC_NONE;
        Motion_Task(a->motion, dt_s);
        return;
    }

    /* ── ③ 切换/首次执行 → 下发 ── */
    if (a->winner != (Motion_Source)best || !a->slots[best].running) {
        arb_dispatch(a, best);
        a->winner = (Motion_Source)best;
        a->slots[best].running = 1;
    }

    /* STOP 为瞬时命令, 派发后立即释放 */
    if (a->slots[best].cmd.type == MCMD_STOP) {
        a->slots[best].active  = 0;
        a->slots[best].running = 0;
        a->winner = MOTION_SRC_NONE;
        return;
    }

    Motion_Task(a->motion, dt_s);

    /* ── ④ 有限运动完成 → 自动释放请求槽 ── */
    cmd = &a->slots[best].cmd;
    if ((cmd->type == MCMD_GOTO_XY  || cmd->type == MCMD_PATH_XY ||
         cmd->type == MCMD_GOTO_ANGLE || cmd->type == MCMD_PATH_ANGLE) &&
        Motion_IsDone(a->motion)) {
        a->slots[best].active  = 0;
        a->slots[best].running = 0;
        a->winner = MOTION_SRC_NONE;
    }
}

Motion_Source MotionArbiter_GetWinner(const MotionArbiter *a) {
    return a ? a->winner : MOTION_SRC_NONE;
}

uint8_t MotionArbiter_IsIdle(const MotionArbiter *a) {
    int i;
    if (!a || a->paused || a->estop) return 0;
    for (i = 0; i < MOTION_SRC_COUNT; i++) {
        if (a->slots[i].active && a->slots[i].cmd.type != MCMD_NONE) return 0;
    }
    return 1;
}
