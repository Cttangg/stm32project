/**
 ******************************************************************************
 * @file    motion.c
 * @brief   运动库实现 (L2 运动层) — 轨迹发生器 + 仿射运动学 + 视觉外环
 ******************************************************************************
 */
#include "motion.h"
#include <math.h>

/* ========================================================================= */
/*  内部辅助                                                                    */
/* ========================================================================= */
static void motion_axis_hold(Motion *m);
static void motion_axis_start(Motion *m);

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float wrap180f(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

/* 前馈: 屏幕坐标 → 轴角 (deg) */
static void kin_forward(const Motion_Calib *c, float x, float y,
                        float *pan, float *tilt) {
    *pan  = c->k[0][0] * x + c->k[0][1] * y + c->b[0];
    *tilt = c->k[1][0] * x + c->k[1][1] * y + c->b[1];
}

/* 逆运动学: 轴角增量 (相对 b, 已卷绕) → 屏幕坐标 (cm) */
static void kin_inverse(const Motion_Calib *c, float dpan, float dtilt,
                        float *x, float *y) {
    float det = c->k[0][0] * c->k[1][1] - c->k[0][1] * c->k[1][0];
    if (fabsf(det) < 1e-9f) { *x = 0.0f; *y = 0.0f; return; }
    *x = ( c->k[1][1] * dpan - c->k[0][1] * dtilt) / det;
    *y = (-c->k[1][0] * dpan + c->k[0][0] * dtilt) / det;
}

/* 光斑反馈是否新鲜 */
static uint8_t spot_fresh(const Motion *m) {
    return (m->has_spot &&
            (uint32_t)(HAL_GetTick() - m->spot_tick) <= MOTION_SPOT_TIMEOUT_MS) ? 1U : 0U;
}

/* 估计当前光斑位置: 优先视觉反馈, 否则由编码器角度逆解 */
static void motion_current_xy(Motion *m, float *x, float *y) {
    if (spot_fresh(m)) { *x = m->spot_x; *y = m->spot_y; return; }
    {
        float dp = wrap180f(MotorAxis_GetDeg(m->pan)  - m->calib.b[0]);
        float dt = wrap180f(MotorAxis_GetDeg(m->tilt) - m->calib.b[1]);
        kin_inverse(&m->calib, dp, dt, x, y);
    }
}

/* 沿直线向 (tx,ty) 推进, 消耗 *budget (cm); 返回 1 = 已到达目标 */
static uint8_t step_toward(Motion *m, float *budget, float tx, float ty) {
    float dx = tx - m->sp_x;
    float dy = ty - m->sp_y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist <= 1e-4f) { m->sp_x = tx; m->sp_y = ty; return 1U; }
    if (*budget >= dist) {
        m->sp_x = tx; m->sp_y = ty;
        *budget -= dist;
        return 1U;
    }
    if (*budget <= 0.0f) return 0U;
    {
        float k = (*budget) / dist;
        m->sp_x += dx * k;
        m->sp_y += dy * k;
        *budget = 0.0f;
    }
    return 0U;
}

/* 启动原语前的公共准备 (XY 模式): 使能 + 启动轴 + 设定点初始化到当前位置 */
static void motion_begin(Motion *m) {
    if (!m->enabled) m->enabled = 1;
    motion_axis_start(m);
    m->mode = MOTION_MODE_XY;
    motion_current_xy(m, &m->sp_x, &m->sp_y);

    /* 保护: 标定退化(k 接近奇异)时逆解会给出巨大/非有限坐标, 会把轨迹拉成
       "永远走不完" → 电机一直转. 这里夹到合理范围 (屏幕最大 ~1m). */
    if (!isfinite(m->sp_x) || fabsf(m->sp_x) > MOTION_XY_LIMIT) m->sp_x = 0.0f;
    if (!isfinite(m->sp_y) || fabsf(m->sp_y) > MOTION_XY_LIMIT) m->sp_y = 0.0f;

    m->move_tick = HAL_GetTick();        /* 有限运动超时保护起点 */
}

/* 启动原语前的公共准备 (ANGLE 模式): 设定点 = 当前轴角 (不经屏幕坐标/逆解) */
static void motion_begin_angle(Motion *m) {
    if (!m->enabled) m->enabled = 1;
    motion_axis_start(m);
    m->mode = MOTION_MODE_ANGLE;
    m->sp_x = MotorAxis_GetDeg(m->pan);   /* 轴角 (deg) */
    m->sp_y = MotorAxis_GetDeg(m->tilt);
    m->move_tick = HAL_GetTick();
}

/* ========================================================================= */
/*  初始化 / 标定                                                              */
/* ========================================================================= */
void Motion_Init(Motion *m, MotorAxis *pan, MotorAxis *tilt) {
    m->pan  = pan;
    m->tilt = tilt;

    Motion_DefaultCalib(&m->calib, 100.0f);

    m->enabled     = 0;
    m->axes_active = 0;
    m->mode        = MOTION_MODE_XY;
    m->state       = MOTION_STATE_IDLE;
    m->npts        = 0;
    m->seg         = 0;
    m->closed      = 0;
    m->approaching = 0;
    m->segs_left   = 0;
    m->speed       = MOTION_DEFAULT_SPEED;
    m->tgt_x = m->tgt_y = 0.0f;
    m->sp_x  = m->sp_y  = 0.0f;
    m->goto_tol = MOTION_DEFAULT_GOTO_TOL;
    m->move_tick = 0;
    m->spot_x = m->spot_y = 0.0f;
    m->spot_tick = 0;
    m->has_spot = 0;
    m->ff_pan = m->ff_tilt = 0.0f;
    m->corr_pan = m->corr_tilt = 0.0f;
}

void Motion_SetCalib(Motion *m, const Motion_Calib *calib) {
    if (calib) m->calib = *calib;
}

void Motion_DefaultCalib(Motion_Calib *calib, float dist_cm) {
    float g = (dist_cm > 1.0f) ? (57.2957795f / dist_cm) : 0.5729578f;

    calib->k[0][0] = g;  calib->k[0][1] = 0.0f;
    calib->k[1][0] = 0.0f; calib->k[1][1] = g;
    calib->b[0] = calib->b[1] = 0.0f;

    calib->corr[0][0] = 0.30f; calib->corr[0][1] = 0.0f;
    calib->corr[1][0] = 0.0f;  calib->corr[1][1] = 0.30f;
    calib->corr_max   = 5.0f;      /* deg */
    calib->vision_en  = 0;         /* 默认关闭, 接好摄像头再开 */
}

void Motion_Enable(Motion *m, uint8_t en) {
    m->enabled = en;
    if (en) {
        if (MOTION_HOLD_AT_IDLE) {
            /* 空闲: TMC 通电静态锁位, 但**不跑 PID** → 无脉冲 = 无抖动 */
            if (!m->pan->enabled)  MotorAxis_Enable(m->pan, 1);
            if (!m->tilt->enabled) MotorAxis_Enable(m->tilt, 1);
            motion_axis_hold(m);
        } else {
            Motion_Release(m);      /* 空闲释放, 完全静音 */
        }
        m->state = MOTION_STATE_IDLE;
    } else {
        Motion_Stop(m);
    }
}

void Motion_Release(Motion *m) {
    m->enabled = 0;                      /* 释放后禁止运动层再驱动 */
    m->state   = MOTION_STATE_IDLE;
    MotorAxis_Enable(m->pan, 0);         /* 停脉冲 + 关 PID + EN 拉高 (完全释放) */
    MotorAxis_Enable(m->tilt, 0);
    m->axes_active = 0;
}

/* 停止轴闭环并保持: 设定点/目标 = 当前位置, 消除"回到上次设定点"的跳变.
   只停脉冲/关 PID (TMC EN 仍低 → 静态锁位), 不改变 state (由调用者决定). */
static void motion_axis_hold(Motion *m) {
    if (m->mode == MOTION_MODE_ANGLE) {
        m->sp_x = MotorAxis_GetDeg(m->pan);    /* 轴角空间 */
        m->sp_y = MotorAxis_GetDeg(m->tilt);
    } else {
        MotorAxis_Refresh(m->pan);             /* 空闲时无 Update, 主动刷新角度 */
        MotorAxis_Refresh(m->tilt);
        motion_current_xy(m, &m->sp_x, &m->sp_y);
    }
    m->tgt_x = m->sp_x;
    m->tgt_y = m->sp_y;
    MotorAxis_Stop(m->pan);
    MotorAxis_Stop(m->tilt);
    m->axes_active = 0;
}

/* 启动轴闭环 (有运动任务时才跑 PID) */
static void motion_axis_start(Motion *m) {
    if (m->axes_active) return;
    MotorAxis_Enable(m->pan, 1);
    MotorAxis_Enable(m->tilt, 1);
    m->axes_active = 1;
}

void Motion_SetVisionEnable(Motion *m, uint8_t en) {
    m->calib.vision_en = en ? 1 : 0;
}

/* ========================================================================= */
/*  运动原语                                                                    */
/* ========================================================================= */
void Motion_GotoXY(Motion *m, float x, float y, float speed) {
    motion_begin(m);
    m->tgt_x = x;
    m->tgt_y = y;
    m->speed = (speed > 0.05f) ? speed : MOTION_DEFAULT_SPEED;
    m->state = MOTION_STATE_GOTO;
}

void Motion_FollowPathXY(Motion *m, const Motion_PointXY *pts, uint16_t npts,
                         uint8_t closed, uint32_t loops, float speed) {
    uint16_t i;

    if (!pts || npts < 2) return;
    if (npts > MOTION_MAX_PATH_PTS) npts = MOTION_MAX_PATH_PTS;

    motion_begin(m);

    for (i = 0; i < npts; i++) m->path[i] = pts[i];
    m->npts  = npts;
    m->closed = closed ? 1 : 0;
    m->speed  = (speed > 0.05f) ? speed : MOTION_DEFAULT_SPEED;
    m->approaching = 1;

    if (m->closed) {
        /* 闭合: 从离当前位置最近的顶点切入, 再按 pts 顺序行进 */
        uint16_t best = 0;
        float bestd = 1e30f;
        for (i = 0; i < npts; i++) {
            float dx = m->path[i].x - m->sp_x;
            float dy = m->path[i].y - m->sp_y;
            float d  = dx * dx + dy * dy;
            if (d < bestd) { bestd = d; best = i; }
        }
        m->seg       = best;
        m->segs_left = (loops == 0) ? MOTION_LOOPS_INF : (uint32_t)loops * npts;
    } else {
        m->seg       = 0;
        m->segs_left = (uint32_t)npts - 1u;
    }
    m->state = MOTION_STATE_PATH;
}

/* ── 角度空间原语: 直接以标定轴角为目标, 不经屏幕坐标/逆解 ── */

void Motion_GotoAngle(Motion *m, float pan_deg, float tilt_deg, float rate_dps) {
    motion_begin_angle(m);
    /* 目标取相对当前的最短路径 (跨越 0/360 接缝无跳变) */
    m->tgt_x = m->sp_x + wrap180f(pan_deg  - m->sp_x);
    m->tgt_y = m->sp_y + wrap180f(tilt_deg - m->sp_y);
    m->speed = (rate_dps > 0.1f) ? rate_dps : MOTION_DEFAULT_ANG_RATE;
    m->state = MOTION_STATE_GOTO;
}

void Motion_FollowAnglePath(Motion *m, const Motion_PointAngle *pts, uint16_t npts,
                            uint8_t closed, uint32_t loops, float rate_dps) {
    uint16_t i;

    if (!pts || npts < 2) return;
    if (npts > MOTION_MAX_PATH_PTS) npts = MOTION_MAX_PATH_PTS;

    motion_begin_angle(m);

    /* 顶点解卷绕到当前角附近, 保证相邻顶点插值走最短路径 */
    for (i = 0; i < npts; i++) {
        m->path[i].x = m->sp_x + wrap180f(pts[i].pan  - m->sp_x);
        m->path[i].y = m->sp_y + wrap180f(pts[i].tilt - m->sp_y);
    }
    m->npts  = npts;
    m->closed = closed ? 1 : 0;
    m->speed  = (rate_dps > 0.1f) ? rate_dps : MOTION_DEFAULT_ANG_RATE;
    m->approaching = 1;

    if (m->closed) {
        uint16_t best = 0;
        float bestd = 1e30f;
        for (i = 0; i < npts; i++) {
            float dx = m->path[i].x - m->sp_x;
            float dy = m->path[i].y - m->sp_y;
            float d  = dx * dx + dy * dy;
            if (d < bestd) { bestd = d; best = i; }
        }
        m->seg       = best;
        m->segs_left = (loops == 0) ? MOTION_LOOPS_INF : (uint32_t)loops * npts;
    } else {
        m->seg       = 0;
        m->segs_left = (uint32_t)npts - 1u;
    }
    m->state = MOTION_STATE_PATH;
}

void Motion_TrackXY(Motion *m, float x, float y) {
    if (!m->enabled) m->enabled = 1;
    motion_axis_start(m);
    m->mode  = MOTION_MODE_XY;
    m->tgt_x = x;
    m->tgt_y = y;
    m->state = MOTION_STATE_TRACK;
}

void Motion_Stop(Motion *m) {
    m->enabled = 0;
    m->state   = MOTION_STATE_IDLE;
    m->axes_active = 0;
    MotorAxis_Stop(m->pan);      /* 停脉冲 + 关 PID, 线圈仍锁定 */
    MotorAxis_Stop(m->tilt);
}

/* ========================================================================= */
/*  视觉反馈                                                                   */
/* ========================================================================= */
void Motion_SetSpotXY(Motion *m, float x, float y) {
    m->spot_x = x;
    m->spot_y = y;
    m->has_spot = 1;
    m->spot_tick = HAL_GetTick();
}

/* ========================================================================= */
/*  周期任务                                                                   */
/* ========================================================================= */
void Motion_Task(Motion *m, float dt_s) {
    float budget;
    float cp, ct;

    if (dt_s <= 0.0f || !m->enabled) return;
    if (m->state == MOTION_STATE_IDLE) return;   /* 空闲不驱动轴 (避免上电跑向 0°) */

    /* 有限运动超时保护: 轨迹异常 (标定退化/编码器异常) 时强制结束, 防电机一直转 */
    if ((m->state == MOTION_STATE_GOTO || m->state == MOTION_STATE_PATH) &&
        (uint32_t)(HAL_GetTick() - m->move_tick) > MOTION_MOVE_TIMEOUT_MS) {
        m->state = MOTION_STATE_DONE;
    }

    /* ── 轨迹发生器 ── */
    budget = m->speed * dt_s;

    switch (m->state) {
    case MOTION_STATE_GOTO:
        if (step_toward(m, &budget, m->tgt_x, m->tgt_y)) {
            if (m->mode == MOTION_MODE_ANGLE) {
                /* 角度空间: 设定点到位后, 等两轴闭环收敛再结束 (提高定位精度) */
                if (MotorAxis_IsSettled(m->pan) && MotorAxis_IsSettled(m->tilt))
                    m->state = MOTION_STATE_DONE;
            } else {
                m->state = MOTION_STATE_DONE;
            }
        }
        break;

    case MOTION_STATE_PATH: {
        int guard = 0;
        while (m->state == MOTION_STATE_PATH && guard++ < 64) {
            float vx = m->path[m->seg].x;
            float vy = m->path[m->seg].y;

            if (!step_toward(m, &budget, vx, vy)) break;   /* 未到顶点, 本周期结束 */

            if (m->approaching) {
                /* 到达切入顶点, 开始计段 */
                m->approaching = 0;
                m->seg = m->closed ? (uint16_t)((m->seg + 1u) % m->npts)
                                   : (uint16_t)(m->seg + 1u);
            } else {
                if (m->segs_left != MOTION_LOOPS_INF) {
                    if (m->segs_left > 0) m->segs_left--;
                    if (m->segs_left == 0) { m->state = MOTION_STATE_DONE; break; }
                }
                if (m->closed) {
                    m->seg = (uint16_t)((m->seg + 1u) % m->npts);
                } else {
                    if (m->seg + 1u >= m->npts) { m->state = MOTION_STATE_DONE; break; }
                    m->seg++;
                }
            }
            if (budget <= 0.0f) break;
        }
        break;
    }

    case MOTION_STATE_TRACK:
        m->sp_x = m->tgt_x;
        m->sp_y = m->tgt_y;
        break;

    case MOTION_STATE_DONE:
    case MOTION_STATE_IDLE:
    default:
        break;
    }

    /* ── ANGLE 模式: 设定点即轴目标角, 直接角度闭环 (无屏幕坐标/逆解/视觉) ── */
    if (m->mode == MOTION_MODE_ANGLE) {
        if (m->state == MOTION_STATE_DONE) {
            if (m->axes_active) motion_axis_hold(m);
            return;
        }
        MotorAxis_SetTargetDeg(m->pan,  m->sp_x);
        MotorAxis_SetTargetDeg(m->tilt, m->sp_y);
        MotorAxis_Update(m->pan,  dt_s);
        MotorAxis_Update(m->tilt, dt_s);
        return;
    }

    /* ── XY 模式: 运动学前馈 ── */
    kin_forward(&m->calib, m->sp_x, m->sp_y, &m->ff_pan, &m->ff_tilt);

    /* ── 视觉外环修正: 驱动光斑趋近设定点 ── */
    cp = 0.0f; ct = 0.0f;
    if (m->calib.vision_en && spot_fresh(m)) {
        float ex = m->sp_x - m->spot_x;
        float ey = m->sp_y - m->spot_y;
        cp = m->calib.corr[0][0] * ex + m->calib.corr[0][1] * ey;
        ct = m->calib.corr[1][0] * ex + m->calib.corr[1][1] * ey;
        cp = clampf(cp, -m->calib.corr_max, m->calib.corr_max);
        ct = clampf(ct, -m->calib.corr_max, m->calib.corr_max);
    }
    m->corr_pan  = cp;
    m->corr_tilt = ct;

    /* ── 运动完成 → 停轴闭环, 由 TMC 通电静态锁位 (不再高频调节).
       state 保持 DONE, 供仲裁层识别完成并释放请求槽. ── */
    if (m->state == MOTION_STATE_DONE) {
        if (m->axes_active) motion_axis_hold(m);
        return;
    }

    /* ── 下发轴目标 + 两轴闭环 ── */
    MotorAxis_SetTargetDeg(m->pan,  m->ff_pan  + cp);
    MotorAxis_SetTargetDeg(m->tilt, m->ff_tilt + ct);
    MotorAxis_Update(m->pan,  dt_s);
    MotorAxis_Update(m->tilt, dt_s);
}

/* ========================================================================= */
/*  状态查询                                                                   */
/* ========================================================================= */
MotionState Motion_GetState(const Motion *m) {
    return m->state;
}

uint8_t Motion_IsDone(const Motion *m) {
    return (m->state == MOTION_STATE_DONE) ? 1U : 0U;
}

void Motion_GetSetpointXY(const Motion *m, float *x, float *y) {
    if (x) *x = m->sp_x;
    if (y) *y = m->sp_y;
}

uint8_t Motion_GetSpotXY(const Motion *m, float *x, float *y) {
    if (!spot_fresh(m)) return 0U;
    if (x) *x = m->spot_x;
    if (y) *y = m->spot_y;
    return 1U;
}
