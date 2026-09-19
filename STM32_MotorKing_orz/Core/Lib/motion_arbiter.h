/**
 ******************************************************************************
 * @file    motion_arbiter.h
 * @brief   最终运动仲裁库 — 多请求源按优先级仲裁为唯一最终运动 (L3 仲裁层)
 *
 * 分层定位:
 *   L3 仲裁层, 位于运动层 (motion) 之上、应用层 (按键/串口/视觉) 之下.
 *   每个子系统把自己的「运动意图」提交到各自的请求槽 (slot), 仲裁器每周期:
 *
 *     ① 暂停/急停 → 立即覆盖一切, 制动 (Motion_Stop)
 *     ② 否则选优先级最高的活动请求, 下发给运动层 (必要时抢占切换)
 *     ③ 有限运动 (GOTO / PATH) 完成后自动释放该请求槽
 *
 *   这样应用层只负责「提交意图」, 无需关心当前谁在动、如何切换、如何停.
 *
 * 请求源与优先级 (枚举值越大优先级越高):
 *   CALIB > RESET > A4 > BORDER > TRACK > MANUAL
 *
 * 优先级用途: 同时存在多个活动请求时决定谁生效.
 *   (摄像机修正属于 motion 层内部, 不占用请求槽.)
 *
 * 模式切换语义:
 *   Submit(*, preempt = 1) 会清除所有优先级不高于自己的请求槽, 实现"新按键抢占旧模式";
 *   如需保留其它源, 传 preempt = 0.
 *   典型的独占模式按键处理: CancelAll() 后再 Submit().
 *
 * 典型用法 (main.c USER CODE):
 *   static Motion motion;
 *   static MotionArbiter arbiter;
 *
 *   Motion_Init(&motion, &pan_axis, &tilt_axis);
 *   MotionArbiter_Init(&arbiter, &motion);
 *
 *   // 按键: 复位
 *   MotionArbiter_CancelAll(&arbiter);
 *   MotionArbiter_Goto(&arbiter, MOTION_SRC_RESET, 0.0f, 0.0f, 8.0f, 1);
 *
 *   // 按键: 沿屏幕边线 (顺时针, 4 顶点, 1 圈)
 *   static const Motion_PointXY border[4] = {
 *       {-25, 25}, {25, 25}, {25, -25}, {-25, -25}
 *   };
 *   MotionArbiter_CancelAll(&arbiter);
 *   MotionArbiter_FollowPath(&arbiter, MOTION_SRC_BORDER,
 *                            border, 4, 1, 1, 10.0f, 1);
 *
 *   while (1) {
 *       MotionArbiter_Task(&arbiter, 0.005f);   // 5ms
 *       // 收到摄像头光斑坐标: Motion_SetSpotXY(&motion, x, y);
 *   }
 ******************************************************************************
 */
#ifndef __MOTION_ARBITER_H
#define __MOTION_ARBITER_H

#include "motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  请求源 (枚举顺序 = 优先级升序, 不可随意调整顺序)                            */
/* ========================================================================= */
typedef enum {
    MOTION_SRC_NONE = -1,    /* 无请求 (哨兵, 不占槽) */
    MOTION_SRC_MANUAL = 0,   /* 手动/串口调试 (最低) */
    MOTION_SRC_TRACK,        /* 一键追踪 (绿系统) */
    MOTION_SRC_A4,           /* 沿 A4 靶纸 */
    MOTION_SRC_BORDER,       /* 沿屏幕四周边线 */
    MOTION_SRC_RESET,        /* 复位到原点 */
    MOTION_SRC_CALIB,        /* 标定 (最高) */
    MOTION_SRC_COUNT
} Motion_Source;

/* ========================================================================= */
/*  请求命令                                                                   */
/* ========================================================================= */
typedef enum {
    MCMD_NONE = 0,
    MCMD_GOTO_XY,       /* 屏幕坐标点到点 */
    MCMD_PATH_XY,       /* 屏幕坐标折线/多边形 (A4) */
    MCMD_GOTO_ANGLE,    /* 角度空间点到点 (RESET: 用 CALIB 预设轴角) */
    MCMD_PATH_ANGLE,    /* 角度空间折线/多边形 (BORDER: 用 CALIB 角点) */
    MCMD_TRACK_XY,      /* 持续跟踪目标点 */
    MCMD_STOP,          /* 立即制动 */
} Motion_ArbCmdType;

typedef struct {
    Motion_ArbCmdType type;

    /* 屏幕坐标 (XY) */
    float    x, y;              /* GOTO_XY / TRACK_XY 目标 (cm) */
    float    speed;             /* GOTO_XY / PATH_XY 速度 (cm/s) */
    const Motion_PointXY *pts;  /* PATH_XY 顶点 (派发前需保持有效) */
    uint16_t npts;              /* PATH_XY 顶点数 */
    uint8_t  closed;            /* 路径是否闭合 */
    uint32_t loops;             /* 圈数 (0 = 无限) */

    /* 角度空间 (ANGLE) */
    float    pan, tilt;         /* GOTO_ANGLE 目标轴角 (deg) */
    float    rate;              /* 速率 deg/s */
    const Motion_PointAngle *apts;  /* PATH_ANGLE 顶点 (派发前需保持有效) */
    uint16_t anpts;             /* PATH_ANGLE 顶点数 */
} Motion_ArbCmd;

/* ========================================================================= */
/*  仲裁器                                                                     */
/* ========================================================================= */
typedef struct {
    Motion_ArbCmd  cmd;
    uint8_t        active;      /* 请求有效 */
    uint8_t        running;     /* 已下发给运动层 */
} Motion_ArbSlot;

typedef struct {
    Motion       *motion;                       /* 绑定的运动层实例 */
    Motion_ArbSlot slots[MOTION_SRC_COUNT];     /* 每个源一个槽 */
    Motion_Source  winner;                      /* 当前生效源 */
    uint8_t        paused;                      /* 暂停 (闩锁, 恢复后继续原请求) */
    uint8_t        estop;                       /* 急停 (闩锁, 需手动清除) */
} MotionArbiter;

/* ========================================================================= */
/*  公开 API                                                                   */
/* ========================================================================= */

/** @brief 绑定运动库实例并清零 */
void MotionArbiter_Init(MotionArbiter *a, Motion *motion);

/**
 * @brief 提交/覆盖某源的运动请求
 * @param preempt 1 = 同时清除所有优先级 <= 本源的其它请求 (模式抢占)
 *                0 = 仅更新本槽, 保留其它请求
 */
void MotionArbiter_Submit(MotionArbiter *a, Motion_Source src,
                          const Motion_ArbCmd *cmd, uint8_t preempt);

/* ── 便捷提交 (内部构造 Motion_ArbCmd) ── */
void MotionArbiter_Goto(MotionArbiter *a, Motion_Source src,
                        float x, float y, float speed, uint8_t preempt);
void MotionArbiter_FollowPath(MotionArbiter *a, Motion_Source src,
                              const Motion_PointXY *pts, uint16_t npts,
                              uint8_t closed, uint32_t loops, float speed,
                              uint8_t preempt);
void MotionArbiter_GotoAngle(MotionArbiter *a, Motion_Source src,
                             float pan, float tilt, float rate, uint8_t preempt);
void MotionArbiter_FollowAnglePath(MotionArbiter *a, Motion_Source src,
                                   const Motion_PointAngle *pts, uint16_t npts,
                                   uint8_t closed, uint32_t loops, float rate,
                                   uint8_t preempt);
void MotionArbiter_Track(MotionArbiter *a, Motion_Source src,
                         float x, float y, uint8_t preempt);
void MotionArbiter_StopCmd(MotionArbiter *a, Motion_Source src, uint8_t preempt);

/** @brief 取消某源的请求 */
void MotionArbiter_Cancel(MotionArbiter *a, Motion_Source src);

/** @brief 取消所有请求 (模式按键切换前调用) */
void MotionArbiter_CancelAll(MotionArbiter *a);

/* ── 暂停 / 急停 (最高优先, 总覆盖) ── */
void    MotionArbiter_Pause(MotionArbiter *a, uint8_t on);
void    MotionArbiter_TogglePause(MotionArbiter *a);
uint8_t MotionArbiter_IsPaused(const MotionArbiter *a);
void    MotionArbiter_EStop(MotionArbiter *a);
void    MotionArbiter_ClearEStop(MotionArbiter *a);

/** @brief 控制周期调用: 仲裁 + 下发 + 推进运动层 */
void MotionArbiter_Task(MotionArbiter *a, float dt_s);

/** @brief 当前生效的请求源 */
Motion_Source MotionArbiter_GetWinner(const MotionArbiter *a);

/** @brief 是否空闲 (无活动请求且未暂停/急停) */
uint8_t MotionArbiter_IsIdle(const MotionArbiter *a);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_ARBITER_H */
