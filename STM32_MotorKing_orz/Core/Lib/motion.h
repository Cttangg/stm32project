/**
 ******************************************************************************
 * @file    motion.h
 * @brief   运动库 — 二维云台运动原语 + 屏幕坐标运动学 + 视觉外环 (L2 运动层)
 *
 * 分层定位:
 *   L2 运动层, 位于轴控层 (motor_axis) 之上, 仲裁层 (motion_arbiter) 之下.
 *   对上层只暴露「屏幕坐标 (cm)」, 内部完成:
 *
 *     屏幕坐标 (x,y) cm ──仿射运动学──► pan/tilt 目标角 (deg) ──MotorAxis──► 电机
 *              ▲                        ▲
 *              │ 视觉外环 (光斑反馈)     │ 编码器角度反馈
 *              └── Motion_SetSpotXY()   └── MotorAxis_Update()
 *
 * 坐标定义 (与题目屏幕一致):
 *   原点 = 屏幕中心 (铅笔画的 0.5m 正方形中心)
 *   +x   = 向右,  +y = 向上,  单位 cm
 *   正方形边线: x/y = ±25cm;  A4 靶纸: 210×297mm 长方形, 任意位置/旋转角.
 *
 * 运动学标定 (仿射模型, 吸收齿比/安装/屏幕倾斜):
 *   [pan_deg ]   [k00 k01] [x]   [b0]
 *   [tilt_deg] = [k10 k11] [y] + [b1]
 *   默认按「云台距屏 dist_cm, 光束角≈轴角」给出 k = 57.2958/dist, b = 0, 需实测标定.
 *
 * 视觉外环 (摄像头闭环修正):
 *   前馈角 = 运动学(setpoint_xy);  修正角 = corr · (setpoint_xy - spot_xy)
 *   轴目标 = 前馈角 + 修正角  → 抵消运动学标定残差, 提高落点精度.
 *   通过 Motion_SetSpotXY() 喂入摄像头观测到的光斑坐标, 超时未更新则自动停用.
 *
 * 运动原语 (全部非阻塞, 由 Motion_Task 周期推进):
 *   Motion_GotoXY     点到点绝对定位 (题目: 复位到原点)
 *   Motion_FollowPathXY 折线/多边形路径跟随 (题目: 屏幕边线 / A4 靶纸, 支持任意旋转)
 *   Motion_TrackXY    持续跟踪一个移动目标点 (绿系统追踪红点)
 *   Motion_Stop       立即制动 (暂停键)
 *
 * 典型用法:
 *   Motion motion;
 *   Motion_Init(&motion, &pan_axis, &tilt_axis);
 *   Motion_DefaultCalib(&motion.calib, 100.0f);   // 云台距屏 100cm
 *   Motion_GotoXY(&motion, 0.0f, 0.0f, 8.0f);    // 回原点
 *   while (1) Motion_Task(&motion, 0.005f);       // 5ms
 ******************************************************************************
 */
#ifndef __MOTION_H
#define __MOTION_H

#include "motor_axis.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  配置                                                                       */
/* ========================================================================= */
#define MOTION_MAX_PATH_PTS     16          /* 单条路径最多顶点数 (矩形=4) */
#define MOTION_LOOPS_INF        0xFFFFFFFFu /* loops=0 时内部表示: 无限循环 */
#define MOTION_DEFAULT_SPEED    8.0f        /* 默认速度 cm/s */
#define MOTION_DEFAULT_GOTO_TOL 0.5f        /* 默认到位容差 cm */
#define MOTION_SPOT_TIMEOUT_MS  200u        /* 光斑反馈超时 (ms) */
#define MOTION_XY_LIMIT         200.0f      /* 起始坐标夹取上限 cm (防标定退化时逆解爆炸) */
#define MOTION_MOVE_TIMEOUT_MS  20000u      /* 有限运动总时长上限 ms (超时强制停止) */
#define MOTION_DEFAULT_ANG_RATE 20.0f       /* 角度空间默认速率 deg/s */

/* 空闲保持策略: 1 = TMC 通电锁位 (光斑不跑, 电机发热);
   0 = 空闲释放力矩 (完全静音免发热, 但光斑可被外力推动) */
#ifndef MOTION_HOLD_AT_IDLE
#define MOTION_HOLD_AT_IDLE     1
#endif

/* ========================================================================= */
/*  类型                                                                       */
/* ========================================================================= */
typedef struct { float x, y; } Motion_PointXY;        /* 屏幕坐标 (cm) */
typedef struct { float pan, tilt; } Motion_PointAngle; /* 轴角 (deg) */

/** @brief 运动坐标系模式 */
typedef enum {
    MOTION_MODE_XY = 0,     /* 屏幕坐标 cm → 仿射运动学 → 轴角 (视觉 / A4) */
    MOTION_MODE_ANGLE,      /* 轴角 deg 直接驱动 (CALIB 标定开环路径, 无逆解) */
} Motion_Mode;

/** @brief 仿射标定 + 视觉外环参数 */
typedef struct {
    float   k[2][2];        /* 运动学矩阵: [pan;tilt](deg) = k·[x;y](cm) */
    float   b[2];           /* 运动学偏置 (deg) */
    float   corr[2][2];     /* 视觉外环增益 (deg/cm) */
    float   corr_max;       /* 视觉修正限幅 (deg) */
    uint8_t vision_en;      /* 1 = 启用视觉外环修正 */
} Motion_Calib;

/** @brief 运动状态 */
typedef enum {
    MOTION_STATE_IDLE = 0,      /* 未使能/已停止 */
    MOTION_STATE_GOTO,          /* 点到点定位中 */
    MOTION_STATE_PATH,          /* 路径跟随中 */
    MOTION_STATE_TRACK,         /* 持续跟踪中 */
    MOTION_STATE_DONE,          /* 有限运动完成, 保持并继续视觉修正 */
} MotionState;

/** @brief 运动库实例 */
typedef struct {
    /* ── 硬件 ── */
    MotorAxis   *pan;               /* 水平轴 */
    MotorAxis   *tilt;              /* 竖直轴 */
    Motion_Calib calib;             /* 标定 */

    /* ── 状态 ── */
    uint8_t      enabled;
    uint8_t      axes_active;       /* 1 = 轴 PID 正在运行 (有运动任务) */
    Motion_Mode  mode;              /* XY 屏幕坐标 / ANGLE 轴角 */
    MotionState  state;

    /* ── 轨迹 ── */
    Motion_PointXY path[MOTION_MAX_PATH_PTS];
    uint16_t     npts;              /* 路径顶点数 */
    uint16_t     seg;               /* 当前目标顶点索引 */
    uint8_t      closed;            /* 闭合路径 */
    uint8_t      approaching;       /* 1 = 正在接近首个顶点 (不计入圈数) */
    uint32_t     segs_left;         /* 剩余段数 (MOTION_LOOPS_INF = 无限) */
    float        speed;             /* cm/s */

    float        tgt_x, tgt_y;      /* 任务终点 (GOTO / TRACK) */
    float        sp_x,  sp_y;       /* 当前设定点 (沿轨迹推进) */
    float        goto_tol;          /* 到位容差 cm */
    uint32_t     move_tick;         /* 本次有限运动起始时刻 (超时保护) */

    /* ── 视觉反馈 ── */
    float        spot_x, spot_y;    /* 摄像头观测光斑坐标 */
    uint32_t     spot_tick;         /* 最近反馈时刻 (HAL_GetTick) */
    uint8_t      has_spot;

    /* ── 调试 ── */
    float        ff_pan, ff_tilt;   /* 前馈角 (deg) */
    float        corr_pan, corr_tilt;/* 视觉修正角 (deg) */
} Motion;

/* ========================================================================= */
/*  初始化 / 标定                                                              */
/* ========================================================================= */

/** @brief 绑定两轴 (不使能) */
void Motion_Init(Motion *m, MotorAxis *pan, MotorAxis *tilt);

/** @brief 设置标定参数 (拷贝) */
void Motion_SetCalib(Motion *m, const Motion_Calib *calib);

/**
 * @brief 生成默认标定 (光束角≈轴角, 云台距屏 dist_cm)
 *        k00 = k11 = 57.2958/dist, 其余为 0; 视觉外环轻微使能
 */
void Motion_DefaultCalib(Motion_Calib *calib, float dist_cm);

/** @brief 使能/停止运动库 (0 = 关闭两轴并回到 IDLE) */
void Motion_Enable(Motion *m, uint8_t en);

/** @brief 立即释放两轴力矩 (空闲彻底静音/免发热; 光斑不再保持) */
void Motion_Release(Motion *m);

/** @brief 启用/停用视觉外环修正 */
void Motion_SetVisionEnable(Motion *m, uint8_t en);

/* ========================================================================= */
/*  运动原语 (非阻塞)                                                          */
/* ========================================================================= */

/** @brief 点到点定位到屏幕坐标 (复位: x=y=0); speed<=0 用默认速度 */
void Motion_GotoXY(Motion *m, float x, float y, float speed);

/**
 * @brief 沿折线/多边形路径运动
 * @param pts    顶点数组 (按运动顺序; 闭合并要求顺时针则由调用者排好序)
 * @param npts   顶点数 (2..MOTION_MAX_PATH_PTS)
 * @param closed 1 = 闭合多边形 (走一周回到起点)
 * @param loops  圈数; 0 = 无限循环 (仅 closed 有意义)
 * @param speed  速度 cm/s; <=0 用默认
 * @note 闭合路径会从离当前位置最近的顶点切入, 再按 pts 顺序行进 (不破坏顺时针).
 */
void Motion_FollowPathXY(Motion *m, const Motion_PointXY *pts, uint16_t npts,
                         uint8_t closed, uint32_t loops, float speed);

/* ── 角度空间原语 (开环: 直接用 CALIB 标定得到的轴角, 不经屏幕坐标/逆解) ── */

/**
 * @brief 角度空间点到点: 设定点从当前轴角按 rate 平滑推进到目标角, 角度 PID 闭环
 *        (自动取最短路径, 跨越 0/360 接缝无跳变). 到达并稳定后 state = DONE.
 * @param pan_deg/tilt_deg 预设轴角 (deg, 即 CALIB 记录值)
 * @param rate_dps 设定点速率 deg/s; <=0 用默认
 */
void Motion_GotoAngle(Motion *m, float pan_deg, float tilt_deg, float rate_dps);

/**
 * @brief 角度空间折线/多边形: 顶点为标定轴角, 相邻顶点间线性插值
 * @param pts 顶点数组 (Motion_PointAngle, deg)
 * @param rate_dps 设定点速率 deg/s; <=0 用默认
 */
void Motion_FollowAnglePath(Motion *m, const Motion_PointAngle *pts, uint16_t npts,
                            uint8_t closed, uint32_t loops, float rate_dps);

/** @brief 持续跟踪移动目标点 (每周期更新目标, 直到被替换/停止) */
void Motion_TrackXY(Motion *m, float x, float y);

/** @brief 立即制动 (暂停): 关两轴 PID 并停脉冲, 线圈保持位置 */
void Motion_Stop(Motion *m);

/* ========================================================================= */
/*  视觉反馈                                                                   */
/* ========================================================================= */

/** @brief 喂入摄像头观测到的光斑屏幕坐标 (cm); 调用即刷新超时计时 */
void Motion_SetSpotXY(Motion *m, float x, float y);

/* ========================================================================= */
/*  周期任务 / 状态查询                                                        */
/* ========================================================================= */

/** @brief 控制周期调用: 推进轨迹 → 运动学 → 轴目标 → 两轴闭环更新 */
void Motion_Task(Motion *m, float dt_s);

/** @brief 当前运动状态 */
MotionState Motion_GetState(const Motion *m);

/** @brief 有限运动是否完成 (状态 == DONE) */
uint8_t Motion_IsDone(const Motion *m);

/** @brief 读取当前设定点 (cm) */
void Motion_GetSetpointXY(const Motion *m, float *x, float *y);

/** @brief 读取最近光斑反馈 (cm); 返回 0 表示无有效反馈 */
uint8_t Motion_GetSpotXY(const Motion *m, float *x, float *y);

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_H */
