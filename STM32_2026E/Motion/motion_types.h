/**
 ******************************************************************************
 * @file    motion_types.h
 * @brief   Motion 层公共类型：状态、结果码、请求/状态结构 (无硬件依赖)
 *
 * 分层: Motion 层向上只暴露这些类型; 不包含 HAL / GPIO / 定时器头文件.
 ******************************************************************************
 */
#ifndef __MOTION_TYPES_H
#define __MOTION_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  单轴状态机 (对应设计指南 §三)                                             */
/* ========================================================================= */
typedef enum {
    AXIS_STATE_DISABLED = 0,   /* 未使能 */
    AXIS_STATE_IDLE,           /* 使能且空闲, 可接受新请求 */
    AXIS_STATE_MOVING,         /* 正在走步 */
    AXIS_STATE_STOPPING,       /* 收到停止请求, 等待底层确认 */
    AXIS_STATE_DONE,           /* 上一段运动完成 (查询后由新请求/复位离开) */
    AXIS_STATE_ERROR           /* 故障 (超时/驱动/限位), 需 ClearFault */
} AxisState_t;

/* ========================================================================= */
/*  统一结果码 (设计指南 §十二: 禁止只返回 true/false)                        */
/* ========================================================================= */
typedef enum {
    AXIS_RESULT_OK = 0,
    AXIS_RESULT_BUSY,
    AXIS_RESULT_INVALID_PARAM,
    AXIS_RESULT_NOT_ENABLED,
    AXIS_RESULT_LIMIT_TRIGGERED,
    AXIS_RESULT_DRIVER_FAULT,
    AXIS_RESULT_TIMEOUT,
    AXIS_RESULT_INTERNAL_ERROR
} AxisResult_t;

/* ========================================================================= */
/*  位置反馈回调 (Motion 层不依赖具体传感器, 由 App 绑定)                      */
/* ========================================================================= */
/**
 * @brief 读取连续角度 (deg, 多圈累计; 正方向与电机 CCW 一致)
 * @param ctx 反馈上下文 (如 MT6701 句柄指针)
 */
typedef float (*AxisFeedbackReadFn)(void *ctx);

/* ========================================================================= */
/*  运动请求 (设计指南 §八: 请求与状态分离)                                   */
/* ========================================================================= */
typedef struct {
    int32_t  steps;                     /* 有符号步数: + 正方向, - 反方向 */
    uint32_t velocity_steps_per_sec;    /* 目标步率 */
} AxisMoveRequest_t;

/* ========================================================================= */
/*  运动状态快照                                                              */
/* ========================================================================= */
typedef struct {
    AxisState_t state;
    int32_t     current_steps;          /* 软件累计位置 (开环, 单位: 步) */
    int32_t     target_steps;           /* 当前目标位置 (开环, 单位: 步) */
    int32_t     remaining_steps;        /* 剩余步数 */
    uint32_t    velocity_steps_per_sec;
    float       current_deg;            /* 闭环: 反馈角度 (deg, 多圈累计) */
    float       current_mm;             /* 闭环: 反馈换算的直线位置 (mm) */
    float       target_mm;              /* 闭环: 目标直线位置 (mm) */
    uint8_t     enabled;
    uint8_t     fault;
    uint8_t     closed_loop;
    AxisResult_t last_error;
} AxisStatus_t;

#ifdef __cplusplus
}
#endif

#endif /* __MOTION_TYPES_H */
