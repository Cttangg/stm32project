/**
 ******************************************************************************
 * @file    axis.h
 * @brief   单轴抽象层 — 统一封装 编码器 + 位置PID + 步进驱动器 (Pan/Tilt 共用)
 *
 * 职责:
 *   - 读取编码器反馈, 更新当前角度
 *   - 校验目标合法性, 位置限幅
 *   - 调用 MotorPID 位置闭环
 *   - 判断是否到位, 维护 Axis 状态机
 *   - 编码器异常/限位触发 → 进入 FAULT 并停止输出
 *
 * Axis 不负责:
 *   - 视觉 / UART / 目标检测 / 坐标转换 / 搜索策略 / 系统模式
 *
 * 对应 architecture.md §9 / requirements.md §5-7.
 ******************************************************************************
 */
#ifndef __AXIS_H
#define __AXIS_H

#include <stdbool.h>
#include <stdint.h>
#include "mt6701.h"
#include "tmc2209.h"
#include "motor_pid.h"
#include "motor_stepper.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  状态 / 故障                                                                */
/* ========================================================================= */
typedef enum
{
    AXIS_DISABLED = 0,
    AXIS_IDLE,
    AXIS_MOVING,
    AXIS_HOLD,
    AXIS_FAULT
} AxisState;

typedef enum
{
    AXIS_OK = 0,
    AXIS_ERR_ENCODER,
    AXIS_ERR_DRIVER,
    AXIS_ERR_LIMIT,
    AXIS_ERR_TIMEOUT,
    AXIS_ERR_INVALID_TARGET
} AxisFault;

/* ========================================================================= */
/*  轴实例                                                                    */
/* ========================================================================= */
typedef struct
{
    /* ── 角度状态 (deg) ── */
    float position;          /* 当前角度 (编码器反馈) */
    float target_position;   /* 目标角度 */
    float velocity;          /* 当前测量速度 deg/s */
    float target_velocity;   /* 当前目标速度 deg/s */

    /* ── 运动限制 ── */
    float min_position;      /* 角度下限 deg */
    float max_position;      /* 角度上限 deg */
    float max_velocity;      /* 最大速度 deg/s (透传到 PID) */
    float max_acceleration;  /* 最大加速度 deg/s² (透传到 PID) */
    float position_tolerance;/* 到位判定阈值 deg */

    /* ── 状态 ── */
    bool enabled;
    bool in_position;
    AxisState state;
    AxisFault  fault;
    uint8_t    enc_fail_cnt;   /* 编码器连续失败计数 (偶发容忍, 连续失败才 FAULT) */

    /* ── 底层依赖 (由应用层注入) ── */
    MotorPID               *pid;
    TMC2209_HandleTypeDef  *driver;
    MT6701_HandleTypeDef   *encoder;
    MotorStepper           *stepper;
} Axis;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定 PID/驱动/编码器/步进引擎, 状态 DISABLED) */
void Axis_Init(Axis *axis,
               MotorPID              *pid,
               TMC2209_HandleTypeDef *driver,
               MT6701_HandleTypeDef  *encoder,
               MotorStepper          *stepper);

/** @brief 使能: 驱动使能, 状态 → IDLE */
void Axis_Enable(Axis *axis);

/** @brief 禁止: 停止 PID + 步进, 释放驱动, 状态 → DISABLED */
void Axis_Disable(Axis *axis);

/** @brief 设定目标角度 (deg), 超出限位则钳位 */
void Axis_SetTarget(Axis *axis, float target);

/** @brief 设定角度限位 [min_position, max_position] */
void Axis_SetLimits(Axis *axis, float min_position, float max_position);

/** @brief 周期更新 (主循环调用): 读编码器 → PID → 驱动步进 */
void Axis_Update(Axis *axis, float dt);

/** @brief 是否已到位 */
bool Axis_IsInPosition(const Axis *axis);

/** @brief 是否有故障 */
bool Axis_IsFault(const Axis *axis);

/** @brief 清除故障 (状态回 IDLE, 需重新使能运动) */
void Axis_ClearFault(Axis *axis);

/** @brief 查询状态 */
AxisState Axis_GetState(const Axis *axis);

/** @brief 查询当前角度 deg */
float Axis_GetPosition(const Axis *axis);

#ifdef __cplusplus
}
#endif

#endif /* __AXIS_H */