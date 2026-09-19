/**
 ******************************************************************************
 * @file    motor_axis.h
 * @brief   单轴闭环执行器 — MT6701 编码器 + 位置 PID + STEP 脉冲引擎 (L1 轴控层)
 *
 * 分层定位:
 *   L1 轴控层, 位于驱动层 (mt6701 / motor_pid / motor_stepper / tmc2209) 之上,
 *   向上为运动层 (motion.c) 提供「目标角度 → 闭环执行」的能力.
 *
 * 职责:
 *   - 每个控制周期: 读 MT6701 角度 → PID → 输出 vel_ref (steps/s) 给 MotorStepper
 *   - 目标角度设定 (绝对角度, 内部按 [-180,180] 卷绕走最短路径)
 *   - 使能/释放 (TMC2209 EN); 立即制动 (停脉冲但线圈仍锁定, 光斑冻结)
 *
 * 三态语义:
 *   MotorAxis_Enable(1)  → 使能驱动 + PID, 并锁定当前位置
 *   MotorAxis_SetTarget  → 连续更新目标角 (不清积分, 适合轨迹跟随)
 *   MotorAxis_Stop       → 停止脉冲 + 关 PID, 但 TMC EN 仍低 (线圈锁定, 位置保持)
 *                         注意: 若 TMC 未使能, 停止后电机无力矩, 位置不再保持
 *   MotorAxis_Enable(0)  → 额外拉高 EN, 完全释放电机 (光斑不再保持)
 *
 * 坏点过滤: I2C 读失败时 MT6701 返回 0°, 本层对单周期 >20° 的跳变沿用上次
 *           角度, 防止 PID 因假误差输出打满而产生抖动.
 *
 * 典型用法:
 *   MotorAxis pan_axis, tilt_axis;
 *
 *   MotorAxis_Init(&pan_axis,  &enc_pan,  &stepper_pan,  &motor_pan);
 *   MotorAxis_Init(&tilt_axis, &enc_tilt, &stepper_tilt, &motor_tilt);
 *   MotorAxis_Enable(&pan_axis, 1);
 *   MotorAxis_Enable(&tilt_axis, 1);
 *   MotorAxis_SetTargetDeg(&pan_axis, 12.5f);
 *
 *   while (1) {                               // 5ms 控制周期
 *       MotorAxis_Update(&pan_axis,  0.005f);
 *       MotorAxis_Update(&tilt_axis, 0.005f);
 *   }
 *
 * 备注: 增益整定/细分标定可直接访问 ax->pid (MotorPID), 见 motor_pid.h.
 ******************************************************************************
 */
#ifndef __MOTOR_AXIS_H
#define __MOTOR_AXIS_H

#include "mt6701.h"
#include "motor_pid.h"
#include "motor_stepper.h"
#include "tmc2209.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  轴句柄                                                                     */
/* ========================================================================= */
typedef struct {
    MT6701_HandleTypeDef  *enc;        /* 角度反馈 (I2C 磁编码器) */
    MotorStepper          *stepper;    /* STEP 脉冲引擎 (含 TMC2209 句柄) */
    TMC2209_HandleTypeDef *motor;      /* 驱动板 (EN 使能/释放) */
    MotorPID               pid;        /* 位置闭环控制器 */

    float   target_deg;                /* 目标角度 (deg) */
    float   current_deg;               /* 最近一次编码器角度 (deg) */
    uint8_t enabled;                   /* 1 = 已使能 */
} MotorAxis;

/* ========================================================================= */
/*  公开 API                                                                   */
/* ========================================================================= */

/** @brief 绑定编码器 / 脉冲引擎 / 驱动板 (不使能; 在 Stepper+编码器 Init 之后调用) */
void   MotorAxis_Init(MotorAxis *ax, MT6701_HandleTypeDef *enc,
                      MotorStepper *stepper, TMC2209_HandleTypeDef *motor);

/**
 * @brief 使能/释放轴
 * @param en 1 = 使能驱动并把目标锁定到当前角度; 0 = 关 PID + 释放电机 (EN 拉高)
 */
void   MotorAxis_Enable(MotorAxis *ax, uint8_t en);

/** @brief 连续更新目标角度 (deg, 绝对值; 不清积分, 适合轨迹跟随) */
void   MotorAxis_SetTargetDeg(MotorAxis *ax, float target_deg);

/** @brief 设置 PID 增益 (透传 MotorPID_SetGains) */
void   MotorAxis_SetGains(MotorAxis *ax, float kp, float ki, float kd);

/** @brief 立即制动: 关 PID + 停 STEP 脉冲, 线圈仍锁定 (位置保持) */
void   MotorAxis_Stop(MotorAxis *ax);

/** @brief 控制周期调用: 读编码器 → PID → 更新 STEP 速率 */
void   MotorAxis_Update(MotorAxis *ax, float dt_s);

/** @brief 只刷新一次编码器角度 (空闲时读一次, 供运动学逆解使用) */
void   MotorAxis_Refresh(MotorAxis *ax);

/** @brief 最近一次编码器角度 (deg) */
float  MotorAxis_GetDeg(const MotorAxis *ax);

/** @brief 当前目标角度 (deg) */
float  MotorAxis_GetTargetDeg(const MotorAxis *ax);

/** @brief 当前角度误差 (deg, 已卷绕 [-180,180]) */
float  MotorAxis_GetErrorDeg(const MotorAxis *ax);

/** @brief 是否到位 (PID 处于 HOLDING) */
uint8_t MotorAxis_IsSettled(const MotorAxis *ax);

/** @brief PID 状态机状态 */
MotorPID_State MotorAxis_GetState(const MotorAxis *ax);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_AXIS_H */
