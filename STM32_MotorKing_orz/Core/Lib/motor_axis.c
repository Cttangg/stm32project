/**
 ******************************************************************************
 * @file    motor_axis.c
 * @brief   单轴闭环执行器实现 (L1 轴控层)
 ******************************************************************************
 */
#include "motor_axis.h"

/* I2C 读失败时 MT6701_ReadDegrees 返回 0.0°; 突变过滤:
   单周期内角度跳变超过 MOTOR_AXIS_MAX_JUMP_DEG 视为坏点, 沿用上次角度.
   (防止 PID 看到几十度假误差 → 输出打满 → 高频反转抖动) */
#define MOTOR_AXIS_MAX_JUMP_DEG   20.0f

static float axis_read_deg(MotorAxis *ax) {
    float raw = MT6701_ReadDegrees(ax->enc);
    float d   = raw - ax->current_deg;
    if (d >  180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    if (d >  MOTOR_AXIS_MAX_JUMP_DEG || d < -MOTOR_AXIS_MAX_JUMP_DEG)
        return ax->current_deg;      /* 坏点: 保持上次角度 */
    return raw;
}

void MotorAxis_Init(MotorAxis *ax, MT6701_HandleTypeDef *enc,
                    MotorStepper *stepper, TMC2209_HandleTypeDef *motor) {
    ax->enc     = enc;
    ax->stepper = stepper;
    ax->motor   = motor;

    MotorPID_Init(&ax->pid);

    ax->current_deg = MT6701_ReadDegrees(enc);
    ax->target_deg  = ax->current_deg;
    ax->enabled     = 0;
}

void MotorAxis_Enable(MotorAxis *ax, uint8_t en) {
    if (en) {
        ax->current_deg = axis_read_deg(ax);
        ax->target_deg  = ax->current_deg;
        MotorPID_SetTarget(&ax->pid, ax->current_deg);   /* 上电锁定当前位置 */
        TMC2209_Enable(ax->motor);
        ax->enabled = 1;
    } else {
        MotorPID_Disable(&ax->pid);
        MotorStepper_Stop(ax->stepper);
        TMC2209_Disable(ax->motor);
        ax->enabled = 0;
    }
}

void MotorAxis_SetTargetDeg(MotorAxis *ax, float target_deg) {
    ax->target_deg = target_deg;
    MotorPID_SetTargetContinuous(&ax->pid, target_deg);
}

void MotorAxis_SetGains(MotorAxis *ax, float kp, float ki, float kd) {
    MotorPID_SetGains(&ax->pid, kp, ki, kd);
}

void MotorAxis_Stop(MotorAxis *ax) {
    MotorPID_Disable(&ax->pid);          /* 清除速度输出 */
    MotorStepper_Stop(ax->stepper);      /* 停脉冲 (EN 不动, 线圈保持) */
}

void MotorAxis_Update(MotorAxis *ax, float dt_s) {
    ax->current_deg = axis_read_deg(ax);
    MotorPID_Update(&ax->pid, ax->current_deg, dt_s);
    MotorStepper_SetVelocity(ax->stepper, ax->pid.vel_ref);
}

void MotorAxis_Refresh(MotorAxis *ax) {
    ax->current_deg = axis_read_deg(ax);
}

float MotorAxis_GetDeg(const MotorAxis *ax) {
    return ax->current_deg;
}

float MotorAxis_GetTargetDeg(const MotorAxis *ax) {
    return ax->target_deg;
}

float MotorAxis_GetErrorDeg(const MotorAxis *ax) {
    return MotorPID_GetError(&ax->pid);
}

uint8_t MotorAxis_IsSettled(const MotorAxis *ax) {
    return (ax->pid.enabled && ax->pid.state == MOTOR_PID_HOLDING) ? 1U : 0U;
}

MotorPID_State MotorAxis_GetState(const MotorAxis *ax) {
    return ax->pid.state;
}
