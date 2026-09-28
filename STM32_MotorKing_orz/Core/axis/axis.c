/**
 ******************************************************************************
 * @file    axis.c
 * @brief   单轴抽象层实现 — 包装 MotorPID/MT6701/TMC2209, 非阻塞
 *
 * 迁移策略 (requirements.md §7):
 *   Axis_Update()
 *        │
 *        ▼
 *   MotorPID_Update()  ← 现有控制算法不变, Axis 只做编排
 *
 * 控制参数 (Kp/Ki/Kd/max_speed/max_accel) 保持 motor_pid.c 原值, 不擅自修改.
 ******************************************************************************
 */
#include "axis.h"

/* 误差卷绕到 [-180,180] (与 motor_pid 内部一致) */
static float axis_wrap180(float deg)
{
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float axis_clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 停止本轴输出: 停 PID + 停步进脉冲 */
static void axis_stop_output(Axis *axis)
{
    if (axis->pid)     MotorPID_Disable(axis->pid);
    if (axis->stepper) MotorStepper_Stop(axis->stepper);
}

void Axis_Init(Axis *axis,
               MotorPID              *pid,
               TMC2209_HandleTypeDef *driver,
               MT6701_HandleTypeDef  *encoder,
               MotorStepper          *stepper)
{
    axis->pid                = pid;
    axis->driver             = driver;
    axis->encoder            = encoder;
    axis->stepper            = stepper;

    axis->position           = 0.0f;
    axis->target_position    = 0.0f;
    axis->velocity           = 0.0f;
    axis->target_velocity    = 0.0f;

    axis->min_position       = -180.0f;
    axis->max_position       =  180.0f;
    axis->max_velocity       = 0.0f;   /* 未设置: 沿用 PID 内部 max_speed */
    axis->max_acceleration   = 0.0f;   /* 未设置: 沿用 PID 内部 max_accel */
    axis->position_tolerance = 0.5f;   /* deg, 默认与 PID deadband_exit 一致 */

    axis->enabled      = false;
    axis->in_position  = false;
    axis->state        = AXIS_DISABLED;
    axis->fault        = AXIS_OK;
    axis->enc_fail_cnt = 0;
}

void Axis_Enable(Axis *axis)
{
    if (axis->fault != AXIS_OK) return;   /* 故障中不可使能 */

    if (axis->driver) TMC2209_Enable(axis->driver);
    axis->enabled = true;
    axis->state   = AXIS_IDLE;
}

void Axis_Disable(Axis *axis)
{
    axis_stop_output(axis);
    if (axis->driver) TMC2209_Disable(axis->driver);
    axis->enabled     = false;
    axis->in_position = false;
    axis->state       = AXIS_DISABLED;
}

void Axis_SetTarget(Axis *axis, float target)
{
    /* 目标钳位到机械限位内 */
    axis->target_position = axis_clampf(target, axis->min_position, axis->max_position);
}

void Axis_SetLimits(Axis *axis, float min_position, float max_position)
{
    if (min_position >= max_position) return;
    axis->min_position = min_position;
    axis->max_position = max_position;
    axis->target_position = axis_clampf(axis->target_position,
                                        axis->min_position,
                                        axis->max_position);
}

bool Axis_IsInPosition(const Axis *axis)
{
    return axis->in_position;
}

bool Axis_IsFault(const Axis *axis)
{
    return (axis->fault != AXIS_OK);
}

void Axis_ClearFault(Axis *axis)
{
    axis->fault       = AXIS_OK;
    axis->state       = AXIS_IDLE;
    axis->in_position = false;
}

AxisState Axis_GetState(const Axis *axis)
{
    return axis->state;
}

float Axis_GetPosition(const Axis *axis)
{
    return axis->position;
}

void Axis_Update(Axis *axis, float dt)
{
    float angle, err;

    if (!axis->enabled) return;

    /* ── 1. 读取编码器反馈 ── */
    if (axis->encoder == NULL) {
        axis->fault = AXIS_ERR_ENCODER;
        axis->state = AXIS_FAULT;
        axis_stop_output(axis);
        return;
    }

    if (MT6701_IsMagnetOK(axis->encoder) == 0) {
        /* I2C 偶发故障容忍: 连续失败超过阈值才判定编码器异常 (磁检失败/持续无应答) */
        if (++axis->enc_fail_cnt >= 3) {
            axis->fault = AXIS_ERR_ENCODER;
            axis->state = AXIS_FAULT;
            axis_stop_output(axis);
        }
        return;   /* 本次跳过更新, 保持上次输出 */
    }
    axis->enc_fail_cnt = 0;

    angle = MT6701_ReadDegrees(axis->encoder);
    axis->position = angle;

    /* ── 2. 机械限位检测 ── */
    if (angle <= axis->min_position || angle >= axis->max_position) {
        axis->fault = AXIS_ERR_LIMIT;
        axis->state = AXIS_FAULT;
        axis_stop_output(axis);
        return;
    }

    /* ── 3. 目标合法化 (已由 SetTarget 钳位, 再兜底一次) ── */
    axis->target_position = axis_clampf(axis->target_position,
                                        axis->min_position,
                                        axis->max_position);

    /* ── 4. 下发目标到 PID (仅当目标变化时, 保留 PID 平滑斜坡) ── */
    if (axis->pid->enabled == 0 || axis->pid->target != axis->target_position) {
        MotorPID_SetTarget(axis->pid, axis->target_position);
    }

    /* ── 5. 位置闭环 (现有控制算法不变) ── */
    MotorPID_Update(axis->pid, angle, dt);
    axis->velocity        = axis->pid->filtered_vel;   /* deg/s, 测量速度 */
    axis->target_velocity = axis->pid->vel_ref;         /* steps/s → deg/s 由 PID 承担 */

    /* ── 6. 驱动步进 (PID 输出 velocity_ref 交给硬件定时器) ── */
    if (axis->stepper) {
        MotorStepper_SetVelocity(axis->stepper, axis->pid->vel_ref);
    }

    /* ── 7. 到位判定 (误差进入容差) ── */
    err = axis_wrap180(axis->target_position - angle);
    axis->in_position = (err > -axis->position_tolerance &&
                         err <  axis->position_tolerance);
    axis->state = axis->in_position ? AXIS_HOLD : AXIS_MOVING;
}