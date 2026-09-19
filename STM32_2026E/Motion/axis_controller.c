/**
 ******************************************************************************
 * @file    axis_controller.c
 * @brief   单轴控制器实现 (Motion 层, 非阻塞状态机)
 *
 * 两种模式:
 *   - 开环 (未绑定反馈): 以步数计位置, MoveSteps 定步运动
 *   - 闭环 (绑定反馈):   以编码器角度计位置, SetTargetMM 设定目标,
 *                        PID 输出速度 steps/s → MotorStepper_SetVelocity
 *
 * 闭环位置换算 (丝杆 2mm/r):
 *   feedback 返回连续角度 deg (多圈累计)
 *   current_mm = current_deg × AXIS_MM_PER_DEG
 *   target_deg = target_mm  × AXIS_DEG_PER_MM
 *
 * 无指令不自走: PID 仅在 SetTargetMM 后被使能; 上电/使能均不产生运动。
 ******************************************************************************
 */
#include "axis_controller.h"
#include "motion_config.h"

/* ========================================================================= */
/*  内部辅助                                                                  */
/* ========================================================================= */

/* 开环: 运动期间由硬件计数换算当前位置 */
static int32_t axis_hw_delta(const AxisController *ax)
{
    uint32_t hw = MotorStepper_GetExecutedSteps(ax->stepper);
    return (int32_t)(hw - ax->move_start_hw);
}

/* 闭环: 采样一次反馈, 更新 current_deg / current_mm (mm 相对零点, 上电即 0) */
static void axis_sample_feedback(AxisController *ax)
{
    if (!ax->feedback_read) return;
    ax->current_deg = ax->feedback_read(ax->feedback_ctx);
    ax->current_mm  = (ax->current_deg - ax->zero_deg) * AXIS_MM_PER_DEG;
}

/* 闭环: 一个控制周期 (读反馈 → PID → 下发速度) */
static void axis_update_closed_loop(AxisController *ax)
{
    uint32_t now = HAL_GetTick();
    float    dt;
    float    dev;

    /* ERROR 锁定: 未清故障前不动作, 也不覆盖状态 */
    if (ax->state == AXIS_STATE_ERROR) return;

    if ((now - ax->last_update_tick) < ax->period_ms) return;
    dt = (float)(now - ax->last_update_tick) / 1000.0f;
    ax->last_update_tick = now;
    if (dt <= 0.0f) dt = (float)ax->period_ms / 1000.0f;

    axis_sample_feedback(ax);

    /* 软限位: 相对零点偏移超限 → 立即停止并锁定故障 */
    dev = ax->current_deg - ax->zero_deg;
    if (dev < 0.0f) dev = -dev;
    if (ax->soft_limit_deg > 0.0f && dev > ax->soft_limit_deg) {
        MotorPID_Disable(&ax->pid);
        MotorStepper_Stop(ax->stepper);
        ax->fault      = 1;
        ax->last_error = AXIS_RESULT_LIMIT_TRIGGERED;
        ax->state      = AXIS_STATE_ERROR;
        return;
    }

    MotorPID_Update(&ax->pid, ax->current_deg, dt);
    MotorStepper_SetVelocity(ax->stepper, ax->pid.vel_ref);

    /* 状态映射 */
    if (ax->pid.state == MOTOR_PID_DISABLED) {
        ax->state = ax->enabled ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED;
    } else if (ax->pid.state == MOTOR_PID_HOLDING) {
        ax->state = AXIS_STATE_DONE;
    } else {
        ax->state = AXIS_STATE_MOVING;
    }

    /* 超时保护: 到位判定由 PID HOLDING 负责, 超时仅针对长时间未到位 */
    if (ax->state == AXIS_STATE_MOVING && ax->timeout_ms &&
        (now - ax->move_start_tick) > ax->timeout_ms) {
        MotorPID_Disable(&ax->pid);
        MotorStepper_Stop(ax->stepper);
        ax->fault      = 1;
        ax->last_error = AXIS_RESULT_TIMEOUT;
        ax->state      = AXIS_STATE_ERROR;
    }
}

/* ========================================================================= */
/*  公开 API                                                                  */
/* ========================================================================= */

void AxisController_Init(AxisController *ax, const char *name, MotorStepper *stepper)
{
    ax->name             = name;
    ax->stepper          = stepper;

    ax->closed_loop      = 0;
    ax->feedback_read    = NULL;
    ax->feedback_ctx     = NULL;
    MotorPID_Init(&ax->pid);
    ax->pid.max_speed = AXIS_MAX_SPEED_STEPS;   /* 丝杆机构整定 */
    ax->pid.max_accel = AXIS_MAX_ACCEL_STEPS;
    ax->pid.kp        = AXIS_PID_KP;             /* 内环 PID (可在线调) */
    ax->pid.ki        = AXIS_PID_KI;
    ax->pid.kd        = AXIS_PID_KD;
    ax->pid.d_alpha   = AXIS_PID_D_ALPHA;
    ax->pid.hold_velocity = AXIS_PID_HOLD_VEL;
    ax->pid.vel_clamp = AXIS_PID_VEL_CLAMP_DEG;
    ax->pid.deadband_enter = AXIS_DEADBAND_ENTER_DEG;
    ax->pid.deadband_exit  = AXIS_DEADBAND_EXIT_DEG;
    ax->pid.comp_max_err   = AXIS_COMP_MAX_ERR_DEG;
    ax->pid.stiction_comp  = AXIS_STICTION_COMP_STEPS;
    ax->current_deg      = 0.0f;
    ax->current_mm       = 0.0f;
    ax->target_mm        = 0.0f;
    ax->zero_deg         = 0.0f;
    ax->soft_limit_deg   = 0.0f;
    ax->period_ms        = AXIS_CONTROL_PERIOD_MS;
    ax->last_update_tick = HAL_GetTick();

    ax->current_steps    = 0;
    ax->target_steps     = 0;
    ax->move_start_steps = 0;
    ax->move_start_hw    = 0;
    ax->dir_sign         = 1;
    ax->velocity         = 0;
    ax->timeout_ms       = AXIS_MOVE_TIMEOUT_MS;
    ax->move_start_tick  = 0;

    ax->state            = AXIS_STATE_DISABLED;
    ax->enabled          = 0;
    ax->fault            = 0;
    ax->last_error       = AXIS_RESULT_OK;

    MotorStepper_Stop(stepper);
    MotorStepper_Disable(stepper);
}

void AxisController_SetFeedback(AxisController *ax, AxisFeedbackReadFn read, void *ctx)
{
    ax->feedback_read = read;
    ax->feedback_ctx  = ctx;
    ax->closed_loop   = (read != NULL) ? 1 : 0;
    if (ax->closed_loop) {
        MotorPID_SetMode(&ax->pid, 0);   /* 丝杆多圈: 连续位置, 不卷绕 */
        ax->soft_limit_deg = AXIS_SOFT_LIMIT_DEG;
        AxisController_Home(ax);         /* 默认以上电位置为零点 */
    }
}

/* 自动零点校准 (预留):
 *   当前无回零/限位硬件 → 直接以"上电位置"作为 0 点 (current_mm = 0)。
 *   未来接入限位开关后, 在此实现: 朝限位运动 → 触碰后回退 → 置零。 */
AxisResult_t AxisController_Home(AxisController *ax)
{
    if (!ax || !ax->closed_loop) return AXIS_RESULT_INVALID_PARAM;

    MotorPID_Disable(&ax->pid);
    MotorStepper_Stop(ax->stepper);

    axis_sample_feedback(ax);            /* 读当前角度 */
    ax->zero_deg   = ax->current_deg;    /* 上电位置 = 零点 */
    ax->current_mm = 0.0f;
    ax->target_mm  = 0.0f;
    ax->state      = ax->enabled ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED;
    return AXIS_RESULT_OK;
}

AxisResult_t AxisController_Enable(AxisController *ax)
{
    if (!ax) return AXIS_RESULT_INVALID_PARAM;
    if (ax->fault) return AXIS_RESULT_DRIVER_FAULT;

    MotorStepper_Enable(ax->stepper);
    ax->enabled = 1;
    if (ax->state == AXIS_STATE_DISABLED) ax->state = AXIS_STATE_IDLE;
    return AXIS_RESULT_OK;
}

void AxisController_Disable(AxisController *ax)
{
    if (!ax) return;
    MotorPID_Disable(&ax->pid);
    MotorStepper_Stop(ax->stepper);
    MotorStepper_Disable(ax->stepper);
    ax->enabled = 0;
    ax->state   = AXIS_STATE_DISABLED;
}

static AxisResult_t axis_set_target(AxisController *ax, float target_mm, uint8_t clamp_nonneg)
{
    if (!ax || !ax->closed_loop) return AXIS_RESULT_INVALID_PARAM;
    if (!ax->enabled) return AXIS_RESULT_NOT_ENABLED;
    if (ax->fault)    return AXIS_RESULT_DRIVER_FAULT;

    if (clamp_nonneg && target_mm < 0.0f) target_mm = 0.0f;   /* 工作区: 不允许负值 */

    ax->target_mm       = target_mm;
    ax->move_start_tick = HAL_GetTick();
    ax->last_update_tick = HAL_GetTick();
    ax->last_error      = AXIS_RESULT_OK;

    /* PID 目标使用绝对角度 = 零点角 + 目标 mm 对应角度; 无指令前 PID 处于 DISABLED, 不会自走 */
    MotorPID_SetTarget(&ax->pid, ax->zero_deg + target_mm * AXIS_DEG_PER_MM);
    ax->state = AXIS_STATE_MOVING;
    return AXIS_RESULT_OK;
}

AxisResult_t AxisController_SetTargetMM(AxisController *ax, float target_mm)
{
    return axis_set_target(ax, target_mm, 1);   /* 绝对定位: 钳 >= 0 */
}

AxisResult_t AxisController_MoveRelMM(AxisController *ax, float delta_mm)
{
    if (!ax) return AXIS_RESULT_INVALID_PARAM;
    return axis_set_target(ax, ax->current_mm + delta_mm, 0);   /* 相对手动: 不钳 */
}

AxisResult_t AxisController_MoveSteps(AxisController *ax, int32_t steps, uint32_t velocity)
{
    TMC2209_Dir dir;
    uint32_t    mag;

    if (!ax) return AXIS_RESULT_INVALID_PARAM;
    if (ax->closed_loop) return AXIS_RESULT_INVALID_PARAM;   /* 闭环用 SetTargetMM */
    if (steps == 0 || velocity == 0 || velocity > AXIS_MAX_VELOCITY)
        return AXIS_RESULT_INVALID_PARAM;
    if (!ax->enabled) return AXIS_RESULT_NOT_ENABLED;
    if (ax->fault)    return AXIS_RESULT_DRIVER_FAULT;
    if (ax->state == AXIS_STATE_MOVING || ax->state == AXIS_STATE_STOPPING)
        return AXIS_RESULT_BUSY;

    dir = (steps > 0) ? TMC2209_DIR_CCW : TMC2209_DIR_CW;
    mag = (uint32_t)((steps > 0) ? steps : -steps);

    ax->dir_sign         = (steps > 0) ? 1 : -1;
    ax->move_start_steps = ax->current_steps;
    ax->move_start_hw    = MotorStepper_GetExecutedSteps(ax->stepper);
    ax->target_steps     = ax->current_steps + steps;
    ax->velocity         = velocity;
    ax->move_start_tick  = HAL_GetTick();
    ax->last_error       = AXIS_RESULT_OK;

    MotorStepper_SetDirection(ax->stepper, dir);
    MotorStepper_MoveSteps(ax->stepper, mag, (float)velocity);
    ax->state = AXIS_STATE_MOVING;
    return AXIS_RESULT_OK;
}

AxisResult_t AxisController_MoveTo(AxisController *ax, int32_t absolute_steps, uint32_t velocity)
{
    if (!ax) return AXIS_RESULT_INVALID_PARAM;
    return AxisController_MoveSteps(ax, absolute_steps - ax->current_steps, velocity);
}

AxisResult_t AxisController_Stop(AxisController *ax)
{
    if (!ax) return AXIS_RESULT_INVALID_PARAM;

    if (ax->closed_loop) {
        MotorPID_Disable(&ax->pid);
        MotorStepper_Stop(ax->stepper);
        ax->state = ax->enabled ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED;
        return AXIS_RESULT_OK;
    }

    if (ax->state != AXIS_STATE_MOVING) return AXIS_RESULT_OK;
    ax->state = AXIS_STATE_STOPPING;
    MotorStepper_Stop(ax->stepper);
    ax->current_steps = ax->move_start_steps + ax->dir_sign * axis_hw_delta(ax);
    return AXIS_RESULT_OK;
}

void AxisController_ClearFault(AxisController *ax)
{
    if (!ax || ax->state != AXIS_STATE_ERROR) return;
    ax->fault      = 0;
    ax->last_error = AXIS_RESULT_OK;
    ax->state      = ax->enabled ? AXIS_STATE_IDLE : AXIS_STATE_DISABLED;
}

void AxisController_Update(AxisController *ax)
{
    if (!ax) return;

    if (ax->closed_loop) {
        axis_update_closed_loop(ax);
        return;
    }

    /* ── 开环 ── */
    if (ax->state != AXIS_STATE_MOVING && ax->state != AXIS_STATE_STOPPING) return;

    ax->current_steps = ax->move_start_steps + ax->dir_sign * axis_hw_delta(ax);

    if (ax->state == AXIS_STATE_STOPPING) {
        if (!MotorStepper_IsBusy(ax->stepper)) ax->state = AXIS_STATE_IDLE;
        return;
    }

    if (ax->timeout_ms && (HAL_GetTick() - ax->move_start_tick) > ax->timeout_ms) {
        MotorStepper_Stop(ax->stepper);
        ax->current_steps = ax->move_start_steps + ax->dir_sign * axis_hw_delta(ax);
        ax->fault         = 1;
        ax->last_error    = AXIS_RESULT_TIMEOUT;
        ax->state         = AXIS_STATE_ERROR;
        return;
    }

    if (!MotorStepper_IsBusy(ax->stepper)) {
        ax->current_steps = ax->target_steps;
        ax->state         = AXIS_STATE_DONE;
    }
}

AxisState_t AxisController_GetState(const AxisController *ax)
{
    return ax ? ax->state : AXIS_STATE_DISABLED;
}

AxisStatus_t AxisController_GetStatus(const AxisController *ax)
{
    AxisStatus_t st;
    st.state                  = ax->state;
    st.current_steps          = ax->current_steps;
    st.target_steps           = ax->target_steps;
    st.remaining_steps        = (int32_t)MotorStepper_GetRemainingSteps(ax->stepper);
    st.velocity_steps_per_sec = ax->velocity;
    st.current_deg            = ax->current_deg;
    st.current_mm             = ax->current_mm;
    st.target_mm              = ax->target_mm;
    st.enabled                = ax->enabled;
    st.fault                  = ax->fault;
    st.closed_loop            = ax->closed_loop;
    st.last_error             = ax->last_error;
    return st;
}
