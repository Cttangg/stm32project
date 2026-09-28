/**
 ******************************************************************************
 * @file    motor_pid.c
 * @brief   步进电机位置闭环 PID 控制器实现 (实时、平滑、抗干扰)
 ******************************************************************************
 */
#include "motor_pid.h"

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* 误差卷绕到 [-180, 180] */
static float wrap180(float deg) {
    while (deg >  180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

void MotorPID_Init(MotorPID *p) {
    /* 默认按 1/32 细分标定 (开机即 1/32, 无需重标定).
       PID 增益为用户实测整定值; 死区补偿推过静摩擦, 消除微小静差 */
    p->kp = 1500.0f;        /* (steps/s)/deg */
    p->ki = 100.0f;         /* (steps/s)/(deg·s) 消除稳态误差 */
    p->kd = 125.0f;         /* steps/deg, D-on-measurement 阻尼 */
    p->d_alpha       = 0.8f;
    p->max_speed     = 19200.0f;   /* steps/s (~3 rev/s = 1080°/s @1/32) */
    p->max_accel     = 96000.0f;   /* steps/s² (0.2s 达最大) */
    p->deadband_enter = 0.05f;     /* deg (14bit 传感器 0.022°, 1/32 微步 0.056°) */
    p->deadband_exit  = 0.5f;      /* deg (滞回) */
    p->comp_max_err   = 0.5f;      /* deg, 死区补偿窗口上界 */
    p->stiction_comp  = 150.0f;    /* steps/s @1/32, 静摩擦补偿最小速度 */
    p->hold_velocity  = 0.0f;      /* steps/s */

    p->enabled    = 0;
    p->state      = MOTOR_PID_DISABLED;
    p->steps_per_rev = STEPS_PER_REV;   /* 默认 1/32: 6400 */
    p->target     = 0.0f;
    p->current    = 0.0f;
    p->integral   = 0.0f;
    p->last_current = 0.0f;
    p->filtered_vel = 0.0f;
    p->vel_cmd    = 0.0f;
    p->vel_ref    = 0.0f;
    p->p_term = p->i_term = p->d_term = 0.0f;
    p->output_saturated = 0;
}

void MotorPID_SetGains(MotorPID *p, float kp, float ki, float kd) {
    p->kp = kp; p->ki = ki; p->kd = kd;
}

void MotorPID_RescaleMicrostep(MotorPID *p, float old_ms, float new_ms) {
    float r = (old_ms > 0.0f) ? (new_ms / old_ms) : 1.0f;
    p->kp        *= r;    /* 同 °/s 响应下, 高细分需更多 steps/s */
    p->max_speed *= r;
    p->max_accel *= r;
    p->stiction_comp *= r;   /* 静摩擦补偿速度同样按细分比例缩放 */
    p->steps_per_rev = MOTOR_FULL_STEPS_PER_REV * new_ms;
}

void MotorPID_SetTarget(MotorPID *p, float target_deg) {
    p->target = target_deg;
    /* 消除 derivative kick: D 作用在测量值, 目标改变不产生冲击 */
    p->last_current = p->current;
    p->integral     = 0.0f;      /* 大目标变化清零积分 */
    p->filtered_vel = 0.0f;
    p->vel_cmd      = 0.0f;
    p->vel_ref      = 0.0f;      /* 从静止重新起步 (加速度限制平滑) */
    p->output_saturated = 0;
    p->state        = MOTOR_PID_MOVING;
    p->enabled      = 1;
}

void MotorPID_Disable(MotorPID *p) {
    p->enabled    = 0;
    p->state      = MOTOR_PID_DISABLED;
    p->vel_cmd    = 0.0f;
    p->vel_ref    = 0.0f;
    p->integral   = 0.0f;
    p->filtered_vel = 0.0f;
}

float MotorPID_GetTarget(const MotorPID *p) { return p->target; }

float MotorPID_GetError(const MotorPID *p) {
    return wrap180(p->target - p->current);
}

void MotorPID_Update(MotorPID *p, float angle_deg, float dt) {
    float err, raw_vel, out, dv, vel_limit;

    p->current = angle_deg;

    if (!p->enabled) {
        p->state = MOTOR_PID_DISABLED;
        p->vel_cmd = 0.0f; p->vel_ref = 0.0f;
        p->filtered_vel = 0.0f;
        return;
    }

    err = wrap180(p->target - angle_deg);

    /* ── D 项: 测量速度微分 (D-on-measurement) + 低通 + 限幅 ── */
    if (dt > 0.0001f) {
        raw_vel = (angle_deg - p->last_current) / dt;      /* deg/s */
        p->filtered_vel = p->d_alpha * p->filtered_vel
                        + (1.0f - p->d_alpha) * raw_vel;
    }
    p->last_current = angle_deg;
    /* 传感器跳变限幅: 防止一次跳变把速度/输出打满 */
    p->filtered_vel = clampf(p->filtered_vel, -150.0f, 150.0f);   /* deg/s */

    /* ── 状态机 + 滞回死区 ── */
    if (p->state == MOTOR_PID_HOLDING) {
        if (err <= -p->deadband_exit || err >= p->deadband_exit)
            p->state = MOTOR_PID_MOVING;      /* 误差超出退出阈值, 重新运行 */
    } else {
        if (err > -p->deadband_enter && err < p->deadband_enter) {
            if (p->filtered_vel > -2.0f && p->filtered_vel < 2.0f)
                p->state = MOTOR_PID_HOLDING;  /* 误差小且近停 → 锁定 */
            else
                p->state = MOTOR_PID_APPROACHING;
        } else {
            p->state = MOTOR_PID_MOVING;
        }
    }

    /* ── P ── */
    p->p_term = p->kp * err;

    /* ── I: 条件积分 (anti-windup) ── */
    if (p->state == MOTOR_PID_HOLDING) {
        /* 到位: 不再累计, 保留少量积分 (不反复启停) */
    } else if (p->output_saturated) {
        /* 输出饱和: 不积分, 防 windup */
    } else {
        p->integral += err * dt;
    }
    p->integral = clampf(p->integral, -100.0f, 100.0f);   /* 积分限幅 (deg·s) */
    p->i_term = p->ki * p->integral;

    /* ── D: -kd × 滤波测量速度 (阻尼) ── */
    p->d_term = -p->kd * p->filtered_vel;

    /* ── 输出 velocity_cmd + 死区补偿 + 速度限幅 ── */
    if (p->state == MOTOR_PID_HOLDING) {
        out = p->hold_velocity;    /* 停步, 允许静态误差 */
    } else {
        out = p->p_term + p->i_term + p->d_term;
        /* 死区补偿: 误差在 [deadband_enter, comp_max_err] 内时叠加最小速度,
           推过静摩擦死区, 消除微小静差 (防止 PID 输出低于摩擦阈值而蠕动不动) */
        if (err >  p->deadband_enter && err <  p->comp_max_err) out += p->stiction_comp;
        else if (err < -p->deadband_enter && err > -p->comp_max_err) out -= p->stiction_comp;
    }

    if (out >  p->max_speed) { out =  p->max_speed; p->output_saturated = 1; }
    else if (out < -p->max_speed) { out = -p->max_speed; p->output_saturated = 1; }
    else p->output_saturated = 0;
    p->vel_cmd = out;

    /* ── 加速度限制: velocity_ref 连续斜坡 ── */
    vel_limit = p->max_accel * dt;
    dv = out - p->vel_ref;
    dv = clampf(dv, -vel_limit, vel_limit);
    p->vel_ref += dv;
}
