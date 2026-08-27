/**
 ******************************************************************************
 * @file    motor_pid.h
 * @brief   步进电机位置闭环 PID 控制器 (实时、平滑、抗干扰)
 *
 * 单位 (明确物理含义):
 *   位置误差 err  : deg
 *   velocity_cmd : steps/s   (PID 原始输出, 速度限幅后)
 *   velocity_ref : steps/s   (加速度限制后的实际期望速度, 连续状态量)
 *   加速度限制    : steps/s²
 *   kp           : (steps/s) / deg
 *   ki           : (steps/s) / (deg·s)
 *   kd           : (steps/s) / (deg/s) = steps/deg  (作用在测量速度上)
 *
 * 关键设计:
 *   - D 项 = -kd × 滤波后测量速度 (D-on-measurement): 目标改变无 derivative kick,
 *     并对测量噪声做一阶低通 + 限幅
 *   - 积分: 条件积分 (输出饱和时不积, 防 windup) + 限幅; HOLDING 保留少量积分
 *   - Deadband 滞回: 进入阈值 < 退出阈值, 防边界反复启停
 *   - 状态机: DISABLED → MOVING → APPROACHING → HOLDING
 *
 * 输出: velocity_ref 交给 MotorStepper_SetVelocity() 驱动硬件定时器.
 ******************************************************************************
 */
#ifndef __MOTOR_PID_H
#define __MOTOR_PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  步进/角度单位换算 (电机步距角 × TMC2209 细分)                            */
/* ========================================================================= */
#define MOTOR_FULL_STEPS_PER_REV   200.0f   /* 42 步进电机: 200 全步/圈 (1.8°) */
#define MOTOR_MICROSTEPS           32       /* 默认细分 (MS1=High MS2=Low → 1/32) */
#define STEPS_PER_REV  (MOTOR_FULL_STEPS_PER_REV * MOTOR_MICROSTEPS)  /* 6400 */
#define DEG_PER_STEP   (360.0f / (float)STEPS_PER_REV)                 /* 0.05625 */
#define STEPS_PER_DEG  ((float)STEPS_PER_REV / 360.0f)                 /* 17.78 */

/* ========================================================================= */
/*  状态机                                                                     */
/* ========================================================================= */
typedef enum {
    MOTOR_PID_DISABLED   = 0,  /* 控制关闭 */
    MOTOR_PID_MOVING,          /* 误差大, 按 PID 输出速度运行 */
    MOTOR_PID_APPROACHING,     /* 误差进入较小区间, 减速接近 */
    MOTOR_PID_HOLDING,         /* 到位锁定: 停步, 允许静态误差, 不反复启停 */
} MotorPID_State;

/* ========================================================================= */
/*  控制器实例                                                                 */
/* ========================================================================= */
typedef struct {
    /* ── 参数 ── */
    float kp;             /* (steps/s)/deg */
    float ki;             /* (steps/s)/(deg·s) */
    float kd;             /* (steps/s)/(deg/s)=steps/deg, 作用在测量速度 */
    float d_alpha;        /* D 低通系数 0~1 (越大越平滑) */
    float max_speed;      /* 最大速度 steps/s */
    float max_accel;      /* 加速度限制 steps/s² */
    float deadband_enter; /* 进入 HOLDING 阈值 deg */
    float deadband_exit;  /* 退出 HOLDING 阈值 deg (exit > enter, 滞回) */
    float comp_max_err;   /* 死区补偿窗口上界 deg (enter<|err|<max 时叠加最小速度) */
    float stiction_comp;  /* 静摩擦补偿最小速度 steps/s */
    float hold_velocity;  /* HOLDING 时允许的微小输出 steps/s (默认 0) */

    /* ── 状态 ── */
    uint8_t       enabled;
    MotorPID_State state;
    float   steps_per_rev;  /* 当前细分对应的每圈步数 (切换 ms 时更新) */
    float   target;          /* deg */
    float   current;         /* deg, 最近测量 */
    float   integral;        /* deg·s */
    float   last_current;    /* deg, 用于测量微分 */
    float   filtered_vel;    /* deg/s, 滤波后测量速度 */
    float   vel_cmd;         /* steps/s, PID 原始输出 (限幅后) */
    float   vel_ref;         /* steps/s, 加速度限制后的期望速度 */
    float   p_term, i_term, d_term;  /* steps/s, 分项输出 (调试用) */
    uint8_t output_saturated;        /* 输出是否饱和 (限幅) */
} MotorPID;

/** @brief 初始化 (默认增益, 关闭状态) */
void  MotorPID_Init(MotorPID *p);

/** @brief 设置 PID 增益 */
void  MotorPID_SetGains(MotorPID *p, float kp, float ki, float kd);

/**
 * @brief 切换细分后按比例重标定 kp / max_speed / max_accel
 *        (保持物理行为一致: 1/64 下同样的 rev/s 需要更多 steps/s)
 * @param old_ms 原细分 (8/16/32/64)
 * @param new_ms 新细分
 */
void  MotorPID_RescaleMicrostep(MotorPID *p, float old_ms, float new_ms);

/** @brief 设定目标角度并使能 (无 derivative kick, 平滑起步) */
void  MotorPID_SetTarget(MotorPID *p, float target_deg);

/** @brief 停止控制 (状态→DISABLED) */
void  MotorPID_Disable(MotorPID *p);

/** @brief 查询目标角度 deg */
float MotorPID_GetTarget(const MotorPID *p);

/** @brief 查询当前误差 deg (已卷绕 [-180,180]) */
float MotorPID_GetError(const MotorPID *p);

/**
 * @brief 控制周期调用 (如 5ms): 角度 → PID → 更新 velocity_cmd/velocity_ref
 * @param angle_deg MT6701 角度
 * @param dt        实际控制周期 (秒), dt<=0 不计算微分
 */
void  MotorPID_Update(MotorPID *p, float angle_deg, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_PID_H */
