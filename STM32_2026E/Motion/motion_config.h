/**
 ******************************************************************************
 * @file    motion_config.h
 * @brief   Motion 层可标定参数集中定义 (阶段 B 单轴默认值)
 *
 * 注意: 这些是"控制器参数", 不是机械行程。真实行程/回零方向等待硬件确认后
 *       再由上层传入, 不在本文件硬编码。
 ******************************************************************************
 */
#ifndef __MOTION_CONFIG_H
#define __MOTION_CONFIG_H

#include <stdint.h>

/* 默认/上限步率 (steps/s); 上限需 ≤ motor_stepper 的 50000 保护值 */
#define AXIS_DEFAULT_VELOCITY     1000U
#define AXIS_MAX_VELOCITY         50000U

/* 单轴最小实现演示: 每次请求的固定步数与运动超时 */
#define AXIS_DEFAULT_MOVE_STEPS   2000
#define AXIS_MOVE_TIMEOUT_MS      10000U

/* ========================================================================= */
/*  机械参数: 丝杆导程 2mm/r (角度↔直线换算)                                  */
/* ========================================================================= */
#define AXIS_LEAD_MM_PER_REV      2.0f     /* 丝杆导程: 2mm / 圈 */
#define AXIS_MM_PER_DEG           (AXIS_LEAD_MM_PER_REV / 360.0f)   /* 0.005556 mm/deg */
#define AXIS_DEG_PER_MM           (360.0f / AXIS_LEAD_MM_PER_REV)   /* 180 deg/mm */

/* 步数换算 (1/16 细分, 200 全步/圈; 最粗可用档) */
#define AXIS_FULL_STEPS_PER_REV   200.0f
#define AXIS_MICROSTEPS           16.0f
#define AXIS_STEPS_PER_REV        (AXIS_FULL_STEPS_PER_REV * AXIS_MICROSTEPS)  /* 3200 */
#define AXIS_STEPS_PER_MM         (AXIS_STEPS_PER_REV / AXIS_LEAD_MM_PER_REV)  /* 1600 */

/* 闭环控制周期 (ms) */
#define AXIS_CONTROL_PERIOD_MS    5U

/* 闭环软限位 (相对上电零点): 0 = 禁用; 测试完成, 现禁用 */
#define AXIS_SOFT_LIMIT_TURNS     0.0f
#define AXIS_SOFT_LIMIT_DEG       (AXIS_SOFT_LIMIT_TURNS * 360.0f)

/* ========================================================================= */
/*  闭环整定 (丝杆机构)                                                       */
/* ========================================================================= */
/* 反馈方向: 使"正速度命令 → 反馈角增大". 实测相反, 故取 -1.0;
   若闭环发散, 改 +1.0 (或交换电机 DIR 极性) */
#define AXIS_FEEDBACK_SIGN        (-1.0f)

/* 速度/加速度上限 (steps/s, steps/s²): 最高 STEP 频率 15 kHz (1/16 下稳定) */
#define AXIS_MAX_SPEED_STEPS      15000.0f
#define AXIS_MAX_ACCEL_STEPS      60000.0f

/* 到位死区/静摩擦补偿 (deg): 1/16 步距粗, 放宽以进 HOLDING */
#define AXIS_DEADBAND_ENTER_DEG   2.0f
#define AXIS_DEADBAND_EXIT_DEG    5.0f
#define AXIS_COMP_MAX_ERR_DEG     3.0f
#define AXIS_STICTION_COMP_STEPS  50.0f

/* ========================================================================= */
/*  内环位置 PID 参数 (串口 `pid <kp> <ki> <kd>` 可在线调)                    */
/*  降低 kp / 提高 D 滤波 → 减小到位振动                                      */
/* ========================================================================= */
#define AXIS_PID_KP        200.0f   /* (steps/s)/deg */
#define AXIS_PID_KI        0.0f     /* (steps/s)/(deg·s) */
#define AXIS_PID_KD        10.0f    /* steps/deg, D-on-measurement */
#define AXIS_PID_D_ALPHA   0.90f    /* D 低通系数 (越大越平滑) */
#define AXIS_PID_HOLD_VEL  0.0f     /* HOLDING 允许的微小输出 steps/s */
#define AXIS_PID_VEL_CLAMP_DEG  3000.0f  /* 测量速度限幅 deg/s (须 > 实际最高速) */

#endif /* __MOTION_CONFIG_H */
