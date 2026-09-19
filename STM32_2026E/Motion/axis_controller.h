/**
 ******************************************************************************
 * @file    axis_controller.h
 * @brief   单轴控制器 (Motion 层核心, 对应设计指南 §三 AxisController)
 *
 * 职责:
 *   - 接收相对/绝对移动请求, 管理位置(步数)与目标
 *   - 单轴状态机 (DISABLED/IDLE/MOVING/STOPPING/DONE/ERROR)
 *   - 参数校验、运动超时、停止与故障处理
 *   - 通过 MotorStepper (Devices 层) 驱动底层, 不直接操作 GPIO/定时器
 *
 * 非阻塞: 所有 API 立即返回; 推进由 AxisController_Update() 在主循环调用.
 ******************************************************************************
 */
#ifndef __AXIS_CONTROLLER_H
#define __AXIS_CONTROLLER_H

#include "motion_types.h"
#include "motor_stepper.h"
#include "motor_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  单轴控制器实例                                                            */
/* ========================================================================= */
typedef struct {
    const char   *name;             /* 轴名 (调试用) */
    MotorStepper *stepper;          /* 绑定的步进执行器 (Devices 层) */

    /* ── 闭环 (角度反馈) ── */
    uint8_t            closed_loop;    /* 1=编码器闭环, 0=开环步数 */
    AxisFeedbackReadFn feedback_read;  /* 反馈读取回调 (返回连续角度 deg) */
    void              *feedback_ctx;   /* 反馈上下文 */
    MotorPID           pid;            /* 位置 PID (输出 steps/s) */
    float              current_deg;    /* 最近反馈角度 (deg, 多圈累计) */
    float              current_mm;     /* 换算直线位置 (mm) */
    float              target_mm;      /* 目标直线位置 (mm) */
    float              zero_deg;       /* 上电零点 (SetFeedback 时的角度) */
    float              soft_limit_deg; /* 软限位半宽 (deg, 相对零点; 0=禁用) */
    uint32_t           period_ms;      /* 闭环控制周期 (ms) */
    uint32_t           last_update_tick;

    /* ── 开环 (步数) ── */
    int32_t       current_steps;
    int32_t       target_steps;
    int32_t       move_start_steps;
    uint32_t      move_start_hw;
    int8_t        dir_sign;
    uint32_t      velocity;
    uint32_t      timeout_ms;
    uint32_t      move_start_tick;

    /* ── 状态 ── */
    AxisState_t   state;
    uint8_t       enabled;
    uint8_t       fault;
    AxisResult_t  last_error;
} AxisController;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化并绑定步进执行器 (不使能电机; 默认开环) */
void AxisController_Init(AxisController *ax, const char *name, MotorStepper *stepper);

/**
 * @brief 绑定位置反馈, 切换到角度闭环模式 (读回调返回连续角度 deg)
 *        传 read=NULL 回到开环。建议在 Init 后、Enable 前调用。
 */
void AxisController_SetFeedback(AxisController *ax, AxisFeedbackReadFn read, void *ctx);

/**
 * @brief 自动零点校准 (预留)
 *        当前无回零硬件: 直接以"上电位置"为 0 点 (current_mm=0)。
 *        未来接入限位开关后在此实现回零动作。
 */
AxisResult_t AxisController_Home(AxisController *ax);

/** @brief 使能电机, DISABLED → IDLE */
AxisResult_t AxisController_Enable(AxisController *ax);

/** @brief 停止并禁止电机, → DISABLED */
void AxisController_Disable(AxisController *ax);

/**
 * @brief 闭环: 设定目标直线位置 (mm), 非阻塞 (PID 在 Update 中推进)
 *        无指令不会自动运动; 仅在调用本函数后开始运动
 */
AxisResult_t AxisController_SetTargetMM(AxisController *ax, float target_mm);

/**
 * @brief 开环: 相对移动 (非阻塞, 闭环模式下返回 INVALID_PARAM)
 * @param steps    有符号步数 (+ 正方向 / - 反方向), 0 非法
 * @param velocity 步率 steps/s
 */
AxisResult_t AxisController_MoveSteps(AxisController *ax, int32_t steps, uint32_t velocity);

/** @brief 开环: 绝对移动到指定步数位置 (非阻塞) */
AxisResult_t AxisController_MoveTo(AxisController *ax, int32_t absolute_steps, uint32_t velocity);

/**
 * @brief 闭环: 相对移动 delta_mm (不钳负值, 用于手动 cw/ccw 点动)
 *        与 SetTargetMM 的区别: 不受"工作区 >=0"约束。
 */
AxisResult_t AxisController_MoveRelMM(AxisController *ax, float delta_mm);

/** @brief 请求停止当前运动 (非阻塞, 立即断脉冲, 下一次 Update 确认) */
AxisResult_t AxisController_Stop(AxisController *ax);

/** @brief 清除 ERROR 故障, 回到 IDLE/DISABLED */
void AxisController_ClearFault(AxisController *ax);

/** @brief 周期调用 (主循环): 推进状态机/位置/超时 (禁止阻塞) */
void AxisController_Update(AxisController *ax);

/** @brief 查询当前状态 */
AxisState_t AxisController_GetState(const AxisController *ax);

/** @brief 查询完整状态快照 */
AxisStatus_t AxisController_GetStatus(const AxisController *ax);

#ifdef __cplusplus
}
#endif

#endif /* __AXIS_CONTROLLER_H */
