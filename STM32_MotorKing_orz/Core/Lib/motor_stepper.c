/**
 ******************************************************************************
 * @file    motor_stepper.c
 * @brief   步进脉冲引擎 — 硬件定时器产生 STEP, 非阻塞 (句柄化, 多实例)
 *
 * 原理:
 *   - 每个 TIM 更新中断翻转一次 STEP 引脚
 *   - 一个步进 = 一次高电平 + 一次低电平 = 2 个更新周期
 *   - 步率 steps/s → 更新率 = 2×steps/s → 动态算 PSC/ARR
 *   - 方向切换: 停表 → 设 DIR → 定时器首个周期(≥1/2rate)天然满足 DIR setup time → 重启
 ******************************************************************************
 */
#include "motor_stepper.h"

/* TIM13/TIM14 定时器时钟 (APB1=42MHz, ×2 = 84MHz) */
#define STEP_TIM_CLK    84000000UL

/* ========================================================================= */
/*  内部辅助                                                                    */
/* ========================================================================= */
static void step_pin_set(MotorStepper *s) {
    HAL_GPIO_WritePin(s->motor->pins.step.port, s->motor->pins.step.pin, GPIO_PIN_SET);
}
static void step_pin_reset(MotorStepper *s) {
    HAL_GPIO_WritePin(s->motor->pins.step.port, s->motor->pins.step.pin, GPIO_PIN_RESET);
}

/* 计算 PSC/ARR, 使更新率 = 2×rate.
 * 定时器运行中只改影子寄存器(PSC 自动缓冲, ARR 开 ARPE), 不重置计数器、
 * 不产生多余更新事件 —— 否则运动层每周期更新速度时会把脉冲序列不断清零. */
static void stepper_set_rate(MotorStepper *s, float rate) {
    TIM_HandleTypeDef *htim = s->htim;
    uint64_t d;
    uint32_t psc, arr;
    uint8_t  running;

    if (rate <= 0.0f) { MotorStepper_Stop(s); return; }
    if (rate > 50000.0f) rate = 50000.0f;      /* 保护上限 */

    /* (PSC+1)(ARR+1) = 时钟 / (2×rate) */
    d = (uint64_t)((double)STEP_TIM_CLK / (2.0 * (double)rate));
    if (d < 1) d = 1;
    psc = (uint32_t)((d + 65535) / 65536) - 1; /* 保证 ARR ≤ 65535 */
    if (psc > 65535) psc = 65535;
    arr = (uint32_t)(d / (psc + 1)) - 1;
    if (arr > 65535) arr = 65535;

    running = (htim->Instance->CR1 & TIM_CR1_CEN) ? 1U : 0U;

    if (!running) {
        /* 启动: 停表 → 写 PSC/ARR → 更新装载 → 清标志 → 重启 */
        __HAL_TIM_DISABLE(htim);
        __HAL_TIM_SET_PRESCALER(htim, psc);
        __HAL_TIM_SET_AUTORELOAD(htim, arr);
        __HAL_TIM_SET_COUNTER(htim, 0);
        htim->Instance->EGR = TIM_EGR_UG;      /* 生成更新事件, 装载 PSC/ARR */
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
        s->phase = 0;
        __HAL_TIM_ENABLE(htim);
    } else {
        /* 运行中: 仅更新影子寄存器, 当前周期结束后自然生效 */
        htim->Instance->CR1 |= TIM_CR1_ARPE;   /* 使能 ARR 预装载 */
        __HAL_TIM_SET_PRESCALER(htim, psc);
        __HAL_TIM_SET_AUTORELOAD(htim, arr);
    }
    s->step_rate = rate;
}

/* ========================================================================= */
/*  公开 API                                                                    */
/* ========================================================================= */
void MotorStepper_Init(MotorStepper *s, TMC2209_HandleTypeDef *motor, TIM_HandleTypeDef *htim) {
    s->motor     = motor;
    s->htim      = htim;
    s->direction = (uint8_t)motor->dir;
    s->phase     = 0;
    s->step_rate = 0.0f;
    s->step_count   = 0;
    s->target_steps = 0;

    /* STEP 引脚初始低 (idle) */
    step_pin_reset(s);

    /* 使能 TIM 更新中断 (NVIC 优先级/使能由 CubeMX MspInit 统一管理) */
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);

    MotorStepper_Stop(s);
}

void MotorStepper_SetVelocity(MotorStepper *s, float steps_per_sec) {
    TMC2209_Dir dir;
    float rate;

    if (steps_per_sec > 0.0f)      { dir = TMC2209_DIR_CCW; rate =  steps_per_sec; }
    else if (steps_per_sec < 0.0f) { dir = TMC2209_DIR_CW;  rate = -steps_per_sec; }
    else                           { MotorStepper_Stop(s); return; }

    /* 方向切换: 停表(ISR 不再触发) → 设 DIR → 重启时首个周期天然提供 DIR setup time */
    if (s->direction != (uint8_t)dir) {
        __HAL_TIM_DISABLE(s->htim);
        __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);
        s->direction = (uint8_t)dir;
        TMC2209_SetDirection(s->motor, dir);
    }
    stepper_set_rate(s, rate);
}

void MotorStepper_MoveSteps(MotorStepper *s, uint32_t steps, float rate) {
    if (steps == 0) return;
    s->target_steps = s->step_count + steps;
    stepper_set_rate(s, rate);   /* 用当前 DIR, 不改变方向 */
}

void MotorStepper_Stop(MotorStepper *s) {
    if (s->htim) {
        __HAL_TIM_DISABLE(s->htim);
        __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);
    }
    s->phase = 0;
    s->step_rate = 0.0f;
    s->target_steps = 0;
    step_pin_reset(s);           /* STEP 拉低 idle */
}

uint32_t MotorStepper_GetSteps(const MotorStepper *s) {
    return s->step_count;
}

/* TIM 更新中断服务. 极短: 翻转引脚 + 计数.
   IRQHandler 由应用层定义并分发 (TIM14 与 TIM8 TRG/COM 共用向量,
   TIM13 与 TIM8 UP 共用向量). */
void MotorStepper_IRQHandler(MotorStepper *s) {
    if (__HAL_TIM_GET_FLAG(s->htim, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(s->htim, TIM_FLAG_UPDATE);

        if (s->phase) {
            step_pin_reset(s);                   /* STEP 低 */
        } else {
            step_pin_set(s);                     /* STEP 高 = 上升沿 = 一步 */
            s->step_count++;
            if (s->target_steps && s->step_count >= s->target_steps) {
                s->target_steps = 0;
                __HAL_TIM_DISABLE(s->htim);      /* 到达目标, 自动停 */
            }
        }
        s->phase ^= 1;
    }
}