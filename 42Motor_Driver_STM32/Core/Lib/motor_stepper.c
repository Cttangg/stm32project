/**
 ******************************************************************************
 * @file    motor_stepper.c
 * @brief   步进脉冲引擎 — TIM14 硬件定时器产生 STEP, 非阻塞
 *
 * 原理:
 *   - 每个 TIM14 更新中断翻转一次 STEP 引脚
 *   - 一个步进 = 一次高电平 + 一次低电平 = 2 个更新周期
 *   - 步率 steps/s → 更新率 = 2×steps/s → 动态算 PSC/ARR
 *   - 方向切换: 停表 → 设 DIR → 定时器首个周期(≥1/2rate)天然满足 DIR setup time → 重启
 ******************************************************************************
 */
#include "motor_stepper.h"

/* TIM14 定时器时钟 (APB1=42MHz, ×2 = 84MHz) */
#define STEP_TIM_CLK    84000000UL

static TMC2209_HandleTypeDef *g_motor = 0;
static TIM_HandleTypeDef     *g_htim  = 0;
static uint8_t  g_direction = 0;              /* TMC2209_Dir */
static uint8_t  g_phase     = 0;              /* 0→STEP 高(步进), 1→STEP 低 */
static float    g_step_rate = 0.0f;           /* 当前步率 steps/s */
static volatile uint32_t g_step_count   = 0;  /* 累计步数 */
static volatile uint32_t g_target_steps = 0;  /* >0 = 走到该值停 */

static void step_pin_set(void) {
    HAL_GPIO_WritePin(g_motor->pins.step.port, g_motor->pins.step.pin, GPIO_PIN_SET);
}
static void step_pin_reset(void) {
    HAL_GPIO_WritePin(g_motor->pins.step.port, g_motor->pins.step.pin, GPIO_PIN_RESET);
}

/* 计算 PSC/ARR, 使更新率 = 2×rate; 停表重配后重启 */
static void stepper_set_rate(float rate) {
    uint64_t d;
    uint32_t psc, arr;

    if (rate <= 0.0f) { MotorStepper_Stop(); return; }
    if (rate > 50000.0f) rate = 50000.0f;      /* 保护上限 */

    /* (PSC+1)(ARR+1) = 时钟 / (2×rate) */
    d = (uint64_t)((double)STEP_TIM_CLK / (2.0 * (double)rate));
    if (d < 1) d = 1;
    psc = (uint32_t)((d + 65535) / 65536) - 1; /* 保证 ARR ≤ 65535 */
    if (psc > 65535) psc = 65535;
    arr = (uint32_t)(d / (psc + 1)) - 1;
    if (arr > 65535) arr = 65535;

    /* 停表 → 写 PSC/ARR → 生成更新装载 → 清标志 → 重启 */
    __HAL_TIM_DISABLE(g_htim);
    __HAL_TIM_SET_PRESCALER(g_htim, psc);
    __HAL_TIM_SET_AUTORELOAD(g_htim, arr);
    __HAL_TIM_SET_COUNTER(g_htim, 0);
    g_htim->Instance->EGR = TIM_EGR_UG;      /* 生成更新事件, 装载 PSC/ARR */
    __HAL_TIM_CLEAR_FLAG(g_htim, TIM_FLAG_UPDATE);
    g_phase = 0;
    __HAL_TIM_ENABLE(g_htim);
    g_step_rate = rate;
}

void MotorStepper_Init(TMC2209_HandleTypeDef *motor, TIM_HandleTypeDef *htim) {
    g_motor     = motor;
    g_htim      = htim;
    g_direction = (uint8_t)motor->dir;
    g_phase     = 0;
    g_step_rate = 0.0f;
    g_step_count   = 0;
    g_target_steps = 0;

    /* STEP 引脚初始低 (idle) */
    step_pin_reset();

    /* 使能 TIM14 更新中断 + NVIC (优先级低于 DMA0/UART1, 避免干扰收发) */
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    HAL_NVIC_SetPriority(TIM8_TRG_COM_TIM14_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM8_TRG_COM_TIM14_IRQn);

    MotorStepper_Stop();
}

void MotorStepper_SetVelocity(float steps_per_sec) {
    TMC2209_Dir dir;
    float rate;

    if (steps_per_sec > 0.0f)      { dir = TMC2209_DIR_CCW; rate =  steps_per_sec; }
    else if (steps_per_sec < 0.0f) { dir = TMC2209_DIR_CW;  rate = -steps_per_sec; }
    else                           { MotorStepper_Stop(); return; }

    /* 方向切换: 停表(ISR 不再触发) → 设 DIR → 重启时首个周期天然提供 DIR setup time */
    if (g_direction != (uint8_t)dir) {
        __HAL_TIM_DISABLE(g_htim);
        __HAL_TIM_CLEAR_FLAG(g_htim, TIM_FLAG_UPDATE);
        g_direction = (uint8_t)dir;
        TMC2209_SetDirection(g_motor, dir);
    }
    stepper_set_rate(rate);
}

void MotorStepper_MoveSteps(uint32_t steps, float rate) {
    if (steps == 0) return;
    g_target_steps = g_step_count + steps;
    stepper_set_rate(rate);   /* 用当前 DIR, 不改变方向 */
}

void MotorStepper_Stop(void) {
    if (g_htim) {
        __HAL_TIM_DISABLE(g_htim);
        __HAL_TIM_CLEAR_FLAG(g_htim, TIM_FLAG_UPDATE);
    }
    g_phase = 0;
    g_step_rate = 0.0f;
    g_target_steps = 0;
    step_pin_reset();          /* STEP 拉低 idle */
}

uint32_t MotorStepper_GetSteps(void) {
    return g_step_count;
}

/* TIM14 更新中断 (与 TIM8 TRG/COM 共用向量). 极短: 翻转引脚 + 计数 */
void MotorStepper_IRQHandler(void) {
    if (__HAL_TIM_GET_FLAG(g_htim, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(g_htim, TIM_FLAG_UPDATE);

        if (g_phase) {
            step_pin_reset();                    /* STEP 低 */
        } else {
            step_pin_set();                      /* STEP 高 = 上升沿 = 一步 */
            g_step_count++;
            if (g_target_steps && g_step_count >= g_target_steps) {
                g_target_steps = 0;
                __HAL_TIM_DISABLE(g_htim);       /* 到达目标, 自动停 */
            }
        }
        g_phase ^= 1;
    }
}

/* 启动向量引用此名 (TIM14 与 TIM8 TRG/COM 共用中断线) */
void TIM8_TRG_COM_TIM14_IRQHandler(void) {
    MotorStepper_IRQHandler();
}
