/**
 ******************************************************************************
 * @file    motor_stepper.h
 * @brief   步进脉冲引擎 — 硬件定时器产生 STEP, 非阻塞 (句柄化, 多实例)
 *
 * 职责:
 *   - 用 TIM 更新中断翻转 STEP 引脚 (TIM13/TIM14 无定时器通道, 用 ISR)
 *   - 由 velocity_ref 动态设置 STEP 频率 (steps/s)
 *   - 方向切换状态机 (停表→设 DIR→定时器首个周期提供 setup time→重启)
 *   - MoveSteps: 手动走 N 步 (非阻塞, 到目标自动停)
 *
 * 多电机: 每电机一个 MotorStepper 实例, 绑定各自的 TMC2209 句柄 + TIM 句柄.
 *   电机1 → TIM14 (TIM8_TRG_COM_TIM14_IRQn 向量)
 *   电机2 → TIM13 (TIM8_UP_TIM13_IRQn 向量)
 *   IRQHandler 由应用层分发 (见下方示例).
 *
 * 定时器配置 (CubeMX TIM13/TIM14):
 *   APB1 定时器时钟 84MHz, PSC/ARR 动态计算, 更新率 = 2 × 步率
 *   (每个更新翻转一次, 一个步进 = 高电平 + 低电平)
 *
 * 应用层示例 (main.c USER CODE 区):
 *   MotorStepper g_tilt, g_pan;
 *
 *   MotorStepper_Init(&g_tilt, &motor_tilt, &htim14);
 *   MotorStepper_Init(&g_pan,  &motor_pan,  &htim13);
 *
 *   void TIM8_TRG_COM_TIM14_IRQHandler(void) { MotorStepper_IRQHandler(&g_tilt); }
 *   void TIM8_UP_TIM13_IRQHandler(void)       { MotorStepper_IRQHandler(&g_pan); }
 ******************************************************************************
 */
#ifndef __MOTOR_STEPPER_H
#define __MOTOR_STEPPER_H

#include "tmc2209.h"
#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  脉冲引擎实例                                                              */
/* ========================================================================= */
typedef struct {
    TMC2209_HandleTypeDef *motor;        /* 绑定的 TMC2209 句柄 */
    TIM_HandleTypeDef     *htim;         /* STEP 定时器 (TIM13/TIM14) */
    uint8_t  direction;                  /* TMC2209_Dir 当前方向 */
    uint8_t  phase;                      /* 0→STEP 高(步进), 1→STEP 低 */
    float    step_rate;                  /* 当前步率 steps/s */
    volatile uint32_t step_count;        /* 累计步数 (ISR 递增) */
    volatile uint32_t target_steps;      /* >0 = 走到该值停 */
} MotorStepper;

/** @brief 初始化 STEP 定时器 + NVIC (在 TMC2209_Init 之后调用) */
void MotorStepper_Init(MotorStepper *s, TMC2209_HandleTypeDef *motor, TIM_HandleTypeDef *htim);

/**
 * @brief 设置 STEP 速度并启动/更新 (非阻塞)
 * @param steps_per_sec 正=CCW(角度增大), 负=CW(角度减小), 0=停止
 *                     速度改变/方向切换内部处理, 不阻塞
 */
void MotorStepper_SetVelocity(MotorStepper *s, float steps_per_sec);

/**
 * @brief 手动走 N 步 (非阻塞, 用当前 DIR 方向)
 * @param steps 步数
 * @param rate  步率 (steps/s)
 *              到目标步数后 ISR 自动停表
 */
void MotorStepper_MoveSteps(MotorStepper *s, uint32_t steps, float rate);

/** @brief 停止 STEP (定时器禁用, STEP 引脚拉低) */
void MotorStepper_Stop(MotorStepper *s);

/** @brief 累计步数 (ISR 递增) */
uint32_t MotorStepper_GetSteps(const MotorStepper *s);

/** @brief 定时器更新中断服务 (由对应 IRQHandler 调用, 极短) */
void MotorStepper_IRQHandler(MotorStepper *s);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_STEPPER_H */