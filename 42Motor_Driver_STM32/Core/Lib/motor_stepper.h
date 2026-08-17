/**
 ******************************************************************************
 * @file    motor_stepper.h
 * @brief   步进脉冲引擎 — 硬件定时器产生 STEP, 非阻塞
 *
 * 职责:
 *   - 用 TIM14 Update 中断翻转 STEP 引脚 (PC13 无定时器通道, 用 ISR)
 *   - 由 velocity_ref 动态设置 STEP 频率 (steps/s)
 *   - 方向切换状态机 (停表→设 DIR→定时器首个周期提供 setup time→重启)
 *   - MoveSteps: 手动走 N 步 (非阻塞, 到目标自动停)
 *
 * 定时器配置 (CubeMX TIM14):
 *   APB1 定时器时钟 84MHz, PSC/ARR 动态计算, 更新率 = 2 × 步率
 *   (每个更新翻转一次, 一个步进 = 高电平 + 低电平)
 ******************************************************************************
 */
#ifndef __MOTOR_STEPPER_H
#define __MOTOR_STEPPER_H

#include "tmc2209.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 STEP 定时器 + NVIC (在 TMC2209_Init 之后调用) */
void MotorStepper_Init(TMC2209_HandleTypeDef *motor, TIM_HandleTypeDef *htim);

/**
 * @brief 设置 STEP 速度并启动/更新 (非阻塞)
 * @param steps_per_sec 正=CCW(角度增大), 负=CW(角度减小), 0=停止
 *                     速度改变/方向切换内部处理, 不阻塞
 */
void MotorStepper_SetVelocity(float steps_per_sec);

/**
 * @brief 手动走 N 步 (非阻塞, 用当前 DIR 方向)
 * @param steps 步数
 * @param rate  步率 (steps/s)
 *              到目标步数后 ISR 自动停表
 */
void MotorStepper_MoveSteps(uint32_t steps, float rate);

/** @brief 停止 STEP (定时器禁用, STEP 引脚拉低) */
void MotorStepper_Stop(void);

/** @brief 累计步数 (ISR 递增) */
uint32_t MotorStepper_GetSteps(void);

/** @brief TIM8_TRG_COM_TIM14 更新中断服务 (由 ISR 调用, 极短) */
void MotorStepper_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_STEPPER_H */
