/**
 ******************************************************************************
 * @file    delay.h
 * @brief   STM32 延时功能接口 — 基于 DWT 内核周期计数器的微秒延时
 *
 * 设计说明:
 *   - delay_us(): 基于 Cortex-M4 内核 DWT->CYCCNT 周期计数器忙等待实现,
 *     不受 SysTick/中断抢占影响, 适合触摸采样等需要稳定短延时的场合
 *   - delay_ms(): 直接包装 HAL_Delay() (基于 SysTick)
 *
 * @note 使用 delay_us() 前必须先调用 delay_init()
 ******************************************************************************
 */
#ifndef __DELAY_H
#define __DELAY_H

#include "stm32f4xx_hal.h"

/**
 * @brief  初始化 DWT 延时计数 (使能 CYCCNT)
 * @note   系统上电后调用一次即可, 重复调用自动忽略.
 *         必须在 SystemClock_Config() 之后调用 (依赖 SystemCoreClock)
 */
void delay_init(void);

/**
 * @brief  微秒级延时 (阻塞, 忙等待)
 * @param  nus: 延时微秒数
 * @note   基于 DWT->CYCCNT, 精度不受 SysTick 中断影响
 */
void delay_us(uint32_t nus);

/**
 * @brief  毫秒级延时 (阻塞)
 * @param  nms: 延时毫秒数
 * @note   直接包装 HAL_Delay()
 */
void delay_ms(uint16_t nms);

#endif /* __DELAY_H */
