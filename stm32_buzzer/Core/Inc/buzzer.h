/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   发声模块: 用定时器 PWM 驱动无源蜂鸣器 / 扬声器.
  *          与具体定时器解耦, 通过 Buzzer_Init() 绑定句柄.
  ******************************************************************************
  */
#ifndef __BUZZER_H
#define __BUZZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"

/**
  * @brief  绑定 PWM 定时器.
  * @param  htim           已由 CubeMX 初始化好的定时器句柄 (如 &htim3)
  * @param  channel        输出通道 (如 TIM_CHANNEL_2)
  * @param  counter_clk_hz 定时器计数时钟 (分频后), 用于计算 ARR.
  *                        例: 100MHz / (PSC+1) = 1MHz 时传 1000000
  */
void Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t counter_clk_hz);

/* 输出指定频率方波 (50% 占空比), freq_hz = 0 停止 */
void Buzzer_SetTone(uint32_t freq_hz);

/* 输出指定频率与占空比 (duty_percent 0..100) 的 PWM, freq_hz = 0 停止 */
void Buzzer_SetToneDuty(uint32_t freq_hz, uint8_t duty_percent);

/* 停止输出 */
void Buzzer_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H */
