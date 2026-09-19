/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   发声模块实现.
  ******************************************************************************
  */
#include "buzzer.h"
#include <stddef.h>

static TIM_HandleTypeDef *s_htim = NULL;
static uint32_t           s_channel = 0U;
static uint32_t           s_counter_clk_hz = 1000000U;

void Buzzer_Init(TIM_HandleTypeDef *htim, uint32_t channel, uint32_t counter_clk_hz)
{
  s_htim = htim;
  s_channel = channel;
  s_counter_clk_hz = (counter_clk_hz != 0U) ? counter_clk_hz : 1000000U;
}

void Buzzer_SetTone(uint32_t freq_hz)
{
  Buzzer_SetToneDuty(freq_hz, 50U);
}

void Buzzer_SetToneDuty(uint32_t freq_hz, uint8_t duty_percent)
{
  uint32_t arr;
  uint32_t ccr;

  if (s_htim == NULL)
  {
    return;
  }

  if (freq_hz == 0U)
  {
    Buzzer_Stop();
    return;
  }

  /* PWM 频率 = counter_clk / (ARR+1) */
  arr = s_counter_clk_hz / freq_hz;
  if (arr == 0U)
  {
    arr = 1U;
  }
  arr -= 1U;

  if (duty_percent > 100U)
  {
    duty_percent = 100U;
  }
  ccr = ((arr + 1U) * duty_percent) / 100U;

  __HAL_TIM_SET_AUTORELOAD(s_htim, arr);
  __HAL_TIM_SET_COMPARE(s_htim, s_channel, ccr);
  HAL_TIM_PWM_Start(s_htim, s_channel);
}

void Buzzer_Stop(void)
{
  if (s_htim != NULL)
  {
    HAL_TIM_PWM_Stop(s_htim, s_channel);
  }
}
