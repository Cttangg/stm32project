#include "delay.h"

static uint8_t g_delay_dwt_ready = 0;

void delay_init(void)
{
    if (g_delay_dwt_ready) return;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    g_delay_dwt_ready = 1;
}

void delay_us(uint32_t nus)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = nus * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

void delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
}
