/**
 ******************************************************************************
 * @file    delay.c
 * @brief   STM32 延时功能实现 — 基于 DWT 内核周期计数器
 *
 * 实现原理:
 *   - delay_init(): 使能 CoreDebug 调试跟踪 (TRCENA) 并启动 DWT->CYCCNT
 *   - delay_us(n): 计算 n * (SystemCoreClock / 1MHz) 个 CPU 周期,
 *     忙等待至 CYCCNT 差值达到目标 (无符号减法自动处理计数器回绕)
 *
 * @note  SystemCoreClock 在 SystemClock_Config() 之后才有正确值,
 *        因此 delay_init() 应在主时钟配置完成后调用
 ******************************************************************************
 */
#include "delay.h"

/** DWT 周期计数是否已初始化 (0=未初始化, 1=已初始化) */
static uint8_t g_delay_dwt_ready = 0;

/**
 * @brief  初始化 DWT 延时计数
 * @note   使能 TRCENA 调试跟踪后启动 CYCCNT, 只执行一次
 */
void delay_init(void)
{
    if (g_delay_dwt_ready) return;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能调试跟踪 */
    DWT->CYCCNT = 0;                                  /* 周期计数器清零 */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;              /* 启动周期计数 */
    g_delay_dwt_ready = 1;
}

/**
 * @brief  微秒级延时 (忙等待)
 * @param  nus: 延时微秒数
 */
void delay_us(uint32_t nus)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = nus * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

/**
 * @brief  毫秒级延时
 * @param  nms: 延时毫秒数
 */
void delay_ms(uint16_t nms)
{
    HAL_Delay(nms);
}
