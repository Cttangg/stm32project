/**
 ******************************************************************************
 * @file    self_test.c
 * @brief   驱动自检实现 — LCD 色条/读点校验, 触摸有效点检测, 串口回环
 *
 * 依赖:
 *   - LCD_SelfTest(): lcd 模块 (LCD_ReadPoint 需驱动 IC 支持读 GRAM)
 *   - TP_SelfTest():  touch 模块 (tp_dev/TP_Scan)
 *   - UART_SelfTest(): uart 模块 + 应用注册的回显帧
 ******************************************************************************
 */
#include "self_test.h"
#include "lcd.h"
#include "touch.h"
#include "delay.h"
#include <string.h>

/** 自检状态文本经串口输出 */
static void st_print(UART_Port port, const char *s)
{
    uint16_t w;
    UART_Send(port, (const uint8_t *)s, (uint16_t)strlen(s), &w);
}

/** 颜色比较 (忽略每通道 1 LSB, 兼容驱动 IC 读 GRAM 的低位偏差) */
static uint8_t st_color_eq(u16 got, u16 want)
{
    return (got & ~0x0821U) == (want & ~0x0821U);
}

uint8_t LCD_SelfTest(void)
{
    u16 i, x;
    u16 w = lcddev.width;
    u16 h = lcddev.height;
    u16 bar_w = w / 5;
    static const u16 bars[5] = {RED, GREEN, BLUE, WHITE, BLACK};
    uint8_t pass = 1;

    /* 五色条 */
    for (i = 0; i < 5; i++) {
        LCD_Fill((u16)(i * bar_w), 0, (u16)((i + 1) * bar_w - 1), h - 1, bars[i]);
    }
    /* 边框线 */
    POINT_COLOR = YELLOW;
    LCD_DrawRectangle(1, 1, w - 2, h - 2);
    /* 读点回读校验: 每色条中部取 3 点 */
    for (i = 0; i < 5; i++) {
        x = (u16)(i * bar_w + bar_w / 2);
        if (!st_color_eq(LCD_ReadPoint(x, h / 2), bars[i])) pass = 0;
        if (!st_color_eq(LCD_ReadPoint(x, h / 4), bars[i])) pass = 0;
        if (!st_color_eq(LCD_ReadPoint(x, 3 * h / 4), bars[i])) pass = 0;
    }
    return pass;
}

uint8_t TP_SelfTest(void)
{
    uint32_t start = HAL_GetTick();
    u16 x, y;

    POINT_COLOR = RED;
    LCD_ShowString(20, 150, 200, 16, 16, "TP SelfTest: Touch!");
    while ((HAL_GetTick() - start) < 5000) {
        TP_Scan(0);
        x = tp_dev.x[0];
        y = tp_dev.y[0];
        if ((tp_dev.sta & TP_PRES_DOWN) && (x != 0xFFFF) &&
            (x < lcddev.width) && (y < lcddev.height)) {
            return 1;   /* 捕获到有效按压 */
        }
        delay_ms(5);
    }
    return 0;
}

uint8_t UART_SelfTest(UART_Port port)
{
    static const uint8_t test_frame[] = "ST-ECHO\r\n";
    uint8_t rx[16];
    uint8_t n = 0;
    uint32_t start = HAL_GetTick();
    uint16_t w;

    UART_Open(port);
    UART_Send(port, test_frame, (uint16_t)(sizeof(test_frame) - 1), &w);
    while ((HAL_GetTick() - start) < 1000) {
        UART_Task();
        if (UART_Available(port)) {
            UART_Read(port, &rx[n], 1);
            n++;
        }
        if (n >= sizeof(test_frame) - 1) break;
    }
    if (n != sizeof(test_frame) - 1) return 0;
    return (memcmp(rx, test_frame, sizeof(test_frame) - 1) == 0);
}

uint8_t LIB_SelfTest(UART_Port port)
{
    uint8_t pass = 1;

    st_print(port, "\r\n=== LIB SelfTest v1.3.0 ===\r\n");
    st_print(port, "[LCD] ");
    if (LCD_SelfTest()) st_print(port, "PASS\r\n");
    else { st_print(port, "FAIL\r\n"); pass = 0; }
    st_print(port, "[TP]  ");
    if (TP_SelfTest()) st_print(port, "PASS\r\n");
    else { st_print(port, "FAIL\r\n"); pass = 0; }
    st_print(port, "[UART] ");
    if (UART_SelfTest(port)) st_print(port, "PASS\r\n");
    else { st_print(port, "FAIL\r\n"); pass = 0; }
    st_print(port, pass ? "=== ALL PASS ===\r\n" : "=== HAS FAILURE ===\r\n");
    return pass;
}
