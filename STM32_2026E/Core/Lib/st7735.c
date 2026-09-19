/**
 ******************************************************************************
 * @file    st7735.c
 * @brief   ST7735 SPI TFT-LCD 驱动 — 预留骨架 (待实现)
 *
 * 硬件接线见 st7735.h. 本文件为占位实现, 保证工程可编译;
 * 完整驱动 (初始化序列/清屏/画点/填充) 待实现.
 ******************************************************************************
 */
#include "st7735.h"

void ST7735_Init(ST7735_HandleTypeDef *h, SPI_HandleTypeDef *hspi,
                 const ST7735_PinDef *cs, const ST7735_PinDef *dc,
                 const ST7735_PinDef *res, const ST7735_PinDef *bl)
{
    h->hspi   = hspi;
    h->cs     = *cs;
    h->dc     = *dc;
    h->res    = *res;
    h->bl     = *bl;
    h->width  = 128;
    h->height = 160;

    /* TODO: 上电复位时序 + ST7735 初始化序列 */
}

void ST7735_SetBacklight(ST7735_HandleTypeDef *h, uint8_t on)
{
    HAL_GPIO_WritePin(h->bl.port, h->bl.pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ST7735_FillScreen(ST7735_HandleTypeDef *h, uint16_t color)
{
    (void)h; (void)color;
    /* TODO: 填充全屏 */
}

void ST7735_DrawPixel(ST7735_HandleTypeDef *h, uint16_t x, uint16_t y, uint16_t color)
{
    (void)h; (void)x; (void)y; (void)color;
    /* TODO: 画点 */
}