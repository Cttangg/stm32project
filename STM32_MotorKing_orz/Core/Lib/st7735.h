/**
 ******************************************************************************
 * @file    st7735.h
 * @brief   ST7735 SPI TFT-LCD 驱动 — 预留骨架 (待实现)
 *
 * 硬件: 1.8 寸 SPI 彩屏, ST7735 控制器
 *   接口: SPI1 (SCK=PA5, MOSI=PA7) + GPIO 控制线
 *   LCD_CS  = PB1   片选 (低有效)
 *   LCD_DC  = PC4   数据/命令选择 (1=数据, 0=命令)
 *   LCD_RES = PC5   复位 (低有效)
 *   LCD_BL  = PB0   背光 (高有效)
 *
 * 预留: 句柄化多实例, 用法与 Core/Lib 其他驱动一致.
 * 待实现: 初始化序列 / 清屏 / 画点 / 填充 / 文字 / 图片.
 ******************************************************************************
 */
#ifndef __ST7735_H
#define __ST7735_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  引脚与句柄定义                                                             */
/* ========================================================================= */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} ST7735_PinDef;

typedef struct {
    SPI_HandleTypeDef *hspi;      /* 绑定的 SPI 句柄 (SPI1) */
    ST7735_PinDef cs;             /* 片选   (低有效) */
    ST7735_PinDef dc;             /* 数据/命令 */
    ST7735_PinDef res;            /* 复位   (低有效) */
    ST7735_PinDef bl;             /* 背光   (高有效) */
    uint16_t width;               /* 面板宽 (默认 128) */
    uint16_t height;              /* 面板高 (默认 160) */
} ST7735_HandleTypeDef;

/* ========================================================================= */
/*  公开 API (预留骨架, 待实现)                                                */
/* ========================================================================= */

/** @brief 绑定 SPI + 引脚并初始化 (上电复位 + 初始化序列) */
void ST7735_Init(ST7735_HandleTypeDef *h, SPI_HandleTypeDef *hspi,
                 const ST7735_PinDef *cs, const ST7735_PinDef *dc,
                 const ST7735_PinDef *res, const ST7735_PinDef *bl);

/** @brief 背光控制 (1=亮 0=灭) */
void ST7735_SetBacklight(ST7735_HandleTypeDef *h, uint8_t on);

/** @brief 清屏 */
void ST7735_FillScreen(ST7735_HandleTypeDef *h, uint16_t color);

/** @brief 画点 */
void ST7735_DrawPixel(ST7735_HandleTypeDef *h, uint16_t x, uint16_t y, uint16_t color);

#ifdef __cplusplus
}
#endif

#endif /* __ST7735_H */