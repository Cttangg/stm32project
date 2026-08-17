#ifndef __TOUCH_H
#define __TOUCH_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "lcd.h"

/* ===================== 类型别名 (兼容原驱动) ===================== */
typedef int32_t s32;

/* 触摸屏状态 */
#define TP_PRES_DOWN 0x80
#define TP_CATH_PRES 0x40

/* 触摸控制器 */
typedef struct
{
    u8  (*init)(void);
    u8  (*scan)(u8);
    void (*adjust)(void);
    u16 x[5];
    u16 y[5];
    u8  sta;
    float xfac;
    float yfac;
    short xoff;
    short yoff;
    u8 touchtype;
} _m_tp_dev;

extern _m_tp_dev tp_dev;

/* ===================== 电阻触摸引脚 (软件模拟 SPI) =====================
   T_PEN=PC5  T_MISO=PB14  T_MOSI=PB15  T_SCK=PB13  T_CS=PB12  */
#define TP_PEN_GPIO     GPIOC
#define TP_PEN_PIN      GPIO_PIN_5
#define TP_MISO_GPIO    GPIOB
#define TP_MISO_PIN     GPIO_PIN_14
#define TP_MOSI_GPIO    GPIOB
#define TP_MOSI_PIN     GPIO_PIN_15
#define TP_SCK_GPIO     GPIOB
#define TP_SCK_PIN      GPIO_PIN_13
#define TP_CS_GPIO      GPIOB
#define TP_CS_PIN       GPIO_PIN_12

#define TP_PEN      HAL_GPIO_ReadPin(TP_PEN_GPIO, TP_PEN_PIN)
#define TP_DOUT     HAL_GPIO_ReadPin(TP_MISO_GPIO, TP_MISO_PIN)
#define TP_TDIN(v)  HAL_GPIO_WritePin(TP_MOSI_GPIO, TP_MOSI_PIN, (v))
#define TP_TCLK(v)  HAL_GPIO_WritePin(TP_SCK_GPIO, TP_SCK_PIN, (v))
#define TP_TCS(v)   HAL_GPIO_WritePin(TP_CS_GPIO, TP_CS_PIN, (v))

/* 触摸功能函数 */
void TP_Write_Byte(u8 num);
u16 TP_Read_AD(u8 CMD);
u16 TP_Read_XOY(u8 xy);
u8  TP_Read_XY(u16 *x, u16 *y);
u8  TP_Read_XY2(u16 *x, u16 *y);
void TP_Drow_Touch_Point(u16 x, u16 y, u16 color);
void TP_Draw_Big_Point(u16 x, u16 y, u16 color);
void TP_Save_Adjdata(void);
u8  TP_Get_Adjdata(void);
void TP_Adjust(void);
void TP_Adj_Info_Show(u16 x0, u16 y0, u16 x1, u16 y1, u16 x2, u16 y2,
                      u16 x3, u16 y3, u16 fac);
u8  TP_Scan(u8 tp);
u8  TP_Init(void);

#endif /* __TOUCH_H */
