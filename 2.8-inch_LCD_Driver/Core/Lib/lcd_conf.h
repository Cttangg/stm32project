/**
 ******************************************************************************
 * @file    lcd_conf.h
 * @brief   LCD 驱动板级配置 — 移植到新板/新屏时只需修改本文件
 *
 * 使用说明:
 *   1. 本文件由 lcd.h 自动包含, 无需手动 include
 *   2. 修改本文件后, 若与 lcd.h 版本不匹配会触发编译期 #error
 *   3. 所有宏取值均与迁移前的驱动定义保持一致 (仅位置变化)
 *
 * 移植清单:
 *   - 背光引脚:      LCD_BL_GPIO_PORT / LCD_BL_PIN
 *   - FSMC 引脚组:   LCD_FSMC_D_* / LCD_FSMC_E_*
 *   - FSMC 时序:     LCD_FSMC_* (总线速度变化时需调整)
 *   - 默认显示:      LCD_DEFAULT_DIR / LCD_DFT_SCAN_DIR / LCD_PANEL_W/H
 ******************************************************************************
 */
#ifndef __LCD_CONF_H
#define __LCD_CONF_H

/** 配置版本号: 与 lcd.h 中 LCD_CONF_VER_EXPECT 匹配, 不一致时编译报错 */
#define LCD_CONF_VER        0x0100

/* ========================================================================= */
/*  背光引脚                                                                  */
/* ========================================================================= */
#define LCD_BL_GPIO_PORT    GPIOB
#define LCD_BL_PIN          GPIO_PIN_1

/* ========================================================================= */
/*  FSMC 引脚组 (GPIO AF12)                                                   */
/* ========================================================================= */
/** 数据总线 & 控制线 (PD 组) */
#define LCD_FSMC_D_GPIO_PORT    GPIOD
#define LCD_FSMC_D_GPIO_PINS    (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5 \
                                 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 \
                                 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15)
/** 数据线 (PE7~PE15) */
#define LCD_FSMC_E_GPIO_PORT    GPIOE
#define LCD_FSMC_E_GPIO_PINS    (GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 \
                                 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 \
                                 | GPIO_PIN_14 | GPIO_PIN_15)

/* ========================================================================= */
/*  FSMC 读写时序 (单位: FSMC 时钟周期, 总线速度变化时调整)                    */
/* ========================================================================= */
#define LCD_FSMC_READ_ADDR_SETUP    0x0F    /* 读: 地址建立时间 */
#define LCD_FSMC_READ_DATA_SETUP    60      /* 读: 数据建立时间 */
#define LCD_FSMC_WRITE_ADDR_SETUP   9       /* 写: 地址建立时间 */
#define LCD_FSMC_WRITE_DATA_SETUP   8       /* 写: 数据建立时间 */
#define LCD_FSMC_ADDR_HOLD          0x00    /* 地址保持时间 */
#define LCD_FSMC_BUS_TURNAROUND     0x00    /* 总线周转时间 */
#define LCD_FSMC_CLK_DIVISION       2       /* 时钟分频 */
#define LCD_FSMC_DATA_LATENCY       2       /* 数据延迟 */
#define LCD_FSMC_ACCESS_MODE        FSMC_ACCESS_MODE_A

/**
 * GRAM 批量写方式:
 *   0 = 16 位逐像素写 (实机验证稳定, 默认)
 *   1 = 32 位批量写 (一次写 2 像素, 理论提速; 依赖 FSMC 16 位总线对 32 位
 *       访问的拆分行为, 部分板卡/屏实测异常, 请实机验证后再开启)
 */
#define LCD_USE_32BIT_GRAM_WRITE    0

/* ========================================================================= */
/*  默认显示参数                                                              */
/* ========================================================================= */
/** 默认显示方向: 0=竖屏 1=横屏 */
#define LCD_DEFAULT_DIR     0
/** 默认扫描方向 (取值见 lcd.h 中 L2R_U2D 等 8 个方向宏) */
#define LCD_DFT_SCAN_DIR    L2R_U2D
/**
 * 面板物理尺寸覆盖 (0=按驱动 IC 自动, 如 ILI9341 自动为 240x320).
 * 换装不同物理尺寸的同一 IC 屏时, 在此强制指定
 */
#define LCD_PANEL_W         0
#define LCD_PANEL_H         0

#endif /* __LCD_CONF_H */
