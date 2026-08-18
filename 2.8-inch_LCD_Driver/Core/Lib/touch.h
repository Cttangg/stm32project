/**
 ******************************************************************************
 * @file    touch.h
 * @brief   电阻触摸屏驱动接口 — 软件模拟 SPI (兼容 XPT2046/ADS7843 等)
 *
 * 硬件连接:
 *   T_PEN = PC5    T_MISO = PB14
 *   T_MOSI = PB15  T_SCK = PB13   T_CS = PB12
 *
 * 使用流程:
 *   1. 初始化 LCD 后调用 TP_Init() (自动装载预存校准参数, 跳过四角校准)
 *   2. 主循环周期调用 TP_Scan(0), 触摸状态与坐标存放于 tp_dev
 *   3. 若触摸偏差大, 调用 TP_Adjust() 重新四角校准
 *
 * @note 板载无 24CXX EEPROM: 校准数据不保存
 ******************************************************************************
 */
#ifndef __TOUCH_H
#define __TOUCH_H

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include "lcd.h"

/* ===================== 板级配置 (touch_conf.h) ===================== */
#define TOUCH_CONF_VER_EXPECT  0x0101
#include "touch_conf.h"
#if (TOUCH_CONF_VER != TOUCH_CONF_VER_EXPECT)
#error "touch_conf.h 与 touch.h 版本不匹配: 请按新版本重新迁移板级配置"
#endif

/* ===================== 类型别名 (兼容原驱动) ===================== */
typedef int32_t s32;

/* 触摸屏状态 */
#define TP_PRES_DOWN 0x80   /**< 触摸按下标记位 */
#define TP_CATH_PRES 0x40   /**< 一次按压事件捕获标记位 */

/**
 * @brief 触摸屏设备结构体 (函数指针 + 坐标/校准参数)
 */
typedef struct
{
    u8  (*init)(void);      /**< 触摸初始化函数指针 */
    u8  (*scan)(u8);        /**< 触摸扫描函数指针 */
    void (*adjust)(void);   /**< 触摸校准函数指针 */
    u16 x[5];               /**< X 坐标缓存: [0]=当前值, [4]=按下瞬间值 */
    u16 y[5];               /**< Y 坐标缓存: [0]=当前值, [4]=按下瞬间值 */
    u8  sta;                /**< 触摸状态 (TP_PRES_DOWN/TP_CATH_PRES) */
    float xfac;             /**< X 方向比例因子 (物理坐标->屏幕坐标) */
    float yfac;             /**< Y 方向比例因子 */
    short xoff;             /**< X 方向偏移量 */
    short yoff;             /**< Y 方向偏移量 */
    u8 touchtype;           /**< 触摸类型: 0=X/Y 与屏幕同向; 1=相反 */
} _m_tp_dev;

/** 全局触摸设备实例 */
extern _m_tp_dev tp_dev;

/* ===================== 电阻触摸引脚 (软件模拟 SPI) =====================
   引脚定义见 touch_conf.h (T_PEN=PC5, T_MISO=PB14, T_MOSI=PB15,
   T_SCK=PB13, T_CS=PB12); 以下为引脚访问宏 */

#define TP_PEN      HAL_GPIO_ReadPin(TP_PEN_GPIO, TP_PEN_PIN)        /**< 读 PEN 引脚 (低电平=按下) */
#define TP_DOUT     HAL_GPIO_ReadPin(TP_MISO_GPIO, TP_MISO_PIN)      /**< 读 MISO 数据 */
#define TP_TDIN(v)  HAL_GPIO_WritePin(TP_MOSI_GPIO, TP_MOSI_PIN, (v)) /**< 写 MOSI 数据 */
#define TP_TCLK(v)  HAL_GPIO_WritePin(TP_SCK_GPIO, TP_SCK_PIN, (v))   /**< 写 SCK 时钟 */
#define TP_TCS(v)   HAL_GPIO_WritePin(TP_CS_GPIO, TP_CS_PIN, (v))     /**< 写 CS 片选 */

/* ===================== 触摸功能函数 ===================== */

/**
 * @brief  软件 SPI 写入 1 字节
 * @param  num: 待写入数据
 */
void TP_Write_Byte(u8 num);

/**
 * @brief  读取 12 位 ADC 转换值
 * @param  CMD: 控制指令 (如 CMD_RDX/CMD_RDY)
 * @retval 12 位 ADC 数据 (高 12 位有效)
 */
u16 TP_Read_AD(u8 CMD);

/**
 * @brief  读取一个轴的坐标 (多次采样滤波)
 * @param  xy: 指令 (CMD_RDX / CMD_RDY)
 * @retval 滤波后的读数
 */
u16 TP_Read_XOY(u8 xy);

/**
 * @brief  读取 X/Y 坐标
 * @param  x: X 坐标指针
 * @param  y: Y 坐标指针
 * @retval 0=失败, 1=成功
 */
u8  TP_Read_XY(u16 *x, u16 *y);

/**
 * @brief  双采样校验读坐标
 * @param  x: X 坐标指针
 * @param  y: Y 坐标指针
 * @retval 0=失败, 1=成功
 */
u8  TP_Read_XY2(u16 *x, u16 *y);

/**
 * @brief  画触摸校准点 (十字线 + 中心圈)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  color: 颜色
 */
void TP_Drow_Touch_Point(u16 x, u16 y, u16 color);

/**
 * @brief  画大点 (2x2 像素)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  color: 颜色
 */
void TP_Draw_Big_Point(u16 x, u16 y, u16 color);

/** @brief 保存校准参数到 EEPROM (板载无 24CXX, 实际为空操作) */
void TP_Save_Adjdata(void);

/**
 * @brief  读取校准参数
 * @retval 1=成功获取; 0=失败, 需重新校准
 */
u8  TP_Get_Adjdata(void);

/** @brief 触摸屏四角校准 */
void TP_Adjust(void);

/**
 * @brief  显示校准过程信息
 * @param  x0~y3: 4 个采样点坐标
 * @param  fac: 比例因子 (x100, 正常范围 95~105)
 */
void TP_Adj_Info_Show(u16 x0, u16 y0, u16 x1, u16 y1, u16 x2, u16 y2,
                      u16 x3, u16 y3, u16 fac);

/**
 * @brief  触摸扫描 (主循环周期调用)
 * @param  tp: 0=输出屏幕坐标; 1=输出物理坐标 (校准等特殊场合用)
 * @retval 当前触屏状态: 0=无触摸; 1=有触摸
 */
u8  TP_Scan(u8 tp);

/**
 * @brief  触摸初始化: GPIO 配置 + 装载预存校准参数
 * @retval 0=未进行四角校准 (使用预存校准值)
 */
u8  TP_Init(void);

/**
 * @brief  PEN 中断服务函数 (由 EXTI9_5_IRQHandler 调用)
 * @note   仅 TP_PEN_INT_ENABLE=1 时有效; EXTI9_5_IRQHandler 由驱动强定义
 *         (见 touch.c 说明); 若在 CubeMX 中另行使能 EXTI9-5 引脚导致
 *         链接冲突, 请删除驱动内的 ISR 定义, 在 CubeMX 生成的
 *         EXTI9_5_IRQHandler USER CODE 段调用本函数
 */
void TP_PenIRQHandler(void);

/* ===================== 手势识别 (上层唯一入口) ===================== */

/**
 * 触摸手势状态机 (单击立即上报, 无双击):
 *   IDLE ──DOWN──> PRESSING ──按住≥MIN_PRESS_TIME──> PRESSED
 *     ↑                    │松开(<MIN_PRESS, 误触忽略)
 *     │                    ↓
 *     │                 IDLE
 *     │
 *   PRESSED ──移动≥阈值──> SWIPE (终态) ──UP──> IDLE
 *     │──时间≥阈值──────> LONG_PRESS (终态) ──UP──> IDLE
 *     └──松开(≥RELEASE_DEBOUNCE 消抖确认)──> SINGLE_CLICK ──> IDLE
 *
 * 防误触设计:
 *   - 按下消抖: 按压必须持续 ≥ TOUCH_MIN_PRESS_TIME, 误碰/噪声尖峰不产生事件
 *   - 释放消抖: 松开必须保持 ≥ TOUCH_RELEASE_DEBOUNCE, PEN 弹跳合并为同一次按压
 *   - 移动优先于长按; 长按/滑动为终态, 松开只复位不产生单击
 */
typedef enum {
    TOUCH_STATE_IDLE = 0,    /**< 待命 */
    TOUCH_STATE_PRESSING,    /**< 按住确认中 (最短按压时间消抖) */
    TOUCH_STATE_PRESSED,     /**< 按下 (识别中: 滑动/长按/松开=单击) */
    TOUCH_STATE_LONG_PRESS,  /**< 长按 (终态, 等待松开) */
    TOUCH_STATE_SWIPE,       /**< 滑动 (终态, 等待松开) */
} TouchState_t;

/** 手势事件 (上层唯一可见的触摸输入, 不暴露 DOWN/UP) */
typedef enum {
    TOUCH_EVENT_NONE = 0,    /**< 无事件 */
    TOUCH_EVENT_SINGLE_CLICK, /**< 单击 (松开确认后立即上报, 无确认延迟) */
    TOUCH_EVENT_LONG_PRESS,   /**< 长按 (按下时间 ≥ TOUCH_LONG_PRESS_TIME 且未滑动) */
    TOUCH_EVENT_SWIPE,        /**< 滑动 (移动 ≥ TOUCH_SWIPE_THRESHOLD) */
} TouchEvent_t;

/**
 * @brief  手势识别 (主循环周期调用, 内部调用 TP_Scan 采样)
 * @retval 本次周期产生的手势事件; 无事件返回 TOUCH_EVENT_NONE
 * @note   上层只消费手势事件, 不感知 DOWN/UP 原始状态;
 *         阈值参数见 touch_conf.h (TOUCH_DOUBLE_CLICK_TIMEOUT 等)
 */
TouchEvent_t TP_GetGesture(void);

/**
 * @brief  查询当前手势状态 (调试/诊断用)
 * @retval 当前状态 (TouchState_t)
 */
TouchState_t TP_GetGestureState(void);

#endif /* __TOUCH_H */
