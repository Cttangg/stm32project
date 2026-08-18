/**
 ******************************************************************************
 * @file    lcd.h
 * @brief   TFT-LCD 驱动接口 — FSMC 并口, 多驱动 IC 自适应
 *
 *          支持驱动 IC: ILI9341/ILI9325/ILI9328/ILI9320/RM68042/RM68021/
 *          NT35310/NT35510/LGDP4531/LGDP4535/SPFD5408/1505/B505/C505 等
 *
 * 硬件连接:
 *   - FSMC Bank1 NE1 (基址 0x60000000), A18 作命令/数据区分线
 *   - LCD 背光: PB1
 *
 * @note   类型别名 u8/u16/u32 与 lcd.c 保持一致 (兼容原驱动代码)
 ******************************************************************************
 */
#ifndef __LCD_H
#define __LCD_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* ===================== 版本 ===================== */
#define LCD_VER_MAJOR  1
#define LCD_VER_MINOR  3
#define LCD_VER_PATCH  0

/* ===================== 错误码 ===================== */
/** LCD 错误码 */
typedef enum {
    LCD_OK                 =  0,   /**< 成功 */
    LCD_ERR_PARAM          = -1,   /**< 参数错误 */
    LCD_ERR_INIT           = -2,   /**< GPIO/FSMC 初始化失败 */
    LCD_ERR_UNSUPPORTED_ID = -3,   /**< 驱动 IC ID 无法识别 */
} LCD_Status;

/* ===================== 扫描方向 (lcd_conf.h 的 LCD_DFT_SCAN_DIR 需要引用) ===================== */
#define L2R_U2D  0   /**< 从左到右, 从上到下 */
#define L2R_D2U  1   /**< 从左到右, 从下到上 */
#define R2L_U2D  2   /**< 从右到左, 从上到下 */
#define R2L_D2U  3   /**< 从右到左, 从下到上 */
#define U2D_L2R  4   /**< 从上到下, 从左到右 */
#define U2D_R2L  5   /**< 从上到下, 从右到左 */
#define D2U_L2R  6   /**< 从下到上, 从左到右 */
#define D2U_R2L  7   /**< 从下到上, 从右到左 */

/* ===================== 板级配置 (lcd_conf.h) ===================== */
#define LCD_CONF_VER_EXPECT  0x0100
#include "lcd_conf.h"
#if (LCD_CONF_VER != LCD_CONF_VER_EXPECT)
#error "lcd_conf.h 与 lcd.h 版本不匹配: 请按新版本重新迁移板级配置"
#endif

/* ===================== 类型别名 (兼容原驱动) ===================== */
typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef volatile uint8_t   vu8;
typedef volatile uint16_t  vu16;
typedef volatile uint32_t  vu32;

/** LCD 主要参数结构体 */
typedef struct
{
    u16 width;          /**< LCD 宽度 */
    u16 height;         /**< LCD 高度 */
    u16 id;             /**< LCD 驱动 IC ID */
    u8  dir;            /**< 显示方向: 0=竖屏 1=横屏 */
    u16 wramcmd;        /**< 开始写 GRAM 指令 */
    u16 setxcmd;        /**< 设置 X 坐标指令 */
    u16 setycmd;        /**< 设置 Y 坐标指令 */
} _lcd_dev;

/** 全局 LCD 参数 */
extern _lcd_dev lcddev;
/** 画笔颜色 */
extern u16 POINT_COLOR;
/** 背景色 */
extern u16 BACK_COLOR;

/* LCD 地址结构体 (FSMC Bank1 NE1, A18 作命令/数据区分线) */
typedef struct
{
    u16 LCD_REG;
    u16 LCD_RAM;
} LCD_TypeDef;

/** LCD 寄存器区基址 (FSMC Bank1 NE1) */
#define LCD_BASE        ((u32)(0x60000000 | 0x00007FFFE))
/** LCD 操作指针: LCD_REG=命令地址, LCD_RAM=数据地址 */
#define LCD             ((LCD_TypeDef *) LCD_BASE)

/* 扫描方向 (左右/上下排列组合) */
#define L2R_U2D  0   /**< 从左到右, 从上到下 */
#define L2R_D2U  1   /**< 从左到右, 从下到上 */
#define R2L_U2D  2   /**< 从右到左, 从上到下 */
#define R2L_D2U  3   /**< 从右到左, 从下到上 */
#define U2D_L2R  4   /**< 从上到下, 从左到右 */
#define U2D_R2L  5   /**< 从上到下, 从右到左 */
#define D2U_L2R  6   /**< 从下到上, 从左到右 */
#define D2U_R2L  7   /**< 从下到上, 从右到左 */

/* 常用颜色 (RGB565) */
#define WHITE         0xFFFF
#define BLACK         0x0000
#define BLUE          0x001F
#define BRED          0XF81F
#define GRED          0XFFE0
#define GBLUE         0X07FF
#define RED           0xF800
#define MAGENTA       0xF81F
#define GREEN         0x07E0
#define CYAN          0x7FFF
#define YELLOW        0xFFE0
#define BROWN         0XBC40
#define BRRED         0XFC07
#define GRAY          0X8430
#define DARKBLUE      0X01CF
#define LIGHTBLUE     0X7D7C
#define GRAYBLUE      0X5458
#define LIGHTGREEN    0X841F
#define LGRAY         0XC618
#define LGRAYBLUE     0XA651
#define LBBLUE        0X2B12

/**
 * @brief  LCD 初始化: GPIO/FSMC 配置 + 驱动 IC 识别 + 初始化序列
 * @note   须在系统时钟配置完成后调用
 * @note   兼容接口: 内部失败时进入 Error_Handler, 需错误处理请用 LCD_InitEx()
 */
void LCD_Init(void);
/**
 * @brief  LCD 初始化 (带错误返回)
 * @retval LCD_OK: 成功; LCD_ERR_INIT: GPIO/FSMC 初始化失败;
 *         LCD_ERR_UNSUPPORTED_ID: 驱动 IC ID 无法识别
 * @note   与 LCD_Init() 二选一调用
 */
LCD_Status LCD_InitEx(void);
/** @brief 开启 LCD 显示 */
void LCD_DisplayOn(void);
/** @brief 关闭 LCD 显示 */
void LCD_DisplayOff(void);
/**
 * @brief  清屏 (全屏填充单一颜色)
 * @param  Color: 填充色 (RGB565)
 */
void LCD_Clear(u16 Color);
/**
 * @brief  设置光标位置 (写 GRAM 的起始坐标)
 * @param  Xpos: 横坐标
 * @param  Ypos: 纵坐标
 */
void LCD_SetCursor(u16 Xpos, u16 Ypos);
/**
 * @brief  画点 (颜色取全局画笔颜色 POINT_COLOR)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 */
void LCD_DrawPoint(u16 x, u16 y);
/**
 * @brief  快速画点 (直接指定颜色)
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @param  color: 颜色 (RGB565)
 */
void LCD_Fast_DrawPoint(u16 x, u16 y, u16 color);
/**
 * @brief  读取指定点颜色
 * @param  x: 横坐标
 * @param  y: 纵坐标
 * @retval 该点颜色 (RGB565); 坐标越界返回 0
 */
u16  LCD_ReadPoint(u16 x, u16 y);
/**
 * @brief  画圆 (Bresenham 中点画圆算法)
 * @param  x0: 圆心横坐标
 * @param  y0: 圆心纵坐标
 * @param  r: 半径
 */
void LCD_Draw_Circle(u16 x0, u16 y0, u8 r);
/**
 * @brief  画线 (Bresenham 算法)
 * @param  x1: 起点横坐标
 * @param  y1: 起点纵坐标
 * @param  x2: 终点横坐标
 * @param  y2: 终点纵坐标
 */
void LCD_DrawLine(u16 x1, u16 y1, u16 x2, u16 y2);
/**
 * @brief  画矩形边框
 * @param  x1: 矩形一角横坐标
 * @param  y1: 矩形一角纵坐标
 * @param  x2: 对角横坐标
 * @param  y2: 对角纵坐标
 */
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2);
/**
 * @brief  在指定矩形区域填充单一颜色
 * @param  sx: 矩形左上角横坐标
 * @param  sy: 矩形左上角纵坐标
 * @param  ex: 矩形右下角横坐标
 * @param  ey: 矩形右下角纵坐标
 * @param  color: 填充色
 */
void LCD_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 color);
/**
 * @brief  在指定矩形区域按颜色数组逐点填充
 * @param  sx: 起始横坐标
 * @param  sy: 起始纵坐标
 * @param  ex: 结束横坐标
 * @param  ey: 结束纵坐标
 * @param  color: 颜色数组指针 (RGB565, 按行存放)
 */
void LCD_Color_Fill(u16 sx, u16 sy, u16 ex, u16 ey, u16 *color);
/**
 * @brief  在指定位置显示一个 ASCII 字符
 * @param  x: 起始横坐标
 * @param  y: 起始纵坐标
 * @param  num: 要显示的字符 (ASCII ' ' ~ '~')
 * @param  size: 字体大小 (12/16/24)
 * @param  mode: 0=非叠加; 1=叠加
 */
void LCD_ShowChar(u16 x, u16 y, u8 num, u8 size, u8 mode);
/**
 * @brief  显示数字 (高位为 0 时不显示)
 * @param  x: 起点横坐标
 * @param  y: 起点纵坐标
 * @param  num: 数值 (0~4294967295)
 * @param  len: 数字位数
 * @param  size: 字体大小
 */
void LCD_ShowNum(u16 x, u16 y, u32 num, u8 len, u8 size);
/**
 * @brief  显示数字 (高位为 0 时按 mode 填充 '0' 或空格)
 * @param  x: 起点横坐标
 * @param  y: 起点纵坐标
 * @param  num: 数值 (0~999999999)
 * @param  len: 显示位数
 * @param  size: 字体大小
 * @param  mode: bit7=1 高位填 '0'; bit0=1 叠加显示
 */
void LCD_ShowxNum(u16 x, u16 y, u32 num, u8 len, u8 size, u8 mode);
/**
 * @brief  显示字符串 (超出宽度自动换行, 超出区域自动截断)
 * @param  x: 起点横坐标
 * @param  y: 起点纵坐标
 * @param  width: 显示区域宽度
 * @param  height: 显示区域高度
 * @param  size: 字体大小
 * @param  p: 字符串起始地址
 */
void LCD_ShowString(u16 x, u16 y, u16 width, u16 height, u8 size, const char *p);

/**
 * @brief  写 LCD 寄存器 (寄存器地址 + 数据)
 * @param  LCD_Reg: 寄存器地址
 * @param  LCD_RegValue: 要写入的数据
 */
void LCD_WriteReg(u16 LCD_Reg, u16 LCD_RegValue);
/**
 * @brief  读 LCD 寄存器
 * @param  LCD_Reg: 寄存器地址
 * @retval 读到的数据
 */
u16 LCD_ReadReg(u16 LCD_Reg);
/**
 * @brief  写 LCD 寄存器序号 (仅写 REG 地址)
 * @param  regval: 寄存器值
 */
void LCD_WR_REG(u16 regval);
/**
 * @brief  写 LCD 数据 (仅写 RAM 地址)
 * @param  data: 要写入的值
 */
void LCD_WR_DATA(u16 data);
/** @brief 读 LCD 数据 */
u16 LCD_RD_DATA(void);
/** @brief 发送开始写 GRAM 命令 (设置光标后调用) */
void LCD_WriteRAM_Prepare(void);
/**
 * @brief  写一个像素颜色到 GRAM
 * @param  RGB_Code: RGB565 颜色值
 */
void LCD_WriteRAM(u16 RGB_Code);
/**
 * @brief  设置 LCD 自动扫描方向
 * @param  dir: 0~7, 代表 8 个方向 (见上方定义)
 * @warning 一般保持默认 L2R_U2D; 其他方向可能导致显示异常 (尤其 9341/6804)
 */
void LCD_Scan_Dir(u8 dir);
/**
 * @brief  设置 LCD 显示方向 (在线切换)
 * @param  dir: 0=竖屏 (240x320); 1=横屏 (320x240)
 */
void LCD_Display_Dir(u8 dir);
/**
 * @brief  设置显示窗口, 并自动将画点坐标移到窗口左上角
 * @param  sx: 窗口起始横坐标 (左上角)
 * @param  sy: 窗口起始纵坐标 (左上角)
 * @param  width: 窗口宽度 (必须 > 0)
 * @param  height: 窗口高度 (必须 > 0)
 * @note   RM68042 横屏时不支持窗口设置
 */
void LCD_Set_Window(u16 sx, u16 sy, u16 width, u16 height);
/**
 * @brief  显示图片 (阻塞): 同步刷完整帧后返回 (RGB565 数组, img 存于 Flash)
 * @param  x: 图片左上角横坐标
 * @param  y: 图片左上角纵坐标
 * @param  w: 图片宽度
 * @param  h: 图片高度
 * @param  img: 图片数据指针 (RGB565, 逐行存放)
 */
void LCD_ShowImage(u16 x, u16 y, u16 w, u16 h, const u16 *img);
/**
 * @brief  异步显示图片: 登记任务后立即返回, 由 LCD_ShowImage_Task() 逐行刷出
 * @param  x: 图片左上角横坐标
 * @param  y: 图片左上角纵坐标
 * @param  w: 图片宽度
 * @param  h: 图片高度
 * @param  img: 图片数据指针 (RGB565, 逐行存放)
 * @note   同一时刻仅支持一个异步任务; 与阻塞版 LCD_ShowImage() 不可并发
 */
void LCD_ShowImage_Start(u16 x, u16 y, u16 w, u16 h, const u16 *img);
/**
 * @brief  异步刷屏进度 (主循环周期调用, 每轮刷一行)
 * @retval 0: 任务进行中; 1: 已完成或当前无任务
 */
uint8_t LCD_ShowImage_Task(void);
/** @brief 取消异步刷屏任务 */
void LCD_ShowImage_Stop(void);

/* LCD 寄存器 */
#define R0             0x00
#define R1             0x01
#define R2             0x02
#define R3             0x03
#define R4             0x04
#define R5             0x05
#define R6             0x06
#define R7             0x07
#define R8             0x08
#define R9             0x09
#define R10            0x0A
#define R12            0x0C
#define R13            0x0D
#define R14            0x0E
#define R15            0x0F
#define R16            0x10
#define R17            0x11
#define R18            0x12
#define R19            0x13
#define R20            0x14
#define R21            0x15
#define R22            0x16
#define R23            0x17
#define R24            0x18
#define R25            0x19
#define R26            0x1A
#define R27            0x1B
#define R28            0x1C
#define R29            0x1D
#define R30            0x1E
#define R31            0x1F
#define R32            0x20
#define R33            0x21
#define R34            0x22
#define R36            0x24
#define R37            0x25
#define R40            0x28
#define R41            0x29
#define R43            0x2B
#define R45            0x2D
#define R48            0x30
#define R49            0x31
#define R50            0x32
#define R51            0x33
#define R52            0x34
#define R53            0x35
#define R54            0x36
#define R55            0x37
#define R56            0x38
#define R57            0x39
#define R59            0x3B
#define R60            0x3C
#define R61            0x3D
#define R62            0x3E
#define R63            0x3F
#define R64            0x40
#define R65            0x41
#define R66            0x42
#define R67            0x43
#define R68            0x44
#define R69            0x45
#define R70            0x46
#define R71            0x47
#define R72            0x48
#define R73            0x49
#define R74            0x4A
#define R75            0x4B
#define R76            0x4C
#define R77            0x4D
#define R78            0x4E
#define R79            0x4F
#define R80            0x50
#define R81            0x51
#define R82            0x52
#define R83            0x53
#define R96            0x60
#define R97            0x61
#define R106           0x6A
#define R118           0x76
#define R128           0x80
#define R129           0x81
#define R130           0x82
#define R131           0x83
#define R132           0x84
#define R133           0x85
#define R134           0x86
#define R135           0x87
#define R136           0x88
#define R137           0x89
#define R139           0x8B
#define R140           0x8C
#define R141           0x8D
#define R143           0x8F
#define R144           0x90
#define R145           0x91
#define R146           0x92
#define R147           0x93
#define R148           0x94
#define R149           0x95
#define R150           0x96
#define R151           0x97
#define R152           0x98
#define R153           0x99
#define R154           0x9A
#define R157           0x9D
#define R192           0xC0
#define R193           0xC1
#define R229           0xE5

#endif /* __LCD_H */
