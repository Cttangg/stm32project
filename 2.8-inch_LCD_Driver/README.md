# 2.8-inch LCD Driver

STM32F407VET6 的 TFT-LCD + 电阻触摸屏驱动组件库（可移植），基于 STM32CubeMX + CMake + arm-none-eabi GCC 构建。

## 1. 项目简介

| 项 | 说明 |
|---|---|
| 主控 | STM32F407VET6 (168 MHz, 512 KB Flash / 128 KB SRAM) |
| LCD | 2.8 寸 TFT，ILI9341（自动识别，兼容 ILI9325/9328/9320/RM68042/NT35310/NT35510/LGDP4531/4535/SPFD5408/1505/B505/C505 等） |
| 接口 | FSMC Bank1 NE1 16 位并口（A18 作命令/数据区分线） |
| 触摸 | XPT2046 / TSC2046 / ADS7843 系列，软件模拟 SPI |
| 串口 | USART1 115200-8N1，HAL + DMA 环形缓冲 |
| 启动方式 | 应用区 0x08008000 起（32 KB 软 Bootloader 之后），ST Open Bootloader 协议经串口升级 |

驱动源码位于 `Core/Lib/`：`lcd.c/h`、`touch.c/h`、`uart.c/h`、`delay.c/h`、`FONT.H`、`app_boot_interface.h`。

## 2. 硬件接线

### 2.1 LCD（FSMC 16 位并口，20 线）

| 信号 | MCU 引脚 | 信号 | MCU 引脚 |
|------|---------|------|---------|
| FSMC_NE1（片选） | PD7 | FSMC_A18（命令/数据） | PD13 |
| FSMC_NWE（写） | PD5 | FSMC_NOE（读） | PD4 |
| D0~D3 | PD14/PD15/PD0/PD1 | D4~D7 | PE7~PE10 |
| D8~D12 | PE11~PE15 | D13~D15 | PD8/PD9/PD10 |
| 背光 `BL` | PB1（`lcd_conf.h`） | 复位 `RST` | 系统 NRST |

### 2.2 触摸（软件 SPI，5 线）

| 信号 | MCU 引脚 | 方向 |
|------|---------|------|
| T_PEN（按压检测） | PC5 | 输入 |
| T_MISO（数据输出 DOUT） | PB14 | 输入 |
| T_MOSI（数据输入 TDIN） | PB15 | 输出 |
| T_SCK（时钟） | PB13 | 输出 |
| T_CS（片选） | PB12 | 输出 |

## 3. 构建与烧录

```bash
# 构建（Ninja + arm-none-eabi）
cmake --preset Debug
cmake --build --preset Debug
# 产物: build/Debug/2.8-inch_LCD_Driver.bin / .elf

# 烧录: 经串口连板载软 Bootloader（应用区 0x08008000 起，不动 BL 区）
STM32_Programmer_CLI.exe -c port=COMx -u 0x08008000 build/Debug/2.8-inch_LCD_Driver.bin
```

> 烧录前需先让 MCU 处于软 Bootloader（上电前按住 Boot 键，或运行中通过串口发送 `dfu`/`boot` 命令）。

## 4. 初始化顺序与依赖

```c
APP_SetVectorTable();        /* 必须在 main 第一条指令调用 (0x08008000) */
HAL_Init();
SystemClock_Config();

LCD_Init();                  /* 兼容接口: 失败进入 Error_Handler */
/* 或: LCD_InitEx() 返回 LCD_Status 错误码 (LCD_OK / LCD_ERR_INIT / LCD_ERR_UNSUPPORTED_ID) */
TP_Init();                   /* 依赖 LCD; 使用预存校准值, 跳过四角校准 */

UART_Init();
UART_Open(UART_P1);
/* UART_RegisterFrame(...) 注册帧协议/回调 */
```

主循环周期调用：`UART_Task()`（串口收帧解析）、`TP_GetGesture()`（手势识别：单击/双击/长按/滑动）。

- **触摸手势**：上层只消费 `TP_GetGesture()` 返回的手势事件（`TOUCH_EVENT_SINGLE_CLICK/LONG_PRESS/SWIPE`），DOWN/UP 原始状态留在驱动内部状态机；单击在松开确认后立即上报（无双击）；防误触：最短按压时间 + 释放消抖（阈值参数见 `touch_conf.h`）
- **图片显示**：`LCD_ShowImage()` 阻塞刷完整帧；需要与主循环并行的场景用 `LCD_ShowImage_Start()` + 每轮调用 `LCD_ShowImage_Task()`（每轮刷一行，渐进显示，不阻塞其他任务）
- **上电自检**：`main.c` 中 `SELF_TEST_ENABLE` 置 1，初始化完成后自动执行 LCD 色条/读点校验、触摸有效点检测、串口回环（`LIB_SelfTest()`），结果经串口输出
- **PEN 中断**：`touch_conf.h` 的 `TP_PEN_INT_ENABLE=1`（默认）时触摸由 EXTI9_5 事件驱动，无按压时不轮询；驱动内置弱定义 ISR，若在 CubeMX 中另行使能 EXTI（会生成强定义），请在其 `EXTI9_5_IRQHandler` 的 USER CODE 段调用 `TP_PenIRQHandler()`

## 5. 移植到新板/新屏

1. **修改板级配置**：`Core/Lib/lcd_conf.h`（背光引脚、FSMC 引脚组与读写时序、默认方向/扫描方向、面板尺寸覆盖）和 `Core/Lib/touch_conf.h`（触摸引脚、校准开关、采样滤波参数）
2. **新屏/换模组**：将 `touch_conf.h` 的 `TP_CAL_PRESET_ENABLE` 改为 `0`，上电后调用 `TP_Adjust()` 重新四角校准
3. **版本防呆**：若 conf 头与驱动版本不匹配，编译会触发 `#error`，按提示重新迁移配置
4. **引脚所有权**：LCD/触摸/FSMC 引脚由驱动自行初始化，不依赖 CubeMX 生成的引脚配置；CubeMX 中可将这些引脚留空
5. **驱动版本**：见 `lcd.h` 的 `LCD_VER_MAJOR/MINOR/PATCH`

## 6. 串口命令协议

- 帧协议：USART1，逐字节回调回显；收到完整一行命令（回车/换行结束）后解析
- `dfu` 或 `boot` + 回车：写 magic token 到 RAM 并软复位，进入 Bootloader 等待编程器
- 其他输入原样回显；另有 `hello\r\n` 每 1 秒周期性上报（见 `main.c`）

## 7. Bootloader 接口（`Core/Lib/app_boot_interface.h`）

| 接口 | 说明 |
|------|------|
| `APP_SetVectorTable()` | 重定位向量表到 0x08008000，须为 main 首条指令 |
| `APP_JumpToBootloader()` | 写 `APP_BOOT_MAGIC` 到 SRAM 末 4 字节并软复位 |

常量须与 Bootloader 工程（`openbootloader_conf.h` + `openbl/main.c`）保持一致。

## 8. 版本记录

- **v1.3.0**：手势状态机重构——移除双击，单击在松开确认后**立即上报**（无确认延迟）；新增防误触：`TOUCH_MIN_PRESS_TIME`（最短按压，误碰忽略）+ `TOUCH_RELEASE_DEBOUNCE`（释放消抖，PEN 弹跳合并）；移除 `TOUCH_EVENT_DOUBLE_CLICK`/`TOUCH_STATE_WAIT_DOUBLE`
- **v1.2.2**：修复上电卡死在 TP_Init——`EXTI9_5_IRQHandler` 由弱定义改为强定义（startup 的 `.weak` 兜底会指向 Default_Handler 死循环）；触摸灵敏度优化：采样失败持续重试、`TP_ERR_RANGE` 50→100、双击窗口 300→200ms
- **v1.2.1**：修复图片不显示——GRAM 32 位批量写在部分板卡/屏上失效，默认恢复 16 位逐像素写（`LCD_USE_32BIT_GRAM_WRITE` 开关，默认 0）
- **v1.2.0**：触摸手势识别状态机——新增 `TP_GetGesture()`（单击/双击/长按/滑动），上层不再感知 DOWN/UP；移动优先于长按，长按/滑动为终态不参与双击；阈值可配置（`TOUCH_DOUBLE_CLICK_TIMEOUT`/`TOUCH_LONG_PRESS_TIME`/`TOUCH_SWIPE_THRESHOLD`）
- **v1.1.0**：新增 `LCD_InitEx()` 错误码接口（`LCD_OK`/`LCD_ERR_INIT`/`LCD_ERR_UNSUPPORTED_ID`）；`LCD_Clear`/`LCD_ShowImage` 32 位批量写提速；新增异步分片刷屏 `LCD_ShowImage_Start/Task/Stop`；PEN 中断事件驱动（`TP_PEN_INT_ENABLE`）；新增自检模块 `Core/Lib/self_test.c/h`
- **v1.0.0**：组件库化——新增 `lcd_conf.h`/`touch_conf.h` 板级配置、版本宏与 conf 防呆检查、FSMC/引脚初始化收归驱动、触摸读数失败防护、全库 Doxygen 注释
- 历史硬件/接线变更见 [变更记录.md](变更记录.md)（GBK 编码）

## 9. 文档生成

```bash
doxygen Doxyfile    # 输出到 docs/ (已 gitignore)
```

## 10. 出处声明

LCD/触摸驱动基于正点原子 STM32F4 例程（TFT 驱动 V2.8 20140721、触摸驱动 V1.1 20140721）改造；本项目按原样使用其公开代码，版权归原作者所有。
