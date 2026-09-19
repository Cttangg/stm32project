# STM32_2026E — 红色激光云台控制系统

> 2023 年全国大学生电子设计竞赛 E 题「运动目标控制与自动追踪系统」— 红色光斑位置控制系统
> MCU: STM32F407VETx (168 MHz) | 执行器: 42 步进电机 ×2 + TMC2209 | 反馈: MT6701 磁编码器 ×2

## 题目要求（红色系统部分）

- 复位：红色光斑从屏幕任意位置回到原点，误差 ≤2cm
- 沿屏幕四周边线顺时针移动一周（30s 内，距边线 ≤2cm）
- 沿 A4 靶纸（黑色电工胶带框，可任意旋转角度）边缘移动一周
- 暂停键：立即制动，便于测量光斑位置

## 系统架构

```
摄像头(待约定) ──USART2──► STM32F407 ──TIM13/14 STEP──► TMC2209×2 ──► 42步进电机×2 ──► 云台(pan/tilt)
                              │                                        ▲
                              ├── SPI1 ◄── ST7735 LCD (显示/交互)
                              └── I2C1/I2C2 ◄──── MT6701 编码器×2 ──────┘
USART1 ──► 调试口
按键×6 ──► 输入
```

## 引脚分配

### 电机 1 — Tilt（竖直轴）

| 信号 | 引脚 | 说明 |
|------|------|------|
| EN_1 | PD8 | TMC2209 使能（低有效） |
| DIR_1 | PC6 | 方向 |
| STEP_1 | PD14 | 步进脉冲（TIM14 ISR 翻转） |
| MS1_1 | PD10 | 细分 |
| MS2_1 | PD12 | 细分 |
| MT6701_1 | I2C1 (PB6/PB7) | 编码器 400kHz |

### 电机 2 — Pan（水平轴）

| 信号 | 引脚 | 说明 |
|------|------|------|
| EN_2 | PE15 | TMC2209 使能（低有效） |
| DIR_2 | PE7 | 方向 |
| STEP_2 | PE9 | 步进脉冲（TIM13 ISR 翻转） |
| MS1_2 | PE13 | 细分 |
| MS2_2 | PE11 | 细分 |
| MT6701_2 | I2C2 (PB10/PB11) | 编码器 400kHz |

### 按键（GPIO 输入, NOPULL）

| 信号 | 引脚 | 说明 |
|------|------|------|
| KEY_PAUSE | PB3 | 暂停/继续 |
| KEY_RESET | PB5 | 复位回原点 |
| KEY_TRACK | PB8 | 一键追踪 |
| KEY_BORDER | PB9 | 沿屏幕边线移动 |
| KEY_CALIB | PE0 | 标定 |
| KEY_A4 | PE1 | A4 靶纸 |

### LCD（ST7735, SPI1 + GPIO 控制线）

| 信号 | 引脚 | 说明 |
|------|------|------|
| SCK | PA5 | SPI1_SCK |
| MOSI | PA7 | SPI1_MOSI |
| LCD_CS | PB1 | 片选（低有效） |
| LCD_DC | PC4 | 数据/命令 |
| LCD_RES | PC5 | 复位（低有效） |
| LCD_BL | PB0 | 背光（高有效） |

### 串口

| 串口 | 用途 | 引脚 |
|------|------|------|
| USART1 | 调试口 | PA9 (TX) / PA10 (RX) |
| USART2 | 摄像头数据（协议待约定） | PA2 (TX) / PA3 (RX) |

> USART2 RX DMA 需改为 **Circular**（当前 .ioc 为 Normal，uart 库要求 Circular）。

## 外设分配

| 外设 | 用途 | 状态 |
|------|------|------|
| TIM14 | 电机1 (Tilt) STEP 脉冲 | 已注册（NVIC 已勾选，OC1 No Output） |
| TIM13 | 电机2 (Pan) STEP 脉冲 | 已注册（NVIC 已勾选，OC1 No Output） |
| I2C1 / I2C2 | MT6701 ×2 | 已配置 |
| USART1 + DMA | 调试口 | 已配置 |
| USART2 + DMA | 摄像头 | 已配置（RX 模式待改） |
| SPI1 | ST7735 LCD | 已配置 |

## 驱动移植（Core/Lib，源自 DriverLib_King_orz）

| 驱动 | 说明 | 状态 |
|------|------|------|
| `uart.c/h` | DMA 串口库（句柄化：`UART_Device`，uart_dbg=USART1 / uart_cam=USART2） | 已移植 |
| `mt6701.c/h` | 磁编码器（句柄化改造，支持 I2C1/I2C2 双实例） | 已移植 |
| `tmc2209.c/h` | TMC2209 驱动板（原生句柄化） | 已移植 |
| `motor_stepper.c/h` | STEP 脉冲引擎（句柄化改造，TIM13/TIM14 双实例） | 已移植 |
| `motor_pid.c/h` | 位置闭环 PID（原生句柄化） | 已移植 |
| `st7735.c/h` | ST7735 SPI 屏驱动（预留骨架，待实现） | 已建 |

> 注意事项：
> - 两个电机在 `main.c` USER CODE 区实例化：`tilt`（TIM14+I2C1）、`pan`（TIM13+I2C2）
> - TIM 中断分发（`TIM8_TRG_COM_TIM14_IRQHandler`/`TIM8_UP_TIM13_IRQHandler`）在 main.c USER CODE BEGIN 0 区
> - CubeMX 重新生成前请确认 `.ioc` 中 TIM13/TIM14 已注册（已注册），生成后 `stm32f4xx_hal_conf.h` 会自动启用 HAL_TIM

## 构建

```
cmake --preset Debug
cmake --build --preset Debug
# 产物: build/Debug/STM32_2026E.elf
```

## 待办

- [ ] 摄像头帧协议约定（USART2）
- [x] 6 个按键注册（KEY_PAUSE/RESET/TRACK/BORDER/CALIB/A4）
- [x] 电机2 引脚变更（已移至 PE7/PE9/PE11/PE13/PE15）
- [ ] USART2 RX DMA 改 Circular
- [ ] 位置闭环标定（细分按 TMC2209 驱动板说明）
- [ ] 应用逻辑：状态机（复位/沿框移动/视觉追踪）
- [ ] ST7735 LCD 驱动实现（骨架已建，Core/Lib/st7735.c/h）