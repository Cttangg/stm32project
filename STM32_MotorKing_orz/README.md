# STM32_MotorKing_orz — 红色激光云台控制系统

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

> USART2 RX DMA 已为 **Circular**（与 USART1 一致，uart 库要求）。

## 外设分配

| 外设 | 用途 | 状态 |
|------|------|------|
| TIM14 | 电机1 (Tilt) STEP 脉冲 | 已注册（NVIC 已勾选，OC1 No Output） |
| TIM13 | 电机2 (Pan) STEP 脉冲 | 已注册（NVIC 已勾选，OC1 No Output） |
| I2C1 / I2C2 | MT6701 ×2 | 已配置 |
| USART1 + DMA | 调试口 | 已配置 |
| USART2 + DMA | 摄像头 | 已配置（RX 模式待改） |
| SPI1 | ST7735 LCD | 已配置 |

## 分层架构

```
L4 应用/输入    按键 ×6 + 串口命令          产生"运动请求"
L3 仲裁层       motion_arbiter             多源请求按优先级仲裁 + 暂停/急停总覆盖
L2 运动层       motion                     轨迹原语 + 屏幕cm坐标运动学 + 视觉外环
L1 轴控层       motor_axis                 每轴 MT6701→PID→Stepper 闭环
L0 驱动层       motor_pid / motor_stepper / tmc2209 / mt6701 / uart / st7735
```

### 题目 → 运动原语映射

| 题目 | 原语 | 说明 |
|------|------|------|
| 基1 复位到原点 | `Motion_GotoAngle(pan0,tilt0,rate)` | **角度空间**：目标=CALIB 第0点轴角，误差→PID，≤2cm |
| 基2 沿屏边线顺时针一周 | `Motion_FollowAnglePath(4角点, closed=1, loops=1)` | **角度空间**：顶点=CALIB 1..4 轴角，角点精确 |
| 基3/4 A4 靶纸（任意位置/旋转角） | `Motion_FollowPathXY(4角点, closed=1, loops=1)` | 屏幕坐标：角点由视觉给出（待接入） |
| 发2 暂停键立即制动 | `MotionArbiter_Pause(1)` | 停脉冲、线圈锁定，光斑冻结 |
| 发1 一键追踪（绿系统） | `Motion_TrackXY(x,y)` | 持续跟踪红点坐标 |

### 坐标与标定

- 屏幕坐标：原点=屏幕中心，+x 向右，+y 向上，单位 cm。
- 运动学（仿射）：`[pan;tilt] = k·[x;y] + b`，`Motion_DefaultCalib(dist_cm)` 给默认值，需实测标定。
- 视觉外环：`Motion_SetSpotXY(x,y)` 喂入光斑坐标 → 前馈角 + `corr·(设定点−光斑)` 修正，超时自动停用。

### 仲裁优先级（枚举升序，值越大越高）

`CALIB > RESET > A4 > BORDER > TRACK > MANUAL`；暂停/急停为闩锁总覆盖，恢复后继续原请求。

## 应用层逻辑（Core/Lib/app.c，L4）

轴定义：**X 轴 = 电机2 = Pan（TIM13/I2C2）**，**Y 轴 = 电机1 = Tilt（TIM14/I2C1）**。

### 串口（USART1 调试口，115200-8N1）

- 每 200ms 打印实时角度：`[IDLE] X=34.56 Y=-12.34`
- 状态/事件即时打印（`>> ...`）：模式切换、标定采样、完成、暂停等
- 标定态格式：`[CALIB 2/4 TOP-RIGHT] X=.. Y=..`

### 调试命令（USART1，行尾 `\r`/`\n`，大小写不敏感）

目标电机由 `app.c` 的 `APP_DBG_*` 绑定块决定，**当前 = 电机2（X 轴 / Pan，PE15/PE13/PE11/PE9/PE7）**；切回电机1 只需改该绑定块。

| 命令 | 说明 |
|------|------|
| `en1 0\|1` | 目标电机 EN（0=使能/低，1=禁止/高）；无参数=回读 |
| `ms1 0\|1` / `ms2 0\|1` | 目标电机 MS1/MS2；无参数=回读 |
| `step 0\|1` / `dir 0\|1` | 目标电机 STEP/DIR 静态电平（手动点动排障） |
| `pins` | 打印目标电机全部引脚实测电平（EN/MS1/MS2/STEP/DIR） |
| `status` | 全量状态：模式 / 两轴 EN 实测电平 / PID 状态 / 步率 / 步数（排障用） |
| `mode [0..3]` | 显示/设置细分（0=1/8 1=1/16 2=1/32 3=1/64） |
| `stop` / `release` | 立即制动 / 释放两轴力矩 |
| `help` | 命令列表 + 当前目标电机 |

> 调试口命令直接操作 GPIO，绕过运动库；用 `release` 后再手动点动可判断是**驱动器故障**还是**控制逻辑问题**。

### 按键

| 按键 | 引脚 | 功能 |
|------|------|------|
| KEY_CALIB | PE0 | 单击依次标定 0→1→2→3→4，点 4 停留 3s 自动结束回 IDLE（也可单击提前结束） |
| KEY_RESET | PB5 | 开环复位到原点（用标定坐标，角度闭环） |
| KEY_BORDER | PB9 | 开环沿屏幕边线顺时针一周 |
| KEY_PAUSE | PB3 | 暂停/继续（立即制动，线圈锁定） |
| KEY_TRACK / KEY_A4 | PB8 / PE1 | 预留（追踪属绿系统；A4 需视觉角点） |

> 按键默认按**低电平有效**（`app.c` 中 `APP_KEY_PRESSED_LEVEL`），高有效电路改成 `GPIO_PIN_SET`。
> `.ioc` 中 6 个按键已配为 `GPIO_PULLUP`（由 `MX_GPIO_Init` 生效，按键接地）；若某块板在 CubeMX 里改回
> NOPULL 且无外部上拉，把 `app.c` 的 `APP_KEY_CFG_PULL` 置 1 即可由驱动补内部上拉。
> 另有上电 300ms 按键屏蔽窗口（`APP_KEY_STARTUP_MS`），确保**上电稳定停在 IDLE，不会误进 CALIB**。

### CALIB 标定流程

> 标定期间 `app_run()` **每周期无条件拉高两轴 EN**（`MotorAxis_Enable(...,0)`），
> 任何路径（含调试命令）重新使能都会被立即撤销，保证标定全程电机完全自由。
> 进入标定时串口打印 `EN1(PD8)=x EN2(PE15)=x (1=released)` 可直接核对。

1. 按 KEY_CALIB 进入标定点 0（中心），**两轴解锁**，手动把光斑摆到目标点
2. 期间每 1s 自动采样当前 X/Y 角度并**覆写**到 `s_cal_pan[5]/s_cal_tilt[5]`，串口打印 `>> CALIB rec ...`
3. 按 KEY_CALIB 进入下一个点（1 左上→2 右上→3 右下→4 左下）
4. 点 4 处每 1s 继续采样，满 3 次（3s）**自动结束**；也可再单击一次立即结束
5. 结束时由 5 点求仿射运动学 `k/b`，两轴重新锁定，回 IDLE

标定点屏幕坐标（cm，原点=屏中心，x 右 y 上）：`0(0,0) 1(-25,25) 2(25,25) 3(25,-25) 4(-25,-25)`。

### RESET / BORDER（角度空间开环 + 角度闭环）

**不经过屏幕坐标/逆解**：直接以 CALIB 记录的**轴角**为目标，误差 = 预设角 − 当前角 → 角度 PID 闭环。
（屏幕坐标+仿射只留给需要视觉的 A4/追踪。）

- **RESET**：目标 = 标定点 0（中心）的 pan/tilt 角。设定点从当前角按 `APP_ANG_RATE`（20°/s）推进到目标，
  自动取最短路径（跨 0/360 无跳变）；到达后等两轴闭环收敛（PID HOLDING）才结束。
- **BORDER**：目标路径 = 标定点 1→2→3→4（左上→右上→右下→左下，顺时针），相邻顶点角度线性插值，闭合 1 圈。
- 标定点即路径顶点 → **角点处精确**；标定矩阵退化也不会出现"轨迹无限长导致一直转"。
- 另有保护：起始坐标夹取 `MOTION_XY_LIMIT`、有限运动总时长上限 `MOTION_MOVE_TIMEOUT_MS`（20s）强制停止。

## 驱动移植（Core/Lib，源自 DriverLib_King_orz）

| 驱动 | 说明 | 状态 |
|------|------|------|
| `uart.c/h` | DMA 串口库（句柄化：`UART_Device`，uart_dbg=USART1 / uart_cam=USART2） | 已移植 |
| `mt6701.c/h` | 磁编码器（句柄化改造，支持 I2C1/I2C2 双实例） | 已移植 |
| `tmc2209.c/h` | TMC2209 驱动板（原生句柄化） | 已移植 |
| `motor_stepper.c/h` | STEP 脉冲引擎（句柄化改造，TIM13/TIM14 双实例） | 已移植 |
| `motor_pid.c/h` | 位置闭环 PID（原生句柄化） | 已移植 |
| `motor_axis.c/h` | **L1 轴控层**：单轴闭环（MT6701+PID+Stepper），角度→执行 | 新增 |
| `motion.c/h` | **L2 运动层**：屏幕 cm 坐标仿射运动学 + 轨迹原语 + 视觉外环 | 新增 |
| `motion_arbiter.c/h` | **L3 仲裁层**：多请求源优先级仲裁 + 暂停/急停总覆盖 | 新增 |
| `app.c/h` | **L4 应用层**：按键状态机 + CALIB 标定 + 开环 RESET/BORDER + 串口打印 | 新增 |
| `st7735.c/h` | ST7735 SPI 屏驱动（预留骨架，待实现） | 已建 |

> 注意事项：
> - 两个电机在 `main.c` USER CODE 区实例化：`tilt`（TIM14+I2C1）、`pan`（TIM13+I2C2）
> - TIM 中断分发（`TIM8_TRG_COM_TIM14_IRQHandler`/`TIM8_UP_TIM13_IRQHandler`）在 main.c USER CODE BEGIN 0 区
> - CubeMX 重新生成前请确认 `.ioc` 中 TIM13/TIM14 已注册（已注册），生成后 `stm32f4xx_hal_conf.h` 会自动启用 HAL_TIM

## 构建

```
cmake --preset Debug
cmake --build --preset Debug
# 产物: build/Debug/STM32_MotorKing_orz.elf
```

## 待办

- [ ] 摄像头帧协议约定（USART2）
- [x] 6 个按键注册（KEY_PAUSE/RESET/TRACK/BORDER/CALIB/A4）
- [x] 电机2 引脚变更（已移至 PE7/PE9/PE11/PE13/PE15）
- [x] 6 个按键 GPIO 上拉（`.ioc` GPIO_PULLUP，低有效）
- [x] USART2 RX DMA 改 Circular
- [ ] 位置闭环标定（细分按 TMC2209 驱动板说明）
- [x] 运动分层库（L1 `motor_axis` / L2 `motion` / L3 `motion_arbiter`）
- [x] 应用层（L4 `app`）：串口角度打印、CALIB 标定、开环 RESET/BORDER、暂停
- [x] 运动学仿射标定（CALIB 模式 5 点实测自动求 k/b）
- [x] `main.c` 集成：`APP_Init()` / `APP_Task()`
- [ ] 实机标定（确认 pan/tilt 正负方向与 `APP_KEY_PRESSED_LEVEL`）
- [ ] 视觉闭环：USART2 光斑坐标 → `Motion_SetSpotXY`（协议待约定）
- [ ] A4 靶纸（需视觉四角点）与绿系统追踪
- [ ] ST7735 LCD 驱动实现（骨架已建，Core/Lib/st7735.c/h）