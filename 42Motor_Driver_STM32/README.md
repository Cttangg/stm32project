# 42 步进电机 + MT6701 编码器 位置闭环系统

> **MCU**: STM32F407VETx (168MHz) | **执行器**: 42 步进电机 + TMC2209 驱动板 | **反馈**: MT6701 14-bit 磁编码器

## 1. 系统架构

```
         ┌─────────────┐   位置误差(deg)          ┌───────────────┐
 MT6701 ─┤  编码器反馈  ├────────────► PID 位置环 │ velocity_ref  │
(14bit)  │  0.022°/LSB │  (5ms 采样)             │  (steps/s)    │
         └──────┬──────┘                         └──────┬────────┘
                ▲                                       │
                │                                       ▼
                │                              ┌──────────────────┐
                │                              │ MotorStepper     │
                │                              │ TIM14 硬件定时器  │
                │                              │ (非阻塞 STEP)     │
                │                              └──────┬───────────┘
                │                                     ▼
                └────────────────── 42步进电机 ◄── TMC2209 (EN/DIR/MS)
```

- **MT6701**：读取绝对角度 → 位置反馈
- **MotorPID**：位置闭环，输出连续速度参考（steps/s），含 D 滤波/抗积分/滞回/死区补偿
- **MotorStepper**：TIM14 硬件定时器把速度转成 STEP 脉冲（CPU 零等待）
- **TMC2209**：GPIO 控制使能/方向/细分
- **UART**：调试 shell + VOFA+ firewater 实时输出

## 2. 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| MT6701 SCL | PB6 (I2C1) | 400kHz |
| MT6701 SDA | PB7 (I2C1) | |
| 电机 EN | PA1 | 低有效 |
| 电机 STEP | PC13 | TIM14 ISR 翻转 |
| 电机 DIR | PE5 | |
| 电机 MS1 | PC1 | 细分 |
| 电机 MS2 | PC3 | 细分 |
| USART1 TX/RX | PA9/PA10 | 115200 调试口 |
| TIM14 | — | STEP 脉冲定时器 |

## 3. 模块与上层接口

### 3.1 MT6701 编码器 — `Core/Lib/mt6701.h`

| 函数 | 说明 |
|------|------|
| `int MT6701_Init(void)` | 初始化（I2C 地址 0x06 应答校验），0=成功 |
| `uint16_t MT6701_ReadRaw(void)` | 原始 14-bit 值 0~16383 |
| `float MT6701_ReadDegrees(void)` | 角度 0~360° |
| `float MT6701_ReadRadians(void)` | 弧度 0~2π |
| `float MT6701_ReadMultiTurn(int32_t *turns)` | 累积圈数 + 圈内角度 |
| `uint8_t MT6701_IsMagnetOK(void)` | 磁铁状态 1=正常 |
| `uint8_t MT6701_IsDataUpdated(void)` | 数据更新标志 |
| `int MT6701_SetZeroPosition(void)` | 当前角度设为 0° |
| `void MT6701_SetFilter(uint8_t level)` | 滤波级别 |

### 3.2 TMC2209 驱动 — `Core/Lib/tmc2209.h`（详见头文件内文档+接线图）

| 函数 | 说明 |
|------|------|
| `void TMC2209_Init(&h, &PinConfig)` | 绑定 GPIO，默认 1/32 细分 |
| `void TMC2209_Enable(&h)` / `Disable` | 使能/释放（EN 低有效） |
| `void TMC2209_SetDirection(&h, dir)` | `TMC2209_DIR_CCW/CW` |
| `void TMC2209_SetMicrostep(&h, ms)` | `TMC2209_MICROSTEP_8/16/32/64` |

### 3.3 步进脉冲引擎 — `Core/Lib/motor_stepper.h`

| 函数 | 说明 |
|------|------|
| `void MotorStepper_Init(&h, &htim14)` | 初始化 TIM14 + NVIC |
| `void MotorStepper_SetVelocity(float steps_per_sec)` | **核心**：正=CCW 负=CW 0=停，硬件定时器发脉冲，非阻塞 |
| `void MotorStepper_MoveSteps(uint32_t n, float rate)` | 手动走 N 步（到数自动停） |
| `void MotorStepper_Stop(void)` | 停止 |
| `uint32_t MotorStepper_GetSteps(void)` | 累计步数 |

### 3.4 位置闭环 PID — `Core/Lib/motor_pid.h`

单位：误差 deg，输出 steps/s，加速度 steps/s²，kp=(steps/s)/deg，kd=steps/deg

| 函数 | 说明 |
|------|------|
| `void MotorPID_Init(&p)` | 默认 1/32 标定（kp=1500,ki=100,kd=125） |
| `void MotorPID_SetGains(&p, kp, ki, kd)` | 在线调参 |
| `void MotorPID_SetTarget(&p, deg)` | 设定目标并使能（无 derivative kick） |
| `void MotorPID_Disable(&p)` | 停止控制 |
| `float MotorPID_GetTarget/GetError(&p)` | 查询目标/误差(deg) |
| `void MotorPID_Update(&p, angle, dt)` | 5ms 周期调用：角度→vel_ref |
| `void MotorPID_RescaleMicrostep(&p, old, new)` | 切细分后重标定 kp/限幅/补偿 |

关键参数（`motor_pid.c` 默认）：
- `kp=1500, ki=100, kd=125, d_alpha=0.8`
- `max_speed=19200 steps/s（~3 rev/s）`, `max_accel=96000 steps/s²`
- `deadband_enter=0.05°, deadband_exit=0.5°（滞回）`
- `stiction_comp=150 steps/s（死区补偿, 0.05°<|e|<0.5° 时叠加推过静摩擦）`

### 3.5 串口框架 — `Core/Lib/uart.h`（详见 README_UART.md）

| 函数 | 说明 |
|------|------|
| `UART_Init/Open/Close` | 生命周期 |
| `UART_Send(port,data,len,&written)` | 非阻塞 DMA 发送 |
| `UART_Available/Read` | 原始字节接收 |
| `UART_RegisterFrame(port,&frame)` | 注册帧协议 |
| `UART_Task()` | 主循环周期调用（帧解析/回调） |

## 4. 调试串口命令（USART1, 115200）

```
enable 0/1        使能/禁止电机
ms 8/16/32/64     切换细分 (自动重标定 PID)
dir 0/1           手动方向
step N            手动走 N 步 (非阻塞)
pos <deg>         位置闭环: 锁定到指定角度
pid on/off        使能/停止 PID
pid               查看增益
pkp <v> pki <v> pkd <v>   在线调参
help
```

VOFA+ firewater（10ms 一行）：
```
angle,target,error,vel_cmd,vel_ref,d_term,i_term
```

## 5. 单位换算（1/32 细分）

| 常量 | 值 |
|------|-----|
| STEPS_PER_REV | 6400 (200 全步 × 32) |
| DEG_PER_STEP | 0.05625° |
| STEPS_PER_DEG | 17.78 |
| max_speed | 19200 steps/s ≈ 3 rev/s ≈ 1080°/s |

## 6. 构建

VSCode + CMake + Ninja + arm-none-eabi-gcc，CubeMX 生成 HAL。固件输出 `build/Debug/42Motor_Driver_STM32.elf`。
