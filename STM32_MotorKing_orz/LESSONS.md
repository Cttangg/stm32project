# 调试教训记录 (Lessons Learned)

> 本文件记录 STM32_MotorKing_orz 开发/调试过程中发现的问题与教训,
> 供后续开发与排障参考。按时间顺序追加。

---

## 2026-09-29 裸 PID 控制电机 1 (Tilt) 调试

### L1. 步进定时器每 5ms 停表重配 → 丢步/抖动

**现象**: PID 闭环时电机剧烈抖动并丢步; 开环 MoveSteps 恒定频率时平滑安静。

**根因**: `MotorStepper_SetVelocity()` → `stepper_set_rate()` 被 PID 每 5ms 调用一次,
旧代码**无条件**执行 "停表 → 清计数器 → EGR 装载 → phase=0 → 重启"。
- 停表重启丢失定时器连续性, 相位强制归零可能连续两次 STEP 或丢半脉冲。
- 计数清零使正在走的周期被截断。

**修复**: `stepper_set_rate()` 中频率差 < 1 steps/s 时直接返回, 不触碰定时器。
PID 斜坡收敛阶段 vel_ref 变化小时不再频繁停表。

**教训**: 硬件定时器配置是"状态", 不是"每次调用都重写"的操作。
高频周期任务 (PID) 调用底层驱动时, 驱动必须先判断"是否需要变化"再操作硬件。

---

### L2. PID 速度方向与编码器读数方向不匹配 → 永远追不上目标

**现象**: `go 26` (目标 > 当前) 后电机持续转向读数**减小**方向 (绕过 0° 到 359°),
读数平滑但永远追不上目标, 最终停在 180° 附近 (wrap 边界), 电机长时间满速转。

**根因**: 实测 (spin 开环): 电机 CCW 转 → 编码器读数**递减**。
但 PID 假设 "正速度 = 读数增大"。误差计算 `wrap180(target - current)` 的方向
与真实机械方向相反 → PID 往错误方向追。

**修复**: 应用层对 PID 输出符号反转:
```c
MotorStepper_SetVelocity(&stepper_tilt, -pid_tilt.vel_ref);
```

**教训**: 方向标定不能靠代码假设, 必须实测 (开环走已知步数观察编码器变化)。
符号不匹配的闭环比没有闭环更危险 (电机满速追错方向)。

---

### L3. I2C 偶发故障: 读失败不能返回 0

**现象**: 编码器读失败返回 0 (0°), 被 Axis 限位检查 `angle <= min` 误判为 LIMIT FAULT;
或 `IsMagnetOK` 读失败返回 0 被误判为磁铁异常 FAULT → 单轴停摆。

**根因**: 0 是 MT6701 的合法角度 (0°), 不能用作读失败哨兵值。

**修复**:
- `MT6701_ReadRaw()` 读失败返回 `last_raw` (上次有效值)。
- `MT6701_IsMagnetOK()` 读失败保持 `mag_ok` 上次判定。
- 句柄增加 `last_raw` / `mag_ok` 字段。

**教训**: 传感器读失败返回值必须区分 "读失败" 与 "合法值", 不能共用一个哨兵。

---

### L4. I2C 总线恢复 (9 SCK) 要限频, 不能用 HAL_Delay

**现象**: I2C 持续故障时, 每次读失败都执行 BusRecover (含 22×HAL_Delay(1) ≈ 22ms),
主循环被拖死 → 串口失联。

**修复**:
- BusRecover 的 `HAL_Delay(1)` 换成 `for (volatile d=0; d<1000; d++)` 忙等 (~µs 级)。
- 恢复限频: 句柄 `last_recover_ms`, 1 秒最多恢复一次。

**教训**: 任何驱动里的阻塞延时在故障路径上都会放大成主循环卡死;
恢复/重试操作必须限频。

---

### L5. I2C 阻塞读 + 高频 STEP 中断同优先级 → 读数错位

**现象**: PID 高速运动时编码器读数跳到半圈错位 (26° ↔ 208°, 差 ~2^13 LSB)。

**修复 (辅助)**: STEP 定时器中断优先级从 (0,0) 降到 (2,0), 低于 I2C 事件中断。
I2C 事务需要原子完成, 不能被高频中断打断时序。

**教训**: 高频外设中断 (STEP) 不应与需要原子性的事务外设 (I2C) 同优先级。
(注: 实际主因是 L2 方向问题, 此修复为辅助加固)

---

### L6. 启动时 PID 目标默认 0° → 全速转半圈

**现象**: App 重启后 Pan 从 180° 全速追默认目标 0°, 大幅旋转。

**修复**: 启动时 `MotorPID_SetTarget(&pid, MT6701_ReadDegrees(&enc))` 目标=当前位置。

**教训**: 位置闭环的初始目标必须是当前位置, 不能是默认 0/未定义值。

---

### L7. newlib-nano 的 sscanf/snprintf %f 不可用

**现象**: `snprintf %f` 输出乱码 (0x1F), `sscanf %f` 解析失败, 命令不生效。

**根因**: `--specs=nano.specs` 默认不链接浮点格式化/解析, `%f` 无效。

**修复**:
- 链接加 `-u _printf_float -u _scanf_float` (FLASH +16KB)。
- 命令解析改用自写 `DbgCmd_ParseFloat()` (纯整数运算), 不依赖库。

**教训**: 嵌入式 nano libc 的浮点 I/O 默认关闭, 需显式链接或自实现。

---

### L8. 编码器读数跳变校验的坑

**设计**: `MT6701_ReadRaw` 中, 读数与 last_raw 的带环绕差 > 30° 视为脏数据丢弃。
- 首读用 `last_raw = 0xFFFF` 标记, 直接接受。
- 正常运动 5ms 采样变化 < 6°, 阈值 30° 安全。

**注意**: 校验基于 last_raw, 若 last_raw 被连续错误值污染则校验失效
(见 L3 修复: 读失败保持 last_raw 缓解)。

**教训**: 数据校验要防"连续一致的错误值", 单值校验无法完全替代硬件可靠性。

---

## 其他 (Bootloader / 烧录)

### L9. BL 烧录后不自动跳 App, 需 `-g` 启动

**现象**: 烧录成功后串口无响应, 以为 App 没跑; 实际 BL 停在等待。

**修复**: CubeProgrammer 烧录后执行:
```
STM32_Programmer_CLI -c port=COM10 mode=OB -g 0x08008000
```
(BL 协议 GO 命令跳转 App)

### L10. 软件回 BL: App 写 magic + 复位

App 侧 `APP_JumpToBootloader()`: 写 `0x2001FFFC = 0xDEADBEEF` → `NVIC_SystemReset()`。
BL 上电检测 magic 决定留在等待还是跳 App。串口发 `boot` 命令可远程进 BL, 免按复位键。

---

## 文件位置速查

| 问题 | 文件 |
|---|---|
| 定时器重配优化 | `Core/Lib/motor_stepper.c` (`stepper_set_rate`) |
| PID 方向标定 | `Core/Src/main.c` (`-pid_tilt.vel_ref`) |
| 编码器容错/恢复 | `Core/Lib/mt6701.c/.h` |
| 中断优先级 | `Core/Src/stm32f4xx_hal_msp.c` |
| 链接浮点 | `cmake/gcc-arm-none-eabi.cmake` |
| App 布局/BL 接口 | `STM32F407xx_FLASH.ld`, `Core/Lib/app_boot_interface.h` |