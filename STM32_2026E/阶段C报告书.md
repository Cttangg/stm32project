# STM32_2026E 阶段 C 报告书 — 单轴角度闭环（丝杆 2mm/r）

| 项目 | 内容 |
|------|------|
| 阶段 | C（单轴角度闭环） |
| 本阶段范围 | 编码器角度闭环、丝杆 2mm/r 换算、无指令不自走 |
| 不在本阶段 | 串口解析（由设计者实现）、限位/回零、XY/Z 协调、视觉、取放 |
| 编译状态 | **未编译**（工具链缺失，见 §8） |
| 硬件测试 | **未进行**（硬件搭建中） |

---

## 1. 本阶段目标

按指令："加入角度闭环，实际结构为丝杆 2mm/r，目前无限位开关所以不要自己走步，串口解析逻辑稍后设计。"

1. 将 MT6701 编码器角度反馈接入 `AxisController`，实现**角度位置闭环**（PID 输出速度给 `MotorStepper`）。
2. 引入丝杆导程参数 **2mm/r**，建立 `mm ↔ deg ↔ steps` 换算。
3. **无指令不自走**：上电/使能不产生运动，仅当上层调用 `App_SetTargetMM()` 后才运动。
4. 提供命令接口供后续串口解析层调用；**不实现**解析逻辑。

---

## 2. 修改文件清单

| 文件 | 改动 |
|------|------|
| `Core/Lib/motor_pid.h` | 新增字段 `uint8_t angular;`；新增 `MotorPID_SetMode()` |
| `Core/Lib/motor_pid.c` | 新增 `axis_error()`；`GetError`/`Update` 按模式决定是否卷绕；Init 默认 `angular=1` |
| `Core/Lib/motor_stepper.c` | `SetVelocity` 增加"同向且速度几乎不变则不重配定时器"防抖 |
| `Motion/motion_types.h` | 新增 `AxisFeedbackReadFn`；`AxisStatus_t` 增加 `current_mm/target_mm/closed_loop` |
| `Motion/motion_config.h` | 新增丝杆参数 `AXIS_LEAD_MM_PER_REV=2.0f`、`AXIS_MM_PER_DEG`、`AXIS_DEG_PER_MM`、`AXIS_STEPS_PER_MM`、`AXIS_CONTROL_PERIOD_MS=5` |
| `Motion/axis_controller.h` | 新增闭环字段与 `SetFeedback` / `SetTargetMM` 接口 |
| `Motion/axis_controller.c` | 新增闭环分支 `axis_update_closed_loop()`；开环/闭环双模式 |
| `App/app.h` | 新增 `App_Init(stepper, encoder, uart)` 与命令接口 `App_SetTargetMM/Stop/Enable/Disable/GetStatus` |
| `App/app.c` | 绑定编码器反馈回调；移除临时调试命令；无自动运动 |
| `Core/Src/main.c` | `App_Init(&stepper_tilt, &enc_tilt, &uart_dbg)` |

---

## 3. 架构变化

```
App/app.c  ── 命令接口 (App_SetTargetMM / App_Stop ...)
   │              ▲ 串口解析层 (待设计) 将调用
   ▼
Motion/axis_controller.c
   │  closed_loop: MT6701 反馈回调 → MotorPID(deg) → vel_ref
   │  open_loop:   步数计数 (保留)
   ▼
Core/Lib/motor_stepper.c  →  TMC2209 + TIM 中断
```

- Motion 层通过 `AxisFeedbackReadFn` 回调获取反馈，**不直接依赖 MT6701**（解耦）。
- PID 单位仍为**角度 deg**（连续、多圈累计、不卷绕），增益沿用原角度整定值。

---

## 4. 接口变化

```c
/* Motion */
void         AxisController_SetFeedback(AxisController *ax, AxisFeedbackReadFn read, void *ctx);
AxisResult_t AxisController_SetTargetMM(AxisController *ax, float target_mm);

/* PID */
void         MotorPID_SetMode(MotorPID *p, uint8_t angular);   /* 0=连续位置不卷绕 */

/* App (供串口解析层) */
AxisResult_t App_SetTargetMM(float target_mm);
AxisResult_t App_Stop(void);
AxisResult_t App_Enable(void);
void         App_Disable(void);
AxisStatus_t App_GetStatus(void);
```

---

## 5. 关键实现说明

1. **换算**（`motion_config.h`）
   - `AXIS_LEAD_MM_PER_REV = 2.0`，`AXIS_DEG_PER_MM = 180`，`AXIS_MM_PER_DEG ≈ 0.005556`
   - `AXIS_STEPS_PER_MM = 6400 / 2 = 3200`
2. **反馈**：`enc_read_deg()` 调 `MT6701_ReadMultiTurn()`，返回 `turns*360 + 单圈角`（连续 deg）。
3. **闭环周期**：`AxisController_Update()` 每 `AXIS_CONTROL_PERIOD_MS = 5ms` 执行一次：读反馈 → `MotorPID_Update(pid, deg, dt)` → `MotorStepper_SetVelocity(pid.vel_ref)`。
4. **不卷绕**：`MotorPID_SetMode(0)` → `axis_error()` 直接返回 `target-current`（丝杆多圈不能卷绕）。
5. **无指令不自走**：`MotorPID` 初始 `DISABLED`，仅 `SetTargetMM → MotorPID_SetTarget` 才使能；上电无目标 → `vel_ref=0` → 不运动。
6. **防抖**：`MotorStepper_SetVelocity` 在"同向且速度变化 <0.5 steps/s"时跳过定时器重配，避免 5ms 周期反复启停定时器。
7. **保护**：闭环 MOVING 超时（10s）→ 停表 + `ERROR`；编码器未应答时自动退回开环（`App_Init` 检查 `inited`）。

---

## 6. 状态机说明

闭环下 `AxisController.state` 由 PID 状态映射：

```
PID DISABLED  → AXIS DISABLED / IDLE
PID MOVING/APPROACHING → AXIS MOVING
PID HOLDING   → AXIS DONE
超时          → AXIS ERROR (需 ClearFault)
```

`App_SetTargetMM` 可从 IDLE/DONE 重新进入 MOVING（支持连续指令）。

---

## 7. 安全影响分析

| 项 | 现状 |
|----|------|
| 急停/限位 | **无硬件、无接口**；`AxisController_Stop()` 为唯一软件停止 |
| 无指令不自走 | **已满足**：PID 未使能时不输出速度 |
| 编码器缺失保护 | **已实现**：`inited==0` → 开环，不闭环乱走 |
| 超时 | **已实现**（闭环/开环均有） |
| 闭环极性 | **未验证**：若编码器正方向与电机 CCW 相反，闭环将发散；须硬件阶段确认（可交换反馈符号） |
| 越界 | **未实现**（无限位、行程未知，不硬编码） |

**结论**：具备"无指令不动 + 超时保护"的软件骨架；**不满足任务书 §6 的真实安全能力**，待硬件确认。

---

## 8. 测试命令与测试结果

```
cmake --preset Debug
cmake --build --preset Debug
```
**结果：失败**（`arm-none-eabi-gcc` 不在 PATH，bundle 无 `gnu-tools-for-stm32`）→ 未编译、未静态检查、未硬件测试。

**硬件就绪后验收**：
1. 手转丝杆，读 `App_GetStatus().current_mm`，核对 1 圈 = 2.00mm。
2. `App_SetTargetMM(+2.0f)`：观察方向、到位误差、是否 HOLDING。
3. 若发散，交换反馈符号或电机 DIR 后复测。
4. 断编码器上电：应报 `OPEN-LOOP` 且不自走。

---

## 9. 已知问题

1. **无法编译**：缺 `gnu-tools-for-stm32`。
2. **闭环极性未验证**（见 §7）。
3. **PID 增益沿用云台角度整定值**，丝杆负载不同，需重新整定（kp/ki/kd、deadband、stiction_comp）。
4. **`MT6701` 读为阻塞 I2C**（≤50ms 超时），在 5ms 控制回路中可能抖动（阶段 A R3）。
5. **无软限位/回零**，`current_mm` 以上电位置为 0，且为多圈相对值。
6. **USART1 调试口未读取**，RX 环形会溢出（待串口解析层接入）。
7. **超时固定 10s**，长距离运动可能误判，需按行程调整。

---

## 10. 下一阶段建议

1. 解决工具链 → 编译 + 单轴闭环硬件验收（§8）。
2. 实现串口命令解析层（设计者），调用 `App_SetTargetMM/Stop/...`。
3. 硬件到位后补限位/急停/回零，替换"上电即零点"的相对位置模型。
4. 重新整定闭环参数并记录。
5. 之后扩展多轴 MotionManager → 视觉 → 取放 → 拼图任务。

---

### 附：未验证声明
本阶段代码**未经编译、未经硬件运行**；"实现完成"仅指代码按接口落地，**不代表功能正确**。
