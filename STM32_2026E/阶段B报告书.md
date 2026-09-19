# STM32_2026E 阶段 B 报告书 — 单轴最小实现（MotorStepper + AxisController）

| 项目 | 内容 |
|------|------|
| 阶段 | B（设备与底层 / 单轴最小实现） |
| 范围依据 | 《龙门架移动拼图系统：总体框架设计指南》§五、§十四 |
| 本阶段范围 | **仅一个轴**：固定方向/步数/速度、启动、停止、完成检测、再次启动、异常状态 |
| 不在本阶段 | XY/Z 协调、视觉、Host 协议、取放机构、回零、运动学 |
| 编译状态 | **未编译通过**（工具链缺失，见 §9-1） |
| 硬件测试 | **未进行**（硬件搭建中） |

---

## 1. 本阶段目标

按设计指南"先建立可靠的 MotorStepper → 再建立 AxisController → 用一个轴完成完整走步测试"的路线，落地：

1. 新增 **Motion 层** `AxisController`：单轴状态机、位置/目标管理、速度限制、停止与超时/故障处理、错误码。
2. 在现有 **Devices 层** `MotorStepper` 上补齐设计指南要求的接口（Enable/Disable/SetDirection/IsBusy/GetRemaining/GetExecuted）。
3. 新增 **Application 层** `app`：非阻塞主循环调度 + 串口调试命令，用于验证状态机与错误处理。
4. **不引入** MotionManager / Planner / 取放 / 视觉等未到阶段的模块。

---

## 2. 修改文件清单

### 新增
| 文件 | 说明 |
|------|------|
| `Motion/motion_types.h` | `AxisState_t` / `AxisResult_t` / `AxisMoveRequest_t` / `AxisStatus_t`（无硬件依赖） |
| `Motion/motion_config.h` | 默认速度、速度上限、演示步数、运动超时（可标定） |
| `Motion/axis_controller.h` | 单轴控制器接口 |
| `Motion/axis_controller.c` | 单轴状态机实现（非阻塞） |
| `App/app.h` | 应用层接口 `App_Init` / `App_Update` |
| `App/app.c` | 单轴调度 + USART1 调试命令 + 状态上报 |

### 修改
| 文件 | 改动 |
|------|------|
| `Core/Lib/motor_stepper.h` | 新增 `MotorStepper_Enable/Disable/SetDirection/IsBusy/GetRemainingSteps/GetExecutedSteps` 声明 |
| `Core/Lib/motor_stepper.c` | 对应实现；`SetDirection` 采用"停表→设 DIR"安全切换 |
| `Core/Src/main.c` | 引入 `app.h`；`USER CODE 2` 调 `App_Init(&stepper_tilt, &uart_dbg)`；主循环改为 `UART_Task(&uart_cam)` + `App_Update()`（USART1 改由 App 直接读原始字节） |
| `CMakeLists.txt` | 增加源 `Motion/axis_controller.c`、`App/app.c`；增加包含目录 `Motion`、`App` |

> 现有 `Core/Lib/tmc2209.*`、`mt6701.*`、`uart.*`、`motor_pid.*` **未改动**（`motor_pid` 本阶段未接入）。

---

## 3. 架构变化

引入设计指南四层中的三层（BSP 由现有 CubeMX + `Core/Lib` 驱动承担）：

```
App/app.c                      ← 本阶段新增 (调度 + 调试命令)
   │  只调用 AxisController
   ▼
Motion/axis_controller.c       ← 本阶段新增 (单轴状态机, 唯一运动入口)
   │  只调用 MotorStepper
   ▼
Core/Lib/motor_stepper.c       ← 已有, 本阶段补齐接口 (Devices)
   │  只操作 TMC2209 GPIO + TIM
   ▼
Core/Lib/tmc2209.c + TIM13/14  ← 已有 (BSP/Drivers)
```

**依赖方向**：App → Motion → Devices → BSP，无反向依赖。Motion 层头文件不含 HAL/GPIO/TIM 头（`motion_types.h` 仅 `<stdint.h>`），符合解耦要求。

**与指南的偏差（待确认）**：现有驱动仍在 `Core/Lib/`，未迁入指南建议的 `Devices/`、`BSP/` 目录，以遵循"最小改动、不过早搬动已验证代码"原则。是否迁移请总设计师裁定。

---

## 4. 接口变化

### 4.1 Devices 层新增（`motor_stepper.h`）
```c
void     MotorStepper_Enable(MotorStepper *s);
void     MotorStepper_Disable(MotorStepper *s);
void     MotorStepper_SetDirection(MotorStepper *s, TMC2209_Dir dir);
uint8_t  MotorStepper_IsBusy(const MotorStepper *s);
uint32_t MotorStepper_GetRemainingSteps(const MotorStepper *s);
uint32_t MotorStepper_GetExecutedSteps(const MotorStepper *s);
```
`MotorStepper` 现在自包含 EN/DIR 控制（转发 TMC2209），Motion 层**不再直接依赖 TMC2209**。

### 4.2 Motion 层新增（`axis_controller.h`）
```c
void         AxisController_Init(AxisController *ax, const char *name, MotorStepper *stepper);
AxisResult_t AxisController_Enable(AxisController *ax);
void         AxisController_Disable(AxisController *ax);
AxisResult_t AxisController_MoveSteps(AxisController *ax, int32_t steps, uint32_t velocity);
AxisResult_t AxisController_MoveTo(AxisController *ax, int32_t absolute_steps, uint32_t velocity);
AxisResult_t AxisController_Stop(AxisController *ax);
void         AxisController_ClearFault(AxisController *ax);
void         AxisController_Update(AxisController *ax);
AxisState_t  AxisController_GetState(const AxisController *ax);
AxisStatus_t AxisController_GetStatus(const AxisController *ax);
```
**所有 API 非阻塞**；推进靠 `AxisController_Update()`。

### 4.3 App 层
```c
void App_Init(MotorStepper *axis_x_stepper, UART_Device *debug_uart);
void App_Update(void);
```

---

## 5. 关键实现说明

1. **非阻塞脉冲**：STEP 仍由 TIM13/TIM14 更新中断翻转 GPIO（`motor_stepper.c`），主循环只做状态推进，无 `while(!done)`、无 `HAL_Delay`。
2. **位置模型（开环）**：`current_steps` = 软件累计步数。运动期间按 `move_start_steps + dir_sign × (硬件计数 − 起始计数)` 计算，完成时直接赋 `target_steps` 以消除采样误差（`axis_controller.c`）。**未接编码器/回零，该值只代表"控制器认为走过的步数"**。
3. **方向切换安全**：`MotorStepper_SetDirection` 先停表再设 DIR，随后 `MoveSteps` 启动，首个更新周期提供 DIR 建立时间。
4. **超时**：`AxisController_Update` 在 MOVING 态用 `HAL_GetTick()` 判超时，超时即 `MotorStepper_Stop` 并进入 ERROR（`last_error=AXIS_RESULT_TIMEOUT`）。
5. **错误码**：所有请求返回 `AxisResult_t`，而非 bool（指南 §十二）。
6. **调试命令**（USART1，逐字节）：`g` 正走 / `r` 反走 / `s` 停 / `e` 使能 / `d` 禁止 / `c` 清故障 / `?` 状态；状态变化自动上报。

---

## 6. 状态机说明

```
        Init
         │
         ▼
   ┌──────────┐  Enable   ┌──────┐  MoveSteps   ┌────────┐
   │ DISABLED │──────────►│ IDLE │─────────────►│ MOVING │
   └──────────┘           └──────┘              └───┬────┘
        ▲                    ▲                      │
        │ Disable            │                      ├── 底层完成 ─► DONE ──(新请求)─► MOVING
        │                    │                      │
        │                    │   Stop               ├── Stop ────► STOPPING ─(确认)─► IDLE
        │                    │◄─────────────────────┤
        │                    │                      └── 超时 ────► ERROR
        │                    │                                     │ ClearFault
        └────────────────────┴─────────────────────────────────────┘
```

- `DONE` 表示上一段完成；收到新 `MoveSteps` 直接回 `MOVING`（支持"再次启动"）。
- `STOPPING` 为停止确认过渡态（底层立即断脉冲，下一次 `Update` 转 `IDLE`）。
- `ERROR` 仅能由 `ClearFault` 离开（回到 IDLE/DISABLED）。

---

## 7. 安全影响分析

| 项 | 现状 |
|----|------|
| 急停 | **无硬件、无接口**；`AxisController_Stop` 仅软件停止。急停链路待硬件确认。 |
| 限位 | **无**；已在 `motion_types.h` 预留 `AXIS_RESULT_LIMIT_TRIGGERED`，未接线。 |
| 驱动器故障 | 已预留 `AXIS_RESULT_DRIVER_FAULT` 与 `fault` 标志，**未接 TMC DIAG**。 |
| 超时 | 已实现（`AXIS_MOVE_TIMEOUT_MS`）。 |
| 越界 | **未实现软限位**（行程未知，不硬编码）。 |
| Z 轴失电下落 | 不适用（本阶段无 Z）。 |

**结论**：本阶段仅建立"可停止/可超时"的软件安全骨架，**不具备满足任务书 §6 的真实安全能力**，需硬件确认后补齐。

---

## 8. 测试命令与测试结果

### 8.1 编译
```
cmake --preset Debug
cmake --build --preset Debug
```
**结果：失败（未进入编译阶段）**
```
The CMAKE_C_COMPILER: arm-none-eabi-gcc is not a full path and was not found in the PATH.
The CMAKE_CXX_COMPILER: arm-none-eabi-g++ is not a full path and was not found in the PATH.
```
原因：本机 `%LOCALAPPDATA%\stm32cube\bundles` 无 `gnu-tools-for-stm32` 包，`arm-none-eabi-*` 不在 PATH。

### 8.2 静态检查
- 无可用编译器/静态分析工具 → **未执行**。
- 已人工复核：头文件包含链、函数声明/定义一致、volatile 快照、状态迁移分支。

### 8.3 硬件测试
**未进行**（硬件搭建中，且无固件可烧录）。

> 依据任务书 §7：**未测试部分明确声明为未验证**，不声称已验证。

### 8.4 待硬件就绪后的验收步骤
1. 底层脉冲：示波器/逻辑分析仪测 STEP 频率、脉宽、DIR 建立时间。
2. 单轴控制器：`g`/`r` 正反走、`s` 运动中停止、重复启动、`d` 后启动应返回 `NOT_ENABLED`、超时应进 ERROR。
3. 机械：低/中/高速、负载、长距离、急停后状态。

---

## 9. 已知问题

1. **无法编译**：缺 `gnu-tools-for-stm32` 工具链（阻断验收，见 §8.1）。
2. **未做 Bootloader 迁移**：本工程仍从 `0x08000000` 运行，未调用 `APP_SetVectorTable()`；烧录流程与其它工程不一致。
3. **无真实安全输入**：急停/限位/驱动故障仅预留接口。
4. **位置为开环**：无编码器/回零，失步后 `current_steps` 不可信。
5. **`MoveSteps` 极小值边界**：`steps == INT32_MIN` 时取负溢出（未防护，实际不会出现）。
6. **调试口占用**：USART1 由 App 直接读原始字节，未走 `UART_RegisterFrame`；后续接入协议时应改为帧回调。
7. **未接 `motor_pid`**：位置闭环留待有编码器/直线反馈后接入。
8. **NVIC 优先级仍全 0**：运动与通信中断同级（阶段 A R4 未处理）。

---

## 10. 下一阶段建议

1. **先解决工具链**（安装 `gnu-tools-for-stm32`），完成 §8.1 编译与 §8.4 单轴验收，这是进入下一步的前提。
2. 单轴验证通过后：将 `AxisController` **参数化**为多实例，落地 **MotionManager**（X/Y/Z 坐标、多轴占用权、急停）。
3. 按硬件确认结果补齐 **LimitSwitch / EmergencyStop / DriverFault** 设备与安全策略。
4. 接入编码器/回零，替换开环位置模型（同时处理阶段 A R6：PID 角度卷绕）。
5. 之后依次：视觉坐标转换 → 取放机构 → PuzzleTaskManager 与 Host 协议。

---

### 附：本阶段未验证声明
本阶段所有代码**均未经编译、未经静态分析工具检查、未经硬件运行**。所有"实现完成"仅指代码已按接口与状态机落地，**不代表功能正确**。请在工具链就绪后先执行 §8.1 与 §8.4。
