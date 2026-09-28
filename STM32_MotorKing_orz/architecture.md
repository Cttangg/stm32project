# 二维追踪云台软件架构规范

> 文档类型：项目顶层架构规范（Architecture Specification）  
> 适用对象：STM32 主控制器 + 双轴步进/闭环电机 + 编码器 + 上位机视觉系统  
> 核心目标：实时、非阻塞、可维护、可测试、可扩展  
> 架构角色：**总设计师（Architecture Agent）负责架构与约束；执行 Agent 负责按规范实现，不得擅自改变顶层职责边界。**

---

## 1. 项目目标

本项目实现一个二维视觉追踪云台。系统通过上位机/视觉设备获取目标在图像中的位置，由 STM32 完成目标数据接收、目标状态管理、像素到云台运动目标的转换、双轴运动规划，以及双轴步进电机的闭环控制，使云台持续将目标保持在指定视觉区域。

系统不要求 STM32 运行视觉算法。视觉算法属于系统外部输入源，STM32 只处理经过协议封装后的目标观测数据。

系统第一优先级不是“追踪算法复杂度”，而是：

1. 控制链路实时可靠。
2. 任何业务逻辑不得阻塞电机控制。
3. 视觉异常、通信异常、电机异常必须可控地降级。
4. 两个轴的软件必须采用统一抽象，禁止为 Pan/Tilt 复制两套不同控制逻辑。
5. 每个模块拥有明确职责和边界，避免跨层调用。
6. 后续能够从简单像素比例模型平滑升级到相机标定、预测控制、搜索模式等功能，而无需重构整个工程。

---

## 2. 系统范围与基本假设

### 2.1 硬件角色

系统至少包含：

- STM32 主控制器。
- Pan 轴步进电机。
- Tilt 轴步进电机。
- 每轴位置反馈编码器。
- 每轴步进/闭环驱动器。
- STM32 与视觉主机之间的串口或其他非阻塞通信接口。
- 可选：机械限位、急停、按键、调试串口、显示设备。

### 2.2 外部视觉系统

视觉系统负责：

- 相机采集。
- 目标检测/识别。
- 目标跟踪算法。
- 输出目标中心坐标及有效性。

STM32 默认接收如下信息：

```c
typedef struct
{
    float x_px;             // 目标图像坐标 X，单位 px
    float y_px;             // 目标图像坐标 Y，单位 px
    bool valid;             // 当前观测是否有效
    uint32_t timestamp_ms;  // 观测时间戳
} VisionTarget;
```

后续允许扩展：

- 目标置信度。
- 目标尺寸。
- 目标速度估计。
- 帧序号。
- 图像宽高。
- 目标类型。

但扩展不得破坏已有接口的职责。

---

# 3. 顶层架构

```text
                          ┌──────────────────────┐
                          │      Vision Host     │
                          │ Detection / Tracking │
                          └──────────┬───────────┘
                                     │
                              VisionTarget
                                     │
                                     ▼
┌──────────────────────────────────────────────────────────┐
│                    STM32 Application                     │
│                                                          │
│  ┌──────────────┐       ┌─────────────────────────────┐  │
│  │ Application  │──────▶│      Tracking Manager       │  │
│  │ State Machine│       │ target / transform / filter │  │
│  └──────────────┘       └──────────────┬──────────────┘  │
│                                        │                 │
│                                 desired pose             │
│                                        ▼                 │
│                           ┌────────────────────────┐     │
│                           │     Motion Manager     │     │
│                           │ limit / trajectory /   │     │
│                           │ coordination          │     │
│                           └───────────┬────────────┘     │
│                                       │                  │
│                            ┌──────────┴──────────┐       │
│                            ▼                     ▼       │
│                     ┌────────────┐       ┌────────────┐ │
│                     │  Axis Pan  │       │ Axis Tilt  │ │
│                     └─────┬──────┘       └─────┬──────┘ │
│                           │                    │         │
│                           └────────┬───────────┘         │
│                                    ▼                     │
│                        ┌──────────────────────┐         │
│                        │    Motor Control     │         │
│                        │ Position / velocity  │         │
│                        └──────────┬───────────┘         │
│                                   │                     │
│                 ┌─────────────────┴─────────────────┐   │
│                 ▼                                   ▼   │
│          Stepper Driver                         Encoder │
│                 │                                   ▲   │
│                 ▼                                   │   │
│               Motor ────────────────────────────────┘   │
│                                                          │
│  Communication / Safety / BSP provide cross-cutting     │
│  services but do not own application decisions.         │
└──────────────────────────────────────────────────────────┘
```

---

# 4. 分层原则

系统划分为以下层次：

```text
L7  Application
L6  Tracking
L5  Motion
L4  Axis
L3  Motor Control / Device Abstraction
L2  Driver / Communication / Safety Services
L1  BSP / HAL
```

## 4.1 上层只能依赖下层

允许：

```text
Application → Tracking → Motion → Axis → Motor
```

禁止：

```text
Motor → Tracking
BSP → Application
Encoder driver → State Machine
UART ISR → Application logic
```

跨层需求必须通过接口或事件传递。

## 4.2 ISR 不承载业务逻辑

中断仅允许：

- 捕获硬件事件。
- 推送/弹出缓冲区数据。
- 更新时间戳/计数器。
- 设置事件标志。
- 执行明确、固定时间上界的极短逻辑。

禁止在 ISR 中执行：

- PID 大量计算。
- UART 阻塞发送。
- 等待外设完成。
- 状态机长流程。
- printf。
- 动态内存分配。
- 任何不确定耗时操作。

---

# 5. 模块职责规范

## 5.1 Application

### 职责

负责系统级状态机和模式切换。

主要状态：

```text
INIT
IDLE
SEARCH
ACQUIRE
TRACK
LOST
FAULT
```

### 不负责

- 读取编码器寄存器。
- 生成 Step 脉冲。
- 解析 UART 字节流。
- 实现 PID。
- 直接处理像素坐标。

### 核心接口

```c
void App_Init(void);
void App_Update(uint32_t now_ms);
AppState App_GetState(void);
void App_RequestStart(void);
void App_RequestStop(void);
void App_RequestSearch(void);
```

---

## 5.2 Communication Manager

### 职责

提供非阻塞通信能力：

```text
RX byte stream
      ↓
Ring Buffer
      ↓
Frame Parser
      ↓
Validated Message
```

负责：

- 接收视觉帧。
- 校验帧头、长度、CRC/校验和。
- 处理错误帧。
- 发送状态遥测。
- 维护通信超时信息。

### 核心接口

```c
void Comm_Init(void);
void Comm_Update(uint32_t now_ms);
bool Comm_PopVisionTarget(VisionTarget *target);
bool Comm_SendTelemetry(const TelemetryFrame *frame);
uint32_t Comm_GetLastRxTime(void);
bool Comm_IsLinkHealthy(uint32_t now_ms);
```

### 强制要求

必须采用以下任一种组合：

```text
DMA + Ring Buffer + Parser
```

或：

```text
ISR + Ring Buffer + Parser
```

不得使用会阻塞主循环/控制循环的接收或发送接口。

---

# 6. Tracking Manager

Tracking Manager 是视觉闭环与运动控制之间的核心适配层。

## 6.1 输入

```c
VisionTarget
CurrentGimbalPose
TrackingConfig
```

## 6.2 输出

```c
GimbalTargetPose
```

建议：

```c
typedef struct
{
    float pan_deg;
    float tilt_deg;
    bool valid;
} GimbalTargetPose;
```

## 6.3 内部处理流程

```text
VisionTarget
    ↓
Validate
    ↓
Timestamp Check
    ↓
Filter
    ↓
Coordinate Transform
    ↓
Deadband
    ↓
Target Pose
```

### 必须包含

- 数据有效性判断。
- 视觉超时判断。
- 可选的一阶低通/EMA 滤波。
- X/Y 独立死区。
- 输出角度限幅。

### 后续可扩展

- 目标速度估计。
- 运动预测。
- 相机内参模型。
- 畸变补偿。
- 非线性坐标映射。
- PID 外环。
- Kalman Filter。

这些属于 Tracking 层，不得下沉到 Motor 层。

### 核心接口

```c
void Tracking_Init(const TrackingConfig *cfg);
void Tracking_Update(uint32_t now_ms);
void Tracking_SetVisionTarget(const VisionTarget *target);
bool Tracking_GetTargetPose(GimbalTargetPose *pose);
TrackingState Tracking_GetState(void);
```

---

# 7. 坐标转换模块

坐标转换必须独立为模块。

## 7.1 第一阶段：线性模型

```text
ex = x_target - x_center
ey = y_target - y_center

Δpan  = Kx * ex
Δtilt = Ky * ey
```

同时考虑机械方向符号：

```c
pan_cmd  = pan_zero  + pan_sign  * Kx * ex;
tilt_cmd = tilt_zero + tilt_sign * Ky * ey;
```

## 7.2 后续：相机模型

可升级为：

```text
pixel
 ↓
normalized coordinate
 ↓
camera ray
 ↓
pan / tilt desired angle
```

### 核心接口

```c
bool CoordTransform_PixelToPose(
    float x_px,
    float y_px,
    GimbalPose current_pose,
    GimbalTargetPose *target);
```

该模块不能访问电机驱动器，也不能发送运动命令。

---

# 8. Motion Manager

Motion Manager 是“目标姿态”到“可执行轨迹”的中间层。

## 8.1 职责

- 接收 Tracking 给出的目标姿态。
- 执行 Pan/Tilt 角度限幅。
- 执行最大速度限制。
- 执行最大加速度限制。
- 处理目标跳变。
- 双轴协调。
- 生成各 Axis 的实时目标。

## 8.2 数据结构

```c
typedef struct
{
    float position_deg;
    float velocity_deg_s;
    float acceleration_deg_s2;
} AxisSetpoint;
```

### 核心接口

```c
void Motion_Init(const MotionConfig *cfg);
void Motion_Update(float dt_s);
void Motion_SetTargetPose(const GimbalTargetPose *target);
void Motion_GetAxisSetpoint(AxisId axis, AxisSetpoint *setpoint);
```

### 设计要求

Motion Manager 不直接操作 GPIO、Timer、Step/Dir。

---

# 9. Axis 抽象

Pan 和 Tilt 必须使用统一 Axis 类型。

```c
typedef struct
{
    AxisId id;

    float position_deg;
    float target_deg;
    float velocity_deg_s;
    float target_velocity_deg_s;

    float min_deg;
    float max_deg;
    float max_velocity_deg_s;
    float max_acceleration_deg_s2;

    bool enabled;
    bool homed;
    bool fault;
} Axis;
```

### 核心接口

```c
void Axis_Init(Axis *axis, const AxisConfig *cfg);
void Axis_Enable(Axis *axis);
void Axis_Disable(Axis *axis);
void Axis_SetTarget(Axis *axis, float target_deg);
void Axis_Update(Axis *axis, float dt_s);
float Axis_GetPosition(const Axis *axis);
bool Axis_IsAtTarget(const Axis *axis);
```

### Axis 负责

- 单轴状态。
- 单轴限位。
- 单轴目标管理。
- 单轴控制链路。

### Axis 不负责

- 视觉。
- 全局状态机。
- 串口协议。
- 双轴协调策略。

---

# 10. Motor Controller

Motor Controller 负责将 Axis 的目标转化为电机执行量。

推荐控制链：

```text
Position Target
      ↓
Position Error
      ↓
Position Controller
      ↓
Velocity / Step Rate Command
      ↓
Stepper Driver
```

如果实际电机闭环方案只需要位置环，则不要为了“看起来完整”强制加入串级 PID。

是否使用速度环/串级控制取决于：

- 驱动器能力。
- 电机动态特性。
- 编码器更新率。
- Step 指令接口。
- 实际系统辨识结果。

### 核心接口

```c
void MotorCtrl_Init(MotorController *ctrl, const MotorCtrlConfig *cfg);
void MotorCtrl_Update(MotorController *ctrl, float dt_s);
void MotorCtrl_SetTarget(MotorController *ctrl, float target_deg);
void MotorCtrl_SetFeedback(MotorController *ctrl, float position_deg);
MotorOutput MotorCtrl_GetOutput(const MotorController *ctrl);
```

### 强制要求

控制器必须具备：

- 输出限幅。
- 积分限幅（若使用积分）。
- 目标变化限制或由 Motion 层负责限制。
- 编码器异常检测。
- 控制周期固定或可测量。

---

# 11. Encoder 模块

编码器模块只负责获得可靠位置反馈。

```c
typedef struct
{
    float angle_deg;
    float velocity_deg_s;
    bool valid;
    uint32_t timestamp_ms;
} EncoderFeedback;
```

### 核心接口

```c
void Encoder_Init(Encoder *enc);
void Encoder_Update(Encoder *enc);
EncoderFeedback Encoder_GetFeedback(const Encoder *enc);
```

### 约束

Encoder 模块不得：

- 决定目标角度。
- 修改全局状态机。
- 控制 Step 输出。
- 直接执行 PID。

---

# 12. Stepper Driver 模块

该模块对实际驱动器进行硬件抽象。

例如：

```c
void Stepper_Enable(StepperDriver *drv);
void Stepper_Disable(StepperDriver *drv);
void Stepper_SetDirection(StepperDriver *drv, bool dir);
void Stepper_SetStepRate(StepperDriver *drv, float step_hz);
void Stepper_Stop(StepperDriver *drv);
```

上层不得直接调用：

```c
HAL_GPIO_WritePin(...)
HAL_TIM_PWM_Start(...)
```

这些内容应该隔离在 BSP/Driver 层。

---

# 13. Safety Manager

Safety 是横向模块，不依赖追踪算法是否正常。

## 13.1 必须监控

```text
机械限位
电机故障
编码器失效
视觉超时
通信超时
角度越界
控制输出异常
```

## 13.2 故障优先级

```text
Emergency Stop
      ↓
Hard Fault
      ↓
Motor Fault
      ↓
Encoder Fault
      ↓
Communication Fault
      ↓
Vision Lost
```

不同故障对应不同降级策略。

例如：

```text
Vision Lost
    ↓
停止追踪目标更新
    ↓
进入 LOST
    ↓
保持/回安全姿态/进入 SEARCH
```

而：

```text
Motor Fault
    ↓
立即禁止该轴输出
    ↓
系统进入 FAULT
```

### 核心接口

```c
void Safety_Init(const SafetyConfig *cfg);
void Safety_Update(uint32_t now_ms);
bool Safety_IsFault(void);
SafetyFault Safety_GetFault(void);
void Safety_EmergencyStop(void);
```

---

# 14. 系统状态机

## 14.1 状态定义

### INIT

执行：

- 外设初始化。
- 参数初始化。
- 编码器读取。
- 电机初始状态确认。
- 通信模块启动。
- 安全检查。

禁止进入正常追踪前存在未处理故障。

### IDLE

- 电机保持安全状态。
- 不主动追踪。
- 可以接受启动、校准等命令。

### SEARCH

用于目标不存在时主动搜索。

第一阶段可以暂时实现为：

```text
保持当前安全 Tilt
Pan 按扫描轨迹运动
```

### ACQUIRE

检测到目标后：

- 验证连续多帧有效性。
- 逐步接近目标。
- 限制最大运动速度。

目标稳定后进入 TRACK。

### TRACK

正常二维追踪模式。

```text
Vision → Tracking → Motion → Axis → Motor
```

### LOST

目标暂时丢失。

由配置决定：

```text
保持当前位置
```

或：

```text
返回预设姿态
```

或：

```text
进入 SEARCH
```

### FAULT

所有普通运动请求失效。

只有明确的故障恢复流程才能退出。

---

# 15. 实时调度模型

系统采用周期任务模型，禁止任何任务无限等待。

推荐初始调度：

| 周期 | 任务 | 优先级 |
|---|---|---|
| 1 ms / 1 kHz | Encoder / Motor Control / Safety Critical | 最高 |
| 5~10 ms | Motion Update | 高 |
| 10~20 ms | Tracking Update | 中高 |
| 事件驱动 | Communication Parser | 中 |
| 20~100 ms | Telemetry / UI / Debug | 低 |

实际周期根据 MCU 性能、编码器采样率、驱动器和视觉帧率调整。

## 15.1 控制主循环示例

```c
while (1)
{
    uint32_t now = SysTick_GetMs();

    Comm_Update(now);
    App_Update(now);
    Tracking_Update(now);
    Motion_Update(dt_motion);

    Axis_Update(&axis_pan, dt_axis);
    Axis_Update(&axis_tilt, dt_axis);

    Safety_Update(now);
    Telemetry_Update(now);
}
```

电机控制若由定时器更新，则主循环只更新目标值，真正的固定周期控制在定时器/控制任务中完成。

---

# 16. 非阻塞规范

项目中禁止以下模式：

```c
HAL_Delay(...);
while (!ready) {}
HAL_UART_Receive(..., HAL_MAX_DELAY);
HAL_UART_Transmit(..., ...);
等待电机到位;
等待视觉回复;
等待编码器响应;
```

应采用：

```text
状态机
定时器
事件标志
Ring Buffer
DMA
时间戳
超时机制
```

任何函数都应满足：

> 调用者能够在有限、可预测的时间内返回。

---

# 17. 数据流规范

## 17.1 视觉数据

```text
Camera
  ↓
Vision Algorithm
  ↓
Protocol Frame
  ↓
Communication Manager
  ↓
VisionTarget
  ↓
Tracking Manager
```

## 17.2 运动数据

```text
VisionTarget
  ↓
Tracking
  ↓
GimbalTargetPose
  ↓
Motion
  ↓
AxisSetpoint
  ↓
Motor Controller
  ↓
Stepper Driver
```

## 17.3 反馈数据

```text
Encoder
  ↓
EncoderFeedback
  ↓
Axis
  ↓
Motor Controller
  ↓
Telemetry
```

---

# 18. 双轴同步规范

Pan/Tilt 不允许相互直接修改对方状态。

错误方式：

```c
pan.c 修改 tilt.target;
tilt.c 修改 pan.target;
```

正确方式：

```text
          Motion Manager
           /          \
          /            \
       Pan Target    Tilt Target
          ↓             ↓
       Axis Pan      Axis Tilt
```

双轴协调只存在于 Motion Manager。

例如需要实现“两个轴同时到位”，由 Motion Manager 根据：

- 角度差。
- 最大速度。
- 最大加速度。

计算两轴轨迹，而不是让 Axis 自己相互干涉。

---

# 19. 参数管理

所有可调参数统一集中管理。

建议：

```c
typedef struct
{
    float pan_zero_deg;
    float tilt_zero_deg;

    float pixel_to_pan;
    float pixel_to_tilt;

    float deadband_x_px;
    float deadband_y_px;

    float max_pan_velocity;
    float max_tilt_velocity;

    float max_pan_acceleration;
    float max_tilt_acceleration;

    uint32_t vision_timeout_ms;
    uint32_t acquire_frames;
} GimbalConfig;
```

禁止把魔法数字散落在业务代码中：

```c
if (error > 12.5f) ...
```

应改为：

```c
if (error > cfg->deadband_x_px) ...
```

---

# 20. 调试与遥测

必须允许在不修改控制逻辑的情况下观察：

```text
System State
Vision Target
Vision Age
Pan Actual
Pan Target
Tilt Actual
Tilt Target
Pan Error
Tilt Error
Motor Output
Encoder Status
Communication Status
Safety Fault
```

建议定义统一遥测结构：

```c
typedef struct
{
    AppState state;

    VisionTarget vision;
    uint32_t vision_age_ms;

    float pan_actual_deg;
    float pan_target_deg;
    float pan_output;

    float tilt_actual_deg;
    float tilt_target_deg;
    float tilt_output;

    uint32_t fault_flags;
} TelemetryFrame;
```

---

# 21. 初始化顺序

建议固定为：

```text
HAL/BSP
  ↓
Communication
  ↓
Encoder
  ↓
Stepper Driver
  ↓
Motor Controller
  ↓
Axis
  ↓
Motion
  ↓
Tracking
  ↓
Safety
  ↓
Application
```

初始化成功后才允许进入 IDLE。

禁止在模块初始化函数中隐藏长时间等待。

---

# 22. 运行时调用关系

```text
App_Update()
      │
      ├── 根据状态决定 Tracking 是否工作
      │
      ▼
Comm_Update()
      │
      ▼
VisionTarget
      │
      ▼
Tracking_Update()
      │
      ▼
GimbalTargetPose
      │
      ▼
Motion_Update()
      │
      ├──────────────┐
      ▼              ▼
Axis Pan         Axis Tilt
      │              │
      ▼              ▼
MotorCtrl        MotorCtrl
      │              │
      ▼              ▼
Driver           Driver
```

安全检查可以独立运行，并拥有高于普通追踪逻辑的优先级。

---

# 23. API 设计规范

## 23.1 禁止全局变量横向共享

错误：

```c
extern float pan_target;
extern float tilt_target;
```

正确：

通过模块 API：

```c
Motion_SetTargetPose(&target);
```

## 23.2 输出优先使用结构体

优先：

```c
MotorOutput MotorCtrl_GetOutput(...);
```

而不是大量独立全局变量。

## 23.3 状态访问只读

外部只能通过：

```c
Axis_GetState();
```

获取内部状态，不得直接修改内部对象字段。

---

# 24. 故障与降级策略

| 故障 | 典型处理 |
|---|---|
| Vision Timeout | TRACK → LOST |
| Vision Invalid | 忽略当前目标 |
| 通信 CRC 错误 | 丢弃当前帧 |
| Encoder Invalid | 禁止对应轴闭环 |
| 电机驱动 Fault | 对应轴停机并进入 FAULT |
| 机械限位触发 | 禁止继续向限位方向运动 |
| 姿态越界 | 目标钳位 |
| 控制输出超限 | 输出限幅并记录故障/告警 |
| 急停 | 立即停止电机输出 |

具体行为可按硬件重新定义，但必须形成显式策略，不能依赖“代码刚好如此运行”。

---

# 25. 测试策略

系统必须支持分层测试。

## 25.1 单轴测试

在完全关闭视觉的情况下：

```text
目标角度
    ↓
Axis
    ↓
Motor
```

验证：

- 方向。
- 零点。
- 位置闭环。
- 静态误差。
- 到位判定。
- 限位。
- 故障。

## 25.2 双轴测试

验证：

```text
Pan = target
Tilt = target
```

并确认两轴不会相互干扰。

## 25.3 虚拟视觉输入

无需连接摄像头，直接构造：

```c
VisionTarget target = {
    .x_px = 700,
    .y_px = 300,
    .valid = true,
    .timestamp_ms = now
};
```

测试完整 Tracking → Motion → Axis 链路。

## 25.4 异常测试

必须验证：

- 视觉突然丢失。
- 目标瞬移。
- 数据乱序。
- CRC 错误。
- 时间戳过期。
- 编码器失效。
- 电机堵转/驱动 Fault。
- 触发限位。

---

# 26. 第一阶段实现范围

项目初版禁止一次性加入所有高级功能。

### Phase 1：单轴闭环

```text
Encoder
  ↓
Axis
  ↓
Motor Controller
  ↓
Stepper Driver
```

目标：单轴能可靠控制到指定角度。

### Phase 2：双轴闭环

```text
Axis[2]
```

目标：Pan/Tilt 都可靠运行。

### Phase 3：视觉通信

实现：

```text
Host → STM32
VisionTarget
```

目标：可靠收发，不阻塞控制。

### Phase 4：二维追踪

实现：

```text
Pixel → Pose → Motion → Motor
```

目标：静态目标锁定。

### Phase 5：鲁棒性

加入：

- 滤波。
- Deadband。
- Timeout。
- ACQUIRE。
- LOST。

### Phase 6：搜索模式

实现：

```text
LOST → SEARCH → ACQUIRE
```

### Phase 7：高级追踪

再考虑：

- 目标预测。
- 相机标定。
- 速度前馈。
- 更高阶滤波。
- 双轴轨迹协调。

---

# 27. 当前版本的“最小可运行闭环”

第一版软件真正需要闭合的链路只有：

```text
                    ┌──────────────┐
                    │ Vision Target│
                    └──────┬───────┘
                           │
                           ▼
                  ┌────────────────┐
                  │   Tracking     │
                  └───────┬────────┘
                          │
                          ▼
                  ┌────────────────┐
                  │    Motion      │
                  └───────┬────────┘
                          │
                 ┌────────┴────────┐
                 ▼                 ▼
              Axis Pan          Axis Tilt
                 │                 │
                 ▼                 ▼
              Motor Pan         Motor Tilt
                 ▲                 ▲
                 │                 │
              Encoder           Encoder
```

但实际开发顺序应为：

```text
Axis → Motor → 双轴 → 通信 → Tracking → 完整状态机
```

而不是直接从“完整视觉追踪”开始。

---

# 28. 架构变更规则

任何执行 Agent 在修改工程前必须：

1. 阅读本文件。
2. 阅读当前工程目录及已有模块。
3. 找出与本架构冲突的旧逻辑。
4. 先提交架构分析，再修改代码。
5. 不得未经确认删除现有可用功能。
6. 不得为了通过编译而破坏模块边界。
7. 不得引入阻塞式控制流程。
8. 不得重复实现已有底层能力。
9. 修改公共 API 时必须同步检查所有调用者。
10. 每次改动后必须报告：修改模块、原因、接口变化、测试结果、遗留问题。

---

# 29. Agent 协作规范

本项目采用“双 Agent”模式：

```text
                总设计师 Agent
                       │
             架构 / 约束 / 验收标准
                       │
                       ▼
                执行 Agent
                       │
              代码分析 / 修改 / 测试
                       │
                       ▼
                工程实施结果
                       │
                       └────→ 总设计师复核
```

## 29.1 总设计师 Agent

负责：

- 解释系统目标。
- 维护架构边界。
- 决定模块职责。
- 决定接口。
- 审查实时性。
- 审查状态机。
- 审查安全策略。
- 对执行结果进行验收。

不负责直接替代执行 Agent 完成全部代码实现。

## 29.2 执行 Agent

负责：

- 阅读当前代码。
- 按架构实现模块。
- 修复具体工程问题。
- 运行编译和测试。
- 输出修改报告。

执行 Agent 不得自行改变顶层架构，例如：

```text
为了实现方便
→ 把 Tracking 写进 Motor.c
→ 把 UART 解析写进 main.c
→ 把两个 Axis 合并成特殊分支
```

这类改动属于架构变更，必须回到总设计师层重新定义。

---

# 30. 验收标准

本项目在进入“完整二维追踪”前，应满足以下条件：

### 架构

- [ ] Pan/Tilt 使用统一 Axis 抽象。
- [ ] Tracking 与 Motor 解耦。
- [ ] Communication 与业务逻辑解耦。
- [ ] Safety 独立存在。
- [ ] 参数集中管理。

### 实时性

- [ ] 无 HAL_Delay 等阻塞等待。
- [ ] 无阻塞 UART 收发。
- [ ] ISR 不执行复杂业务。
- [ ] Motor Control 周期稳定。
- [ ] 任何超时均有明确处理。

### 电机

- [ ] 单轴闭环正常。
- [ ] 双轴方向正确。
- [ ] 限位正常。
- [ ] 编码器反馈可靠。
- [ ] 目标跳变不会直接导致无限大输出。

### 视觉

- [ ] 有效目标能正确解析。
- [ ] 过期目标不会持续控制。
- [ ] 目标丢失有明确状态。
- [ ] 有死区避免目标中心附近频繁抖动。

### 工程

- [ ] 编译通过。
- [ ] 无明显警告。
- [ ] 关键模块有最小单元测试或仿真输入。
- [ ] 修改均有变更说明。

---

# 31. 未来扩展

当前架构允许自然扩展：

```text
二维云台
   │
   ├── 更高精度相机标定
   ├── 目标速度预测
   ├── Kalman Filter
   ├── 目标优先级
   ├── 自动搜索
   ├── 多目标选择
   ├── 云台姿态零点标定
   ├── 参数在线调整
   └── 三轴扩展
```

特别是三轴扩展时，原则上只需要将：

```c
Axis axis_pan;
Axis axis_tilt;
```

扩展为：

```c
Axis axes[AXIS_COUNT];
```

上层 Tracking / Motion / Safety 仍可保持相同思想。

---

# 32. 最终架构原则

整个项目最终遵循以下一句话：

> **视觉负责“目标在哪里”，Tracking 负责“目标对应什么姿态”，Motion 负责“如何安全地到达这个姿态”，Axis/Motor 负责“真实执行”，Encoder 负责“告诉系统实际到了哪里”，Safety 负责“任何异常情况下让系统停在可控状态”。**

以及四条不可破坏的工程原则：

```text
1. 不阻塞实时控制。
2. 不跨层乱调用。
3. 不复制双轴业务逻辑。
4. 不让执行 Agent 自行改变顶层架构。
```

---

## Appendix A：推荐文件结构

```text
Core/
├── app/
│   ├── app.c
│   ├── app.h
│   └── app_state.c
│
├── tracking/
│   ├── tracking_manager.c
│   ├── tracking_manager.h
│   ├── target_filter.c
│   ├── target_filter.h
│   ├── coordinate_transform.c
│   └── coordinate_transform.h
│
├── motion/
│   ├── motion_manager.c
│   ├── motion_manager.h
│   ├── trajectory.c
│   └── trajectory.h
│
├── axis/
│   ├── axis.c
│   └── axis.h
│
├── motor/
│   ├── motor.c
│   ├── motor.h
│   ├── motor_controller.c
│   └── motor_controller.h
│
├── encoder/
│   ├── encoder.c
│   └── encoder.h
│
├── driver/
│   ├── stepper_driver.c
│   └── stepper_driver.h
│
├── communication/
│   ├── protocol.c
│   ├── protocol.h
│   ├── uart_port.c
│   └── uart_port.h
│
├── safety/
│   ├── safety_manager.c
│   └── safety_manager.h
│
└── bsp/
    ├── bsp_timer.c
    ├── bsp_uart.c
    ├── bsp_gpio.c
    └── bsp_spi.c
```

---

## Appendix B：第一阶段实现任务清单

```text
[ ] 建立 Axis 抽象
[ ] 实现 Encoder 接口
[ ] 实现 Stepper Driver 接口
[ ] 实现单轴位置闭环
[ ] 验证单轴方向与零点
[ ] 扩展为 Axis[2]
[ ] 实现 Pan/Tilt 双轴控制
[ ] 实现非阻塞通信
[ ] 定义 VisionFrame
[ ] 定义 VisionTarget
[ ] 实现 Tracking Manager
[ ] 实现 Pixel → Pose
[ ] 加入 Deadband
[ ] 加入 Vision Timeout
[ ] 加入 ACQUIRE / TRACK / LOST
[ ] 加入 Motion 限速/加速度限制
[ ] 加入 Safety Manager
[ ] 加入 Telemetry
[ ] 做异常工况测试
```

---

**文档结束**
