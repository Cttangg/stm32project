# 二维追踪云台软件架构重构落地计划

## 1. 重构目标

本次重构目标不是改变现有电机控制算法，而是在保持现有硬件驱动和闭环控制行为基本稳定的前提下，将现有工程逐步迁移到 `architecture.md` 定义的分层架构。

最终软件结构：

```text
Application
    │
    ▼
Tracking Manager
    │
    ▼
Coordinate Transform
    │
    ▼
Motion Manager
    │
    ▼
Axis[2]
    │
    ▼
Motor Controller
    │
    ├── Encoder
    └── Stepper Driver
```

辅助链路：

```text
Vision Host
    │
    ▼
Communication Manager
    │
    ▼
VisionTarget
    │
    ▼
Tracking Manager
```

安全链路：

```text
Key / Limit / Encoder / Communication
              │
              ▼
        Safety Manager
              │
              ▼
         Axis / Gimbal
```

必须继续满足：

* 全系统非阻塞
* 电机 STEP 由定时器 ISR 产生
* UART 使用 DMA / RingBuffer
* 不在 ISR 中执行复杂控制算法
* Pan/Tilt 使用统一的数据结构和 API
* 上层不得直接操作 TMC2209
* 驱动层不得依赖业务层
* 重构过程中不能破坏现有单轴闭环能力

---

# 2. 重构原则

## 2.1 不进行“大爆炸式重构”

严禁一次性删除现有：

```text
main.c
motor_pid.c
motion.c
tracking.c
```

然后重新实现全部功能。

必须采用：

```text
旧系统
   │
   ├── 保留现有可工作的驱动
   │
   ├── 新建抽象层
   │
   ├── 将旧逻辑逐步迁移
   │
   └── 每一步都可以编译、运行、验证
```

每完成一个阶段必须保证工程至少能够：

```text
编译通过
↓
单轴运行
↓
闭环运行
↓
双轴运行
```

---

# 3. 重构优先级

按照以下顺序实施：

```text
P0  Axis 抽象
 ↓
P1  Safety Manager
 ↓
P2  Motion Manager
 ↓
P3  Communication / VisionTarget
 ↓
P4  Coordinate Transform
 ↓
P5  Tracking Manager
 ↓
P6  Application 状态机
 ↓
P7  Telemetry / Debug
```

原因：

`Axis` 是后面所有运动模块的基础。

`Safety` 必须在二维运动真正接入视觉之前完成。

视觉和 Tracking 不应该建立在仍然存在于 `main.c` 的裸 PID 控制之上。

---

# 4. Phase 0：建立当前工程基线

## 目标

在任何结构修改之前确认当前系统可工作。

记录：

```text
Pan：
    当前角度
    目标角度
    最大速度
    最大加速度
    到位误差
    PID 参数

Tilt：
    当前角度
    目标角度
    最大速度
    最大加速度
    到位误差
    PID 参数
```

同时记录：

```text
Encoder 通信状态
TMC2209 状态
UART 状态
STEP 定时器频率
主循环周期
MotorPID.Update() 周期
```

建立基准测试：

```text
Test A
Pan 单轴移动到指定角度

Test B
Tilt 单轴移动到指定角度

Test C
Pan/Tilt 同时移动

Test D
停止电机

Test E
编码器异常

Test F
UART 连续接收
```

在基线未确认之前，不修改控制算法。

---

# 5. Phase 1：实现 Axis 抽象

新增：

```text
axis/
├── axis.c
└── axis.h
```

建议数据结构：

```c
typedef enum
{
    AXIS_DISABLED = 0,
    AXIS_IDLE,
    AXIS_MOVING,
    AXIS_HOLD,
    AXIS_FAULT
} AxisState;

typedef enum
{
    AXIS_OK = 0,
    AXIS_ERR_ENCODER,
    AXIS_ERR_DRIVER,
    AXIS_ERR_LIMIT,
    AXIS_ERR_TIMEOUT,
    AXIS_ERR_INVALID_TARGET
} AxisFault;

typedef struct
{
    float position;
    float target_position;

    float velocity;
    float target_velocity;

    float max_velocity;
    float max_acceleration;

    float position_tolerance;

    bool enabled;
    bool in_position;

    AxisState state;
    AxisFault fault;

    MotorPID *pid;
    TMC2209_HandleTypeDef *driver;
    MT6701_HandleTypeDef *encoder;
} Axis;
```

核心 API：

```c
void Axis_Init(Axis *axis,
               MotorPID *pid,
               TMC2209_HandleTypeDef *driver,
               MT6701_HandleTypeDef *encoder);

void Axis_Enable(Axis *axis);
void Axis_Disable(Axis *axis);

void Axis_SetTarget(Axis *axis, float target);
void Axis_SetLimits(Axis *axis,
                    float min_position,
                    float max_position);

void Axis_Update(Axis *axis, float dt);

bool Axis_IsInPosition(const Axis *axis);
bool Axis_IsFault(const Axis *axis);

void Axis_ClearFault(Axis *axis);
AxisState Axis_GetState(const Axis *axis);
```

---

# 6. Axis 的内部职责

Axis 负责：

```text
读取反馈
↓
更新当前位置
↓
检查目标是否合法
↓
位置限幅
↓
调用 MotorPID
↓
判断是否到位
↓
维护 Axis 状态
```

Axis 不负责：

```text
视觉
UART
目标检测
二维坐标转换
搜索策略
系统模式
```

---

# 7. 最重要的迁移策略

现有：

```c
MotorPID_Update(&pan_pid);
MotorPID_Update(&tilt_pid);
```

不要马上删除。

第一阶段应该变成：

```c
Axis_Update(&axis_pan, dt);
Axis_Update(&axis_tilt, dt);
```

然后：

```text
Axis_Update()
      │
      ▼
MotorPID_Update()
```

即：

```text
旧 PID
   ↓
Axis
   ↓
旧 PID
```

先建立“包装层”，确认行为一致后，再清理旧接口。

---

# 8. Phase 2：Safety Manager

新增：

```text
safety/
├── safety_manager.c
└── safety_manager.h
```

统一管理：

```text
KEY_PAUSE
KEY_RESET
KEY_TRACK
KEY_BORDER
KEY_CALIB
KEY_A4

Pan Limit
Tilt Limit

Encoder Magnet Status

Motor Driver Fault

Vision Timeout
```

定义：

```c
typedef enum
{
    SAFETY_OK = 0,
    SAFETY_WARNING,
    SAFETY_STOP,
    SAFETY_FAULT
} SafetyLevel;
```

核心 API：

```c
void Safety_Init(void);
void Safety_Update(uint32_t now);

bool Safety_IsMotionAllowed(void);
bool Safety_IsTrackingAllowed(void);

void Safety_EmergencyStop(void);
void Safety_ClearFault(void);
```

---

# 9. Safety 的优先级

出现以下任意情况：

```text
机械限位触发
编码器异常
驱动器故障
紧急停止
系统故障
```

必须能够直接阻止运动。

安全机制优先级：

```text
Safety
  >
Application
  >
Tracking
  >
Motion
  >
Axis
```

Tracking 不允许绕过 Safety。

---

# 10. Phase 3：Motion Manager

新增：

```text
motion/
├── motion.c
└── motion.h
```

Motion Manager 负责把：

```text
目标姿态
```

转换成：

```text
Pan Axis Setpoint
Tilt Axis Setpoint
```

核心接口：

```c
typedef struct
{
    float pan;
    float tilt;
} GimbalSetpoint;

void Motion_Init(void);

void Motion_SetTarget(float pan, float tilt);
void Motion_Stop(void);

void Motion_Update(float dt);

GimbalSetpoint Motion_GetSetpoint(void);
```

内部处理：

```text
target
 ↓
position limit
 ↓
velocity limit
 ↓
acceleration limit
 ↓
Axis
```

---

# 11. 双轴协调

Motion Manager 必须支持：

```text
Pan = 20°
Tilt = 10°
```

同时运动。

第一阶段只需要做到：

```text
两轴同时启动
两轴各自限速
两轴各自限加速度
```

暂时不要加入复杂轨迹规划。

第二阶段再考虑：

```text
同步到位
速度比例
S 曲线
梯形轨迹
```

---

# 12. Phase 4：Communication Manager

现在：

```text
uart.c
RingBuffer
UART_RegisterFrame
UART_Task
```

这些属于底层通信能力。

新增一层：

```text
communication/
├── communication.c
├── communication.h
└── vision_protocol.c
```

职责：

```text
UART
 ↓
Frame
 ↓
Protocol Validation
 ↓
VisionTarget
```

---

# 13. VisionTarget 数据结构

新增：

```c
typedef struct
{
    float x;
    float y;

    bool valid;

    uint32_t timestamp;
    uint32_t sequence;
} VisionTarget;
```

建议同时增加：

```c
typedef struct
{
    VisionTarget target;

    uint32_t receive_time;
    uint32_t last_valid_time;

    bool timeout;
} VisionState;
```

---

# 14. 通信协议

协议层必须至少支持：

```text
Frame Header
Length
Message Type
Payload
Sequence
Timestamp
CRC
```

推荐：

```text
+--------+--------+--------+---------+---------+-----+
| HEAD   | TYPE   | LEN    | PAYLOAD | SEQ/TS  | CRC |
+--------+--------+--------+---------+---------+-----+
```

不得让 Tracking Manager 直接解析 UART 原始字节。

Tracking Manager 只能看到：

```c
VisionTarget
```

---

# 15. Vision Timeout

通信层 / Tracking 层必须判断视觉数据是否超时。

逻辑：

```text
收到目标
    │
    ▼
更新时间戳
    │
    ▼
正常 Tracking
    │
    ├── 数据持续更新
    │
    └── 超过 timeout
             │
             ▼
          TARGET_LOST
```

禁止：

```text
最后一帧 valid = 1
↓
永远使用旧目标
```

---

# 16. Phase 5：Coordinate Transform

新增：

```text
tracking/
└── coordinate_transform.c
```

第一版本只实现线性模型：

```text
ex = target_x - center_x
ey = target_y - center_y

pan_error  = ex * pixel_to_angle_x;
tilt_error = ey * pixel_to_angle_y;
```

输出：

```c
typedef struct
{
    float pan_delta;
    float tilt_delta;
} TrackingOffset;
```

不要直接修改电机。

---

# 17. 后续坐标模型升级

第一版本：

```text
Pixel
 ↓
Linear Transform
```

未来可以替换成：

```text
Pixel
 ↓
Camera Intrinsic
 ↓
Normalized Ray
 ↓
Gimbal Geometry
 ↓
Pan/Tilt
```

Coordinate Transform 必须保证：

```text
Tracking Manager
```

不需要知道具体计算方法。

---

# 18. Phase 6：Tracking Manager

新增：

```text
tracking/
├── tracking_manager.c
├── tracking_manager.h
├── target_filter.c
├── target_filter.h
└── coordinate_transform.c
```

Tracking Manager 输入：

```text
VisionTarget
当前 Pan
当前 Tilt
系统参数
```

输出：

```text
GimbalSetpoint
```

---

# 19. Tracking Manager 状态

定义：

```c
typedef enum
{
    TRACKING_DISABLED = 0,
    TRACKING_SEARCH,
    TRACKING_ACQUIRE,
    TRACKING_TRACK,
    TRACKING_LOST
} TrackingState;
```

基本转移：

```text
DISABLED
   │
   ▼
SEARCH
   │ target valid
   ▼
ACQUIRE
   │ stable
   ▼
TRACK
   │ target lost
   ▼
LOST
   │ timeout
   ▼
SEARCH
```

---

# 20. Tracking Filter

第一版使用简单滤波即可：

```text
EMA / Low Pass
```

例如：

```c
filtered_x =
    alpha * current_x +
    (1.0f - alpha) * filtered_x;
```

不要第一版就加入复杂预测器。

必须先确认：

```text
视觉抖动
电机抖动
追踪延迟
```

再决定是否增加：

```text
速度估计
目标预测
Kalman Filter
```

---

# 21. Deadband

Tracking Manager 增加：

```c
float deadband_x;
float deadband_y;
```

例如：

```text
|ex| < deadband_x
```

则：

```text
Pan 不继续调整
```

避免目标已经基本位于中心时产生持续微动。

Deadband 必须属于 Tracking 层，而不是 Motor PID 层。

---

# 22. Phase 7：Application 状态机

新增：

```text
app/
├── app.c
└── app.h
```

Application 只管理：

```text
系统当前模式
用户命令
模块启动/停止
状态转换
```

推荐：

```c
typedef enum
{
    APP_INIT = 0,
    APP_IDLE,
    APP_SEARCH,
    APP_TRACK,
    APP_LOST,
    APP_FAULT
} AppState;
```

Application 不允许：

```c
TMC2209_SetDirection(...)
MotorPID_Update(...)
MT6701_ReadDegrees(...)
```

这些都必须由下层完成。

---

# 23. main.c 最终目标

最终 `main.c` 应该尽量接近：

```c
int main(void)
{
    BSP_Init();

    Communication_Init();
    Encoder_Init();
    Motor_Init();
    Axis_Init();
    Motion_Init();
    Tracking_Init();
    Safety_Init();
    App_Init();

    while (1)
    {
        Communication_Task();
        Safety_Task();
        Tracking_Task();
        Motion_Task();
        App_Task();

        Telemetry_Task();
    }
}
```

真正的高速控制：

```text
Timer ISR
   │
   ▼
MotorStepper_IRQHandler()
```

继续保持现有设计。

---

# 24. 实时调度约束

推荐：

```text
Timer ISR
    │
    └── STEP / 极短硬件状态处理

Motor control task
    │
    └── ~1 kHz

Motion task
    │
    └── 100~200 Hz

Tracking task
    │
    └── 30~100 Hz

Communication task
    │
    └── 由接收数据驱动 / 高频轮询

Telemetry
    │
    └── 10~50 Hz
```

不得出现：

```c
HAL_Delay()
while(!flag);
while(UART_BUSY);
while(!encoder_ready);
```

不得在 ISR 中：

```text
浮点大计算
复杂 PID
UART 发送
视觉处理
大量日志
```

---

# 25. Phase 8：Telemetry

新增：

```text
telemetry/
├── telemetry.c
└── telemetry.h
```

调试数据建议：

```c
typedef struct
{
    float pan_position;
    float pan_target;
    float pan_velocity;

    float tilt_position;
    float tilt_target;
    float tilt_velocity;

    uint8_t app_state;
    uint8_t tracking_state;

    uint8_t fault_flags;

    uint32_t vision_age;
} Telemetry;
```

Telemetry 只读取状态，不参与控制。

---

# 26. 重构后的最终依赖关系

必须满足：

```text
BSP
 ↑
Driver
 ↑
Motor / Encoder
 ↑
Axis
 ↑
Motion
 ↑
Tracking
 ↑
Application
```

通信：

```text
BSP
 ↑
UART Driver
 ↑
Communication
 ↑
Tracking
```

安全：

```text
Safety
 ──→ Axis
 ──→ Motion
 ──→ Application
```

禁止反向依赖，例如：

```text
Motor → Tracking
Encoder → Application
TMC2209 → App
UART → Motor
```

---

# 27. 每阶段验收标准

## Phase 1 Axis

必须做到：

```text
Pan 可以通过 Axis_SetTarget() 移动
Tilt 可以通过 Axis_SetTarget() 移动
```

且：

```text
Axis 不知道视觉
Axis 不知道 UART
```

---

## Phase 2 Safety

验证：

```text
触发限位
↓
禁止继续运动
```

```text
Encoder 异常
↓
进入故障
```

---

## Phase 3 Motion

验证：

```text
Pan/Tilt 同时移动
```

并且：

```text
速度不超过 max_velocity
加速度不超过 max_acceleration
```

---

## Phase 4 Communication

验证：

```text
UART 连续发送视觉帧
↓
VisionTarget 正确产生
```

同时：

```text
CRC 错误
↓
帧丢弃
```

```text
视觉超时
↓
target invalid
```

---

## Phase 5 Coordinate

固定测试：

```text
目标 = 图像中心
→ Pan/Tilt 不应产生额外偏移

目标向右
→ Pan 按定义方向运动

目标向上
→ Tilt 按定义方向运动
```

必须实际确认坐标方向，不允许仅凭代码假设。

---

## Phase 6 Tracking

测试：

```text
目标稳定
目标缓慢移动
目标快速移动
目标抖动
目标突然消失
目标重新出现
```

---

## Phase 7 Application

测试：

```text
IDLE
→ TRACK

TRACK
→ LOST

LOST
→ SEARCH

任意状态
→ FAULT
```

---

# 28. 旧代码清理规则

只有在新模块已经通过测试后才能删除旧逻辑。

例如：

```text
main.c 中的 Pan PID
        ↓
Axis_Pan
        ↓
验证
        ↓
删除 main.c 旧代码
```

禁止：

```text
新 Axis 保留一套 PID
main.c 又保留一套 PID
```

最终每个控制量必须只有一个来源。

---

# 29. Agent 执行规则

执行 Agent 必须遵守：

### 第一原则

**架构优先于局部代码优化。**

不要为了让一个函数更短而破坏层级。

### 第二原则

**先阅读现有代码，再修改。**

尤其检查：

```text
main.c
motor_pid.c
tmc2209.c
mt6701.c
uart.c
```

不得根据文件名猜测 API 行为。

### 第三原则

**每完成一个阶段必须编译。**

推荐：

```text
修改
↓
Build
↓
静态检查
↓
运行测试
↓
记录结果
↓
下一阶段
```

### 第四原则

**不得擅自修改控制参数。**

如果只是架构迁移：

```text
Kp
Kd
Ki
max_speed
max_accel
```

保持原值。

控制参数调整属于独立任务。

### 第五原则

**禁止阻塞式实现。**

新代码不得引入：

```text
HAL_Delay
阻塞 UART
阻塞 SPI
阻塞等待电机
无限轮询
```

---

# 30. 建议的实际执行顺序

执行 Agent 不要一次完成全部任务。

严格按照：

```text
STEP 1
建立 Axis
        ↓
STEP 2
把现有 Pan/Tilt PID 包进 Axis
        ↓
STEP 3
验证双轴闭环
        ↓
STEP 4
建立 Safety
        ↓
STEP 5
建立 Motion
        ↓
STEP 6
迁移视觉通信
        ↓
STEP 7
建立 VisionTarget
        ↓
STEP 8
建立 Coordinate Transform
        ↓
STEP 9
建立 Tracking Manager
        ↓
STEP 10
建立 Application
        ↓
STEP 11
Telemetry
        ↓
STEP 12
清理旧代码
```

---

# 31. 最终完成判据

重构完成后，代码应该能够形成以下闭环：

```text
视觉目标
   ↓
VisionTarget
   ↓
Tracking Manager
   ↓
Coordinate Transform
   ↓
Motion Manager
   ↓
Axis[Pan/Tilt]
   ↓
MotorPID
   ↓
MotorStepper
   ↓
TMC2209
   ↓
Stepper Motor
   ↓
MT6701
   ↓
Axis Feedback
   └──────────────→ 闭环
```

同时：

```text
Encoder / Limit / Driver / Communication
                ↓
          Safety Manager
                ↓
             Stop/Fault
```

最终 `main.c` 不应该再承担业务控制逻辑，而只承担系统初始化和非阻塞任务调度。

---

# 32. 当前阶段禁止做的事情

在 Axis、Safety、Motion 完成并验证之前，暂时不要实现：

```text
复杂目标预测
Kalman Filter
自动搜索算法
复杂轨迹规划
高级 PID
云台姿态解算
复杂视觉算法
```

先建立稳定骨架。

目标不是“代码功能最多”，而是：

```text
单轴稳定
→ 双轴稳定
→ 通信稳定
→ 追踪稳定
→ 再增加高级算法
```
