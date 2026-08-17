# UART — STM32 通用串口库

> **版本**: 1.0.0 | **平台**: STM32F407VETx | **底层**: HAL_UART + DMA

## 目录

- [1. 概述](#1-概述)
- [2. 架构](#2-架构)
- [3. CubeMX 前置配置](#3-cubemx-前置配置)
- [4. 快速开始](#4-快速开始)
- [5. API 参考](#5-api-参考)
- [6. 帧协议](#6-帧协议)
- [7. 多串口扩展](#7-多串口扩展)
- [8. 调试统计](#8-调试统计)
- [9. 设计说明](#9-设计说明)
- [10. 常见问题](#10-常见问题)

---

## 1. 概述

UART 是一个类似 **Linux tty** 的串口抽象层：应用只调 `UART_Send` / `UART_Task` / `UART_RegisterFrame`，把寄存器、DMA、中断、环形缓冲、帧解析全部封装在库内。

### 核心特性

| 特性 | 说明 |
|------|------|
| **非阻塞发送** | 数据入 TX RingBuffer → DMA 自动后台发送，`UART_Send` 立即返回 |
| **非阻塞接收** | DMA CIRCULAR 持续接收 → IDLE 空闲检测 → RX RingBuffer |
| **可自定义帧协议** | `get_length` / `check` / `callback` 回调适配任意协议，多协议链表共存 |
| **多串口** | 端口表预留 UART1~UART6，当前注册 USART1 |
| **零 malloc** | 所有缓冲区编译期静态分配 |
| **ISR 安全** | 单生产者-单消费者无锁 RingBuffer + DMB 屏障 |
| **裸机 / RTOS** | `UART_Task()` 主循环周期调用，或包成 FreeRTOS 任务 |

### 文件

```
Core/Lib/
├── uart.h    # 公开 API 头文件
└── uart.c    # 全部实现
```

### 资源占用

```
RAM:   ~13.5 KB  (rx_dma 256B + rx_ring 1KB + tx_ring 512B 等, 每端口)
FLASH: ~5 KB     (库本体)
```

---

## 2. 架构

```
┌──────────────────────────────────────────────┐
│              Application (main.c)             │
│   UART_Send() / UART_Task() / FrameCallback  │
├──────────────────────────────────────────────┤
│                   uart.h                      │  ← 唯一公开接口
├──────────────────────────────────────────────┤
│                   uart.c                      │
│  ┌──────────────────────────────────────────┐ │
│  │  端口实例表 / 回调分发 (find_inst)         │ │
│  ├─────────────────┬────────────────────────┤ │
│  │  RX 子系统       │  TX 子系统              │ │
│  │  DMA CIRCULAR   │  TX RingBuffer          │ │
│  │  + IDLE 检测     │  → DMA NORMAL 分块      │ │
│  │  + NDTR 位置跟踪 │  + TxCplt 链式续发      │ │
│  ├─────────────────┴────────────────────────┤ │
│  │  Frame Parser — 用户自定义协议状态机       │ │
│  └──────────────────────────────────────────┘ │
├──────────────────────────────────────────────┤
│        STM32 HAL (UART + DMA) + CubeMX 配置    │
├──────────────────────────────────────────────┤
│              USART + DMA 硬件                  │
└──────────────────────────────────────────────┘
```

**数据流:**

```
【接收】                                 【发送】
UART RXD                                UART_Send()
  ↓ DMA CIRCULAR (rx_dma_buf[256])        ↓
  ↓ IDLE/HT/TC 事件回调                   TX RingBuffer[512]
  ↓ 读 NDTR → 增量搬运                    ↓ HAL_UART_Transmit_DMA
RX RingBuffer[1024]                     tx_temp[128] 分块
  ↓ UART_Task()                          ↓ TxCplt 链式续发
Frame Parser 状态机                     USART TXD
  ↓ 帧完整 + 校验通过
Application Callback(data, len)
```

---

## 3. CubeMX 前置配置

库不自己配置 DMA，**依赖 CubeMX 生成的外设初始化**：

1. **USART**: 启用对应 USART，分配 TX/RX 引脚，勾选 `global interrupt`
   - USART1: PA9(TX) / PA10(RX)，AF7
   - USART2: PA2(TX) / PA3(RX)，AF7
2. **DMA**: 给 USART 的 TX/RX 各配一个流
   - **RX 必须 `Circular`**（持续接收回卷）
   - **TX 必须 `Normal`**（一次发完，Circular 会无限重发）
   - 字节对齐，`Memory Increment = Enable`
   - USART1 参考: RX=DMA2_Stream2 Ch4, TX=DMA2_Stream7 Ch4
3. **NVIC**: DMA 流中断 + USART 全局中断全部勾选
4. main.c 中 `MX_DMA_Init()` 和 `MX_USARTx_UART_Init()` 会由 CubeMX 自动生成

库在 `UART_Open()` 里用 `HAL_UARTEx_ReceiveToIdle_DMA()` 启动 RX，无需额外寄存器操作。

---

## 4. 快速开始

```c
#include "uart.h"

/* 逐字节回显 */
static void echo_cb(UART_Port port, uint8_t *data, uint16_t len) {
    uint16_t w;
    UART_Send(port, data, len, &w);
}

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_USART1_UART_Init();

    UART_Init();
    UART_Open(UART_P1);

    static UART_Frame echo_frame = {
        .header_len = 0,      /* 0 = 逐字节回调 */
        .callback   = echo_cb,
    };
    UART_RegisterFrame(UART_P1, &echo_frame);

    while (1) {
        UART_Task();          /* 必须周期调用 */
    }
}
```

---

## 5. API 参考

### 端口枚举

```c
typedef enum {
    UART_P1 = 0,   /* USART1  PA9/PA10  APB2 AF7  DMA2-S7(TX) DMA2-S2(RX) */
    UART_P2 = 1,   /* USART2  PA2/PA3   APB1 AF7  DMA1-S6(TX) DMA1-S5(RX) */
    UART_P3 = 2,   /* USART3  PB10/PB11 APB1 AF7                         */
    UART_P4 = 3,
    UART_P5 = 4,
    UART_P6 = 5,   /* USART6  PC6/PC7   APB2 AF8                         */
    UART_PMAX
} UART_Port;
```

> 命名带 `P` 前缀以避开 CMSIS 外设宏 `UART4`/`UART5`。

### 生命周期

| 函数 | 说明 |
|------|------|
| `void UART_Init(void)` | 初始化端口表、RingBuffer。外设 Init 之后调用一次 |
| `UART_Status UART_Open(port)` | 打开端口并启动 DMA RX（IDLE 接收）。返回 `UART_OK` 或错误码 |
| `UART_Status UART_Close(port)` | 关闭端口，`HAL_UART_Abort` 停止 DMA |

### 发送

| 函数 | 说明 |
|------|------|
| `UART_Status UART_Send(port, data, len, &written)` | 非阻塞入队。`written` 返回实际入队字节数，`UART_ERR_FULL` 表示缓冲满 |
| `uint8_t UART_IsSending(port)` | 1=正在发送，0=空闲 |

### 接收

| 函数 | 说明 |
|------|------|
| `uint16_t UART_Available(port)` | RX 环形缓冲可读字节数 |
| `uint16_t UART_Read(port, buf, max_len)` | 读取原始字节 |
| `UART_Status UART_RegisterFrame(port, &frame)` | 注册帧协议和回调（链表，最多 4 个） |

### 调度

| 函数 | 说明 |
|------|------|
| `void UART_Task(void)` | 主循环周期调用：环形 → 帧解析 → 回调，并处理帧超时 |

### 统计

| 函数 | 说明 |
|------|------|
| `const UART_Stats *UART_GetStats(port)` | 返回累计统计 |

---

## 6. 帧协议

```c
typedef uint16_t (*UART_FrameGetLength)(const uint8_t *buf);  /* 返回帧总长, 0=错误 */
typedef uint8_t  (*UART_FrameCheck)(const uint8_t *buf, uint16_t len); /* 1=通过 */
typedef void     (*UART_FrameCallback)(UART_Port port, uint8_t *data, uint16_t len);

typedef struct UART_Frame {
    uint8_t            header[4];     /* 帧头字节 */
    uint8_t            header_len;    /* 帧头长度 (0=逐字节回调) */
    uint16_t           max_len;       /* 单帧最大长度 */
    UART_FrameGetLength get_length;   /* 长度提取 (NULL=帧头长度即帧长) */
    UART_FrameCheck     check;        /* 校验 (NULL=不校验) */
    UART_FrameCallback  callback;     /* 帧完成回调 */
    uint32_t           timeout_ms;    /* 帧超时 */
    /* 内部状态, 勿动 */
} UART_Frame;
```

### 示例：电机控制协议 (AA 55 LEN CMD DATA CRC16)

```c
static const uint8_t motor_hdr[] = {0xAA, 0x55};

static uint16_t motor_len(const uint8_t *b) { return b[2]; }        /* LEN 字段 */
static uint8_t  motor_chk(const uint8_t *b, uint16_t len) {
    return (crc16_modbus(b, len) == 0);
}
static void motor_cb(UART_Port port, uint8_t *b, uint16_t len) {
    switch (b[3]) {  /* CMD */
        case 0x01: motor_set_position(*(int32_t*)&b[4]); break;
        case 0x02: motor_set_speed(*(int32_t*)&b[4]);    break;
        case 0x03: motor_enable(b[4]);                   break;
    }
}

static UART_Frame motor_frame = {
    .header = {0xAA, 0x55},
    .header_len = 2,
    .max_len = 64,
    .get_length = motor_len,
    .check = motor_chk,
    .callback = motor_cb,
    .timeout_ms = 50,
};
UART_RegisterFrame(UART_P1, &motor_frame);
```

---

## 7. 多串口扩展

当前只注册了 USART1，启用 USART2：

1. **CubeMX**: 配置 USART2 + DMA（RX Circular / TX Normal）+ 中断
2. **uart.c** `UART_Init()` 增加一行：

   ```c
   g_uart[UART_P2].huart = &huart2;   /* usart.h 已声明 extern huart2 */
   ```

3. **使用**: `UART_Open(UART_P2)` / `UART_Send(UART_P2, ...)` / `UART_RegisterFrame(UART_P2, ...)`

> 注意：库的 HAL 回调（`HAL_UARTEx_RxEventCallback` 等）已按 `huart` 指针分发到对应端口，多口无需额外接线。

---

## 8. 调试统计

```c
typedef struct {
    uint32_t rx_bytes;        /* 累计接收字节数 */
    uint32_t tx_bytes;        /* 累计发送字节数 */
    uint32_t rx_overflow;     /* RX 环形溢出次数 */
    uint32_t tx_overflow;     /* TX 环形溢出次数 */
    uint32_t uart_errors;     /* UART 错误 (ORE/FE/NE) 累计 */
    uint32_t rx_events;       /* 接收事件次数 (IDLE/HT/TC) */
} UART_Stats;

const UART_Stats *s = UART_GetStats(UART_P1);
```

---

## 9. 设计说明

### 9.1 为什么 RX 用 Circular + IDLE，TX 用 Normal？

- **RX Circular**：DMA 持续把收到的字节写入 `rx_dma_buf[256]` 并回卷，永不停止。IDLE（线路空闲>1 字节）触发一帧结束；HT/TC（半满/满）也触发，防止缓冲溢出。
- **TX Normal**：每次只发一段（≤128B），发完停止。若用 Circular，DMA 会无限重发同一段数据。
- 在回调里读 `__HAL_DMA_GET_COUNTER`（NDTR）得到 DMA 当前写位置，把 `last_pos → 当前` 的字节增量搬进软件环形，天然处理回卷。

### 9.2 为什么用 HAL_UARTEx_ReceiveToIdle_DMA 而不是手写 DMA？

- 完全走 HAL API，不自写 DMA 寄存器代码，与 CubeMX 生成配置一致。
- `HAL_UARTEx_ReceiveToIdle_DMA` 内部使能 IDLE 中断并在事件时回调 `HAL_UARTEx_RxEventCallback(huart, Size)`，配合 NORMAL 或 CIRCULAR 都可用。

### 9.3 RingBuffer 无锁设计

- `write` 仅生产者（ISR）修改，`read` 仅消费者（Task/主循环）修改，M4 上 16 位指针天然原子。
- 容量 = 2^n，用 `& (size-1)` 替代 `%`。

### 9.4 TX 链式发送

```
UART_Send("ABC")   → TX Ring: [A][B][C]
                        ↓ HAL_UART_Transmit_DMA
                     DMA: A B C → 发送中
                        ↓ TC 中断 → HAL_UART_TxCpltCallback
UART_Send("DEF")   → TX Ring: [D][E][F]
                        ↓ tx_kick 续发
                     DMA: D E F → 继续
```

---

## 10. 常见问题

### Q: `UART_Send` 返回 `UART_ERR_FULL`？
TX RingBuffer 满（512B），部分数据已入队（看 `written`）。可在主循环重试发送剩余部分。

### Q: 如何确认发送完成？
```c
while (UART_IsSending(UART_P1)) { UART_Task(); }
```

### Q: `UART_Task()` 必须调用吗？
必须。RX 数据由 ISR 搬入环形缓冲，但**帧解析和用户回调在 `UART_Task()` 里执行**。主循环里至少每 1ms 调一次。

### Q: 回调里能调用 `UART_Send` 吗？
能。回调运行在 `UART_Task()` 上下文（非 ISR），可安全调用任何 API（回显就是这么做的）。

### Q: 接收乱码/丢字节？
- 检查波特率与主机一致（库不校验）
- 检查 RX DMA 是否为 Circular、TX 是否为 Normal
- 检查 `UART_Stats.uart_errors` 是否有 ORE/FE

### Q: 适配其他 STM32 系列？
改 `uart.c` 的 `#include "usart.h"` 为对应系列头文件，CubeMX 按该系列生成 DMA 配置即可，核心算法无需改。

---

## 版本历史

| 版本 | 日期 | 变更 |
|------|------|------|
| 1.0.0 | 2026-08-14 | 基于 HAL_UART + DMA 的通用串口库，RX Circular+IDLE，TX 链式发送，可插拔帧协议 |

## 许可

MIT License — 可自由用于商业和开源项目。
