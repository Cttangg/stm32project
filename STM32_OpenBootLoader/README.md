# ST Open Bootloader — STM32F407VET6

本工程 `STM32_OpenBootLoader` 即 **Bootloader** 工程（32 KB，位于 0x08000000）。
基于 ST 官方 Open Bootloader（`stm32-mw-openbl`，BSD 许可），仅 USART 接口，
与 CubeProgrammer 的 "Open Bootloader" 模式兼容。

## 分区

| 区域 | 地址 | 大小 |
|---|---|---|
| Bootloader（本工程） | `0x08000000` | 32 KB（扇区 0..1） |
| 用户 App | `0x08008000` | 480 KB（扇区 2..11） |

- Bootloader 自带自保护：扇区 0..1 不可被主机擦除 / 整体擦除 / 写入。
- 上电逻辑：App 请求进入 → 等主机；App 有效 → 等主机 1 s（无主机则跳转 App）；
  无有效 App → 一直等主机。

## 工程内文件位置

```
STM32_OpenBootLoader/
├── Core/                      Bootloader 应用代码（与兄弟项目同结构）
│   ├── Inc/                   CubeMX 生成头文件（main.h 等）
│   ├── Src/                   CubeMX 生成源码（main.c、stm32f4xx_it.c 等）
│   └── Lib/                   用户定制驱动/接口：
│                               platform.h / openbootloader_conf.h / interfaces_conf.h
│                               usart_interface.c/.h  （寄存器级，无需 LL 驱动）
│                               flash_interface.c/.h  （F4 HAL，自保护）
│                               ram_interface.c/.h、optionbytes_interface.c/.h
│                               common_interface.c/.h
│                               app_openbootloader.c/.h（工程级胶水）
│                               app_boot_interface.h  （预留给上层 APP，仅头文件）
├── STM32F407xx_FLASH.ld      链接脚本（FLASH 32 KB @ 0x08000000）
├── stm32-mw-openbl/          ST 官方 MW 仓库（git clone，源码唯一来源，可 git pull）
│   ├── Core/                  openbl_core.c/.h
│   ├── Modules/Mem/           openbl_mem.c/.h
│   └── Modules/USART/         openbl_usart_cmd.c/.h
└── cmake/stm32cubemx/CMakeLists.txt   OpenBL 源文件/头文件路径已接好
```

MW 源码在根目录 `stm32-mw-openbl/`（独立 git 仓库，`git pull` 升级不影响应用代码）；
`Core/Lib` 是 STM32F4 定制实现，升级 MW 时无需改动。

## 硬件准备

| 用途 | 工具 | 说明 |
|---|---|---|
| 首次烧 BL | ST-Link（SWD） | 只烧一次；BL 有自保护，之后基本不需要 |
| 固件升级 | USB-TTL（CH340 等）→ COM | PA9←RX、PA10→TX、GND 共地，115200-8E1 |
| 调试 | ST-Link（SWD） | 全片擦除 / 读寄存器 / 复位 |

## 已验证状态

2026-08：BL 已在目标板实测，`COM8` 直连完成完整协议会话：

```
SYNC(0x7F)     -> 0x79 (ACK)
GET(0x00 0xFF) -> 0x79 0x0B 0x31 <11个命令> 0x79   (协议 V3.1)
GET_ID(0x02)   -> 0x79 0x01 0x04 0x13 0x79        (设备ID 0x413)
读选项字节      -> 0x1FFFC000 读 16 字节成功
```

CubeProgrammer 用 **Open Bootloader** 模式连接即可，无需 BOOT0。

## 构建与烧录

1. 构建（CMake，arm-none-eabi-gcc）：
   ```
   cmake --preset Debug
   cmake --build --preset Debug
   ```
   产物：`build/Debug/STM32_OpenBootLoader.elf`（约 17 KB）。
2. 烧录：SWD 直接写 `0x08000000`。

> 注意：`main.c` 的 OpenBL 代码全部位于 CubeMX `USER CODE` 段内，
> 用 CubeMX 重新生成也不会丢失（.ioc 已清理掉 USART1/DMA）。

## 上层 APP 迁移注意事项

给已有 App 工程（42Motor、LCD 等）接本 Bootloader 时需注意：

1. **链接偏移（必须）**：App 的 `STM32F407xx_FLASH.ld` 改为
   `FLASH (rx) : ORIGIN = 0x8008000, LENGTH = 0x78000`（480 KB）。
   > CubeMX 重新生成会覆盖 .ld，最稳做法是在 CubeMX
   > `Project Manager → Linker Settings` 里设 Flash Origin/Size。

2. **向量表 VTOR（必须，main 第一行）**：`APP_SetVectorTable()` 必须放在
   `HAL_Init()` 之前——`HAL_Init()` 会启用 SysTick 中断，若 VTOR 仍指向
   0x08000000，中断会跳去 Bootloader 的向量表而死循环。放在
   `/* USER CODE BEGIN 1 */` 段内，CubeMX 再生成不会丢。

3. **跳转前后状态（App 无需清理）**：Bootloader 跳转前已关闭并释放 USART
   （PA9/PA10）、停止 SysTick 中断、设好 MSP；App 从 `Reset_Handler` 全新启动，
   `HAL_Init` 复位全部外设，自行 `MX_DMA_Init` + `MX_USART1_UART_Init`。

4. **串口 DMA 无冲突**：Bootloader 用轮询、App 用 DMA（如 USART1 的
   DMA2-Stream2/7）——时间互斥，引脚已释放，App 全新初始化；App 的
   USART1/DMA 中断 handler 由 App 自己的 `stm32f4xx_it.c` 提供。

5. **RAM 末 4 字节 `0x2001FFFC`**：跳转标志，App 不要占用。正常安全
   （栈顶 0x20020000 向下、.bss 向上）；若 App 把大缓冲/大堆设计在 RAM
   顶端附近，需预留这 4 字节。

6. **时钟一致性**：下载协议固定 115200-8E1（BL 按 PCLK2=84 MHz 计算），
   App 后续改时钟不影响 Bootloader。

7. **App 有效性校验（BL 自动做）**：Bootloader 上电检查 App 首 8 字节
   （SP∈RAM 范围 且 Reset 向量∈[0x08008000, 0x08080000)），合法才跳转，
   否则一直等主机——App 被擦/不完整时不会跳转，而是等编程器（防变砖）。

8. **首次烧录顺序**：新板先用 SWD 烧 Bootloader（一次），之后全部走串口；
   Bootloader 有自保护（扇区 0..1 不可擦写），一般不会坏，仅 BL 损坏 /
   误开 RDP / 需要全擦时才回到 SWD。

## 上层 APP 接口调用（已预留）

在 App 工程（如 LCD、42 电机驱动）中 `#include "app_boot_interface.h"`
（把该头文件拷入 App 工程的 `Core/Lib/`）：

```c
/* main() 第一行（HAL_Init 之前） */
APP_SetVectorTable();            // SCB->VTOR = 0x08008000

/* 需要升级时（如串口命令 boot/dfu 触发） */
APP_JumpToBootloader();          // 写 RAM 标志 + 软复位，复位后停在主机等待
```

## 迁移到其他 App 工程（快速清单）

给同芯片（STM32F407VE）的 App 工程接本 BL，只需 4 步：

1. 拷贝 `Core/Lib/app_boot_interface.h` → App 工程的 `Core/Lib/`
2. App 链接脚本改一行：`FLASH (rx) : ORIGIN = 0x8008000, LENGTH = 0x78000`
3. `main()` 第一行（`HAL_Init` 之前）加 `APP_SetVectorTable();`
4. （可选）升级命令里调 `APP_JumpToBootloader();` 即可随时进 BL

> BL 工程本身是独立工程，**不需要**把 openbl 源码拷进 App 工程。
> 详见 `DriverLib_King_orz/基于STM32CubePrg的软BL` 自包含模块（可整体拷走）。

## 与 ROM Bootloader（AN2606）的区别

本 Bootloader 是**自定义 ST Open Bootloader**（Flash 内，上电即运行），
不是芯片出厂 ROM Bootloader（AN2606，需 BOOT0=1 触发）。两者的连接参数不同：

| | ROM Bootloader (AN2606) | 本 Open Bootloader |
|---|---|---|
| 触发 | BOOT0=3.3V | 上电自动运行，无需跳线 |
| 同步字节 | 0x7F | **0x7F** |
| 波特率 | 自动识别 | 固定 115200 |
| CubeProgrammer 模式 | ST Bootloader | **Open Bootloader** |

本 Open Bootloader 的 USART 同步字节与 ROM Bootloader 相同，都是 **0x7F**，
CubeProgrammer 选 **Open Bootloader** 模式即可（不要选 ST Bootloader 模式，
那是给 ROM 的，两者协议虽同但连接目标不同）。

## 串口升级工作流

**硬件**：USB-TTL → PA9(RX)、PA10(TX)、GND 共地，**115200，8 数据位 + 偶校验（8E1）**——这是 ST Open Bootloader 协议的固定帧格式，CubeProgrammer 打开 Open Bootloader 模式时会自动用偶校验。注意 STM32F4 上 8E1 必须配置为 **9 位字长（M=1）+ 使能偶校验（PCE=1）**，否则波形对不上、字节会被丢弃（见 `usart_interface.c`）。

**方式 A：App 运行中升级（推荐，无时序压力）**

1. App 串口发 `boot`（42Motor 的 tty 命令 / LCD 的 `dfu`）触发
   `APP_JumpToBootloader()` → 软复位。
2. Bootloader 检测到跳转标志 → 清除 → 停在"等主机"（不跳 App）。
3. CubeProgrammer：选 UART + 串口号，波特率 115200，连接方式选
   **Open Bootloader** → Connect。
4. 握手：CubeProgrammer 发同步字节 **0x7F** → BL 回 0x79，读到设备 ID `0x413`
   （验证连接成功）。
5. 加载 App 固件 `.bin`（起始地址填 `0x08008000`）→ Download
   （自动擦扇区 2..11 再写入）。
6. Disconnect → 复位（或 CubeProgrammer 里 `GO 0x08008000`）。
7. 复位后：无跳转标志、App 有效 → 等 1 s 无主机 → 自动跳 App，新固件运行。

**方式 B：App 无法运行 / 首次上电（无 App 时 BL 一直等，不限时）**

1. 连接 CubeProgrammer（同方式 A 第 3-4 步）。
2. 若 App 已存在但想进 BL：**断电 → 点 Connect → 上电**，在 1 s 窗口内
   完成握手（超时就跳 App 了）。
3. 下载 + 复位，同上。

**固件文件**：App 编译出 `.elf` 后转换：

```
arm-none-eabi-objcopy -O binary xxx.elf xxx.bin
```

CubeProgrammer 下载 bin 填起始地址 `0x08008000`；hex 自带地址不用填。

**验证**：GET_ID 返回 `0x413`；下载后可读 `0x08008000`，应为 App 向量表
（SP=`0x20020000`）。
