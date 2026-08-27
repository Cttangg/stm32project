# AGENTS.md

STM32F407VET6 firmware collection. Docs/comments/commit messages are in Chinese.

## Layout

- The git repo root **is** `D:\desktop\stm32project` (github.com/Cttangg/stm32project). Each subdirectory is an independent firmware project with its own CubeMX `.ioc`, CMake build, and `build/` artifacts — not packages of one project.
- `DriverLib_King_orz/` — separate git repo (root `.gitignore`d). Reusable driver modules, each self-contained with a README (UART DMA lib, MT6701+TMC2209 42-stepper, soft Bootloader, LCD/touch drivers).
- `stm32-mw-openbl/` — git clone of ST's Open Bootloader middleware, `.gitignore`d, safe to `git pull`. Source of truth for `STM32_OpenBootLoader`; never edit it, customizations live in `Core/Lib/`.
- `STM32_MotorKing_orz/` — bare CubeMX skeleton (USART1/2 + I2C1/2 + SPI1, no user code, no `Core/Lib/`) and **not yet tracked by git**.

## Build

Per project, run inside the project directory (toolchain file `cmake/gcc-arm-none-eabi.cmake` needs `arm-none-eabi-*` on PATH):

```
cmake --preset Debug
cmake --build --preset Debug
# artifact: build/Debug/<ProjectName>.elf
```

- Toolchain is not system-installed; `2.8-inch_LCD_Driver/build.ps1` shows the discovery pattern: newest bundle under `%LOCALAPPDATA%\stm32cube\bundles\` (cmake, ninja, gnu-tools-for-stm32, programmer). It also builds+flashes via soft Bootloader (`-Flash -ComPort COMx`).
- clangd: each project's `.clangd` uses `build/Debug` as compile DB — configure the Debug preset before relying on clangd. The STM32Cube IDE clangd extension config uses `cube`/`cube-cmake` shims and `CUBE_BUNDLE_PATH` (`.vscode/settings.json`).

## CubeMX rules

- Regenerating from `.ioc` overwrites `Core/Src|Inc` and `cmake/stm32cubemx/CMakeLists.txt`; user code survives **only** inside `/* USER CODE BEGIN/END */` blocks (root `CMakeLists.txt` is generated once and is free to edit).
- `STM32F407xx_FLASH.ld` is also regenerated — set Flash Origin/Size in CubeMX `Project Manager → Linker Settings`, not by hand-editing the `.ld`.

## Bootloader / app split (all app projects)

- Flash map: Bootloader `0x08000000` (32 KB, sectors 0-1, self-protected) | App `0x08008000` (480 KB).
- App must call `APP_SetVectorTable()` as the **first line of main(), before `HAL_Init()`** (from `Core/Lib/app_boot_interface.h`).
- RAM last 4 bytes (`0x2001FFFC`) are the boot-jump magic — reserved, keep buffers away from top of RAM.
- App flashing goes over UART via CubeProgrammer **Open Bootloader** mode (NOT "ST Bootloader"): 115200-8E1, sync byte 0x7F, app `.bin` written at `0x08008000`; App triggers it with `dfu`/`boot` UART commands. `arm-none-eabi-objcopy -O binary x.elf x.bin`.
- `app_boot_interface.h` magic constants must match the `STM32_OpenBootLoader` project (`openbootloader_conf.h` + `openbl/main.c`).

## Shared UART library (Core/Lib/uart.c)

- CubeMX prereqs: RX DMA stream **Circular**, TX DMA stream **Normal**, both DMA + USART interrupts enabled.
- `UART_Task()` must be called from the main loop (frame parsing + callbacks run there, not in ISR).

## Project notes

- `2.8-inch_LCD_Driver/` — LCD/touch drivers in `Core/Lib/` with `lcd_conf.h`/`touch_conf.h` board config; `SELF_TEST_ENABLE` in main.c; image→C conversion via `tools/img2c.py`; `doxygen Doxyfile` for docs (output gitignored).
- `42Motor_Driver_STM32/` — MT6701 encoder + TMC2209 stepper closed-loop in `Core/Lib/`; debug UART shell; `flash.gdb` flashes via gdb extended-remote (OpenOCD-style server).
- `STM32_OpenBootLoader/` — the bootloader itself; protocol details and App-migration checklist in its README.