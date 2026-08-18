# ============================================================================
#  build.ps1 — 一键构建 + 可选串口烧录 (STM32CubeProgrammer 软 Bootloader)
#
# 用法:
#   .\build.ps1                 # 构建 Debug + Release
#   .\build.ps1 -Preset Debug   # 只构建 Debug
#   .\build.ps1 -Flash -ComPort COM9          # 构建 Release 并烧录 (默认串口 COM9)
#   .\build.ps1 -Flash -Preset Debug -ComPort COM9
#
# 烧录说明:
#   - MCU 需先进入软 Bootloader (上电前按住 Boot 键, 或运行时串口发送 dfu/boot 命令)
#   - 应用烧到 0x08008000, 不动 Bootloader 区
# ============================================================================
param(
    [ValidateSet("Debug", "Release", "All")]
    [string]$Preset = "All",
    [switch]$Flash,
    [string]$ComPort = "COM9"
)

$ErrorActionPreference = "Stop"
$root = $PSScriptRoot

# ---- 自动探测 STM32Cube 工具链 (bundles) ----
$bundles = "C:\Users\admin\AppData\Local\stm32cube\bundles"
$cmakeBin = (Get-ChildItem "$bundles\cmake" -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1).FullName + "\bin"
$ninjaBin = (Get-ChildItem "$bundles\ninja" -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1).FullName + "\bin"
$gccBin   = (Get-ChildItem "$bundles\gnu-tools-for-stm32" -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1).FullName + "\bin"
$progBin  = (Get-ChildItem "$bundles\programmer" -Directory -ErrorAction SilentlyContinue | Sort-Object Name -Descending | Select-Object -First 1).FullName + "\bin"

foreach ($d in @($cmakeBin, $ninjaBin, $gccBin)) {
    if (-not (Test-Path $d)) { throw "工具链目录不存在: $d" }
}
$env:Path = "$cmakeBin;$ninjaBin;$gccBin;" + $env:Path

# ---- 构建 ----
$presets = @()
if ($Preset -eq "All") { $presets = @("Debug", "Release") } else { $presets = @($Preset) }

foreach ($p in $presets) {
    Write-Host "== 配置 $p ..." -ForegroundColor Cyan
    cmake --preset $p
    Write-Host "== 构建 $p ..." -ForegroundColor Cyan
    cmake --build --preset $p
    if ($LASTEXITCODE -ne 0) { throw "构建失败: $p" }
}

# ---- 产物清单 ----
Write-Host "`n== 构建产物 ==" -ForegroundColor Green
$elfs = Get-ChildItem "$root\build" -Recurse -Filter "2.8-inch_LCD_Driver.elf" -ErrorAction SilentlyContinue
foreach ($e in $elfs) {
    $bin = Join-Path $e.DirectoryName ($e.BaseName + ".bin")
    if (Test-Path $bin) {
        Write-Host ("{0,-60} {1,10:N0} B" -f $bin.Replace("$root\", ""), (Get-Item $bin).Length)
    }
}

# ---- 烧录 (可选) ----
if ($Flash) {
    $prog = Join-Path $progBin "STM32_Programmer_CLI.exe"
    if (-not (Test-Path $prog)) { throw "未找到 STM32_Programmer_CLI: $prog" }
    $last = $presets[-1]
    $bin = "$root\build\$last\2.8-inch_LCD_Driver.bin"
    if (-not (Test-Path $bin)) { throw "bin 不存在: $bin (请先构建对应 preset)" }

    Write-Host "`n== 经串口 $ComPort 烧录 (软 Bootloader) ..." -ForegroundColor Cyan
    Write-Host "  确保 MCU 已处于软 Bootloader (Boot 键 或 串口发送 dfu/boot)"
    & $prog -c port=$ComPort -u 0x08008000 $bin
    if ($LASTEXITCODE -ne 0) { throw "烧录失败 (检查串口号 / MCU 是否在 Bootloader 中)" }
    Write-Host "`n烧录完成" -ForegroundColor Green
}
