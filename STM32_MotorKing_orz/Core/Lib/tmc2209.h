/**
 ******************************************************************************
 * @file    tmc2209.h
 * @brief   TMC2209 步进电机驱动板 — 句柄化 GPIO 控制驱动
 *
 * 设计:
 *   - 句柄化多电机: GPIO 引脚 + 运行状态封装进 TMC2209_HandleTypeDef,
 *     所有 API 接收句柄指针, 传哪个句柄就控哪个电机.
 *   - 跨 STM32: 本头文件包含 main.h (CubeMX 按芯片自动引入对应 HAL);
 *     GPIO 引脚模式/时钟由 MX_GPIO_Init() 统一处理, 驱动内不写时钟使能.
 *   - 本驱动只做 GPIO 控制 (EN/DIR/MS), 不做 STEP 脉冲:
 *     STEP 由 motor_stepper.c 的 TIM14 硬件定时器非阻塞产生.
 *
 * 上层操作流程:
 *   1. TMC2209_Init(&h, &PinConfig)  绑定 GPIO (用 main.h 生成的 MOTOR_X_*_Pin/Port 宏)
 *   2. MotorStepper_Init(&h, &htim14) 初始化 STEP 定时器 (见 motor_stepper.h)
 *   3. TMC2209_Enable(&h)            使能驱动
 *   4. MotorStepper_SetVelocity(v)   运行: 正=CCW, 负=CW, 0=停 (硬件定时器发脉冲)
 *   5. TMC2209_Disable(&h)           释放电机
 *   可选: TMC2209_SetDirection / TMC2209_SetMicrostep / MotorStepper_MoveSteps
 *
 * 控制方式: STEP / DIR / EN / MS1 / MS2 均为 GPIO 数字控制,
 *          不使用 UART(PDN/USART) 寄存器配置.
 *
 * 板载引脚图 (TMC2209 驱动板):
 *
 *                 +---------------------------+
 *                 |  +---------------------+  |
 *                 |  |  电位器 (电流调节)   |  |
 *                 |  +---------------------+  |
 *                 |                           |
 *           EN  ──┤ [1]                   [1] ├── VM    电机电源 (+12V)
 *          MS1  ──┤ [2]                   [2] ├── GND   电源地
 *          MS2  ──┤ [3]                   [3] ├── M1B   电机 A 相
 *          PDN  ──┤ [4]      TMC2209      [4] ├── M1A   电机 A 相
 *         USART ──┤ [5]                   [5] ├── M2A   电机 B 相
 *          CLK  ──┤ [6]                   [6] ├── M2B   电机 B 相
 *          STEP ──┤ [7]                   [7] ├── VIO   逻辑供电 (3.3V/5V)
 *          DIR  ──┤ [8]                   [8] ├── GND   逻辑地
 *                 +---------------------------+
 *
 * 控制引脚说明:
 *   EN    使能端 (低有效: 低=使能, 高=禁止)
 *   STEP  步进脉冲输入 (上升沿触发一步)
 *   DIR   方向控制
 *   MS1/MS2  细分选择 (见下表)
 *   PDN   关断/串口 (内置下拉, GPIO 控制无需处理)
 *   USART UART 配置 (GPIO 控制无需处理)
 *   CLK   时钟输入 (无需处理)
 *
 * 微步设置 (TMC2209 红排针):
 *   Microstep  | MS1  | MS2
 *   -----------+------+------
 *   1/8  Step  | Low  | Low
 *   1/16 Step  | High | High
 *   1/32 Step  | High | Low
 *   1/64 Step  | Low  | High
 *
 * ========================================================================
 *    TMC2209 (黄排针) 驱动板 与 42 步进电机 (6Pin) 接线示意图
 * ========================================================================
 *
 *   [TMC2209 驱动板]                  [42 步进电机 6Pin 接口]
 *    ┌───────────────┐                (XH2.54, 凸起朝上)
 *    │   [M1A] ──────┼──────────────► Pin1  (红)  线圈1 始端 (A+)
 *    │   [M1B] ──────┼──────────────┐
 *    │   [M2A] ──────┼─────► Pin3   (绿)  线圈2 始端 (B+)
 *    │   [M2B] ──────┼──────────────┐
 *    └───────────────┘              │
 *                                    ├──► Pin4  (黄)  线圈1 末端 (A-)
 *                                    ├──► Pin6  (蓝)  线圈2 末端 (B-)
 *                                    └──► Pin2 / Pin5 空脚, 不接
 *
 *   接线表:
 *   ┌────────────┬──────────┬───────────┬──────────────────┐
 *   │ TMC2209    │ 电机 Pin │ 典型线色   │ 说明             │
 *   ├────────────┼──────────┼───────────┼──────────────────┤
 *   │ M1A        │ Pin1     │ 红        │ 线圈1 始端 (A+)   │
 *   │ M1B        │ Pin4     │ 黄        │ 线圈1 末端 (A-)   │
 *   │ M2A        │ Pin3     │ 绿        │ 线圈2 始端 (B+)   │
 *   │ M2B        │ Pin6     │ 蓝        │ 线圈2 末端 (B-)   │
 *   │ —          │ Pin2     │ —         │ 空脚, 不接        │
 *   │ —          │ Pin5     │ —         │ 空脚, 不接        │
 *   └────────────┴──────────┴───────────┴──────────────────┘
 *
 *   * 线色仅供参考, 请用万用表通断测量确认 (两两相通即同一线圈两端)
 *   * 相定义: M1 = A 相(线圈1), M2 = B 相(线圈2)
 *   * 只要把一个线圈的两头分别接到 MxA / MxB 即可
 *   * 若电机反转, 将任意一相的两根线对调
 * ========================================================================
 *
 * 使用示例 (多电机, STEP 由硬件定时器驱动):
 *   TMC2209_HandleTypeDef motor_x, motor_y;
 *
 *   TMC2209_Init(&motor_x, &(TMC2209_PinConfig){
 *       .en   = {MOTOR_X_EN_GPIO_Port,   MOTOR_X_EN_Pin},
 *       .step = {MOTOR_X_STEP_GPIO_Port, MOTOR_X_STEP_Pin},
 *       .dir  = {MOTOR_X_DIR_GPIO_Port,  MOTOR_X_DIR_Pin},
 *       .ms1  = {MOTOR_X_MS1_GPIO_Port,  MOTOR_X_MS1_Pin},
 *       .ms2  = {MOTOR_X_MS2_GPIO_Port,  MOTOR_X_MS2_Pin},
 *   });
 *   MotorStepper_Init(&motor_x, &htim14);
 *
 *   TMC2209_Enable(&motor_x);
 *   MotorStepper_SetVelocity(500.0f);      — 500 steps/s 运行 (CCW)
 *   MotorStepper_SetVelocity(0.0f);        — 停止
 *   TMC2209_Disable(&motor_x);             — 释放电机
 *
 *   前提: 引脚需在 CubeMX 中配置为 GPIO_Output, 时钟由 MX_GPIO_Init() 使能
 ******************************************************************************
 */
#ifndef __TMC2209_H
#define __TMC2209_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================= */
/*  一、细分选择 (MS1/MS2 硬件引脚)                                           */
/* ========================================================================= */
typedef enum {
    TMC2209_MICROSTEP_8  = 0,  /* MS1=Low  MS2=Low  → 1/8  */
    TMC2209_MICROSTEP_16 = 1,  /* MS1=High MS2=High → 1/16 */
    TMC2209_MICROSTEP_32 = 2,  /* MS1=High MS2=Low  → 1/32 */
    TMC2209_MICROSTEP_64 = 3,  /* MS1=Low  MS2=High → 1/64 */
} TMC2209_Microstep;

/* ========================================================================= */
/*  二、方向                                                                    */
/* ========================================================================= */
typedef enum {
    TMC2209_DIR_CCW = 0,   /* 逆时针 */
    TMC2209_DIR_CW  = 1,   /* 顺时针 */
} TMC2209_Dir;

/* ========================================================================= */
/*  三、引脚与句柄定义                                                         */
/* ========================================================================= */
typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} TMC2209_PinDef;

typedef struct {
    TMC2209_PinDef en;    /* 使能   (低有效) */
    TMC2209_PinDef step;  /* 步进脉冲 */
    TMC2209_PinDef dir;   /* 方向   */
    TMC2209_PinDef ms1;   /* 细分 1 */
    TMC2209_PinDef ms2;   /* 细分 2 */
} TMC2209_PinConfig;

/** @brief 电机句柄: 引脚绑定 + 运行状态 (应用层定义, 每电机一个) */
typedef struct {
    TMC2209_PinConfig pins;        /* GPIO 引脚绑定 */
    uint8_t           enabled;     /* 运行状态: 0=禁止 1=使能 */
    uint8_t           dir;         /* 当前方向 */
    uint32_t          step_count;  /* 累计步数 */
    TMC2209_Microstep microstep;   /* 当前细分 */
} TMC2209_HandleTypeDef;

/* ========================================================================= */
/*  四、公开 API (全部接收句柄指针, 多电机独立控制)                             */
/* ========================================================================= */

/** @brief 绑定 GPIO 并初始化句柄 (引脚模式/时钟由 CubeMX MX_GPIO_Init 处理) */
void TMC2209_Init(TMC2209_HandleTypeDef *h, const TMC2209_PinConfig *cfg);

/** @brief 使能驱动 (EN 拉低) */
void TMC2209_Enable(TMC2209_HandleTypeDef *h);

/** @brief 禁止驱动 (EN 拉高) */
void TMC2209_Disable(TMC2209_HandleTypeDef *h);

/** @brief 设置方向 */
void TMC2209_SetDirection(TMC2209_HandleTypeDef *h, TMC2209_Dir dir);

/** @brief 设置细分 (写 MS1/MS2) */
void TMC2209_SetMicrostep(TMC2209_HandleTypeDef *h, TMC2209_Microstep ms);

/* 注意: STEP 脉冲由硬件定时器生成 (见 motor_stepper.c), 不再提供阻塞式
   TMC2209_Step()/TMC2209_StepN() 软件脉冲接口 */

#ifdef __cplusplus
}
#endif

#endif /* __TMC2209_H */
