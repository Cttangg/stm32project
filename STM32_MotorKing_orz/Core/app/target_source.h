/**
 ******************************************************************************
 * @file    target_source.h
 * @brief   目标数据源接口 + 注册表 — 解耦"目标从哪来"与执行内环
 *
 * 背景 (architecture.md §14 / requirements.md §30):
 *   同一套内环 (Axis → MotorPID → STEP) 需要被不同来源驱动:
 *     - direct: 命令/按键给出的绝对角度      (本题 Q1/Q2: 电机角度直接闭环)
 *     - vision: 视觉像素 → 外环 → 角度      (本题 Q3+)
 *   各来源实现同一 TargetSource 接口, 注册进注册表, 运行期由上层选择激活哪个.
 *
 * 约束:
 *   - 数据源只能通过 Motion_SetTarget() 输出目标, 不得直接操作 Axis/电机/GPIO.
 *   - 注册 ≠ 激活. 视觉源只有 enter() 时才注册摄像头帧 (Communication_Init),
 *     Q1/Q2 全程不激活 → 不注册帧, USART2 保持空闲.
 ******************************************************************************
 */
#ifndef __TARGET_SOURCE_H
#define __TARGET_SOURCE_H

#include <stdbool.h>
#include <stdint.h>
#include "motion.h"     /* GimbalSetpoint */

#ifdef __cplusplus
extern "C" {
#endif

/* 前置声明, 避免头文件强耦合 uart.h */
struct UART_Device;

/* ========================================================================= */
/*  数据源标识                                                                 */
/* ========================================================================= */
typedef enum
{
    TARGET_SRC_DIRECT = 0,   /* 命令/按键给定绝对角度 (Q1/Q2 电机角度直闭环) */
    TARGET_SRC_VISION,       /* 视觉像素 → 外环 → 角度 (Q3+) */
    TARGET_SRC_ID_MAX
} TargetSourceId;

/* ========================================================================= */
/*  数据源接口                                                                 */
/* ========================================================================= */
typedef struct TargetSource
{
    TargetSourceId id;
    const char    *name;
    void (*enter)(void);                       /* 激活: 注册帧/使能外环 (可空) */
    void (*exit)(void);                        /* 停用: 反注册/禁用      (可空) */
    void (*update)(uint32_t now_ms);           /* 周期: 产出目标         (可空) */
    bool (*get_target)(GimbalSetpoint *out);   /* 取当前目标             (可空) */
} TargetSource;

/* ========================================================================= */
/*  注册表 API                                                                 */
/* ========================================================================= */

/** @brief 注册一个数据源 (按 id 存入注册表, 重复注册覆盖) */
void TargetSource_Register(const TargetSource *src);

/** @brief 按 id 查找 (未注册返回 NULL) */
const TargetSource *TargetSource_Find(TargetSourceId id);

/** @brief 激活指定数据源 (先 exit 旧源再 enter 新源; 未注册返回 false) */
bool TargetSource_Activate(TargetSourceId id);

/** @brief 当前激活的源 id (无激活返回 TARGET_SRC_ID_MAX) */
TargetSourceId TargetSource_GetActiveId(void);

/** @brief 当前激活的源 (无激活返回 NULL) */
const TargetSource *TargetSource_GetActive(void);

/* ========================================================================= */
/*  内置数据源                                                                 */
/* ========================================================================= */

/** @brief 注册内置 direct / vision 源 (不激活) */
void TargetSource_Init(void);

/** @brief direct 源输入: 设定绝对目标角度 (deg) */
void TargetSource_DirectSet(float pan_deg, float tilt_deg);

/** @brief vision 源绑定摄像头串口 (须在激活 vision 源之前调用) */
void TargetSource_VisionBind(struct UART_Device *cam);

#ifdef __cplusplus
}
#endif

#endif /* __TARGET_SOURCE_H */
