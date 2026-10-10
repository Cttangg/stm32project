/**
 ******************************************************************************
 * @file    calib.h
 * @brief   KEY_CALIB 标定流程 — 释放电机 → 手动摆点 → 逐点记录
 *
 * 流程 (KEY_CALIB 每次按下):
 *   第 1 次: 释放两轴电机 (TMC2209 EN 断开, 可手动扳动云台) 进入标定
 *   第 2~6 次: 依次记录当前编码器角度到 O → 左上 → 右上 → 右下 → 左下
 *   记录满 5 点: 自动退出, 重新锁定电机 (目标=当前角度, 防跳变)
 *
 * 标定期间读取编码器绝对值 (电机已释放, 角度随手动变化), 不走 Axis 闭环.
 ******************************************************************************
 */
#ifndef __CALIB_H
#define __CALIB_H

#include <stdbool.h>
#include <stdint.h>
#include "axis.h"
#include "mt6701.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 前置声明 (避免强耦合 uart.h) */
struct UART_Device;

/* ========================================================================= */
/*  配置 (依赖注入)                                                            */
/* ========================================================================= */
typedef struct
{
    Axis                 *pan;        /* Pan 轴 (释放/锁定) */
    Axis                 *tilt;       /* Tilt 轴 */
    MT6701_HandleTypeDef *enc_pan;    /* Pan 编码器 (读绝对角度) */
    MT6701_HandleTypeDef *enc_tilt;   /* Tilt 编码器 */
    struct UART_Device   *uart;       /* 调试口 (每次按下打印当前数值, 可空) */
} CalibConfig;

/* ========================================================================= */
/*  API                                                                       */
/* ========================================================================= */

/** @brief 初始化 (绑定轴/编码器) */
void Calib_Init(const CalibConfig *cfg);

/** @brief 周期调用 (主循环): 轮询 KEY_CALIB 边沿并推进标定流程 */
void Calib_Update(uint32_t now_ms);

/** @brief 是否处于标定流程中 */
bool Calib_IsActive(void);

/** @brief 下一个待记录点的序号 (0..GP_COUNT) */
uint8_t Calib_Index(void);

#ifdef __cplusplus
}
#endif

#endif /* __CALIB_H */
