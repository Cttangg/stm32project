/**
 ******************************************************************************
 * @file    app.h
 * @brief   应用层 (L4) — 按键/CALIB/开环运动状态机 + 串口调试打印
 *
 * 使用 (main.c USER CODE 区):
 *   APP_Init();            // 在 MX_xxx_Init() 之后调用一次
 *   while (1) { APP_Task(); }
 ******************************************************************************
 */
#ifndef __APP_H
#define __APP_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化驱动实例 + 运动/仲裁库 + 按键 + 串口 (HAL 外设 Init 之后调用) */
void APP_Init(void);

/** @brief 主循环周期调用: 按键扫描 + 控制 + 串口打印 */
void APP_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_H */
